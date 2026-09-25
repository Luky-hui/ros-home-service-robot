#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import hashlib
import json
import math
import os
import queue
import re
import subprocess
import threading
import time
import uuid

import numpy as np
import rospy
import sherpa_onnx
import sounddevice as sd
import soundfile as sf
import yaml

try:
    import edge_tts
except ImportError:
    edge_tts = None
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, SetBoolResponse
from rei_voice.msg import REIResult, REIResultNlp
from rei_voice.srv import REINlp, REINlpResponse
from rei_voice.srv import REIPlayer, REIPlayerResponse
from rei_voice.srv import REITts, REITtsResponse
from robot_audio.srv import Awake, AwakeResponse
from robot_audio.srv import Collect, CollectResponse
from robot_audio.srv import Control, ControlRequest
from robot_audio.srv import Nav, NavRequest
from robot_audio.srv import robot_iat, robot_iatResponse
from robot_audio.srv import robot_semanteme, robot_semantemeResponse
from robot_audio.srv import robot_tts, robot_ttsResponse
from robot_audio.srv import up_sync, up_syncResponse


PUNCTUATION = "，。！？、,.!?：:；;“”\"'（）() "
CHINESE_DIGITS = {
    "零": 0, "一": 1, "二": 2, "两": 2, "三": 3, "四": 4,
    "五": 5, "六": 6, "七": 7, "八": 8, "九": 9,
}


class LocalVoiceNode:
    def __init__(self):
        rospy.init_node("local_voice_node")
        config_file = rospy.get_param("~config_file")
        with open(config_file, "r", encoding="utf-8") as stream:
            self.config = yaml.safe_load(stream)

        self.continuous_listen = bool(rospy.get_param("~continuous_listen", True))
        self.execute_intents = bool(rospy.get_param("~execute_intents", True))
        self.sample_rate = int(self.config["audio"]["sample_rate"])
        self.block_size = int(self.config["audio"]["block_size"])
        self.input_device = self.config["audio"]["input_device"]
        self.output_device = self.config["audio"]["output_device"]
        self.configure_pulse_devices()
        self.record_seconds = float(self.config["audio"]["record_seconds"])
        self.tts_leading_silence = float(
            self.config["audio"].get("tts_leading_silence", 0.25)
        )
        self.tts_warmup_silence = float(
            self.config["audio"].get("tts_warmup_silence", 0.35)
        )
        self.output_directory = self.config["audio"]["output_directory"]
        os.makedirs(self.output_directory, exist_ok=True)
        self.tts_cache_directory = self.config["audio"]["tts_cache_directory"]
        os.makedirs(self.tts_cache_directory, exist_ok=True)
        audio_config = self.config["audio"]
        self.tts_backend = audio_config.get("tts_backend", "sherpa_onnx")
        self.tts_edge_voice = audio_config.get("tts_edge_voice", "zh-CN-XiaoxiaoNeural")
        self.tts_edge_rate = audio_config.get("tts_edge_rate", "-8%")
        self.tts_edge_volume = audio_config.get("tts_edge_volume", "+0%")
        self.tts_edge_pitch = audio_config.get("tts_edge_pitch", "+0Hz")
        self.tts_edge_online_generation = bool(
            audio_config.get("tts_edge_online_generation", False)
        )
        self.tts_sherpa_speed = float(audio_config.get("tts_sherpa_speed", 1.0))

        self.recording_enabled = True
        self.playing = threading.Event()
        self.audio_queue = queue.Queue(maxsize=40)
        self.tts_lock = threading.Lock()
        self.awake_until = 0.0

        self.asr = self._create_asr()
        self.kws = self._create_kws()
        self.vad = self._create_vad()
        self.tts = self._create_tts()

        topics = self.config["topics"]
        self.command_pub = rospy.Publisher(topics["command"], String, queue_size=10)
        self.transcript_pub = rospy.Publisher(topics["transcript"], String, queue_size=10)
        self.intent_pub = rospy.Publisher(topics["intent"], String, queue_size=10)
        self.level_pub = rospy.Publisher(
            "/local_voice/audio_level",
            Float32,
            queue_size=10,
        )
        self.state_pub = rospy.Publisher("/REITopic/AIUIState", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher("/REITopic/AIUIResult", REIResult, queue_size=10)

        self.control_client = rospy.ServiceProxy("/voice_control", Control)
        self.nav_client = rospy.ServiceProxy("/voice_nav", Nav)

        rospy.Service("/REIService/RecordAudio", SetBool, self.handle_record_audio)
        rospy.Service("/REIService/PcmPlayer", REIPlayer, self.handle_rei_player)
        rospy.Service("/REIService/voice_nlp", REINlp, self.handle_rei_nlp)
        rospy.Service("/REIService/voice_tts", REITts, self.handle_rei_tts)

        rospy.Service("/voice_awake", Awake, self.handle_awake)
        rospy.Service("/voice_collect", Collect, self.handle_collect)
        rospy.Service("/voice_iat", robot_iat, self.handle_iat)
        rospy.Service("/voice_aiui", robot_semanteme, self.handle_semanteme)
        rospy.Service("/voice_tts", robot_tts, self.handle_robot_tts)
        rospy.Service("/voice_up_sync", up_sync, self.handle_up_sync)

        self.state_pub.publish(String("EVENT_STATE: STATE_READY"))
        self.stream = None
        if self.continuous_listen:
            try:
                self.stream = sd.RawInputStream(
                    samplerate=self.sample_rate,
                    blocksize=self.block_size,
                    device=self.input_device,
                    dtype="int16",
                    channels=1,
                    callback=self.audio_callback,
                )
                self.stream.start()
                threading.Thread(target=self.recognition_loop, daemon=True).start()
            except Exception as exc:
                self.stream = None
                self.continuous_listen = False
                rospy.logwarn(
                    "Audio input stream unavailable; continuous listen disabled: %s",
                    exc,
                )

        rospy.loginfo("Local token-free voice bridge is ready")

    def configure_pulse_devices(self):
        sources = self.pactl_names("sources")
        sinks = self.pactl_names("sinks")

        requested_source = os.environ.get("PULSE_SOURCE", "")
        default_source = self.pactl_default("Default Source")
        if requested_source and requested_source not in sources:
            rospy.logwarn("Configured PULSE_SOURCE is unavailable: %s", requested_source)
            requested_source = ""
        if not requested_source and default_source:
            os.environ["PULSE_SOURCE"] = default_source
            rospy.loginfo("Using PulseAudio source: %s", default_source)

        if self.output_device and self.output_device not in sinks:
            rospy.logwarn("Configured audio output is unavailable: %s", self.output_device)
            default_sink = self.pactl_default("Default Sink")
            if default_sink:
                self.output_device = default_sink
                rospy.loginfo("Using PulseAudio sink: %s", self.output_device)

    def pactl_names(self, kind):
        try:
            result = subprocess.run(
                ["pactl", "list", "short", kind],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=3,
            )
        except Exception:
            return set()
        names = set()
        for line in result.stdout.splitlines():
            parts = line.split("\t")
            if len(parts) >= 2:
                names.add(parts[1])
        return names

    def pactl_default(self, field):
        try:
            result = subprocess.run(
                ["pactl", "info"],
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=3,
            )
        except Exception:
            return ""
        prefix = field + ": "
        for line in result.stdout.splitlines():
            if line.startswith(prefix):
                return line[len(prefix):].strip()
        return ""

    def _create_asr(self):
        models = self.config["models"]
        return sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=models["sensevoice_model"],
            tokens=models["sensevoice_tokens"],
            num_threads=4,
            language="zh",
            use_itn=True,
        )

    def _create_kws(self):
        kws_config = self.config.get("kws", {})
        if not bool(kws_config.get("enabled", False)):
            return None
        models = self.config["models"]
        required = ["kws_tokens", "kws_encoder", "kws_decoder", "kws_joiner", "kws_keywords"]
        missing = [name for name in required if not models.get(name)]
        if missing:
            rospy.logwarn("KWS disabled, missing config keys: %s", ",".join(missing))
            return None
        try:
            spotter = sherpa_onnx.KeywordSpotter(
                tokens=models["kws_tokens"],
                encoder=models["kws_encoder"],
                decoder=models["kws_decoder"],
                joiner=models["kws_joiner"],
                keywords_file=models["kws_keywords"],
                num_threads=int(kws_config.get("num_threads", 2)),
                sample_rate=float(self.sample_rate),
                keywords_score=float(kws_config.get("keywords_score", 1.0)),
                keywords_threshold=float(kws_config.get("keywords_threshold", 0.25)),
                provider="cpu",
            )
            rospy.loginfo("KWS ready: %s", models["kws_keywords"])
            return spotter
        except Exception as exc:
            rospy.logwarn("KWS unavailable, falling back to SenseVoice ASR: %s", exc)
            return None

    def _create_vad(self):
        config = sherpa_onnx.VadModelConfig()
        config.silero_vad.model = self.config["models"]["vad_model"]
        config.silero_vad.threshold = 0.5
        audio_config = self.config.get("audio", {})
        config.silero_vad.min_silence_duration = float(audio_config.get("vad_min_silence_duration", 0.45))
        config.silero_vad.min_speech_duration = 0.25
        config.silero_vad.max_speech_duration = float(audio_config.get("vad_max_speech_duration", 10.0))
        config.sample_rate = self.sample_rate
        config.num_threads = 2
        return sherpa_onnx.VoiceActivityDetector(config, 30)

    def _create_tts(self):
        models = self.config["models"]
        vits = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=models["tts_model"],
            lexicon=models["tts_lexicon"],
            tokens=models["tts_tokens"],
            length_scale=1.0,
        )
        model = sherpa_onnx.OfflineTtsModelConfig(
            vits=vits,
            num_threads=4,
            provider="cpu",
            debug=False,
        )
        return sherpa_onnx.OfflineTts(
            sherpa_onnx.OfflineTtsConfig(
                model=model,
                rule_fsts=",".join(self.config["models"]["tts_rule_fsts"]),
                max_num_sentences=1,
                silence_scale=0.2,
            )
        )

    def audio_callback(self, indata, frames, callback_time, status):
        del frames, callback_time
        if status:
            rospy.logwarn_throttle(5.0, "Audio input status: %s", status)
        level_samples = np.frombuffer(indata, dtype=np.int16).astype(np.float32)
        if level_samples.size:
            rms = float(np.sqrt(np.mean(level_samples * level_samples)))
            self.level_pub.publish(Float32(rms))
        if not self.recording_enabled or self.playing.is_set():
            return
        try:
            self.audio_queue.put_nowait(bytes(indata))
        except queue.Full:
            try:
                self.audio_queue.get_nowait()
                self.audio_queue.put_nowait(bytes(indata))
            except queue.Empty:
                pass

    def recognition_loop(self):
        while not rospy.is_shutdown():
            try:
                data = self.audio_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            samples = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
            self.vad.accept_waveform(samples)
            while not self.vad.empty():
                segment = self.vad.front
                keyword_text = self.detect_keyword_samples(segment.samples)
                if keyword_text:
                    text = self.normalize_text(keyword_text)
                    source = "KWS"
                else:
                    text = self.normalize_text(self.transcribe_samples(segment.samples))
                    source = "ASR"
                self.vad.pop()
                if text:
                    self.process_text(text, source=source)

    def detect_keyword_samples(self, samples):
        if self.kws is None:
            return ""
        kws_config = self.config.get("kws", {})
        chunk_samples = int(kws_config.get("chunk_samples", 1600))
        stream = self.kws.create_stream()
        for start in range(0, len(samples), chunk_samples):
            stream.accept_waveform(self.sample_rate, samples[start:start + chunk_samples])
            while self.kws.is_ready(stream):
                self.kws.decode_stream(stream)
            result = self.kws.get_result(stream)
            if result:
                return result
        stream.input_finished()
        while self.kws.is_ready(stream):
            self.kws.decode_stream(stream)
        return self.kws.get_result(stream)

    def transcribe_samples(self, samples):
        stream = self.asr.create_stream()
        stream.accept_waveform(self.sample_rate, samples)
        self.asr.decode_stream(stream)
        return stream.result.text

    def normalize_text(self, text):
        text = text.strip()
        for mark in PUNCTUATION:
            text = text.replace(mark, "")
        return text

    def contains_wake_word(self, text):
        return any(word in text for word in self.config["wake"]["words"])

    def strip_wake_word(self, text):
        for word in self.config["wake"]["words"]:
            text = text.replace(word, "")
        return text

    def process_text(self, text, source="ASR"):
        rospy.loginfo("%s: %s", source, text)
        self.transcript_pub.publish(String(text))
        self.publish_iat(text)

        if self.contains_wake_word(text):
            self.awake_until = time.time() + float(self.config["wake"]["awake_seconds"])
            self.state_pub.publish(String("EVENT_WAKEUP"))
            remaining = self.strip_wake_word(text)
            if not remaining:
                self.synthesize(self.config["wake"]["reply"], True)
                return
            text = remaining

        intent = self.parse_intent(text)
        self.publish_nlp(text, intent)
        self.intent_pub.publish(String(json.dumps(intent, ensure_ascii=False)))
        command_text = text
        if intent["name"] == "service_mission" and intent["slots_value"]:
            rospy.loginfo(
                "Publishing mission command: %s",
                command_text,
            )
        self.command_pub.publish(String(command_text))
        if self.execute_intents:
            self.execute_intent(intent)

    def parse_intent(self, text):
        short_welcome = text in ("广州", "广州馆", "广", "州", "深圳", "深圳馆", "深", "圳", "深镇", "镇", "上海", "上海馆", "上", "海", "吉林", "吉林馆", "吉", "林", "麒麟", "麒麟馆", "麒林", "麒林馆", "吉宾馆", "北京", "北京馆", "北", "京", "光州")
        venue_in_text = any(
            keyword in text
            for keyword in (
                "广州",
                "广州馆",
                "广",
                "州",
                "深圳",
                "深圳馆",
                "深",
                "圳",
                "深镇",
                "镇",
                "上海",
                "上海馆",
                "上",
                "海",
                "吉林",
                "吉林馆",
                "吉",
                "林",
                "麒麟",
                "麒麟馆",
                "麒林",
                "麒林馆",
                "吉宾馆",
                "北京",
                "北京馆",
                "北",
                "京",
                "光州",
            )
        )
        guide_action = any(action in text for action in ("参观", "带我去", "带我到", "去"))
        if (venue_in_text and guide_action) or short_welcome:
            return {
                "name": "service_mission",
                "answer": self.config["answers"]["guide"],
                "slots_name": ["mission"],
                "slots_value": ["welcome"],
            }
        if "巡检" in text and ("任务" in text or text == "巡检"):
            return {
                "name": "service_mission",
                "answer": self.config["answers"]["patrol"],
                "slots_name": ["mission"],
                "slots_value": ["patrol"],
            }
        mission_phrases = self.config["mission_phrases"]
        if any(phrase in text for phrase in mission_phrases["national_guide"]):
            return {
                "name": "home_service_mission",
                "answer": self.config["answers"]["guide"],
                "slots_name": ["mission"],
                "slots_value": ["guide"],
            }
        if any(phrase in text for phrase in mission_phrases["smart_assistant"]):
            return {
                "name": "home_service_mission",
                "answer": "好的，我去看看冰箱里有什么。",
                "slots_name": ["mission"],
                "slots_value": ["assistant"],
            }
        find_targets = {
            "手机": "cell phone",
            "书包": "backpack",
            "背包": "backpack",
            "水瓶": "water bottle",
        }
        find_action = any(action in text for action in ("找", "寻找", "查找", "在哪", "哪里", "哪儿", "位置", "看见", "看到", "有没有"))
        if any(phrase in text for phrase in mission_phrases["find_object"]) or find_action:
            for spoken_name, target_label in find_targets.items():
                if spoken_name in text:
                    return {
                        "name": "home_service_mission",
                        "answer": "好的，我去找" + spoken_name + "。",
                        "slots_name": ["mission", "target"],
                        "slots_value": ["find_object", target_label],
                    }
        for mission_name, phrases in mission_phrases.items():
            if any(phrase in text for phrase in phrases):
                if mission_name == "welcome":
                    answer = self.config["answers"]["guide"]
                elif mission_name == "patrol":
                    answer = self.config["answers"]["patrol"]
                else:
                    answer = "好的，开始执行任务。"
                return {
                    "name": "service_mission",
                    "answer": answer,
                    "slots_name": ["mission"],
                    "slots_value": [mission_name],
                }

        if "取消导航" in text or "停止导航" in text:
            return {
                "name": "robot_nav",
                "answer": self.config["answers"]["cancel_navigation"],
                "slots_name": ["action", "position"],
                "slots_value": ["cancel", "取消导航"],
            }

        if "参观" in text and ("全部" in text or "一圈" in text or "一下" in text):
            return {
                "name": "robot_guid",
                "answer": self.config["answers"]["guide"],
                "slots_name": [],
                "slots_value": [],
            }

        for spoken_name, nav_order in self.config["navigation"]["aliases"].items():
            if spoken_name in text and (
                text == spoken_name
                or any(word in text for word in ("去", "前往", "导航", "带我", "回"))
            ):
                return {
                    "name": "robot_nav",
                    "answer": "好的，这就带您去" + spoken_name + "。",
                    "slots_name": ["action", "position"],
                    "slots_value": ["navigate", nav_order],
                }

        direction = self.find_direction(text)
        if direction:
            value, unit = self.find_measurement(text, direction)
            return {
                "name": "robot_control",
                "answer": "好的，现在就" + direction + "。",
                "slots_name": ["direction", "value", "unit"],
                "slots_value": [direction, self.format_number(value), unit],
            }

        return {
            "name": "chat",
            "answer": self.rule_answer(text),
            "slots_name": [],
            "slots_value": [],
        }

    @staticmethod
    def find_direction(text):
        for direction in ("前进", "后退", "左移", "右移", "左转", "右转"):
            if direction in text:
                return direction
        return ""

    def find_measurement(self, text, direction):
        match = re.search(r"(\d+(?:\.\d+)?)\s*(毫米|厘米|米|度)", text)
        if match:
            return float(match.group(1)), match.group(2)

        match = re.search(r"([零一二两三四五六七八九十百点]+)(毫米|厘米|米|度)", text)
        if match:
            return self.chinese_number(match.group(1)), match.group(2)

        if direction in ("左转", "右转"):
            return 90.0, "度"
        return 0.3, "米"

    @staticmethod
    def chinese_number(text):
        if "点" in text:
            integer, decimal = text.split("点", 1)
            integer_value = LocalVoiceNode.chinese_number(integer) if integer else 0
            decimal_value = 0.0
            for index, char in enumerate(decimal, 1):
                decimal_value += CHINESE_DIGITS.get(char, 0) * (10 ** -index)
            return integer_value + decimal_value
        if text == "十":
            return 10.0
        if "百" in text:
            left, right = text.split("百", 1)
            return CHINESE_DIGITS.get(left, 1) * 100 + LocalVoiceNode.chinese_number(right)
        if "十" in text:
            left, right = text.split("十", 1)
            return CHINESE_DIGITS.get(left, 1) * 10 + CHINESE_DIGITS.get(right, 0)
        value = 0
        for char in text:
            value = value * 10 + CHINESE_DIGITS.get(char, 0)
        return float(value)

    @staticmethod
    def format_number(value):
        if float(value).is_integer():
            return str(int(value))
        return str(value)

    def rule_answer(self, text):
        if "你是谁" in text:
            return "我是魔力元宝机器人，很高兴为您服务。"
        if "你好" in text:
            return "你好，很高兴见到你。"
        if "谢谢" in text:
            return "不客气，很高兴帮到你。"
        return self.config["answers"]["unknown"]

    def execute_intent(self, intent):
        name = intent["name"]
        try:
            if name == "robot_control":
                direction, raw_value, unit = intent["slots_value"]
                value = float(raw_value)
                if unit == "厘米":
                    value /= 100.0
                elif unit == "毫米":
                    value /= 1000.0
                elif unit == "度":
                    value = math.radians(value)
                request = ControlRequest()
                request.controlInfo = [direction, intent["answer"]]
                request.value = value
                self.control_client(request)
            elif name == "robot_nav":
                request = NavRequest()
                request.nav_order = intent["slots_value"][1]
                self.nav_client(request)
            elif name == "robot_guid":
                request = NavRequest()
                request.nav_order = "guidAround"
                self.nav_client(request)
            elif name in ("chat", "service_mission", "home_service_mission"):
                self.synthesize(intent["answer"], True)
        except Exception as exc:
            rospy.logwarn("Intent execution failed: %s", exc)

    def publish_iat(self, text):
        message = REIResult()
        message.sid = uuid.uuid4().hex
        message.type = "iat"
        message.iat = text
        message.anwser = ""
        self.result_pub.publish(message)

    def publish_nlp(self, text, intent):
        message = REIResult()
        message.sid = uuid.uuid4().hex
        message.type = "nlp"
        message.iat = text
        message.anwser = intent["answer"]
        item = REIResultNlp()
        item.name = intent["name"]
        item.query = text
        item.index = 0
        item.slots_name = intent["slots_name"]
        item.slots_value = intent["slots_value"]
        message.intent = [item]
        self.result_pub.publish(message)

    def tts_cache_path(self, text):
        cache_identity = "|".join(
            [
                self.tts_backend,
                self.tts_edge_voice,
                self.tts_edge_rate,
                self.tts_edge_volume,
                self.tts_edge_pitch,
                str(self.tts_sherpa_speed),
                text,
            ]
        )
        cache_key = hashlib.sha256(cache_identity.encode("utf-8")).hexdigest()
        return os.path.join(self.tts_cache_directory, "tts_%s.wav" % cache_key)

    def synthesize_uncached(self, text, filename):
        if self.tts_backend == "edge_tts" and self.tts_edge_online_generation:
            try:
                self.synthesize_edge_tts(text, filename)
                return
            except Exception as exc:
                rospy.logwarn("Edge TTS failed, falling back to local MeloTTS: %s", exc)
        elif self.tts_backend == "edge_tts":
            rospy.logwarn("Edge TTS cache miss, online generation disabled; using local MeloTTS")
        self.synthesize_sherpa_tts(text, filename)

    def synthesize_sherpa_tts(self, text, filename):
        audio = self.tts.generate(text, sid=0, speed=self.tts_sherpa_speed)
        samples = np.asarray(audio.samples, dtype=np.float32)
        self.write_tts_wav(filename, samples, audio.sample_rate)

    def synthesize_edge_tts(self, text, filename):
        if edge_tts is None:
            raise RuntimeError("edge_tts Python package is not installed")
        temporary_mp3 = filename + ".tmp.mp3"
        converted_wav = filename + ".converted.wav"
        try:
            communicate = edge_tts.Communicate(
                text,
                self.tts_edge_voice,
                rate=self.tts_edge_rate,
                volume=self.tts_edge_volume,
                pitch=self.tts_edge_pitch,
            )
            asyncio.run(communicate.save(temporary_mp3))
            subprocess.run(
                [
                    "ffmpeg",
                    "-y",
                    "-loglevel",
                    "error",
                    "-i",
                    temporary_mp3,
                    "-ar",
                    "44100",
                    "-ac",
                    "1",
                    converted_wav,
                ],
                check=True,
                timeout=60,
            )
            samples, sample_rate = sf.read(converted_wav, dtype="float32")
            samples = np.asarray(samples, dtype=np.float32)
            if samples.ndim > 1:
                samples = np.mean(samples, axis=1).astype(np.float32)
            self.write_tts_wav(filename, samples, int(sample_rate))
        finally:
            for path in (temporary_mp3, converted_wav):
                try:
                    os.remove(path)
                except OSError:
                    pass

    def write_tts_wav(self, filename, samples, sample_rate):
        samples = np.asarray(samples, dtype=np.float32)
        peak = float(np.max(np.abs(samples))) if samples.size else 0.0
        if peak > 0.0:
            samples = samples * min(8.0, 0.92 / peak)
        if self.tts_leading_silence > 0.0:
            silence_count = int(sample_rate * self.tts_leading_silence)
            silence = np.zeros(silence_count, dtype=np.float32)
            samples = np.concatenate((silence, samples))
        temporary_filename = filename + ".tmp.wav"
        sf.write(temporary_filename, samples, sample_rate)
        os.replace(temporary_filename, filename)

    def synthesize(self, text, play):
        with self.tts_lock:
            filename = self.tts_cache_path(text)
            self.state_pub.publish(String("EVENT_STATE: STATE_WORKING"))
            if os.path.exists(filename) and os.path.getsize(filename) > 44:
                rospy.loginfo("TTS cache hit: %s", text)
            else:
                rospy.loginfo("TTS cache miss: %s", text)
                self.synthesize_uncached(text, filename)
            if play:
                self.play_audio(filename)
            message = REIResult()
            message.sid = uuid.uuid4().hex
            message.type = "tts"
            message.iat = ""
            message.anwser = text
            self.result_pub.publish(message)
            self.state_pub.publish(String("EVENT_STATE: STATE_READY"))
            return filename

    def play_audio(self, filename):
        self.playing.set()
        try:
            if self.tts_warmup_silence > 0.0:
                subprocess.run(
                    [
                        "paplay",
                        "--volume=1",
                        "--device=" + self.output_device,
                        self.create_silence_file(self.tts_warmup_silence),
                    ],
                    check=False,
                    timeout=5,
                )
            subprocess.run(
                [
                    "paplay",
                    "--volume=65536",
                    "--device=" + self.output_device,
                    filename,
                ],
                check=True,
                timeout=120,
            )
        finally:
            self.playing.clear()
            self._clear_audio_queue()

    def create_silence_file(self, seconds):
        filename = os.path.join(self.output_directory, "warmup_silence.wav")
        sample_rate = 44100
        sample_count = max(1, int(sample_rate * seconds))
        sf.write(filename, np.zeros(sample_count, dtype=np.float32), sample_rate)
        return filename

    def _clear_audio_queue(self):
        while True:
            try:
                self.audio_queue.get_nowait()
            except queue.Empty:
                return

    def transcribe_file(self, filename):
        samples, sample_rate = sf.read(filename, dtype="float32", always_2d=True)
        if samples.shape[1] != 1 or sample_rate != self.sample_rate:
            converted = os.path.join(self.output_directory, "converted_%s.wav" % uuid.uuid4().hex)
            subprocess.run(
                [
                    "ffmpeg", "-loglevel", "error", "-y", "-i", filename,
                    "-ar", str(self.sample_rate), "-ac", "1", "-c:a", "pcm_s16le",
                    converted,
                ],
                check=True,
            )
            samples, sample_rate = sf.read(converted, dtype="float32", always_2d=True)
        return self.normalize_text(self.transcribe_samples(samples[:, 0]))

    def record_file(self, seconds=None):
        seconds = float(seconds if seconds is not None else self.record_seconds)
        filename = os.path.join(self.output_directory, "record_%s.wav" % uuid.uuid4().hex)
        frames = int(seconds * self.sample_rate)
        recording = sd.rec(
            frames,
            samplerate=self.sample_rate,
            channels=1,
            dtype="int16",
            device=self.input_device,
        )
        sd.wait()
        sf.write(filename, recording, self.sample_rate, subtype="PCM_16")
        return filename

    def handle_record_audio(self, request):
        self.recording_enabled = bool(request.data)
        state = "started" if self.recording_enabled else "stopped"
        return SetBoolResponse(success=True, message="Local recording " + state)

    def handle_rei_player(self, request):
        try:
            self.play_audio(request.PcmPath)
            return REIPlayerResponse(success=True, message="played")
        except Exception as exc:
            return REIPlayerResponse(success=False, message=str(exc))

    def handle_rei_nlp(self, request):
        text = self.normalize_text(request.text)
        intent = self.parse_intent(text)
        self.publish_nlp(request.text, intent)
        self.intent_pub.publish(String(json.dumps(intent, ensure_ascii=False)))
        self.command_pub.publish(String(text))
        if self.execute_intents:
            self.execute_intent(intent)
        return REINlpResponse(success=True, message=intent["answer"])

    def handle_rei_tts(self, request):
        try:
            filename = self.synthesize(request.text, request.is_play)
            return REITtsResponse(success=True, filePath=filename, message="ok")
        except Exception as exc:
            return REITtsResponse(success=False, filePath="", message=str(exc))

    def handle_awake(self, request):
        return AwakeResponse(awake_flag=self.contains_wake_word(request.text))

    def handle_collect(self, request):
        if not request.collect_flag:
            return CollectResponse(voice_filename="")
        return CollectResponse(voice_filename=self.record_file())

    def handle_iat(self, request):
        try:
            return robot_iatResponse(text=self.transcribe_file(request.audiopath))
        except Exception as exc:
            rospy.logwarn("IAT failed: %s", exc)
            return robot_iatResponse(text="")

    def handle_semanteme(self, request):
        if request.mode == 1:
            text = self.normalize_text(request.textorpath)
        elif request.mode == 2:
            text = self.transcribe_file(request.textorpath)
        else:
            raise rospy.ServiceException("mode must be 1 or 2")
        intent = self.parse_intent(text)
        response = robot_semantemeResponse()
        response.tech = "local"
        response.iat = text
        response.anwser = intent["answer"]
        response.intent = intent["name"]
        response.slots_name = intent["slots_name"]
        response.slots_value = intent["slots_value"]
        return response

    def handle_robot_tts(self, request):
        return robot_ttsResponse(audiopath=self.synthesize(request.text, request.play))

    @staticmethod
    def handle_up_sync(request):
        del request
        return up_syncResponse(result=True)

    def spin(self):
        rospy.spin()
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()


if __name__ == "__main__":
    LocalVoiceNode().spin()
