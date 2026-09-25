#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import re
import subprocess
import time


PUNCTUATION = " \t\r\n,.!?;:\uff0c\u3002\uff01\uff1f\uff1b\uff1a\u3001\"'\u201c\u201d\u2018\u2019()\uff08\uff09"


def normalize_text(text):
    text = text or ""
    for mark in PUNCTUATION:
        text = text.replace(mark, "")
    return text


def has_any(text, phrases):
    return any(phrase and phrase in text for phrase in phrases)


class SemanticIntentClassifier:
    """Hybrid intent guard for RAICOM service-group spoken commands.

    Deterministic rules own all task execution decisions. The local LLM is only a
    fallback for short interference replies and never has direct navigation authority.
    """

    TASK_NONE = "none"
    TASK_GUIDE = "guide"
    TASK_ASSISTANT = "assistant"
    TASK_FIND = "find_object"

    def __init__(self, config=None):
        config = config or {}
        self.enabled = bool(config.get("enabled", False))
        self.model_enabled = bool(config.get("model_enabled", False))
        self.model_path = config.get("model_path", "")
        self.reply_model_path = config.get("reply_model_path", self.model_path)
        self.llama_binary = config.get("llama_binary", "")
        self.timeout = float(config.get("timeout", 8.0))
        self.max_tokens = int(config.get("max_tokens", 128))
        self.temperature = float(config.get("temperature", 0.0))
        self.polish_rule_replies = bool(config.get("polish_rule_replies", True))
        self.last_model_latency = 0.0

    def classify(self, command, expected_mission=None):
        command = command or ""
        text = normalize_text(command)
        if not text:
            return self._result(False, self.TASK_NONE, reply="")

        rule_result = self._classify_by_rules(command, text, expected_mission)
        if rule_result is not None:
            result = self._with_natural_reply(rule_result, command, expected_mission)
            result["command"] = text
            return result

        unknown_result = self._result(
            False,
            self.TASK_NONE,
            interference_type="unknown",
            reply="抱歉，这个功能我暂时不支持。",
            source="fallback",
        )
        result = self._with_natural_reply(unknown_result, command, expected_mission)
        result["command"] = text
        return result


    def _with_natural_reply(self, result, command, expected_mission):
        if result.get("execute_task"):
            return result
        if result.get("interference_type") != "unknown":
            return result
        if not self.polish_rule_replies:
            return result
        reply = self._generate_reply(command, expected_mission, result)
        if not reply:
            return result
        polished = dict(result)
        polished["reply"] = reply
        polished["source"] = result.get("source", "rule") + "+model_reply"
        return polished

    def _generate_reply(self, command, expected_mission, result):
        if not self.model_enabled:
            return ""
        if not self.llama_binary or not os.path.isfile(self.llama_binary):
            return ""
        if not self.reply_model_path or not os.path.isfile(self.reply_model_path):
            return ""
        fallback = str(result.get("reply", "")).strip()
        prompt = self._build_reply_prompt(command, expected_mission, result, fallback)
        data = self._run_model_json(prompt, self.reply_model_path)
        if not data:
            return ""
        reply = str(data.get("reply", "")).strip()
        if not self._reply_is_safe(reply):
            return ""
        return reply

    def _run_model_json(self, prompt, model_path=None):
        model_path = model_path or self.model_path
        cmd = [
            self.llama_binary,
            "-m", model_path,
            "-p", prompt,
            "-n", str(self.max_tokens),
            "--temp", str(self.temperature),
            "--top-p", "1",
            "-no-cnv",
            "--simple-io",
            "--no-display-prompt",
        ]
        start = time.time()
        try:
            proc = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=self.timeout,
                check=False,
            )
        except Exception:
            return None
        self.last_model_latency = time.time() - start
        return self._extract_json(proc.stdout)

    @staticmethod
    def _reply_is_safe(reply):
        if not reply:
            return False
        if len(reply) > 48:
            return False
        blocked = [
            "\u8bf7\u8ddf\u6211\u6765", "\u5f00\u59cb\u6267\u884c", "\u6211\u8fd9\u5c31\u53bb",
            "\u9a6c\u4e0a\u51fa\u53d1", "\u5df2\u5f00\u59cb", "execute_task", "find_object",
            "assistant", "guide", "JSON",
        ]
        return not has_any(reply, blocked)

    def _build_reply_prompt(self, command, expected_mission, result, fallback):
        return (
            "\u53ea\u8f93\u51faJSON\uff0c\u683c\u5f0f\u4e3a{\\\"reply\\\":\\\"...\\\"}\u3002\n"
            "\u7528\u6237\u8fd9\u53e5\u8bdd\u4e0d\u662f\u4efb\u52a1\u547d\u4ee4\uff0c\u4e0d\u80fd\u5bfc\u822a\uff0c\u4e0d\u80fd\u8bf4\u8bf7\u8ddf\u6211\u6765\uff0c\u4e0d\u80fd\u8bf4\u5f00\u59cb\u6267\u884c\u3002\n"
            "\u4f60\u8981\u76f4\u63a5\u56de\u7b54\u7528\u6237\uff0c\u81ea\u7136\u53cb\u597d\uff0c32\u5b57\u4ee5\u5185\u3002\n"
            "\u4e0d\u8981\u7f16\u9020\u5929\u6c14\u3001\u65f6\u95f4\u3001\u73b0\u573a\u4fe1\u606f\u3002\n"
            "\u5982\u679c\u7528\u6237\u8981\u8bb2\u7b11\u8bdd\uff0c\u7ed9\u4e00\u53e5\u6e29\u548c\u77ed\u7b11\u8bdd\uff1b\u5982\u679c\u4e0d\u786e\u5b9a\uff0c\u8bf7\u7b80\u77ed\u56de\u5e94\u5e76\u7b49\u5f85\u6307\u4ee4\u3002\n"
            "\u7528\u6237:%s\n"
            "JSON:"
        ) % command

    def _classify_by_rules(self, command, text, expected_mission):
        guide_negative = [
            "\u53c2\u89c2\u5b8c\u4e86", "\u53c2\u89c2\u5b8c\u6210", "\u53c2\u89c2\u5b8c\u6bd5", "\u53c2\u89c2\u7ed3\u675f", "\u5bfc\u89c8\u5b8c\u6210", "\u5bfc\u89c8\u7ed3\u675f", "\u4e0d\u7528\u53c2\u89c2", "\u4e0d\u7528\u5e26\u6211\u53c2\u89c2", "\u4e0d\u53c2\u89c2",
            "\u5148\u4e0d\u53c2\u89c2", "\u522b\u53c2\u89c2", "\u4e0d\u8981\u53c2\u89c2", "\u505c\u6b62\u53c2\u89c2", "\u5df2\u7ecf\u53c2\u89c2\u5b8c",
            "\u5df2\u7ecf\u53c2\u89c2\u5b8c\u6210", "\u5df2\u7ecf\u53c2\u89c2\u7ed3\u675f", "\u53c2\u89c2\u8fc7\u4e86", "\u4e0d\u9700\u8981\u53c2\u89c2",
            "参观一下就行了", "先不导览", "不用导览", "导览完了", "已经导览完了",
        ]
        if has_any(text, guide_negative):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="guide_cancel",
                reply="\u660e\u767d\u4e86\uff0c\u6211\u5148\u4e0d\u5e26\u4f60\u53c2\u89c2\uff0c\u9700\u8981\u65f6\u518d\u53eb\u6211\u3002",
                source="rule",
            )

        guide_wrong_place = [
            "厕所", "卫生间", "阳台", "书房", "办公室", "教室", "医院", "学校", "超市", "商店",
            "外面", "门口", "停车场", "电梯", "楼下", "楼上",
        ]
        if has_any(text, ["带我去", "领我去", "去一下", "导航到"]) and has_any(text, guide_wrong_place):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="unsupported_place",
                reply="这个地点不在本次导览范围内，我会按比赛要求介绍餐厅、厨房、客厅和卧室。",
                source="rule",
            )

        weather_words = ["天气", "气温", "温度", "冷不冷", "热不热", "下雨", "带伞", "穿什么", "穿啥", "衣服", "衣物", "外套"]
        if has_any(text, weather_words) or ("\u7a7f" in text and has_any(text, ["\u63a8\u8350", "\u9002\u5408", "\u4eca\u5929", "\u8863\u670d", "\u8863\u7269"])):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="weather_or_clothes",
                reply="\u53ef\u4ee5\uff0c\u5efa\u8bae\u6309\u73b0\u573a\u6e29\u5ea6\u9009\u8212\u9002\u8863\u7269\uff0c\u51b7\u5c31\u52a0\u5916\u5957\u3002",
                source="rule",
            )

        if has_any(text, ["\u4f60\u662f\u8c01", "\u4ecb\u7ecd\u4f60\u81ea\u5df1", "\u4f60\u80fd\u505a\u4ec0\u4e48", "你叫什么", "你叫什么名字", "你会什么", "你有什么功能"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="self_intro",
                reply="\u6211\u662f\u5c45\u5bb6\u670d\u52a1\u673a\u5668\u4eba\uff0c\u80fd\u5bfc\u89c8\u3001\u63a8\u8350\u83dc\u54c1\u548c\u5bfb\u627e\u7269\u54c1\u3002",
                source="rule",
            )

        if has_any(text, ["\u8c22\u8c22", "\u8f9b\u82e6\u4e86", "\u611f\u8c22", "谢谢你", "做得不错", "很好", "可以了", "好的"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="polite",
                reply="\u4e0d\u5ba2\u6c14\uff0c\u5f88\u9ad8\u5174\u5e2e\u5230\u4f60\u3002",
                source="rule",
            )

        if has_any(text, ["\u505c\u4e00\u4e0b", "\u6682\u505c", "\u5148\u505c", "\u53d6\u6d88", "\u522b\u52a8", "停下", "不要动", "先等等", "等一下", "别执行"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="stop_or_cancel",
                reply="\u597d\u7684\uff0c\u6211\u5148\u4e0d\u6267\u884c\u8fd9\u6761\u8bed\u97f3\u6307\u4ee4\u3002",
                source="rule",
            )

        if "\u7b11\u8bdd" in text or has_any(text, ["讲个故事", "唱首歌", "聊天", "陪我聊"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="chat_joke",
                reply="\u5f53\u7136\uff0c\u51b0\u7bb1\u8bf4\uff1a\u6211\u6700\u51b7\u9759\uff0c\u6240\u4ee5\u6700\u4f1a\u4fdd\u9c9c\u3002",
                source="rule",
            )

        if text in ["\u4f60\u597d", "hello", "hi", "嗨", "你好呀", "早上好", "下午好", "晚上好"]:
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="greeting",
                reply="\u4f60\u597d\uff0c\u6211\u5728\u8fd9\u91cc\uff0c\u9700\u8981\u5e2e\u52a9\u5c31\u544a\u8bc9\u6211\u3002",
                source="rule",
            )

        if has_any(text, ["几点", "现在时间", "今天几号", "星期几", "日期"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="time_question",
                reply="我正在比赛演示中，暂时不查询时间，请继续下达任务指令。",
                source="rule",
            )

        guide_positive = [
            "\u53ef\u4ee5\u5e26\u6211\u53c2\u89c2\u4e00\u4e0b\u5417",
            "\u53ef\u4ee5\u5e26\u6211\u53c2\u89c2\u5417",
            "\u5e26\u6211\u53c2\u89c2\u4e00\u4e0b",
            "\u5e26\u6211\u53c2\u89c2",
            "\u5e26\u6211\u5bfc\u89c8",
            "\u8bf7\u5e26\u6211\u53c2\u89c2",
            "\u5e2e\u6211\u53c2\u89c2",
            "\u7ed9\u6211\u5bfc\u89c8",
            "\u5e26\u6211\u4ecb\u7ecd",
            "\u7ed9\u6211\u4ecb\u7ecd\u4e00\u4e0b\u8fd9\u4e2a\u5bb6",
            "\u4ecb\u7ecd\u4e00\u4e0b\u8fd9\u4e2a\u5bb6",
            "\u4ecb\u7ecd\u4e00\u4e0b\u623f\u95f4",
            "带我看一下房间",
            "带我看一下这个家",
            "麻烦带我看一下房间",
            "麻烦带我看一下这个家",
            "带我熟悉一下环境",
            "介绍一下环境",
        ]
        guide_action_words = ["\u5e26\u6211", "\u8bf7\u5e26", "\u5e2e\u6211", "\u7ed9\u6211", "\u53ef\u4ee5", "麻烦", "请"]
        guide_scene_words = ["\u53c2\u89c2", "\u5bfc\u89c8", "\u4ecb\u7ecd", "看一下房间", "看一下这个家", "熟悉一下环境", "看看房间"]
        if has_any(text, guide_positive) or (has_any(text, guide_action_words) and has_any(text, guide_scene_words)):
            return self._result(True, self.TASK_GUIDE, reply="\u597d\u7684\uff0c\u8bf7\u8ddf\u6211\u6765\u3002", source="rule")

        if "\u53c2\u89c2" in text or "\u5bfc\u89c8" in text:
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="ambiguous_guide",
                reply="\u6211\u8fd8\u6ca1\u542c\u6e05\u4efb\u52a1\uff0c\u8bf7\u8bf4\uff1a\u53ef\u4ee5\u5e26\u6211\u53c2\u89c2\u4e00\u4e0b\u5417\u3002",
                source="rule",
            )

        food_interference_words = ["吃什么", "喝什么", "买什么", "玩什么", "做什么运动", "去哪玩", "看什么电影", "听什么歌", "穿什么", "玩游戏", "打游戏", "做游戏"]
        if (has_any(text, ["推荐", "适合", "今天"]) and has_any(text, food_interference_words)) or has_any(text, ["我想玩游戏", "想玩游戏", "陪我玩游戏", "玩一会游戏"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="assistant_interference",
                reply="这个我先不执行。本任务请说推荐今天适合做什么菜，或让我看看冰箱里有什么。",
                source="rule",
            )

        non_food_recommend_objects = [
            "手机", "书包", "背包", "电脑", "平板", "电视", "耳机", "相机", "手表",
            "电影", "电视剧", "视频", "小说", "游戏", "衣服", "鞋子",
        ]
        non_food_recommend_actions = [
            "推荐", "适合", "看什么", "买什么", "用什么", "换什么", "选什么",
        ]
        if has_any(text, non_food_recommend_actions) and has_any(text, non_food_recommend_objects):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="assistant_non_food_recommendation",
                reply="这个推荐我暂时不执行。本任务只推荐做菜，或查看冰箱食材。",
                source="rule",
            )

        if has_any(text, ["不做菜", "不用做菜", "别推荐菜", "换一道菜", "不用看冰箱", "冰箱不用看", "饭做好了"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="assistant_cancel",
                reply="好的，我先不推荐菜品。需要时请说推荐今天适合做什么菜。",
                source="rule",
            )

        assistant_positive = [
            "\u7ed9\u6211\u63a8\u8350\u4e00\u4e0b\u4eca\u5929\u9002\u5408\u505a\u4ec0\u4e48\u83dc", "\u63a8\u8350\u4e00\u4e0b\u4eca\u5929\u9002\u5408\u505a\u4ec0\u4e48\u83dc", "\u63a8\u8350\u505a\u4ec0\u4e48\u83dc",
            "\u9002\u5408\u505a\u4ec0\u4e48\u83dc", "\u505a\u4ec0\u4e48\u83dc", "\u505a\u83dc", "\u63a8\u8350\u4e00\u9053\u83dc", "\u51b0\u7bb1", "\u98df\u6750",
            "\u770b\u770b\u6709\u4ec0\u4e48\u83dc", "\u8fd8\u5269\u4ec0\u4e48\u83dc",
        ]
        if has_any(text, assistant_positive):
            return self._result(True, self.TASK_ASSISTANT, reply="\u597d\u7684\uff0c\u6211\u53bb\u770b\u770b\u51b0\u7bb1\u91cc\u6709\u4ec0\u4e48\u3002", source="rule")

        if expected_mission == self.TASK_ASSISTANT:
            assistant_context_words = [
                "菜", "做饭", "做菜", "下厨", "冰箱", "食材", "食物", "还剩", "剩下",
                "推荐菜", "推荐个菜", "推荐一下", "做什么", "吃的",
            ]
            if has_any(text, assistant_context_words):
                return self._result(True, self.TASK_ASSISTANT, reply="\u597d\u7684\uff0c\u6211\u53bb\u770b\u770b\u51b0\u7bb1\u91cc\u6709\u4ec0\u4e48\u3002", source="rule_context")

        target = ""
        if "\u624b\u673a" in text:
            target = "cell phone"
        elif "\u4e66\u5305" in text or "\u80cc\u5305" in text:
            target = "backpack"
        if target:
            find_action = has_any(text, ["\u627e", "\u5728\u54ea", "\u54ea\u91cc", "\u4f4d\u7f6e", "\u5e2e\u6211", "看见", "看到", "瞧见", "有没有", "寻找", "查找", "哪儿", "哪"])
            if find_action or expected_mission == self.TASK_FIND:
                return self._result(True, self.TASK_FIND, target=target, reply="\u597d\u7684\uff0c\u6211\u53bb\u627e\u3002", source="rule")

        if has_any(text, ["不找了", "不用找", "别找", "找完了", "已经找到了", "找到了", "停止寻找"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="find_cancel",
                reply="好的，我先不继续寻找。需要时请告诉我要找手机还是书包。",
                source="rule",
            )

        unsupported_find = [
            "\u676f\u5b50", "\u6c34\u676f", "\u6c34\u74f6", "\u94a5\u5319", "\u9065\u63a7\u5668",
            "眼镜", "帽子", "衣服", "鞋子", "钱包", "电脑", "平板", "雨伞", "书", "本子", "玩具",
        ]
        if has_any(text, unsupported_find) and has_any(text, ["\u627e", "\u5728\u54ea", "\u54ea\u91cc", "\u4f4d\u7f6e", "\u5e2e\u6211"]):
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="unsupported_find_object",
                reply="\u8fd9\u4e2a\u7269\u54c1\u6211\u6682\u65f6\u4e0d\u627e\uff0c\u73b0\u5728\u53ea\u652f\u6301\u624b\u673a\u548c\u4e66\u5305\u3002",
                source="rule",
            )

        if has_any(text, ["找东西", "找一下东西", "帮我找一下", "东西在哪里", "去哪找", "帮我看看东西", "找个东西"]) and not target:
            return self._result(
                False,
                self.TASK_NONE,
                interference_type="ambiguous_find_object",
                reply="请告诉我要找手机还是书包。",
                source="rule",
            )

        if has_any(text, ["充电", "充电桩", "回桩", "回去充电", "开始充电"]):
            return self._result(True, "charge", reply="好的，我准备返回充电桩。", source="rule")

        return None

    def _classify_by_model(self, command, expected_mission):
        if not self.model_enabled:
            return None
        prompt = self._build_prompt(command, expected_mission)
        data = self._run_model_json(prompt)
        if not data:
            return None

        reply = str(data.get("reply", "")).strip()
        interference_type = str(data.get("interference_type", "unknown")).strip() or "unknown"
        if not reply:
            reply = "\u597d\u7684\uff0c\u6211\u6536\u5230\u4e86\u3002"
        return self._result(
            False,
            self.TASK_NONE,
            interference_type=interference_type,
            reply=reply[:60],
            source="model",
        )

    def _build_prompt(self, command, expected_mission):
        expected_mission = expected_mission or "auto"
        return (
            "\u4f60\u662fRAICOM\u5c45\u5bb6\u670d\u52a1\u673a\u5668\u4eba\u8bed\u97f3\u610f\u56fe\u5224\u522b\u5668\u3002\u53ea\u8f93\u51fa\u4e00\u884cJSON\uff0c\u4e0d\u8981\u89e3\u91ca\u3002\n"
            "\u5b57\u6bb5\u56fa\u5b9a\u4e3a execute_task, task, target, interference_type, reply\u3002\n"
            "task\u53ea\u80fd\u662fguide\u3001assistant\u3001find_object\u3001charge\u3001none\u3002\n"
            "\u5982\u679c\u4e0d\u662f\u660e\u786e\u4efb\u52a1\u547d\u4ee4\uff0cexecute_task\u5fc5\u987b\u662ffalse\uff0ctask\u5fc5\u987b\u662fnone\uff0c\u5e76\u7ed9\u4e00\u53e520\u5b57\u4ee5\u5185\u4e2d\u6587\u77ed\u56de\u7b54\u3002\n"
            "\u4efb\u52a1\u4e00\u662f\u53c2\u89c2\u5bfc\u89c8\uff1b\u4efb\u52a1\u4e8c\u662f\u63a8\u8350\u505a\u83dc\u6216\u67e5\u770b\u51b0\u7bb1\u98df\u6750\uff1b\u4efb\u52a1\u4e09\u53ea\u627e\u624b\u673a\u6216\u4e66\u5305\u3002\n"
            "\u5e72\u6270\u547d\u4ee4\u8981\u56de\u7b54\u4f46\u4e0d\u6267\u884c\u3002\n"
            "\u5f53\u524d\u4efb\u52a1\u4e0a\u4e0b\u6587: %s\n"
            "\u7528\u6237\u8bed\u97f3: %s\n"
            "JSON:"
        ) % (expected_mission, command)

    @staticmethod
    def _extract_json(text):
        text = text or ""
        match = re.search(r"\{.*?\}", text, re.S)
        if not match:
            return None
        try:
            data = json.loads(match.group(0))
        except Exception:
            return None
        if not isinstance(data, dict):
            return None
        return data

    @staticmethod
    def _result(execute_task, task, target="", interference_type="", reply="", source="rule"):
        return {
            "execute_task": bool(execute_task),
            "task": task,
            "target": target,
            "interference_type": interference_type,
            "reply": reply,
            "source": source,
        }


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--text", required=True)
    parser.add_argument("--expected-mission", default="")
    parser.add_argument("--model", default="")
    parser.add_argument("--llama", default="")
    parser.add_argument("--model-enabled", action="store_true")
    args = parser.parse_args()
    classifier = SemanticIntentClassifier({
        "enabled": True,
        "model_enabled": args.model_enabled,
        "model_path": args.model,
        "llama_binary": args.llama,
    })
    print(json.dumps(classifier.classify(args.text, args.expected_mission), ensure_ascii=False))
