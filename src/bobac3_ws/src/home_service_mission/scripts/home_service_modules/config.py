#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import yaml


class HomeMissionConfig:
    def __init__(self, path):
        if not path or not os.path.isfile(path):
            raise RuntimeError("Home mission config file not found: %s" % path)
        self.path = path
        with open(path, "r", encoding="utf-8") as stream:
            self.data = yaml.safe_load(stream)
        if not isinstance(self.data, dict):
            raise RuntimeError("Home mission config is empty or invalid: %s" % path)

    def section(self, name):
        value = self.data.get(name)
        if not isinstance(value, dict):
            raise RuntimeError("Home mission config missing section: %s" % name)
        return value

    def optional_section(self, name):
        value = self.data.get(name, {})
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise RuntimeError("Home mission config section must be a mapping: %s" % name)
        return value

    def validate(self, mission_type="auto"):
        for section in [
            "waypoints",
            "navigation",
            "arrival_pause",
            "interaction",
            "guide",
            "assistant",
            "find_object",
            "finish",
            "chinese_names",
        ]:
            self.section(section)

        required = {"start", "corridor"}
        required.update(self.section("guide").get("route", []))
        self.add_route_items(required, self.section("navigation").get("corridor_route", []))
        self.add_route_items(required, self.section("navigation").get("return_route", []))
        for route in self.section("guide").get("approach_routes", {}).values():
            self.add_route_items(required, route)
        required.add(self.section("assistant").get("target_waypoint", "kitchen"))
        self.add_route_items(required, self.section("assistant").get("approach_route", []))
        self.add_route_items(required, self.section("find_object").get("route", []))
        pre_charge_name = self.section("finish").get("pre_charge_target", "")
        if pre_charge_name:
            required.add(pre_charge_name)
        charge_name = self.section("finish").get("charge_target", "")
        if mission_type in ("auto", "charge") and charge_name:
            required.add(charge_name)

        waypoints = self.section("waypoints")
        missing = []
        for name in sorted(required):
            point = waypoints.get(name)
            if not point:
                missing.append(name)
                continue
            if point.get("x") is None or point.get("y") is None or point.get("yaw") is None:
                missing.append(name)
        if missing:
            raise RuntimeError(
                "Home waypoints are not calibrated: %s. Update the config file: %s"
                % (", ".join(missing), self.path)
            )

    @staticmethod
    def add_route_items(required, route):
        for item in route:
            if isinstance(item, dict):
                required.add(item["waypoint"])
            else:
                required.add(item)

    @staticmethod
    def normalize_command(command):
        command = command or ""
        for mark in " \\t\\r\\n,.!?;:，。！？；：、":
            command = command.replace(mark, "")
        return command

    @classmethod
    def command_matches(cls, command, phrases):
        command = cls.normalize_command(command)
        for phrase in phrases:
            phrase = cls.normalize_command(phrase)
            if not phrase:
                continue
            if command == phrase:
                return True
            if len(phrase) >= 4 and phrase in command:
                return True
        return False

    def parse_find_target(self, command):
        trigger_phrases = self.section("find_object").get("trigger_phrases", {})
        for target, phrases in trigger_phrases.items():
            if self.command_matches(command, phrases):
                return target
        return ""

    def parse_mission_command(self, command):
        command = command or ""
        if self.command_matches(command, self.section("guide")["trigger_phrases"]):
            return "guide", ""
        if self.command_matches(command, self.section("assistant")["trigger_phrases"]):
            return "assistant", ""
        target = self.parse_find_target(command)
        if target:
            return "find_object", target
        if self.command_matches(command, self.section("finish").get("trigger_phrases", [])):
            return "charge", ""
        return "", ""
