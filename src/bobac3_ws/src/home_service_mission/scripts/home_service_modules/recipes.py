#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random


class RecipeBook:
    def __init__(self, assistant_config):
        self.assistant = assistant_config

    def ensure_minimum_foods(self, foods):
        minimum_count = int(self.assistant.get("minimum_report_count", 3))
        maximum_count = int(self.assistant.get("maximum_report_count", minimum_count))
        result = list(foods)
        if len(result) < minimum_count:
            fallback = self.assistant.get("simulation_fallback", {})
            if fallback.get("random_fill_to_minimum", False):
                pool = [label for label in self.assistant["food_labels"] if label not in result]
                random.shuffle(pool)
                for label in pool:
                    result.append(label)
                    if len(result) >= minimum_count:
                        break
            elif fallback.get("enabled", False) and fallback.get("fill_to_minimum", False):
                for label in fallback.get("labels", []):
                    if label in self.assistant["food_labels"] and label not in result:
                        result.append(label)
                    if len(result) >= minimum_count:
                        break
        if maximum_count > 0:
            result = result[:maximum_count]
        return result

    def choose_recipe(self, foods):
        available = set(foods)
        for recipe in self.assistant.get("recipes", []):
            required = set(recipe.get("requires", []))
            if required.issubset(available):
                return recipe["name"], recipe["method"]
        if foods:
            selected = list(foods)[:3]
            if len(selected) >= 3:
                return (
                    "%s、%s和%s家常小炒" % (selected[0], selected[1], selected[2]),
                    "将%s、%s和%s清洗切配，适合熟制的食材先下锅炒熟，再放入剩余食材一起翻炒，最后加入少量盐和调味料即可。"
                    % (selected[0], selected[1], selected[2]),
                )
            return (
                "清爽拼盘",
                "将%s清洗处理，按食材特性煮熟或直接切配后组合。"
                % "、".join(selected),
            )
        return "家常面", "准备面条和基础调味料，将面条煮熟后调味。"
