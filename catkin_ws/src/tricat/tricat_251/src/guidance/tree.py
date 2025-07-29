#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass, field
from typing import List, Optional

@dataclass
class Waypoint:
    x: float
    y: float
    type: str
    num: int
    range: int
    arrive: bool

@dataclass
class TreeNode:
    key: any
    children: List['TreeNode'] = field(default_factory=list)

    def add_child(self, child_node: 'TreeNode'):
        self.children.append(child_node)


class Tree:
    def __init__(self, root_key):
        self.root = TreeNode(root_key)

    def add_child(self, parent_key, child_key):
        parent_node = self.search(self.root, parent_key)
        if parent_node:
            child_node = TreeNode(child_key)
            parent_node.add_child(child_node)
        else:
            print(f"Parent node with key {parent_key} not found.")

    def search(self, node, key):
        if node is None:
            return None
        if node.key == key:
            return node

        for child in node.children:
            result = self.search(child, key)
            if result:
                return result

        return None

    def print_tree(self, node, level=0):
        if node is not None:
            print(' ' * level * 4 + str(node.key))
            for child in node.children:
                self.print_tree(child, level + 1)

