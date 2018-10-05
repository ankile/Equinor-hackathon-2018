#!/usr/bin/env python
"""
class TargetUpdater:
    def __init__(self, world_graph):
        self.world_graph = world_graph
        self.path = None
        self.goal = None
        self.current_target = None

    def set_goal(self, goal):
        if self.goal == goal:
            return

        self.goal = goal

        self.path = None
        self.current_target = None

    def has_new_target(self, drone_x, drone_y):
        pass


    def should_move_to_next_target(self, x0, y0):
        if self.path is None or self.goal is None or not self.path:
            return False

        (x, y) = self.path[0]
        distance = ((x - x0)**2 + (y - y0)**2) ** 0.5

        return distance < 0.3

    def get_target(self, drone_x, drone_y):
        if self.path is None or self.goal is None or self.current_target



        next = self.path[0]
        self.path = self.path[1:]

        return next
"""