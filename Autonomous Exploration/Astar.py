#!/usr/bin/env python3

import math
import heapq

class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, start, end):
        # Euclidean distance heuristic
        dx = abs(start[0] - end[0])
        dy = abs(start[1] - end[1])
        return math.sqrt(dx*dx + dy*dy)

    def is_valid(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols and self.grid[row][col] == 0

    def get_neighbors(self, node):
        row, col = node
        neighbors = [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]
        return [(r, c) for r, c in neighbors if self.is_valid(r, c)]

    def a_star_search(self, start, goal):
        open_set = [(0, start)]  # Priority queue with f-score and node (f, node)
        came_from = {}  # Parent pointers to reconstruct the path
        g_score = {start: 0}  # Cost from start to node
        f_score = {start: self.heuristic(start, goal)}  # Estimated total cost from start to goal through node

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = self.reconstruct_path(came_from, current)
                return path

            for neighbor in self.get_neighbors(current):
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]  # Reverse the path to get it from start to goal
