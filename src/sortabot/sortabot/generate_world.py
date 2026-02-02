import random
import math
import numpy as np
from scipy.spatial import Delaunay
from collections import deque

WORLD = 100
MARGIN = 5
COLORS = ["red", "blue", "green", "yellow"]
CELL_SIZE = 2.0  # grid resolution for connectivity check


def rand_xy():
    return (
        random.uniform(MARGIN, WORLD - MARGIN),
        random.uniform(MARGIN, WORLD - MARGIN),
    )


def points_to_grid(points, obstacles):
    grid_size = int(WORLD / CELL_SIZE) + 1
    grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

    for obs in obstacles:
        x, y = obs["pos"]
        sx, sy = obs["scale"]
        yaw = obs["yaw"]

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        hx, hy = sx / 2, sy / 2

        corners = [
            (x + hx*cos_yaw - hy*sin_yaw, y + hx*sin_yaw + hy*cos_yaw),
            (x - hx*cos_yaw - hy*sin_yaw, y - hx*sin_yaw + hy*cos_yaw),
            (x - hx*cos_yaw + hy*sin_yaw, y - hx*sin_yaw - hy*cos_yaw),
            (x + hx*cos_yaw + hy*sin_yaw, y + hx*sin_yaw - hy*cos_yaw),
        ]

        min_i = max(0, int(min(c[0] for c in corners) / CELL_SIZE))
        max_i = min(grid_size - 1, int(max(c[0] for c in corners) / CELL_SIZE))
        min_j = max(0, int(min(c[1] for c in corners) / CELL_SIZE))
        max_j = min(grid_size - 1, int(max(c[1] for c in corners) / CELL_SIZE))

        for i in range(min_i, max_i + 1):
            for j in range(min_j, max_j + 1):
                grid[i][j] = 1

    return grid


def is_connected(start, targets, obstacles):
    grid = points_to_grid(targets, obstacles)
    size = len(grid)

    def to_cell(p):
        return int(p[0] / CELL_SIZE), int(p[1] / CELL_SIZE)

    sx, sy = to_cell(start)
    visited = set([(sx, sy)])
    q = deque([(sx, sy)])

    while q:
        x, y = q.popleft()
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < size and 0 <= ny < size:
                if grid[nx][ny] == 0 and (nx, ny) not in visited:
                    visited.add((nx, ny))
                    q.append((nx, ny))

    for t in targets:
        if to_cell(t) not in visited:
            return False
    return True


def generate(max_attempts=50):
    for _ in range(max_attempts):
        robot = rand_xy()
        childabots = [{"pos": rand_xy()} for _ in range(3)]

        bins = [{"color": c, "pos": rand_xy()} for c in COLORS]

        objects = []
        total = 0
        while total < 100:
            w = random.randint(5, 18)
            total += w
            objects.append({
                "color": random.choice(COLORS),
                "weight": w,
                "pos": rand_xy()
            })

        obstacles = []
        for _ in range(10):
            axis = random.choice(["x", "y"])
            scale = (random.uniform(3, 6), 1) if axis == "x" else (1, random.uniform(3, 6))
            obstacles.append({
                "pos": rand_xy(),
                "scale": scale,
                "yaw": random.uniform(0, math.pi)
            })

        targets = [b["pos"] for b in bins] + [o["pos"] for o in objects]

        if is_connected(robot, targets, obstacles):
            pts = (
                [robot] +
                [b["pos"] for b in bins] +
                [o["pos"] for o in objects] +
                [c["pos"] for c in childabots]
            )
            graph = Delaunay(np.array(pts))
            return robot, bins, objects, obstacles, childabots, graph

    pts = [robot] + targets
    graph = Delaunay(np.array(pts))
    return robot, bins, objects, obstacles, childabots, graph