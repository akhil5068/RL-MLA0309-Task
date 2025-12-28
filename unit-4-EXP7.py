import numpy as np
import matplotlib.pyplot as plt
import random
import math
MAP_SIZE = 100
START = (5, 5)
GOAL = (95, 95)
OBSTACLES = [
    (30, 30, 12),
    (60, 60, 10),
    (70, 25, 8),
    (40, 75, 10)
]
STEP_SIZE = 4
NEIGHBOR_RADIUS = 10
MAX_ITER = 2000
def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
def is_collision(point):
    for ox, oy, r in OBSTACLES:
        if distance(point, (ox, oy)) <= r:
            return True
    return False
def steer(from_node, to_node):
    theta = math.atan2(to_node[1] - from_node[1],
                       to_node[0] - from_node[0])
    new_x = from_node[0] + STEP_SIZE * math.cos(theta)
    new_y = from_node[1] + STEP_SIZE * math.sin(theta)
    return (new_x, new_y)
nodes = [START]
parent = {START: None}
cost = {START: 0}

for _ in range(MAX_ITER):
    rand_point = (random.uniform(0, MAP_SIZE),
                  random.uniform(0, MAP_SIZE))


    nearest = min(nodes, key=lambda n: distance(n, rand_point))

    new_node = steer(nearest, rand_point)

    if is_collision(new_node):
        continue


    near_nodes = [n for n in nodes if distance(n, new_node) < NEIGHBOR_RADIUS]
    best_parent = nearest
    min_cost = cost[nearest] + distance(nearest, new_node)

    for n in near_nodes:
        c = cost[n] + distance(n, new_node)
        if c < min_cost:
            min_cost = c
            best_parent = n

    nodes.append(new_node)
    parent[new_node] = best_parent
    cost[new_node] = min_cost

    for n in near_nodes:
        if cost[new_node] + distance(new_node, n) < cost[n]:
            parent[n] = new_node
            cost[n] = cost[new_node] + distance(new_node, n)

    if distance(new_node, GOAL) < STEP_SIZE:
        parent[GOAL] = new_node
        nodes.append(GOAL)
        break


path = []
node = GOAL
while node is not None:
    path.append(node)
    node = parent[node]

path.reverse()


plt.figure(figsize=(8, 8))


for ox, oy, r in OBSTACLES:
    plt.gca().add_patch(plt.Circle((ox, oy), r, color='red', alpha=0.4))

for n in parent:
    if parent[n] is not None:
        plt.plot([n[0], parent[n][0]],
                 [n[1], parent[n][1]], 'g-', linewidth=0.5)

px, py = zip(*path)
plt.plot(px, py, 'b-', linewidth=3, label="Optimized UAV Path")

plt.scatter(*START, color='blue', s=80, label="Start")
plt.scatter(*GOAL, color='black', s=80, label="Goal")

plt.title("UAV Surveillance Path Planning using RRT*")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend()
plt.grid(True)
plt.show()
