#!/usr/bin/env python

# class Graph:
#   def size()
#   def adjacent_to(src)
#   def distance(src, dst)
import heapq as h

class ManhattanGraph:
    """
    A graph over a square grid, where movement is restricted to left/right/up/down. I.e. no diagonal movement.
    """
    def __init__(self, grid):
        self.grid = grid
        self.prev_node = None

    def size(self):
        return (len(self.grid[0]), len(self.grid))

    def adjacents_to(self, node):
        (size_x, size_y) = self.size()
        (x, y) = node
        coords = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
        diagcoords = []#[(x + dx, y + dy) for dx in [-1, 1] for dy in [-1, 1] if self.grid[y + dy][x] == self.grid[y][x + dx] and self.grid[y+dy][x] in [0, '\x00']]

        return [(x, y) for (x, y) in coords if 0 <= x < size_x and 0 <= y < size_y and (self.grid[y][x] == 0 or self.grid[y][x] == '\x00')]

    def cost(self, src, dst):
        (x0, y0) = src
        (x1, y1) = dst

        return ((x1 - x0)**2 + (y1 - y0)**2) ** 0.5

def normalize(x, y):
    m = (x**2 + y**2)**0.5
    return (x / m, y / m)


class RecursivePath:
    def __init__(self, prev, node, cost):
        self.prev = prev
        self.node = node
        self.cost = cost


    def appending(self, node, cost):
        return RecursivePath(self, node, self.cost + cost)

    def as_list(self):
        if not self.prev:
            return [self.node]

        return self.prev.as_list() + [self.node]

    def __lt__(self, other):
        return self.cost < other.cost

    def __gt__(self, other):
        return self.cost > other.cost

    def __eq__(self, other):
        return self.cost == other.cost

def are_parallell(p0, p1, p2):
    epsilon = 0.01
    dot = lambda a, b: a[0] * b[0] + a[1] * b[1]
    v1 = normalize(p1[0] - p0[0], p1[1] - p0[1])
    v2 = normalize(p2[0] - p1[0], p2[1] - p1[1])
    return (v1[0] == v2[0] == 0 or v1[1] == v2[1] == 0)
    #return abs(v1[0] - v2[0]) < epsilon and abs(v1[1] - v2[1]) < epsilon

def ring_around(pos):
    return [(pos[0] + dx, pos[1] + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1]]

def unfiltered_shortest_path(graph, source, destination, count = 1, exploration_factor = 1):
    """
    :param graph: A graph to traverse
    :param source: The source node as a tuple of (x, y)
    :param destination: The destination node as a tuple of (x, y)
    :param count: Fetch the `count` best paths
    :param exploration_factor: Determines how much we'll "explore" around the optimal path.
    :return: The `count` best paths from `source` to `destination`
    """
    # A min-heap of the paths, sorted by the total path length
    heap = [RecursivePath(None, source, 0)]
    # We'll return the `count` best paths
    optimal_paths = []

    # A 2d array of the least costs so far.
    (size_x, size_y) = graph.size()
    least_costs = [[float('inf') for x in range(size_x)] for y in range(size_y)]
    least_costs[source[1]][source[0]] = 0

    while len(optimal_paths) < count and heap:
        # The current path with the least total cost
        shortest = h.heappop(heap)
        (x, y) = shortest.node

        # Due to ordering by the least total cost, we've found
        # an optimal path once we reach the destination
        if shortest.node == destination:
            optimal_paths.append(shortest)
            continue

        # If we've found a shorter path to the current edge, we'll simply discard this path
        if least_costs[y][x] < shortest.cost:
            continue

        # Otherwise, we'll visit all the neighbours of `shortest`
        # and add all of those which result in a shorter path.
        for node in graph.adjacents_to(shortest.node):
            edge_cost = graph.cost(shortest.node, node)
            cost_scaling = 1 if shortest.prev is None or are_parallell(shortest.prev.node, shortest.node, node) else 10
            dst_scaling = 1 + 20*len([a for a in ring_around(node) if a in ['\x01', 1]])
            path = shortest.appending(node, dst_scaling * edge_cost)
            (x1, y1) = path.node

            if path.cost < least_costs[y1][x1] * exploration_factor:
                least_costs[y1][x1] = path.cost
                h.heappush(heap, path)

    return optimal_paths[0].as_list()


def shift_reference_point(paths):
    return [(x + 0.5, y + 0.5) for (x, y) in paths]

def shortest_path(graph, source, destination):
    unfiltered = unfiltered_shortest_path(graph, source, destination)
    should_be_removed = [True]

    for i in range(1, len(unfiltered) - 1):
        should_be_removed.append(unfiltered[i - 1][0] == unfiltered[i][0] == unfiltered[i + 1][0] or unfiltered[i - 1][1] == unfiltered[i][1] == unfiltered[i + 1][1])

    should_be_removed.append(False)

    filtered = [unfiltered[i] for i in range(len(unfiltered)) if not should_be_removed[i]]

    return shift_reference_point(filtered)

