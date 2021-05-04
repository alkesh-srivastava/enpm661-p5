import math
from typing import List


class Vertex:
    def __init__(self, pos: (int, int)):
        self.pos = pos
        self.edges_and_costs = {}

    def add_edge_with_cost(self, succ: (int, int), cost: float):
        if succ != self.pos:
            self.edges_and_costs[succ] = cost

    @property
    def edges_and_c_old(self):
        return self.edges_and_costs


class Vertices:
    def __init__(self):
        self.list = []

    def add_vertex(self, v: Vertex):
        self.list.append(v)

    @property
    def vertices(self):
        return self.list


def heuristic(p: (int, int), q: (int, int)) -> float:
    """
    Helper function to compute distance between two points.
    :param p: (x,y)
    :param q: (x,y)
    :return: manhattan distance
    """
    return math.sqrt((p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2)


def get_movements_4n(x: int, y: int) -> List:
    """
    get all possible 4-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(x + 1, y + 0),
            (x + 0, y + 1),
            (x - 1, y + 0),
            (x + 0, y - 1)]


def get_movements_8n(x: int, y: int) -> List:
    """
    get all possible 8-connectivity movements.
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    return [(x + 1, y + 0),
            (x + 0, y + 1),
            (x - 1, y + 0),
            (x + 0, y - 1),
            (x + 1, y + 1),
            (x - 1, y + 1),
            (x - 1, y - 1),
            (x + 1, y - 1)]


def obstacle(y, x, clearance=0):  # This function definition inspects whether a coordinate point x
    y = 99 - y
    # and y lie on the obstacle or not
    inside_circle1 = False
    inside_circle2 = False
    inside_C1 = False
    inside_C2 = False
    inside_C3 = False

    if ((x - 20) ** 2 + (y - 20) ** 2) <= ((10 + clearance) ** 2):
        inside_circle1 = True
    if ((x - 20) ** 2 + (y - 80) ** 2) <= ((10 + clearance) ** 2):
        inside_circle2 = True

    if 2.5 - clearance <= x <= 17.5 + clearance and 42.5 - clearance <= y <= 57.5 + clearance:
        inside_C1 = True

    if 37.5 - clearance <= x <= 62.5 + clearance and 42.5 - clearance <= y <= 57.5 + clearance:
        inside_C2 = True

    if 72.5 - clearance <= x <= 87.5 + clearance and 20 - clearance <= y <= 40 + clearance:
        inside_C3 = True

    if inside_circle1 or inside_circle2 or inside_C1 or inside_C2 or inside_C3:
        return True
    else:
        return False
