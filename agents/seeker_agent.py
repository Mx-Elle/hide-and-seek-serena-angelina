from Mesh.nav_mesh import NavMesh
from Mesh.nav_mesh import NavMeshCell
from agent_base import Agent
from world_state import WorldState
from collections import defaultdict
from time import perf_counter
import shapely
import heapq
import math
import time

# Serena


class WannabeSeeker(Agent):

    def __init__(self, world_map: NavMesh, max_speed: float):
        Agent.__init__(self, world_map, max_speed)
        self.name = "WannabeSeeker"
        self.probabilities = {}

    def astar(
        self, start: shapely.Point, end: shapely.Point
    ) -> list[NavMeshCell] | None:
        start_cell, end_cell = self.map.find_cell(start), self.map.find_cell(end)
        closed_list = set()  # explored
        if not start_cell or not end_cell:
            return None
        g_scores = defaultdict(lambda: float("inf"))
        g_scores[start_cell] = 0
        f_scores = defaultdict(lambda: float("inf"))
        f_scores[start_cell] = start_cell.distance(end_cell)

        frontier = [(f_scores[start_cell], start_cell.id, start_cell)]
        prev: dict[NavMeshCell, NavMeshCell | None] = {start_cell: None}

        while True:
            _, _, current_cell = heapq.heappop(frontier)
            if current_cell == end_cell:
                break

            for neighbor in current_cell.neighbors:
                if neighbor in closed_list:
                    continue

                temp_g = g_scores[current_cell] + 1
                if temp_g < g_scores[neighbor]:
                    g_scores[neighbor] = temp_g
                    f_scores[neighbor] = temp_g + neighbor.distance(end_cell)
                    prev[neighbor] = current_cell  # type: ignore
                    if neighbor not in closed_list:
                        heapq.heappush(
                            frontier, (f_scores[neighbor], neighbor.id, neighbor)
                        )

            closed_list.add(current_cell)

        path = []
        key = end_cell
        while key != None:
            path.append(key)
            key = prev[key]
        return path[::-1]

    def go_straight(
        self, loc: shapely.Point, target: shapely.Point | None
    ) -> tuple[float, float] | None:
        if loc == target or not target or not self.map.in_bounds(target):
            # the issue right now is that temp_point seems to be set as None too often. This likely explains why it's stopping at linestrings?
            return

        dx, dy = (
            target.x - loc.x,
            target.y - loc.y,
        )

        distance = math.dist((dx, dy), (0, 0))
        speed = min(distance, self.max_speed)
        dx, dy = speed * dx / distance, speed * dy / distance
        return dx, dy

    # def after_sighting(self, state: WorldState, last_sighted_loc, self.probabilities: defaultdict):
    # checking if it's possible for hider to be somewhere
    # if change in distance > #frames * 2.7 (hider speed), then impossible
    # curr_time = time.time()
    # curr_node = self.map.find_cell(state.seeker_position)
    # distance = last_sighted_loc - curr_node
    # if curr_time * 2.7 < distance:
    # self.probabilities[curr_node] = 0

    def later_sight(self, state: WorldState):
        frontier = []
        target_cell = None
        current_node = self.map.find_cell(state.seeker_position)
        frontier.append(current_node)
        if not self.map.has_line_of_sight(
            state.seeker_position, state.hider_position
        ):
            # target_value = None
            target_cell = max(self.probabilities, key=lambda c: (self.probabilities[c], -current_node.polygon.centroid.distance(c.polygon.centroid)))
            # for cell, prob in self.probabilities:
            #     if prob > target_value:
            #         target_value = prob
            #         target_cell = cell
            #     elif prob == target_value:
            #         if distance(self, target_cell) < distance(self, cell):
            #             target_cell = cell
            # target_lst = self.astar(self, state.seeker_position, target_cell)
        for cell in self.probabilities:
            if not self.map.has_line_of_sight(
                state.seeker_position, cell.polygon.centroid
            ):
                self.probabilities[cell] += 1
            else:
                self.probabilities[cell] = 0
        return target_cell

    def initial_sight(self, state: WorldState):
        if state.hider_position:
            target = state.hider_position
            for cell in self.probabilities:
                if cell != target:
                    self.probabilities[cell] = 0
        else:
            frontier = []
            current_node = self.map.find_cell(state.seeker_position)
            frontier.append(current_node)
            while len(frontier) > 0:
                current_node = frontier.pop()
                for neighbor in current_node.neighbors:
                    frontier.append(neighbor)
                if self.map.has_line_of_sight(
                    state.seeker_position, current_node.polygon.centroid
                ):
                    self.probabilities[current_node] = 0
                else:
                    self.probabilities[current_node] = float("inf")
            target = max(self.probabilities, key=lambda c: (self.probabilities[c], -current_node.polygon.centroid.distance(c.polygon.centroid)))
        return target

    def act(self, state: WorldState) -> tuple[float, float] | None:
        target = None

        if len(self.probabilities) == 0:
            target = self.initial_sight(state)
        else:
            target = self.later_sight(state)

        # if target == None:

        # if self.map.has_line_of_sight(state.seeker_position, state.hider_position):
        # return self.go_straight(state.seeker_position, state.hider_position)
        # last_sighted_loc = state.location
        # last_sighted_time = time.time()
        # possible_loc(last_sighted_loc, self.probabilities)

        strip = self.astar(state.seeker_position, target)
        if strip is None:
            return None

        portals = [strip[i].neighbors[strip[i + 1]] for i in range(len(strip) - 1)]
        # while self.map.has_line_of_sight(state.location, state.target) == False:
        for portal in portals[::-1]:
            # point = portal.centroid
            edges = [
                shapely.line_interpolate_point(portal, 0.01, normalized=True),
                shapely.line_interpolate_point(portal, 0.99, normalized=True),
            ]

            for point in edges:
                if self.map.has_line_of_sight(state.location, point):
                    return self.go_straight(state.location, point)

    @property
    def is_seeker(self) -> bool:
        return True