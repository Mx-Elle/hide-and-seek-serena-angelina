from agent_base import Agent
from world_state import WorldState
from Mesh.nav_mesh import NavMesh
#can we import these?
from Mesh.nav_mesh import NavMeshCell
from collections import defaultdict
import heapq
import shapely
import math

#Angelinaa

class WannabeHider(Agent):

    def __init__(self, world_map: NavMesh, max_speed: float):
        Agent.__init__(self, world_map, max_speed)
        self.name = "Wannabe Hider"

    #BTW self_location = state.hider_position
    target = NavMesh.random_position

    def astar(self, start: shapely.Point, end: shapely.Point) -> tuple[float, float] | None:
        closed_list = set()
        start_cell = self.map.find_cell(start)
        end_cell = self.map.find_cell(end)
        if not start_cell or not end_cell:
            return None
        g_scores = defaultdict(lambda: float("inf"))
        g_scores[start_cell] = 0
        f_scores = defaultdict(lambda: float("inf"))
        f_scores[start_cell] = start_cell.distance(end_cell)

        # this is based on prior astar, making adjustments
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
    

    def find_optimal(self, state: WorldState) -> defaultdict[NavMeshCell: float] | None:
        current_cell = self.map.find_cell(state.hider_position)
        closed_list = [(current_cell)]
        frontiers = [current_cell.neighbors]

        neighborscore = defaultdict(lambda: float("inf"))
        neighborscore[current_cell] = len(current_cell.neighbors)

#many issues here btw fix them eventually
        while True:
            if all(item in closed_list for item in frontiers):
                break
            for neighbor in frontiers:
                cell, __ = neighbor
                if cell in neighborscore:
                    continue
                current_cell = heapq.heappop(frontiers)
                neighborscore[current_cell] = len(cell.neighbors)
                closed_list.add(current_cell)
        return neighborscore
    
    def find_proximity(self, state: WorldState) -> defaultdict[NavMeshCell: float] | None:
        #for proximity, plugging arbitrary values first
        smelly_cell = self.map.find_cell(state.seeker_position)
        closed_list = [(smelly_cell)]
        frontiers = [current_cell.neighbors]
        # save this total result based on this prior calculation so you don't have to recalculate

        neighborscore = defaultdict(lambda: float("inf"))
        neighborscore[current_cell] = len(current_cell.neighbors)

        while True:
            if all(item in closed_list for item in frontiers):
                break
            for neighbor in frontiers:
                neighbor, __ = neighbor
                if neighbor in neighborscore:
                    continue
                current_cell = heapq.heappop(frontiers)
                neighborscore[current_cell] = len(neighbor.neighbors)
                closed_list.add(current_cell)
        return neighborscore


    def go_straight(
        self, loc: shapely.Point, target: shapely.Point | None
    ) -> tuple[float, float] | None:
        if loc == target or not self.map.in_bounds(target) or target == None: # type: ignore
            return
        dx, dy = (
            target.x - loc.x,
            target.y - loc.y,
        )
        distance = math.dist((dx, dy), (0, 0))
        speed = min(distance, self.max_speed * 0.9999)
        dx, dy = speed * dx / distance, speed * dy / distance
        return dx, dy

    #changes from here. WorldState no longer has some saved location state, so you want to adjust target location 
    def act(self, state: WorldState) -> tuple[float, float] | None:
        target = state.seeker_position
        # target = heapq.heappop(self.find_optimal(state))
        if state.hider_position == target:
            return
        if self.map.has_line_of_sight(state.hider_position, target):
            return self.go_straight(state.hider_position, target)
        strip = self.astar(state.hider_position, target)
        if strip is None:
            return None
        portals = [strip[i].neighbors[strip[i + 1]] for i in range(len(strip) - 1)]
        for portal in portals[::-1]:
            edges = [
                shapely.line_interpolate_point(portal, 0.01, normalized=True),
                shapely.line_interpolate_point(portal, 0.99, normalized=True),
            ]
            for point in edges:
                state.hider_position = state.hider_position
                if self.map.has_line_of_sight(state.hider_position, point):
                    return self.go_straight(state.hider_position, point)

    @property
    def is_seeker(self) -> bool:
        return False