from agent_base import Agent
from world_state import WorldState
from Mesh.nav_mesh import NavMesh
#can we import these?
from Mesh.nav_mesh import NavMeshCell
from collections import defaultdict
import heapq
import shapely
import math
import time

#Angelinaa

class WannabeHider(Agent):

    def __init__(self, world_map: NavMesh, max_speed: float):
        Agent.__init__(self, world_map, max_speed)
        self.name = "Wannabe Hider"
        self.last_decision_frame = 0
        self.current_target = None


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
    

    def find_optimal(self, state: WorldState) -> defaultdict | None:
        current_cell = self.map.find_cell(state.hider_position)
        closed_list = [current_cell]
        frontiers = list(current_cell.neighbors.keys())

        neighborscore = defaultdict(lambda: float("inf"))
        neighborscore[current_cell] = -len(current_cell.neighbors)

        while True:
            if all(item in frontiers for item in closed_list):
                break
            for neighbor in frontiers:
                cell = neighbor
                if cell in neighborscore:
                    continue
                frontiers += list(cell.neighbors.keys())
                frontiers.remove(cell)
                neighborscore[cell] = -len(cell.neighbors)
                closed_list.append(cell)
        print(neighborscore)
        return neighborscore
    
    def find_proximity(self, point: shapely.Point, radius: float) -> defaultdict:
        # #for proximity, plugging arbitrary values first
        # smelly_cell = self.map.find_cell(state.seeker_position)
        # closed_list = [(smelly_cell)]
        # frontiers = [current_cell.neighbors]
        # # save this total result based on this prior calculation so you don't have to recalculate

        # neighborscore = defaultdict(lambda: float("inf"))
        # neighborscore[current_cell] = len(current_cell.neighbors)

        # while True:
        #     if all(item in closed_list for item in frontiers):
        #         break
        #     for neighbor in frontiers:
        #         neighbor, __ = neighbor
        #         if neighbor in neighborscore:
        #             continue
        #         current_cell = heapq.heappop(frontiers)
        #         neighborscore[current_cell] = len(neighbor.neighbors)
        #         closed_list.add(current_cell)
        # return neighborscore
    
        closed_list = set()
        smelly_cell = self.map.find_cell(point)
        stinky = defaultdict(lambda: float(0))
        stinky[smelly_cell] = radius

        frontier = [(stinky[smelly_cell], smelly_cell)]

        while True:
            stink, current_cell = heapq.heappop(frontier)
            if stink < 0:
                break

            for neighbor in current_cell.neighbors:
                if neighbor in closed_list:
                    continue
                temp_s = stinky[current_cell] - 1
                if temp_s > stinky[neighbor]:
                    stinky[neighbor] = temp_s
                    if neighbor not in closed_list:
                        heapq.heappush(
                            frontier, (stinky[neighbor], neighbor)
                        )
            closed_list.add(current_cell)
        return stinky


    def calculate_optimal(
            #dunno if state is necessary
            self, state: WorldState
    ) -> NavMeshCell | None:
        current_cell = self.map.find_cell(state.hider_position)
        if not current_cell:
            return None
        closed_list = [current_cell]
        frontiers = list(current_cell.neighbors.keys())
        optimalmap = self.find_optimal(state)

        probmap = heapq.heapify[(optimalmap[current_cell], current_cell)]

        while True:
            if all(item in frontiers for item in closed_list):
                break
            for neighbor in frontiers:
                cell = neighbor
                if cell in closed_list:
                    continue
                frontiers += list(cell.neighbors.keys())
                frontiers.remove(cell)
                stink_of_seeker = self.find_proximity(state.seeker_position, 5)[cell]
                stink_of_hider = self.find_proximity(state.seeker_position, 3)[cell]
                value = optimalmap[current_cell] - (stink_of_hider * 0.5) + (stink_of_seeker * 3)
                heapq.heappush(probmap, value, cell)
                closed_list.append(cell)

        _, target = heapq.heappop(probmap)
        return target

    def im_crine_please_work(self, state: WorldState) -> shapely.Point:
        ideal_point = self.calculate_optimal(state)
        linestring = ideal_point.neighbors[ideal_point]
        point = shapely.Point(linestring.coords[0])
        return point
        

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
        #choose a target
        if state.frame - self.last_decision_frame > 30 or self.current_target == None:
            # self.current_target = self.map.random_position()
            self.current_target = self.im_crine_please_work(state)
            print(self.current_target)
            self.last_decision_frame = state.frame

        target = self.current_target
        
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
                if self.map.has_line_of_sight(state.hider_position, point):
                    return self.go_straight(state.hider_position, point)

    @property
    def is_seeker(self) -> bool:
        return False