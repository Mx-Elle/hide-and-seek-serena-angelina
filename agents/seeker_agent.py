from agent_base import Agent
from world_state import WorldState
from Mesh.nav_mesh import NavMesh
import heapq

# Serena
"""
Strategy: create a heap of "probabilities" of where the hider is. Eliminate locations
the hider could be by considering how far the hider can go after its last sighting
(knowing that the hider's speed is 90% of the seeker's)
"""

<<<<<<< HEAD

class WannabeSeeker(Agent):
    probabilities = []
    
=======
class WannabeSeeker(Agent):
>>>>>>> 6f88dc9b45260e19d8afd50dae341a1c8de59920

    def __init__(self, world_map: NavMesh, max_speed: float):
        Agent.__init__(self, world_map, max_speed)
        self.name = "WannabeSeeker"

    def act(self, state: WorldState) -> tuple[float, float] | None:
        
        ...

    @property
    def is_seeker(self) -> bool:
        return True
