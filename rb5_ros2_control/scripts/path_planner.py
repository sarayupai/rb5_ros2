#!/usr/bin/env python3
import math
import heapq

# Sample graph (actual graph imported from main) 
GRAPH = {
    "A": {"pos": (0.0, 0.0), "neighbors": ["B", "D"]},
    "B": {"pos": (0.0, 0.01), "neighbors": ["A", "C"]},
    "C": {"pos": (0.0, 0.02), "neighbors": ["B", "F"]},
    "D": {"pos": (0.03, 0.0), "neighbors": ["A", "G"]},
    #"E": {"pos": (0.03, 0.01), "neighbors": ["C"]},
    "F": {"pos": (0.03, 0.02), "neighbors": ["C", "I"]},
    "G": {"pos": (0.06, 0.0), "neighbors": ["D", "H"]},
    "H": {"pos": (0.06, 0.01), "neighbors": ["G", "I"]},
    "I": {"pos": (0.06, 0.02), "neighbors": ["F", "H"]},
}

# Note: positions are x, y (no angle information)
# Note: assumes paths exists (ie no error handling for unreachable/undefined nodes)

class PathPlanner(): 
    def __init__(self, graph):  
        self.map = graph 

    # Input: goal node, current position
    # Output: path to goal  
    def get_path(self, goal, current_pos): 
        # Get first waypoint 
        w1 = self.get_first_waypoint(current_pos)

        # Run A* using first waypoint to build rest of path 
        path = self.run_astar(w1, goal)

        print(f"[Planner] Planned path: {path}")
        return path 
    
    def run_astar(self, start, goal): 
        open_pq = [(0, start)]
        came_from = {}
        closed_set = set()
        g_score = {node: float('inf') for node in self.map} # cost to this node 
        g_score[start] = 0 

        while len(open_pq) > 0: 
            # Get the current node (pop from open list)
            _, current = heapq.heappop(open_pq) # returns node with smallest cost 

            # Skip if already processed
            if current in closed_set:
                continue

            # Add current to closed list
            closed_set.add(current) 

            # Found the goal
            if current == goal:
                # Reconstruct path 
                path = [0.0, self.map[current]["pos"][1], self.map[current]["pos"][0]]
                while current in came_from:
                    current = came_from[current]
                    path.append(0.0)
                    path.append(self.map[current]["pos"][1])
                    path.append(self.map[current]["pos"][0])
                return path[::-1] # Return reversed path
            
            for neighbor in self.map[current]["neighbors"]:
                # Skip already evaluated neighbors 
                if neighbor in closed_set:
                    continue
                # Get positions of current node, neighbor node, & goal node 
                curr_pos = self.map[current]["pos"]
                neighbor_pos = self.map[neighbor]["pos"]
                goal_pos = self.map[goal]["pos"]

                # TODO: can update to add cost map (ie higher cost for pts near obstacles -> obstacle clearance)
                tentative_g = g_score[current] + self.euclidean(curr_pos, neighbor_pos)

                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    # h score -> distance to goal 
                    h_score = self.euclidean(neighbor_pos, goal_pos)

                    f_score = g_score[neighbor] + h_score
                    heapq.heappush(open_pq, (f_score, neighbor))

    # Given a start position, outputs closest node 
    def get_first_waypoint(self, start):
        # Find closest node in graph 
        min_dist = float('inf')
        closest_pt = None 
        # Cycle through all possible waypoints and select one with smallest distance 
        for name, info in self.map.items():
            dist = self.euclidean(start, info['pos'])
            if dist < min_dist: 
                min_dist = dist
                closest_pt = name 
        print(f"[Planner] Current closest node: {closest_pt}")
        return closest_pt

    # Calculates euclidean distance between two points 
    def euclidean(self, p1, p2): 
        x1, y1 = p1
        x2, y2 = p2
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
#planner = PathPlanner(GRAPH)
#path = planner.get_path('I', (0.0, 0.0)) 

