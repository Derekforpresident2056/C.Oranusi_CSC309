# ==========================
# Drone Delivery Navigation
# ==========================

import heapq
from collections import deque

# --------- Graph Definition ---------
graph = {
    'A': {'B': 2, 'C': 5, 'D': 1},
    'B': {'A': 2, 'D': 2, 'E': 3},
    'C': {'A': 5, 'D': 2, 'F': 3},
    'D': {'A': 1, 'B': 2, 'C': 2, 'E': 1, 'F': 4},
    'E': {'B': 3, 'D': 1, 'G': 2},
    'F': {'C': 3, 'D': 4, 'G': 1},
    'G': {'E': 2, 'F': 1, 'H': 3},
    'H': {'G': 3}
}

heuristic = {
    'A': 7,
    'B': 6,
    'C': 6,
    'D': 4,
    'E': 2,
    'F': 2,
    'G': 1,
    'H': 0
}

start = 'A'
goal = 'H'

# ===================================
# Depth-First Search (DFS)
# ===================================
def dfs(graph, start, goal):
    visited = set()
    stack = deque([(start, [start])])       

    while stack:
        
        current_node, path = stack.pop()

        if current_node == goal:
            return path

        if current_node not in visited:
            visited.add(current_node)
            
            
            for neighbor in graph[current_node]:
                if neighbor not in visited:
                    
                    new_path = list(path)
                    new_path.append(neighbor)
                    stack.append((neighbor, new_path))
    
    return None

# ===================================
# Breadth-First Search (BFS)
# ===================================
def bfs(graph, start, goal):
    visited = set()
    queue = deque([(start, [start])])

    while queue:
       
        current_node, path = queue.popleft()

        if current_node == goal:
            return path

        if current_node not in visited:
            visited.add(current_node)
            
            
            for neighbor in graph[current_node]:
                if neighbor not in visited:
                    
                    new_path = list(path)
                    new_path.append(neighbor)
                    queue.append((neighbor, new_path))
    
    return None 

# ===================================
# Uniform Cost Search (UCS)
# ===================================
def ucs(graph, start, goal):
   
    frontier = []
    heapq.heappush(frontier, (0, start, [start]))
    
   
    visited_costs = {start: 0}

    while frontier:
       
        current_cost, current_node, path = heapq.heappop(frontier)

        
        if current_node == goal:
            return path

        for neighbor, weight in graph[current_node].items():
            new_cost = current_cost + weight
            
            
            if neighbor not in visited_costs or new_cost < visited_costs[neighbor]:
                visited_costs[neighbor] = new_cost
                new_path = list(path)
                new_path.append(neighbor)
                heapq.heappush(frontier, (new_cost, neighbor, new_path))
    
    return None

# ===================================
# A* Search
# ===================================
def a_star(graph, start, goal, heuristic):
    
    frontier = []
    
    

    start_f_score = 0 + heuristic[start]
    heapq.heappush(frontier, (start_f_score, 0, start, [start]))
    
    visited_costs = {start: 0} 

    while frontier:
        
        f_score, current_g, current_node, path = heapq.heappop(frontier)

        if current_node == goal:
            return path

        for neighbor, weight in graph[current_node].items():
           
            new_g = current_g + weight
            
            
            if neighbor not in visited_costs or new_g < visited_costs[neighbor]:
                visited_costs[neighbor] = new_g
                
                
                new_f = new_g + heuristic[neighbor]
                
                new_path = list(path)
                new_path.append(neighbor)
                
                heapq.heappush(frontier, (new_f, new_g, neighbor, new_path))
    
    return None

# ===================================
# Run and Compare
# ===================================
if __name__ == "__main__":
    print("DFS Path:", dfs(graph, start, goal))
    print("BFS Path:", bfs(graph, start, goal))
    print("UCS Path:", ucs(graph, start, goal))
    print("A* Path:", a_star(graph, start, goal, heuristic))