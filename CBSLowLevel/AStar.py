import heapq

def reconstruct_path(cameFrom, current):
    total_path = [current]
    while current in cameFrom.keys():
        current = cameFrom[current]
        total_path.insert(0, current)
    return total_path

def A_star(start, goal, h, graph):
    openSet = {start}
    cameFrom = {}
    gScore = {node: float('inf') for node in graph}
    gScore[start] = 0
    fScore = {node: float('inf') for node in graph}
    fScore[start] = h(start, goal)
    
    while openSet:
        current = min(openSet, key=lambda node: fScore[node])
        if current == goal:
            return reconstruct_path(cameFrom, current)
        
        openSet.remove(current)
        for neighbor in graph[current]:
            tentative_gScore = gScore[current] + graph[current][neighbor]
            if tentative_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_gScore + h(neighbor, goal)
                openSet.add(neighbor)
    
    return None  # No path found

def heuristic(node, goal):
    # Example heuristic function (Euclidean distance)
    return abs(ord(node) - ord(goal))


