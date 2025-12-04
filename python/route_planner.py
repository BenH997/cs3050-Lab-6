#!/usr/bin/env python3
"""
Route Planner with Dijkstra, A*, and Bellman-Ford algorithms
"""

import sys
import csv
import heapq
import math
from typing import Dict, List, Tuple, Optional

EARTH_RADIUS = 6371.0  # km

# Updated to contain earliest and latest attributes
class Node:
    """Represents a node in the graph"""
    def __init__(self, node_id: int, lat: float, lon: float, earl: int, late: int):
        self.id = node_id
        self.lat = lat
        self.lon = lon
        self.earliest = earl
        self.latest = late

class Edge:
    """Represents an edge in the graph"""
    def __init__(self, to: int, weight: float):
        self.to = to
        self.weight = weight

# Updated add_node method to work with earliest and latest attributes
class Graph:
    """Graph data structure with adjacency list"""
    def __init__(self):
        self.nodes: Dict[int, Node] = {}
        self.adj_list: Dict[int, List[Edge]] = {}
    
    def add_node(self, node_id: int, lat: float, lon: float, earl: int, late: int):
        """Add a node to the graph"""
        self.nodes[node_id] = Node(node_id, lat, lon, earl, late)
        if node_id not in self.adj_list:
            self.adj_list[node_id] = []
    
    def add_edge(self, from_id: int, to_id: int, weight: float):
        """Add an edge to the graph"""
        if from_id not in self.adj_list:
            self.adj_list[from_id] = []
        self.adj_list[from_id].append(Edge(to_id, weight))

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate haversine distance between two points"""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return EARTH_RADIUS * c


def dijkstra(graph: Graph, start: int, end: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]], int]:
    """
    Dijkstra's algorithm for shortest path
    Returns: (distances, previous nodes, nodes explored)
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0

    pq = [(0, start)]
    nodes_explored = 0
    visited = set()
    
    while pq:
        current_dist, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        
        visited.add(u)
        nodes_explored += 1
        
        if u == end:
            break
        
        if current_dist > dist[u]:
            continue
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            alt = dist[u] + edge.weight
            
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                heapq.heappush(pq, (alt, v))

    return dist, prev, nodes_explored

# Finds path with minimal time constraint violations
def closestPath(graph: Graph, start: int, end: int):
    dist = {n: float('inf') for n in graph.nodes}
    prev = {n: None for n in graph.nodes}
    violations = {n: float('inf') for n in graph.nodes}

    dist[start] = 0
    violations[start] = 0

    # Heap to track (arrival_time, total_violation, node_id)
    pq = [(0, 0, start)]
    nodesExplored = 0

    while pq:
        time, currV, u = heapq.heappop(pq)
        nodesExplored += 1

        if u == end:
            return dist, prev, nodesExplored, violations

        # Evaluate adjacent edges
        for edge in graph.adj_list[u]:
            v = edge.to
            arrival = time + edge.weight
            node = graph.nodes[v]

            # Find violation score
            early = 0
            late = 0
            if node.earliest - arrival > 0:
                early = node.earliest - arrival
            if arrival - node.latest > 0:
                late = arrival - node.latest

            vertViolation = currV + early + late

            # Relax if arrival time is better or violation score is better
            if arrival < dist[v] or vertViolation < violations[v]:
                dist[v] = arrival
                violations[v] = vertViolation
                prev[v] = u
                heapq.heappush(pq, (arrival, vertViolation, v))

    # Path not found
    return dist, prev, nodesExplored, violations

# Modified Dijkstra's algorithm to support time windows
def dijkstraTimeWindow(graph: Graph, start: int, end: int):
    dist = {n: float('inf') for n in graph.nodes}
    prev = {n: None for n in graph.nodes}
    dist[start] = 0

    pq = [(0, start)]
    visited = set()
    nodesExplored = 0

    while pq:
        time, u = heapq.heappop(pq)
        nodesExplored += 1

        if u in visited:
            continue
        visited.add(u)

        node = graph.nodes[u]

        # Wait if early
        if time < node.earliest:
            time = node.earliest
        if time > node.latest:
            continue

        if u == end:
            return dist, prev, nodesExplored, None

        for edge in graph.adj_list[u]:
            v = edge.to
            newT = time + edge.weight

            if newT < dist[v]:
                dist[v] = newT
                prev[v] = u
                heapq.heappush(pq, (newT, v))

    # Failed to find feasable path
    # Find closest path
    dist, prev, explored2, violations = closestPath(graph, start, end)
    return dist, prev, nodesExplored + explored2, violations

def multiDestinationPriority(graph, dests, threshold=None):
    # Group destinations by priority
    priorityGroups = {
        "HIGH": [],
        "MEDIUM": [],
        "LOW": []
    }

    for node, priority in dests:
        priorityGroups[priority].append(node)

    # Init path with first node
    path = []
    for nodes in priorityGroups.values():
        if len(nodes) > 0:
            path.append(nodes[0])
    
    totalDist = 0
    totalNodesVisited = 0
    violations = []

    for priorityLevel in ["HIGH","MEDIUM","LOW"]:
        while priorityGroups[priorityLevel]:
            fromNode = path[-1]
            toNode = priorityGroups[priorityLevel][0]

            dist, prev, nodesExplored = dijkstra(graph, fromNode, toNode)

            originalDistance = dist[toNode]

            if threshold != None:
                # Violate priority contraints if shorter path is found
                for lower in ["MEDIUM","LOW"]:
                    for n in priorityGroups[lower]:
                        altDist, altPrev, altNodesExplored = dijkstra(graph, fromNode, n)
                        if altDist[n] < (originalDistance / threshold):
                            violations.append(f"Visited {n} before {toNode}")
                            toNode = n
                            dist = altDist
                            prev = altPrev
                            originalDistance = altDist[n]

            # Add sub path to main path
            subPath = reconstruct_path(prev, fromNode, toNode)
            
            if subPath != None:
                path.extend(subPath[1:])
                totalDist += originalDistance

            # Remove visited
            for priorityList in priorityGroups.values():
                if toNode in priorityList:
                    priorityList.remove(toNode)

    return path, totalDist, violations

# def multiDestinationPriority(graph: Graph, start: int, destinations: Dict[int, str]):
#     path = [start]
#     totalTravel = 0
#     totalNodesVisited = 0
#     fromNode = start
#     toNode = start
#     violations = []

#     # Separate nodes based on priority
#     highNodes = []
#     medNodes = []
#     lowNodes = []
    
#     # Construct different lists of priorities
#     for index in range(len(destinations[0])):
#         if destinations[1][index] == "HIGH":
#             highNodes.append(destinations[0][index])
#         elif destinations[1][index] == "MEDIUM":
#             medNodes.append(destinations[0][index])
#         else:
#             lowNodes.append(destinations[0][index])

#     nodeLists = (highNodes, medNodes, lowNodes)

#     # Construct path which visits all nodes
#     for nodes in nodeLists:
#         while len(nodes) > 0:
#             fromNode = path[-1]
#             toNode = nodes[0]

#             dist, prev, nodesExplored = dijkstra(graph, fromNode, toNode)
            
#             totalTravel += dist[toNode]
#             totalNodesVisited += nodesExplored
            
#             subPath = reconstruct_path(prev, fromNode, toNode)

#             # Deal with no path found
#             if subPath != None:
#                 # Add sub path to main path
#                 for node in subPath:
#                     if node == subPath[0]:
#                         continue

#                     path.append(node)
#             else:
#                 violations.append(f"Node {toNode} unreachable from {fromNode}.")

#             nodes.remove(nodes[0])
            
#             # If already visited remove from nodes to visit
#             for priorityList in nodeLists:
#                 for node in priorityList:
#                     if node in path:
#                         priorityList.remove(node)

#     print(violations)
#     print(path)
#     print(totalNodesVisited)
#     print(totalTravel)

#     # Todo: return valid path specs
#     return  

def astar(graph: Graph, start: int, end: int) -> Tuple[Dict[int, float], Dict[int, Optional[int]], int]:
    """
    A* algorithm for shortest path
    Returns: (distances, previous nodes, nodes explored)
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    end_node = graph.nodes[end]
    
    def heuristic(node_id: int) -> float:
        node = graph.nodes[node_id]
        return haversine(node.lat, node.lon, end_node.lat, end_node.lon)
    
    pq = [(heuristic(start), start)]
    nodes_explored = 0
    visited = set()
    
    while pq:
        _, u = heapq.heappop(pq)
        
        if u in visited:
            continue
        
        visited.add(u)
        nodes_explored += 1
        
        if u == end:
            break
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            alt = dist[u] + edge.weight
            
            if alt < dist[v]:
                dist[v] = alt
                prev[v] = u
                f_score = alt + heuristic(v)
                heapq.heappush(pq, (f_score, v))
    
    return dist, prev, nodes_explored


def bellman_ford(graph: Graph, start: int, end: int) -> Tuple[Optional[Dict[int, float]], Optional[Dict[int, Optional[int]]], int]:
    """
    Bellman-Ford algorithm for shortest path
    Can handle negative weights and detect negative cycles
    Returns: (distances, previous nodes, nodes explored) or (None, None, 0) if negative cycle detected
    """
    dist = {node_id: float('inf') for node_id in graph.nodes}
    prev = {node_id: None for node_id in graph.nodes}
    dist[start] = 0
    
    nodes_explored = 0
    node_count = len(graph.nodes)
    
    # Relax edges |V| - 1 times
    for i in range(node_count - 1):
        updated = False
        for u in graph.nodes:
            if dist[u] == float('inf'):
                continue
            
            for edge in graph.adj_list.get(u, []):
                v = edge.to
                if dist[u] + edge.weight < dist[v]:
                    dist[v] = dist[u] + edge.weight
                    prev[v] = u
                    updated = True
        
        nodes_explored += 1
        if not updated:
            break
    
    # Check for negative cycles
    for u in graph.nodes:
        if dist[u] == float('inf'):
            continue
        
        for edge in graph.adj_list.get(u, []):
            v = edge.to
            if dist[u] + edge.weight < dist[v]:
                return None, None, 0  # Negative cycle detected
    
    return dist, prev, nodes_explored


def reconstruct_path(prev: Dict[int, Optional[int]], start: int, end: int) -> Optional[List[int]]:
    """Reconstruct path from start to end using previous nodes"""
    if prev[end] is None and start != end:
        return None
    
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = prev[current]
    
    path.reverse()
    return path


def print_path(graph: Graph, prev: Dict[int, Optional[int]], start: int, end: int, distance: float):
    """Print the path from start to end"""
    path = reconstruct_path(prev, start, end)
    
    if path is None:
        print("No path found")
        return
    
    path_str = " -> ".join(str(node) for node in path)
    print(f"Path from {start} to {end}: {path_str}")
    print(f"Total distance: {distance:.2f} km")

# Updated to include the time constraints of nodes
def load_graph(nodes_file: str, edges_file: str) -> Graph:
    """Load graph from CSV files"""
    graph = Graph()
    
    # Load nodes
    with open(nodes_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            node_id = int(row['id'])
            lat = float(row['lat'])
            lon = float(row['lon'])
            earl = int(row['earliest'])
            late = int(row['latest'])
            graph.add_node(node_id, lat, lon, earl, late)
    
    # Load edges
    with open(edges_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            from_id = int(row['from'])
            to_id = int(row['to'])
            distance = float(row['distance'])
            graph.add_edge(from_id, to_id, distance)
    
    return graph


def main():
    if len(sys.argv) != 6:
        print(f"Usage: {sys.argv[0]} <nodes.csv> <edges.csv> <start_node> <end_node> <algorithm>")
        print("Algorithms: dijkstra, dijkstraTimeWindows, astar, bellman-ford")
        sys.exit(1)
    
    nodes_file = sys.argv[1]
    edges_file = sys.argv[2]
    start_node = int(sys.argv[3])
    end_node = int(sys.argv[4])
    algorithm = sys.argv[5]
    
    # Load graph
    graph = load_graph(nodes_file, edges_file)
    
    # Validate nodes
    if start_node not in graph.nodes or end_node not in graph.nodes:
        print("Invalid start or end node")
        sys.exit(1)
    
    # Run selected algorithm
    if algorithm == "dijkstra":
        print("=== Dijkstra's Algorithm ===")
        dist, prev, nodes_explored = dijkstra(graph, start_node, end_node)
    elif algorithm == "astar":
        print("=== A* Algorithm ===")
        dist, prev, nodes_explored = astar(graph, start_node, end_node)
    elif algorithm == "bellman-ford":
        print("=== Bellman-Ford Algorithm ===")
        dist, prev, nodes_explored = bellman_ford(graph, start_node, end_node)
        if dist is None:
            print("Negative cycle detected!")
            sys.exit(1)
    elif algorithm == "dijkstraTimeWindows":
        print("=== Dijkstra's Algorithm with time windows ===")
        dist, prev, nodesExplored, violations = dijkstraTimeWindow(graph, start_node, end_node)
    elif algorithm == "multiDestinationPriority":
        print("=== Multiple Destination Priority Pathing ===")
        """
        Maybe have an option to load destinations from a file or manually????????
        Will eventually ask for the start node via input()
        Will then ask for following destinations to visit
        **** Will ask for priority of each node
        """

        dests = [
            (1, "HIGH"),
            (2, "MEDIUM"),
            (6, "LOW")
        ]

        print(multiDestinationPriority(graph, dests))
    else:
        print(f"Unknown algorithm: {algorithm}")
        print("Available algorithms: dijkstra, dijkstraTimeWindows, astar, bellman-ford")
        sys.exit(1)
    
    # Print results
    if algorithm == "dijkstraTimeWindows" and violations != None:
        printClosestPath(prev, start_node, end_node, dist[end_node], nodesExplored, violations)
    elif algorithm == "multiDestinationPriority":
        pass
    else:
        print_path(graph, prev, start_node, end_node, dist[end_node])
        print(f"Nodes explored: {nodesExplored}")

# Prints a path including violations and distance traveled
def printClosestPath(prev: Dict[int, Optional[int]], start: int, end: int, distance: float, nodesExplored: int, violations: Dict[int, str]):
    path = reconstruct_path(prev, start, end)
    
    if path != None:
        print("No feasable path found satisfying time conditions. Closest path will be provided instead:")
        print(f"Path from {start} to {end}: ", end='')

        for node in path:
            if node == start:
                print(f"{node} -> ", end='')
            elif node == end:
                print(f"{node}({violations[node]} units early/late)")
            else:
                print(f"{node}({violations[node]} units early/late) -> ", end='')

        print(f"Total distance: {distance} km")
        print(f"Nodes explored: {nodesExplored}")
    else:
        print(f"No possible paths exist from {start} to {end}.")
    

if __name__ == "__main__":
    main()