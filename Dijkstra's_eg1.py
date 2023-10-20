import heapq

class Node:
    def __init__(self, value):
        self.value = value
        self.edges = []

    def add_edge(self, node, weight):
        self.edges.append(Edge(self, node, weight))


class Edge:
    def __init__(self, from_node, to_node, weight):
        self.from_node = from_node
        self.to_node = to_node
        self.weight = weight


class Graph:
    def __init__(self, nodes=[]):
        self.nodes = nodes

    def add_node(self, value):
        self.nodes.append(Node(value))


def dijkstra(graph, start_value):
    start_node = next((n for n in graph.nodes if n.value == start_value), None)
    if not start_node:
        raise ValueError("Start node not found in graph")

    distances = {node.value: float('infinity') for node in graph.nodes}
    distances[start_node.value] = 0

    previous_nodes = {node.value: None for node in graph.nodes}

    pq = [(0, start_node)]

    while pq:
        current_distance, current_node = heapq.heappop(pq)

        if current_distance > distances[current_node.value]:
            continue

        for edge in current_node.edges:
            neighbor = edge.to_node
            new_distance = current_distance + edge.weight

            if new_distance < distances[neighbor.value]:
                distances[neighbor.value] = new_distance
                previous_nodes[neighbor.value] = current_node
                heapq.heappush(pq, (new_distance, neighbor))

    return distances, previous_nodes

# Helper function to retrieve the shortest path
def get_shortest_path(previous_nodes, start_value, end_value):
    path = []
    current_value = end_value

    while current_value:
        path.insert(0, current_value)
        current_value = previous_nodes[current_value].value if previous_nodes[current_value] else None

    return path if path[0] == start_value else None

# Example usage:
graph = Graph()
graph.add_node('A')
graph.add_node('B')
graph.add_node('C')
graph.add_node('D')

graph.nodes[0].add_edge(graph.nodes[1], 1)
graph.nodes[0].add_edge(graph.nodes[2], 4)
graph.nodes[1].add_edge(graph.nodes[2], 2)
graph.nodes[1].add_edge(graph.nodes[3], 5)
graph.nodes[2].add_edge(graph.nodes[3], 3)

distances, previous_nodes = dijkstra(graph, 'A')
print(f"Distances: {distances}")
print(f"Shortest path from A to D: {get_shortest_path(previous_nodes, 'A', 'D')}")
