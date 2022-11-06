import json
from math import radians, cos, sin, asin, sqrt

class Graph(object):
    def __init__(self, data=None):
        """Initialize a graph instance.
        Data is the key of the dict and edges are held in a list of names.
        """
        self.graph = {}
        if data:
            for i in data:
                self.add_node(i)

    def nodes(self):
        """Return a list of all nodes in graph."""
        return [key for key in self.graph.keys()]

    def edges(self):
        """Return a list of all edges in the graph."""
        edge_list = []
        if self.nodes:
            for node1 in self.graph:
                for node2 in self.graph[node1]:
                    edge_list.append((node1, node2[0], node2[1]))
        return edge_list

    def add_node(self, node):
        """Add a new node to graph."""
        self.graph.setdefault(node, [])

    def add_edge(self, node1, node2, weight):
        """Add an edge between node1 and node2."""
        self.graph.setdefault(node1, [])
        self.graph.setdefault(node2, [])
        if node2 not in self.graph[node1]:
            self.graph[node1].append((node2, weight))

    def del_node(self, node_delete):
        """Delete the inputted node from the graph."""
        if node_delete not in self.graph:
            raise IndexError('The input node is not in the graph')
        del self.graph[node_delete]
        for node in self.graph:
            for edge in self.graph[node]:
                if edge[0] == node_delete:
                    self.graph[node].remove(edge)

    def del_edge(self, node1, node2):
        """Delete the edge connecting node1 to node 2 if it exists."""
        if node2 not in [edge[0] for edge in self.graph[node1]]:
            raise IndexError('This edge does not exist.')

        removed_edge = [edge for edge in self.graph[node1] if edge[0] == node2]
        self.graph[node1].remove(removed_edge[0])

    def has_node(self, node):
        """Return true if the input node is in the graph, else False."""
        return node in self.graph

    def neighbours(self, node):
        """Return the list of nodes connected to the input node."""
        return [edge[0] for edge in self.graph[node]]

    def adjacent(self, node1, node2):
        """Return True if there is an edge connecting n1 and n2."""
        if node1 not in self.graph or node2 not in self.graph:
            raise KeyError('One or both of these nodes is not in the graph.')
        return node2 in [edge[0] for edge in self.graph[node1]]

    def depth_traversal(self, root, visited=None):
        """Perform depth traversal of graph."""
        if visited is None:
            visited = []
        visited.append(root)
        for edge in self.graph[root]:
            if edge[0] not in visited:
                self.depth_traversal(edge[0], visited)
        return visited

    def breadth_traversal(self, root):
        """Perform breath traversal of graph."""
        visited = [root]
        node_edges = self.graph[root]
        while node_edges:
            edge = node_edges.pop(0)
            if edge[0] not in visited:
                visited.append(edge[0])
                unique_edges = [edge[0] for edge in self.graph[edge]
                                if edge[0] not in visited]
                node_edges.extend(unique_edges)
        return visited

    def dijkstra(self, source, target):
        """Find the shorted path from source node to all other nodes."""
        visited = {source: 0}
        path = {}
        nodes_visit = self.nodes()
        while nodes_visit:
            min_node = None
            for node in nodes_visit:
                if node in visited:
                    if min_node is None:
                        min_node = node
                    elif visited[node] < visited[min_node]:
                        min_node = node
            if min_node is None:
                break

            nodes_visit.remove(min_node)
            current_weight = visited[min_node]

            for edge in self.graph[min_node]:
                weight = current_weight + edge[1]
                if edge[0] not in visited or weight < visited[edge[0]]:
                    visited[edge[0]] = weight
                    path[edge[0]] = min_node

            if min_node == target:
                caminhoA = self._path(source, target, path)
                caminho = []
                for i in caminhoA:
                    caminho.append(list_nomes[i])
                return visited[target], caminho


    def floyd_warshall(self, source, target, infinity=99999999):
        """Use Floyd Marshall algorithm to find shortest path."""
        distance = {}
        path = {}
        nodes = self.nodes()
        for u in nodes:
            distance[u] = {}
            path[u] = {}
            for v in nodes:
                distance[u][v] = infinity #placholder for infinity
                path[u][v] = -1
            distance[u][u] = 0
            for neighbor in self.graph[u]:
                target_node = neighbor[0]
                new_weight = neighbor[1]
                distance[u][target_node] = new_weight
                path[u][target_node] = u

        for t in nodes:
            for u in nodes:
                for v in nodes:
                    new_distance = distance[u][t] + distance[t][v]
                    if new_distance < distance[u][v]:
                        distance[u][v] = new_distance
                        path[u][v] = path[t][v]

        if source not in distance or target not in nodes:
            raise IndexError('These is no exising path between these nodes')
        elif distance[source][target] == infinity:
            return
        exact_path = [target]
        target_path = path[source]
        iter_source = target
        while iter_source != source:
            exact_path.append(target_path[iter_source])
            iter_source = target_path[iter_source]
        caminho = []
        for i in exact_path:
                caminho.append(list_nomes[i])
        return distance[source][target], caminho

    def _path(self, source, target, path):
        """Helper function to return a list of the path."""
        cur_node = target
        ret_path = [target]
        while cur_node != source:
            ret_path.append(path[cur_node])
            cur_node = path[cur_node]
        return ret_path[::-1]


#Ler o arquivo json
with open('cities.json') as json_cidades:
    cidades = json.load(json_cidades)

list_latitude = []
list_longitude = []
list_nomes = []

for i in cidades:
    list_latitude.append(i['latitude'])
    list_longitude.append(i['longitude'])
    list_nomes.append(i['city'])

#calcula a distancia em km de 2 pontos na Terra
def haversine(lon1, lat1, lon2, lat2):
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = (sin(dlat/2)**2 + cos(lat1)) * (cos(lat2) * sin(dlon/2)**2)
    c = 2 * asin(sqrt(a))
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
    return c * r


for i in range(len(cidades)):
    graph = Graph([i])

in_dist = int(input('Digite qual você quer que seja a distância MÁXIMA entre duas cidades para se criar uma aresta: '))

# Vai calcular a distância entre as cidades e ir colocando no grafo.
for i in range(1000):
    for j in range(1000):
        dist = haversine(list_longitude[i], list_latitude[i], list_longitude[j], list_latitude[j])
        #distancia radial
        if(dist<in_dist):
            graph.add_edge(i, j, dist)

# Consegue a cidade de partida do usuário
partida = input("Insira o nome da cidade de partida: ")
while True:
    if partida in list_nomes:
        index1 = list_nomes.index(partida)
        break
    else:
        partida = input("Cidade invalida, insira novamente: ")

# Consegue a cidade destino do usuário
destino = input("Insira o nome da cidade de destino: ")
while True:
    if destino in list_nomes:
        index2 = list_nomes.index(destino)
        break
    else:
        destino = input("Cidade invalida, insira novamente: ")

# Pode escolher qual algoritmo usar mudando o nome dijkstra pra floyd_warshall ou vice-versa
# OBS: O floyd_warshall demora ALGUNS MINUTOS PARA SER EXECUTADO.
print(graph.dijkstra(index1, index2))