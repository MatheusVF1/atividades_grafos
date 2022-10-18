import numpy as np
import sys

class Graph:

    def __init__(self, vertx):
        self.V = vertx
        self.graph = [[0 for column in range(vertx)]
                      for row in range(vertx)]

    def gerar_tabela_dist(self, dist, source):
        print("\nDistância da instalação central ({}) até os vértices:".format(source+1))
        for node in range(self.V):
            print(node+1, "é igual a ->", dist[node])

    def minDistance(self, dist, sptSet):

        min = sys.maxsize

        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v

        return min_index

    def dijk(self, source):

        dist = [sys.maxsize] * self.V
        dist[source] = 0
        sptSet = [False] * self.V

        for cout in range(self.V):

            u = self.minDistance(dist, sptSet)

            sptSet[u] = True

            for v in range(self.V):
                if self.graph[u][v] > 0 and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]:
                    dist[v] = dist[u] + self.graph[u][v]

        self.gerar_tabela_dist(dist, source)
        self.dist_sum_vec(dist)
        self.max_dist_vec(dist)

    def dist_sum_vec(self, dist):
        a = 0
        for node in range(self.V):
            a+= dist[node]

        dist_vec.append(a)

    def max_dist_vec(self, dist):
        b = 0
        for node in range(self.V):
            if dist[node] > b:
                b = dist[node]

        dist_max.append(b)


G = Graph(10)
G.graph = [
#	 1  2   3  4  5   6   7   8   9   10  
	(0, 10, 0, 0, 3,  0,  5,  0,  0,  0),# 1
	(5, 0,  0, 3, 0,  3,  0,  0,  0,  0),# 2
	(6, 0,  0, 8, 0,  8,  0,  0,  0,  0),# 3
	(0, 5,  0, 0, 10, 1,  0,  0,  0,  0),# 4
	(0, 0,  0, 0, 0,  20, 0,  0,  11, 8),# 5
	(0, 0,  0, 0, 0,  0,  20, 5,  0,  12),# 6
	(0, 0,  2, 0, 7,  0,  0,  0,  0,  0),# 7
	(0, 0,  0, 0, 0,  0,  2,  0,  12, 3),# 8
	(0, 0,  0, 0, 2,  20, 0,  4,  0,  0),# 9
	(5, 0,  7, 0, 4,  0,  0,  20, 6,  0),# 10
	]

dist_vec = []
dist_max = []

for i in range (10):
    G.dijk(i)

print("\n", dist_vec, sep="")
print(dist_max)

menor_vec = min(dist_vec)
menor_max = 100
indice = -1

for go in range(10):
    if dist_vec[go] == menor_vec:
        if dist_max[go] < menor_max:
            menor_max = dist_max[go]
            indice = go

print('\nO melhor vértice para ser a estação central: Vértice {}'.format(indice+1))
