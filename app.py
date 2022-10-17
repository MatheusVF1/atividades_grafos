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


G = Graph(12)
G.graph = [
#	 1   2   3   4   5   6   7   8   9   10  11  12
	(0,  17, 25, 0,  21, 0,  0,  0,  0,  0,  0,  0),# 1
	(17, 0,  0,  10, 0,  15, 0,  0,  0,  0,  0,  0),# 2
	(25, 0,  0,  0,  0,  0,  20, 0,  0,  0,  0,  0),# 3
	(0,  10, 0,  0,  0,  9,  0,  23, 0,  0,  0,  0),# 4
	(21, 0,  0,  0,  0,  12, 19, 0,  0,  0,  0,  0),# 5
	(0,  15, 0,  9,  12, 0,  0,  8,  7,  0,  0,  0),# 6
	(0,  0,  20, 0,  19, 0,  0,  0,  17, 0,  12, 22),# 7
	(0,  0,  0,  23, 0,  8,  0,  0,  10, 13, 0,  0),# 8
	(0,  0,  0,  0,  0,  7,  17, 0,  10, 12, 15, 0),# 9
	(0,  0,  0,  0,  0,  0,  0,  0,  13, 0,  14, 21),# 10
	(0,  0,  0,  0,  0,  0,  12, 0,  0,  14, 0,  10),# 11
	(0,  0,  0,  0,  0,  0,  22, 0,  0,  21, 10, 0),# 12
	]

dist_vec = []
dist_max = []

for i in range (12):
    G.dijk(i)

print("\n", dist_vec, sep="")
print(dist_max)


