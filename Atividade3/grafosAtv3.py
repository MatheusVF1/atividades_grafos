import numpy as np
from shapely import geometry
import matplotlib.pyplot as plt
from math import dist
from descartes.patch import PolygonPatch

def lerVertices(arquivo):
    entrada = arquivo.readlines() 
    aux = 0
    V = []

    for i in range(0, 2):
        divi = entrada[i]
        if i == 0:
            divi = divi.split(",")
            pontoInicial = (float(divi[0]), float(divi[1]))
            V.append(pontoInicial)
        else:
            divi = divi.split(",")
            pontoFinal = (float(divi[0]), float(divi[1]))
            V.append(pontoFinal)

    quantObstaculos = int(entrada[2])
    aux = 3
    quantPontos = 0
    polV = []

    for i in range(quantObstaculos):
        quantPontos = int(entrada[aux])
        polG = []
        for j in range(quantPontos):
            aux += 1
            divi = entrada[aux]

            divi = divi.split(", ")
            V.append((float(divi[0]), float(divi[1])))
            polG.append((float(divi[0]), float(divi[1])))
        polV.append(polG)
        aux += 1
    return pontoInicial, pontoFinal, polV, V

#Indo pegar os dados do arquivo txt
f = open("mapa.txt", "r")
comeco, final, Vertices, allVertices = lerVertices(f)

# fazendo o mapa e plotando ele
armazenPoligono = []
fig, gp = plt.subplots()

for polig in Vertices:
    poliC = geometry.Polygon(shell=polig)
    armazenPoligono.append(poliC)

    x, y = poliC.exterior.xy
    gp.fill(x, y, alpha=0.5, fc='black')
    plt.plot(x, y)

pontos = []
i = 0

for p in allVertices:
    cponto = geometry.Point(p)
    pontos.append(cponto)
    x, y = cponto.xy
    plt.plot(x, y, 'o')
    gp.annotate(str(i), (x[0], y[0]))
    i += 1

lines =[]
fig,gp = plt.subplots()

for i in allVertices:
    for j in allVertices:
      if i!=j:
        vaiProximo = False
        line = geometry.LineString([i,j])
        for poliC in armazenPoligono:
          if line.crosses(poliC) or poliC.contains(line) or poliC.covers(line):
            vaiProximo = True
        if vaiProximo == False:

          lines.append(line)
          x,y = line.xy
          plt.plot(x,y)

for polig in armazenPoligono:
  x,y = polig.exterior.xy
  gp.fill(x, y, alpha=1, fc='black')

class Graph:
    def __init__(self, vertices):
        self.quantVertices = vertices
        self.graph = []

    def gprint(self):
        print(self.graph)

    def add_edge(self, u, v, w):
        self.graph.append([u, v, w])

def grafoVisibilidade(Poligonos, V):
    G = Graph(15)
    fig, gp = plt.subplots()

    for i in V:
        for j in V:
            if i!=j:
                vaiProximo = False
                line = geometry.LineString([i,j])
                for poliC in Poligonos:
                    if line.crosses(poliC) or poliC.contains(line) or poliC.covers(line):
                        vaiProximo = True
                if vaiProximo == False:
                    G.add_edge(allVertices.index(i),allVertices.index(j),line.length)
                    x,y = line.xy
                    plt.plot(x,y)
    for polig in Poligonos:
        x,y = polig.exterior.xy
        gp.fill(x, y, alpha=1, fc='black')
    return G

G = grafoVisibilidade(armazenPoligono, allVertices)

def mstKruskal(G):
    T = []
    indice = 0
    aresta = 0
    G.graph = sorted(G.graph, key=lambda vertice: vertice[2])
    vizinho = []
    classif = []

    for vertice in range(G.quantVertices):
        vizinho.append(vertice)
        classif.append(0)
    
    while aresta < G.quantVertices - 1:
        u, v, w = G.graph[indice]
        indice = indice + 1
        x = achar(vizinho, u)
        y = achar(vizinho, v)
        if x != y:
            aresta = aresta + 1
            T.append([u, v, w])
            juncao(vizinho, classif, x, y)

    custoMinimo = 0
    print("Arestas da árvore construída com Kruskal")

    for u, v, weight in T:
        custoMinimo += weight
        print("{} --- {} == {}".format(u, v, weight))
    
    print("\nMenor árvore vista:", custoMinimo)
    return T

def mstPrim(G):
    T = []
    matrizG = [[0 for coluna in range(G.quantVertices)] for fileira in range(G.quantVertices)]
    Vselecionado = [0] * G.quantVertices

    for aresta in G.graph:
        matrizG[aresta[0]][aresta[1]] = aresta[2]
    
    Vselecionado[0] = True
    quantArestas = 0

    while quantArestas < G.quantVertices - 1:
        minimo = np.Inf
        u = 0
        v = 0
        for i in range(G.quantVertices):
            if Vselecionado[i]:
                for j in range(G.quantVertices):
                    if ((not Vselecionado[j]) and matrizG[i][j]):
                        if minimo > matrizG[i][j]:
                            minimo = matrizG[i][j]
                            u = i
                            v = j
        T.append([u, v, matrizG[u][v]])
        Vselecionado[v] = True
        quantArestas += 1
    
    T = sorted(T, key=lambda vertice: vertice[2])
    custoMinimo = 0
    print("\nArestas da árvore construída com Prim")

    for u, v, weight in T:
        custoMinimo += weight
        print("{} --- {} == {}".format(u, v, weight))
    
    print("\nMenor árvore vista:", custoMinimo)
    return T

def menorChave(key, mst, G):
    minimo = np.Int
    for v in range(G.quantVertices):
        if key[v] < minimo and mst[v] == False:
            minimo = key[v]
            indiceMin = v
    return indiceMin

def juncao(vizinho, classif, x, y):
    if classif[x] < classif[y]:
        vizinho[x] = y
    elif classif[x] > classif[y]:
        vizinho[y] = x
    else:
        vizinho[y] = x
        classif[x] += 1

def achar(vizinho, i):
    if vizinho[i] == i:
        return i
    return achar(vizinho, vizinho[i])

kruskal = mstKruskal(G)
prim = mstPrim(G)

def verticeMaisProximo(T, posicao,pontos):
  newV = geometry.Point((posicao[0],posicao[1]))
  minD=float(99999999)
  vertice =-1
  for i in range(len(pontos)):
    if newV.distance(pontos[i])<minD:
      minD=newV.distance(pontos[i])
      vertice=i
  if minD==0.0:
    print("O vértice já estava incluso no caminho: {}".format(vertice))
  else:
    T=T.append([len(pontos)-1,vertice,minD])
    pontos.append(newV)
  return vertice


#teste com ponto inicial
pos = np.array([1,10])
v =verticeMaisProximo(kruskal,pos,pontos)

from matplotlib import Path


def auxiliarCaminho(pais, v_inicial, v_final):
    path = []
    path.append(v_final)
    while path[-1] != v_inicial:
        path.append(pais[path[-1]])
    path.reverse()
    return path


def computarCaminho(T, pos_inicial, pos_final, pontos):
    v_inicial = verticeMaisProximo(T, pos_inicial, pontos)
    v_final = verticeMaisProximo(T, pos_final, pontos)
    matrixV = [[] for row in range(len(pontos))]
    for u, v, w in T:
        matrixV[u].append(v)
        matrixV[v].append(u)

    for i in range(len(matrixV)):
        matrixV[i] = sorted(matrixV[i], key=lambda v: v)

    visited = set()
    queue = []
    vizinho = [0] * 15
    queue.append(v_inicial)
    visited.add(v_inicial)
    while queue:
        currentV = queue.pop(0)

        if currentV == v_final:
            return auxiliarCaminho(vizinho, v_inicial, v_final)
        for n in matrixV[currentV]:
            if n not in visited:
                vizinho[n] = currentV
                visited.add(n)
                queue.append(n)


pos_inicial = np.array([1, 10])
pos_final = np.array([10, 1])

caminho = computarCaminho(kruskal, pos_inicial, pos_final, pontos)

print(caminho)

import random
pos_inicial = np.array([random.uniform(0,10), random.uniform(0,10)])
pos_final = np.array([random.uniform(0,10), random.uniform(0,10)])
print(pos_inicial,pos_final)

path = computarCaminho(kruskal, pos_inicial, pos_final,pontos)
print(path)
