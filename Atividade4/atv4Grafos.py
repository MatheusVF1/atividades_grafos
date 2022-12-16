class Graph():
    def __init__(self, vertices):
        self.graph = [[0 for coluna in range(vertices)]
                            for linha in range(vertices)]
        self.V = vertices

    def verifAdjacente(self, v, pos, path):
        if self.graph[path[pos-1]][v] == 0:   # Vai verificar se o vértice é adjacente ao vertice anterior
            return False
        for vertex in path:
            if vertex == v:
                return False
        return True

    def cicloHamiltoniano(self):
        path = [-1] * self.V
        path[0] = 0

        if self.cicloHamiltonianoUtil(path, 1) == False:    
            #se nao tem solucao
            print("Não possui um ciclo hamiltoniano\n")
            return False
	      #diz se tem solucao
        self.printaResultado(path)
        return True

    def cicloHamiltonianoUtil(self, path, pos):
        if pos == self.V:
            if self.graph[path[pos-1]][path[0]] == 1:      # Verifica se o caminho calculado é um Ciclo Hamiltoniano
                return True
            else:
                return False
        #Tenta com vertices diferentes
        for v in range(1,self.V):
            if self.verifAdjacente(v, pos, path) == True:
                path[pos] = v
                if self.cicloHamiltonianoUtil(path, pos+1) == True:
                    return True
                path[pos] = -1  # Vai remover o vértice se não for ciclo
        return False

    def printaResultado(self, path):
        print ("O ciclo hamiltoniano é: ")   # Exibe o cliclo obtido
        for vertex in path:
            print(vertex, end = " -> ")
        print(path[0])


grafo1 = Graph(6)

grafo1.graph[0][1] = 1
grafo1.graph[1][0] = 1
grafo1.graph[1][2] = 1
grafo1.graph[2][1] = 1
grafo1.graph[2][3] = 1
grafo1.graph[3][2] = 1
grafo1.graph[3][4] = 1
grafo1.graph[4][3] = 1
grafo1.graph[4][5] = 1
grafo1.graph[5][4] = 1
grafo1.graph[5][0] = 1
grafo1.graph[0][5] = 1

print("Resultado do Grafo 1 gerado: ")
print(grafo1.graph, "\n")
grafo1.cicloHamiltoniano()


################################################################################
################################################################################


grafo2 = Graph(10)

grafo2.graph[0][1] = 1
grafo2.graph[0][4] = 1
grafo2.graph[0][6] = 1
grafo2.graph[1][0] = 1
grafo2.graph[1][3] = 1
grafo2.graph[1][5] = 1
grafo2.graph[2][0] = 1
grafo2.graph[2][3] = 1
grafo2.graph[2][5] = 1
grafo2.graph[3][4] = 1
grafo2.graph[3][5] = 1
grafo2.graph[3][1] = 1
grafo2.graph[4][5] = 1
grafo2.graph[4][9] = 1
grafo2.graph[4][8] = 1
grafo2.graph[5][6] = 1
grafo2.graph[5][9] = 1
grafo2.graph[5][7] = 1
grafo2.graph[6][2] = 1
grafo2.graph[6][4] = 1
grafo2.graph[7][6] = 1
grafo2.graph[7][8] = 1
grafo2.graph[7][9] = 1
grafo2.graph[8][4] = 1
grafo2.graph[8][5] = 1
grafo2.graph[8][7] = 1
grafo2.graph[9][0] = 1
grafo2.graph[9][2] = 1
grafo2.graph[9][4] = 1
grafo2.graph[9][7] = 1
grafo2.graph[9][8] = 1

print("Resultado do Grafo 2 gerado: ")
print(grafo2.graph, "\n")
grafo2.cicloHamiltoniano()


################################################################################
################################################################################


grafo3 = Graph(8)

grafo3.graph[0][2] = 1
grafo3.graph[2][0] = 1
grafo3.graph[0][4] = 1
grafo3.graph[4][0] = 1
grafo3.graph[0][6] = 1
grafo3.graph[6][0] = 1
grafo3.graph[1][3] = 1
grafo3.graph[3][1] = 1
grafo3.graph[1][7] = 1
grafo3.graph[7][1] = 1
grafo3.graph[2][4] = 1
grafo3.graph[4][2] = 1
grafo3.graph[2][6] = 1
grafo3.graph[6][2] = 1
grafo3.graph[3][5] = 1
grafo3.graph[5][3] = 1
grafo3.graph[3][7] = 1
grafo3.graph[7][3] = 1
grafo3.graph[4][6] = 1
grafo3.graph[6][4] = 1
grafo3.graph[4][5] = 1
grafo3.graph[5][4] = 1
grafo3.graph[5][7] = 1
grafo3.graph[7][5] = 1

print("Resultado do Grafo 2 gerado: ")
print(grafo3.graph, "\n")
grafo3.cicloHamiltoniano()