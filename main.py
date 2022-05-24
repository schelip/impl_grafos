from classes.Graph import *


def main():
    print('''Grafo orientado DG para testes:
              -5         1
          [1]<----[4]---->[6]
         /     2 ^   \\       \\
      3 v       /  6  v 2     2 v
     [0(s)]---->[2]---->[5]---->[7]
            4   \\     ^ 2    -3
                1 v   / 
                   [3]
    ''')

    dg = DirectedGraph(8)
    dg.add_edge(0, 2)
    dg.add_edge(1, 0)
    dg.add_edge(2, 3)
    dg.add_edge(2, 4)
    dg.add_edge(2, 5)
    dg.add_edge(3, 5)
    dg.add_edge(4, 1)
    dg.add_edge(4, 5)
    dg.add_edge(4, 6)
    dg.add_edge(5, 7)
    dg.add_edge(6, 7)

    i = inf
    #      0  1  2  3  4  5  6  7
    W1 = [[0, i, 4, i, i, i, i, i],  # 0
          [3, 0, i, i, i, i, i, i],  # 1
          [i, i, 0, 1, 2, 6, i, i],  # 2
          [i, i, i, 0, i, 2, i, i],  # 3
          [i, -5, i, i, 0, 2, 1, i],  # 4
          [i, i, i, i, i, 0, i, -3],  # 5
          [i, i, i, i, i, i, 0, 2],  # 6
          [i, i, i, i, i, i, i, 0]]  # 7

    def w1(u, v):
        return W1[u.num][v.num]

    print('''Grafo não orientado UG para testes:
            [0(s)]
         3 /    \\ 5
          /   1  \\     4 
        [1]-------[4]------[2]
        /       /  \\     /2 
     4 /  2   2/   1\\  /  
     [3]-----[5]     [6]
    ''')

    ug = UndirectedGraph(7)
    ug.add_edge(0, 1)
    ug.add_edge(0, 4)
    ug.add_edge(1, 3)
    ug.add_edge(1, 4)
    ug.add_edge(2, 4)
    ug.add_edge(2, 6)
    ug.add_edge(3, 5)
    ug.add_edge(4, 5)
    ug.add_edge(4, 6)

    #      0  1  2  3  4  5  6
    W2 = [[0, 3, i, i, 5, i, i],  # 0
          [3, 0, i, 4, 1, i, i],  # 1
          [i, i, 0, i, 4, i, 2],  # 2
          [i, 4, i, 0, i, 2, i],  # 3
          [5, 1, 4, i, 0, 2, 1],  # 4
          [i, i, i, 2, 2, 0, i],  # 5
          [i, i, 2, i, 1, i, 0]]  # 6

    def w2(u, v):
        return W2[u.num][v.num]

    print('''Rede NG para teste:
                7
          [1] ----> [3]
      4 ^  ^         ^  \\ 5
       /   | 5       |   v 
     [s]   |       7 |   [t]
       \\  |         |   ^ 
      6 v  |         |  / 6
          [2] ----> [4]
               4
    ''')

    ng = Network(6)
    ng.add_edge(0, 1, 4)
    ng.add_edge(0, 2, 6)
    ng.add_edge(1, 3, 7)
    ng.add_edge(2, 1, 5)
    ng.add_edge(2, 4, 4)
    ng.add_edge(3, 5, 6)
    ng.add_edge(4, 3, 7)
    ng.add_edge(4, 5, 6)

    c = 1
    while c != 0:
        print('''Qual algoritmo deseja executar?
    (1) Busca em largura
    (2) Busca em profundidade
    (3) Algoritmo de Bellman-Ford
    (4) Algoritmo de Floyd-Warshall
    (5) Algoritmo de Ford-Fulkerson
    (6) Algoritmo de Prim
    (0) Sair
    ->''', end=" ")
        # Implementados mas sem testes:
        # - Algoritmo para GAOs
        # - Algoritmo de Dijkstra

        c = int(input())

        if c == 1:
            print("Rodando BFS em UG...")
            ug.bfs(ug.V[0])
            ug.describe(D=True, PI=True)
        elif c == 2:
            print("Rodando DFS em DG...")
            dg.dfs()
            dg.describe(D=True, F=True, PI=True)
        elif c == 3:
            print("Rodando Algoritmo de Bellman-Ford em DG...")
            dg.bellman_ford(w1, dg.V[0])
            dg.describe(D=True, PI=True)
        elif c == 4:
            print("Rodando Algoritmo de Floyd-Warshall em DG...")
            D, P = dg.floyd_warshall(W1)
            print("Matriz de caminhos mínimos:")
            for i in range(len(D)):
                print(D[i])
            print("\nMatriz de predecessores:")
            for i in range(len(P)):
                print(P[i])
        elif c == 5:
            print("Rodando Algoritmo de Ford-Fulkerson em NG...")
            print("Fluxo máximo: " + str(ng.ford_fulkerson(ng.V[0], ng.V[5])))
        elif c == 6:
            print("Rodando Algoritmo de Prim em UG...")
            ug.mst_prim(w2, ug.V[0])
            ug.describe(D=True, PI=True)
        elif c == 0:
            print("Finalizando programa")
            break
        else:
            print("Entrada inválida")


if __name__ == '__main__':
    main()
