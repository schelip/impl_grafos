from abc import ABC, abstractmethod
from collections import namedtuple
from math import inf
from typing import Callable, Optional, Iterable, Tuple

from .Color import Color


class Graph(ABC):
    """
    Classe abstrata definindo as características comuns a Grafos Orientados
    e não orientados.

    Attributes:
        V: Lista de vértices
    """

    def __init__(self, n: int):
        """
        Args:
            n: Número de vértices
        """
        self.V = [self.Vertex(i) for i in range(n)]

    def __str__(self):
        return str([str(v) for v in self.V])

    """"""""""""""""""""""""" DEFINIÇÃO DOS VÉRTICES """""""""""""""""""""""""""

    class _Vertex:
        """
        Classe definindo as características de um vértice. Declarada como
        classe interna de Grafo, visto que são uma composição.

        Attributes:
            graph: Grafo a qual pertence
            num: Número identificador
            adj: Vértices adjacentes
            d: Profundidade/Tempo de descobrimento/
                estimativa de caminho mínimo/chave
            f: Tempo de finalização
            pi: Vértice pai
            color: Cor (estado de descobrimento)
        """

        def __init__(self, num: int, graph):
            self.graph = graph
            self.num = num
            self.adj = []
            self.d = inf
            self.f = inf
            self.key = inf
            self.pi = None
            self.color = Color.WHITE

        @property
        def outdegree(self) -> int:
            """
            Grau de saída do vértice
            """
            return len(self.adj)

        @property
        def indegree(self) -> int:
            """
            Grau de entrada do vértice
            """
            n = 0
            for u in self.graph.V:
                if self in u.adj:
                    n += 1
            return n

        def __str__(self):
            return "Vértice(%d)" % self.num

        def initialize(self):
            """
            Retorna o vértice para o estado inicial
            """
            self.d = inf
            self.f = inf
            self.pi = None
            self.color = Color.WHITE

        def describe(self,
                     ADJ: bool = False,
                     D: bool = False,
                     F: bool = False,
                     PI: bool = False):
            """
            Imprime dados do vértice
            Args:
                ADJ: Flag para imprimir vértices adjacentes
                D: Flag para imprimir profundidade/tempo de descoberta/
                    estimativa de caminho mínimo/chave
                F: Flag para imprimir tempo de finalização
                PI: Flag para imprimir vértice pai
            """
            print("%s" % str(self) +
                  ("\nadj: %s" % [str(v) for v in self.adj] if ADJ else "") +
                  ("\nd: %s" % self.d if D else "") +
                  ("\nf: %s" % self.f if F else "") +
                  ("\npai: %s" % self.pi if PI else ""))

    def Vertex(self, num: int) -> _Vertex:
        """
        Wrapper do construtor da classe _Vertex para inicializar vértices sem a
        necessidade de especificar o grafo

        Args:
            num: número identificador do vértice

        Returns:
            Instância de _Vertex com .graph = self
        """
        return self._Vertex(num, self)

    """"""""""""""""""""""""""""""""" ÚTEIS """""""""""""""""""""""""""""""""""
    _all = object()
    """
    Valor utilizado como "flag" representando todos os vértices do
    grafo, visto que propriedades de instância não podem ser utilizadas como
    valor padrão de parâmetros
    """

    Edge = namedtuple('Edge', 'u v')
    """
    Tupla nomeada representando uma aresta do grafo entre os vértices u e v
    """

    @property
    def E(self) -> Iterable[Tuple[_Vertex, _Vertex]]:
        """
        Método gerador para retornar todas as arestas do grafo
        """
        for u in self.V:
            for v in u.adj:
                yield u, v

    @abstractmethod
    def add_edge(self, nu: int, nv: int):
        """
        Método abstrato para adicionar aresta entre dois vértices.
        """
        pass

    def initialize(self, vs: list = _all):
        """
        Inicializa todos ou uma seleção de vértices

        Args:
            vs: Lista de vértices a serem inicializados. Por padrão, todos
        """
        if vs is self._all:
            vs = self.V
        for v in vs:
            v.initialize()

    def initialize_single_source(self, s: _Vertex, vs: list = _all):
        """
        Inicializa todos ou uma seleção de vértices e seta a profundidade de um
        vértice como 0, ou seja, como vértice de entrada

        Args:
            s: Vértice de entrada
            vs: Lista de vértices a serem inicializados
        """
        self.initialize(vs)
        s.d = 0

    def describe(self, vs: list = _all,
                 ADJ: bool = False,
                 D: bool = False,
                 F: bool = False,
                 PI: bool = False):
        """
        Imprime informações sobre todos ou alguns vértices

        Args:
            vs: Lista de vértices a serem descritos
            ADJ: Flag para imprimir vértices adjacentes
            D: Flag para imprimir profundidade/tempo de descoberta/estimativa
                de caminho mínimo
            F: Flag para imprimir tempo de finalização
            PI: Flag para imprimir vértice pai
        """
        if vs is self._all:
            vs = self.V
        for v in vs:
            v.describe(ADJ, D, F, PI)
            print("\n")

    @staticmethod
    def relax(u: _Vertex, v: _Vertex, w: Callable):
        """
        Se possível, melhora a estimativa do caminho mínimo passando por uma
        aresta entre dois vértices

        Args:
            u: Vértice de saída
            v: Vértice de entrada da aresta
            w: Função de peso
        """
        if v.d > u.d + w(u, v):
            v.d = u.d + w(u, v)
            v.pi = u

    def get_path(self, s: _Vertex, v: _Vertex) -> Optional[list[_Vertex]]:
        """
        Obtém caminho entre s e v
        Args:
            s: Vértice de origem
            v: Vértice final

        Returns:
            Lista de vértices partindo de s até v ou None se caminho não existe
        """
        p = [v]
        if v == s:
            return p
        if not v.pi:
            return None
        sp = self.get_path(s, v.pi)
        if sp:
            return sp + p
        return None

    def print_path(self, s: _Vertex, v: _Vertex):
        """
        Imprime o caminho entre dois vértices seguindo as profundidades
        definidas na busca em largura

        Args:
            s: Vértice de origem
            v: Vértice final
        """
        p = self.get_path(s, v)
        if not p:
            print("Nenhum caminho existente de " + str(s) + " para " + str(v))
        else:
            for v in p:
                print(v)

    @staticmethod
    def extract_min(V: list[_Vertex]) -> _Vertex:
        """
        Obtém vértice com menor estimativa de caminho mínimo. Implementada sem
        otimização por estruturas de dados complexas, utilizando apenas um
        arranjo simples, tendo portanto complexidade O(n).
        Args:
            V: Lista de vértices da qual extrair

        Returns:
            Vértice com menor estimativa de caminho mínimo
        """
        min_i = -1
        min_d = inf
        for i in range(len(V)):
            if V[i].d < min_d:
                min_d = V[i].d
                min_i = i
        return V.pop(min_i)

    """""""""""""""""""""""""""" ALGORITMOS DE BUSCA """""""""""""""""""""""""""

    def bfs(self, s: _Vertex):
        """
        Busca primeiro na largura, ou seja, descobre todos os vértices de uma
        mesma profundidade de cada vez

        Args:
            s: Vértice de entrada
        """
        self.initialize_single_source(s, [v for v in self.V if v != s])
        s.color = Color.GRAY
        Q = [s]
        while len(Q) > 0:
            u = Q.pop(0)
            for v in u.adj:
                if v.color == Color.WHITE:
                    v.d = u.d + 1
                    v.pi = u
                    v.color = Color.GRAY
                    Q += [v]
            u.color = Color.BLACK

    def dfs(self):
        """
        Busca primeiro na profundidade, ou seja, descobre um vértice de cada
        profundidade até encontrar uma folha
        """

        def dfs_visit(u: Graph._Vertex):
            nonlocal time
            time += 1
            u.color = Color.GRAY
            u.d = time
            for v in u.adj:
                if v.color == Color.WHITE:
                    v.pi = u
                    dfs_visit(v)
            u.color = Color.BLACK
            time += 1
            u.f = time

        self.initialize()
        time = 0
        for s in self.V:
            if s.color == Color.WHITE:
                dfs_visit(s)

    """"""""""""""""""""""""""""" CAMINHOS MÍNIMOS """""""""""""""""""""""""""""

    def bellman_ford(self, w: Callable, s: _Vertex) -> bool:
        """
        Algoritmo de Bellman-Ford
        Encontra o caminho mínimo de única origem
        Relaxa todas as arestas |V| - 1 vezes e
        detecta ciclos negativos acessíveis

        Args:
            w: Função peso
            s: Vértice de origem

        Returns:
            bool: True se não foi encontrado um ciclo negativo,
                False caso foi encontrado
        """
        self.initialize_single_source(s)
        for i in range(len(self.V)):
            for u, v in self.E:
                self.relax(u, v, w)
        for u, v in self.E:
            if v.d > u.d + w(u, v):
                return False
        return True

    # NAO TESTADO
    # def dijkstra(self, w: Callable, s: _Vertex):
    #     """
    #     Algoritmo de Dijkstra
    #     Encontra o caminho mínimo de única origem de um grafo
    #     utilizando uma versão ponderada da busca em largura.
    #
    #     Args:
    #         w: Função peso
    #         s: Vértice de origem
    #     """
    #     self.initialize_single_source(s)
    #     S = []
    #     Q = self.V[:]
    #     while len(Q) > 0:
    #         u = self.extract_min(Q)
    #         S += u
    #         for v in u.adj:
    #             self.relax(u, v, w)

    @staticmethod
    def floyd_warshall(W: list[list[int]]) \
            -> (list[list[int]], list[list[int]]):
        """
        Algoritmo de Floyd-Warshall
        Calcula o caminho mínimo de todas as origens utilizando
        programação dinâmica
        Args:
            W: Matriz com valores de peso. W[i][j] armazena w(V[i], V[j])

        Returns:
            Matriz contendo custos dos caminhos mínimos entre cada par
        """
        n = len(W)
        D = W[:]
        P = [[None if i == j or W[i][j] == inf else i
              for j in range(n)] for i in range(n)]
        for k in range(n):
            for i in range(n):
                for j in range(n):
                    if D[i][j] > D[i][k] + D[k][j]:
                        D[i][j] = D[i][k] + D[k][j]
                        P[i][j] = P[k][j]
        return D, P

    """"""""""""""""""""""" ÁRVORES GERADORAS MÍNIMAS """""""""""""""""""""""

    def mst_prim(self, w: callable, s: _Vertex):
        """
        Algoritmo de Prim
        Args:
            w: Função de peso
            s: Vértice inicial
        """
        self.initialize_single_source(s)
        Q = self.V[:]
        while len(Q) > 0:
            u = self.extract_min(Q)
            for v in u.adj:
                if v in Q and w(u, v) < v.d:
                    v.pi = u
                    v.d = w(u, v)


class DirectedGraph(Graph):

    def add_edge(self, nu: int, nv: int):
        """
        Adiciona aresta entre dois vértices e adiciona o vértice de saída
        como adjacente do de entrada

        Args:
            nu: Número identificador do vértice de saída
            nv: Número identificador vértice de entrada
        """
        self.V[nu].adj += [self.V[nv]]

    # TODO
    # NAO TESTADO
    # def topological_sort(self):
    #     """
    #     Ordena os vértices topologicamente
    #
    #     Returns:
    #         Uma lista encadeada ordenada de acordo com os tempos
    #         de finalização
    #
    #     Raises:
    #         ValueError: Se o grafo não é um GAO
    #     """
    #
    #     def topological_sort_visit(u: Graph._Vertex):
    #         # Equivalente a dfs_visit, mas inserindo na lista encadeada
    #         nonlocal time, sorted_list
    #         time += 1
    #         u.color = Color.GRAY
    #         u.d = time
    #         for v in u.adj:
    #             if v.color == Color.WHITE:
    #                 v.pi = u
    #                 topological_sort_visit(v)
    #             elif v.color == Color.GRAY:
    #                 raise ValueError
    #         u.color = Color.BLACK
    #         time += 1
    #         u.f = time
    #         sorted_list.insert(0, u)
    #
    #     # Busca em profundidade
    #     self.initialize()
    #     time = 0
    #     sorted_list = []
    #     for s in self.V:
    #         if s.color == Color.WHITE:
    #             topological_sort_visit(s)
    #
    #     return sorted_list

    # TODO
    # NAO TESTADO
    # # Algoritmo para GAOs
    # def dag_shortest_paths(self, w: Callable, s: Graph._Vertex):
    #     """
    #     Encontra o caminho mínimo de uma origem de um GAO
    #     utilizando ordenação topológica
    #
    #     Args:
    #         w: Função peso
    #         s: Vértice de entrada
    #     """
    #     try:
    #         sl = self.topological_sort()
    #         self.initialize_single_source(s)
    #         for u in sl:
    #             for v in u.adj:
    #                 self.relax(u, v, w)
    #     except ValueError:
    #         print("Erro: Grafo não é um GAO")


class UndirectedGraph(Graph):

    def add_edge(self, nu: int, nv: int):
        """
        Adiciona aresta entre dois vértices e adiciona os vértices como
        adjacentes um do outro

        Args:
            nu: Número identificador do vértice de saída
            nv: Número identificador vértice de entrada
        """
        self.V[nu].adj += [self.V[nv]]
        self.V[nv].adj += [self.V[nu]]


class Network(DirectedGraph):
    def __init__(self, n: int):
        super().__init__(n)

        # Capacidade de cada aresta
        self.c = [[0 for _ in range(n)] for _ in range(n)]

        # Fluxo de cada aresta
        self.f = [[0 for _ in range(n)] for _ in range(n)]

    # noinspection PyMethodOverriding
    def add_edge(self, nu: int, nv: int, c: int):
        """
        Adiciona aresta com capacidade
        Args:
            nu: Número do vértice de saída
            nv: Número do vértice de entrada
            c: Capacidade da aresta

        Returns:

        """
        super(Network, self).add_edge(nu, nv)
        self.c[nu][nv] = c

    def get_augmenting_path(self, s: Graph._Vertex, t: Graph._Vertex)\
            -> tuple[Optional[list[Graph.Edge]], int]:
        """
        Constrói a rede residual e usa bfs para identificar se existe um caminho
        aumentador p
        Args:
            s: Vértice fonte
            t: Vértice sumidouro

        Returns:
            p: Lista de vértices do caminho aumentador p ou None se p não existe
            cfp: capacidade residual de p
        """
        Gf = Network(len(self.V))
        for e in [Graph.Edge(u.num, v.num) for u, v in self.E]:
            cf = self.c[e.u][e.v] - self.f[e.u][e.v]
            if cf > 0:
                Gf.add_edge(e.u, e.v, cf)
            cf = self.f[e.u][e.v]
            if cf > 0:
                Gf.add_edge(e.v, e.u, cf)

        Gf.dfs()
        p = Gf.get_path(Gf.V[s.num], Gf.V[t.num])
        if not p:
            return None, 0
        p = [Graph.Edge(p[i].num, p[i + 1].num) for i in range(len(p) - 1)]
        cfp = min(Gf.c[e.u][e.v] for e in p)
        return p, cfp

    def ford_fulkerson(self, s: Graph._Vertex, t: Graph._Vertex) -> int:
        """
        Calcula o fluxo máximo da rede através da identificação de caminhos au-
        mentadores a partir de redes residuais
        Args:
            s: Vértice fonte
            t: Vértice sumidouro

        Returns:
            Valor do fluxo máximo
        """
        self.f = [[0 for _ in range(len(self.f))] for _ in range(len(self.f))]
        max_f = 0
        p, cfp = self.get_augmenting_path(s, t)
        while p:
            for e in p:
                E = [Graph.Edge(u.num, v.num) for u, v in self.E]
                if e in E:
                    self.f[e.u][e.v] = self.f[e.u][e.v] + cfp
                else:
                    self.f[e.v][e.u] = self.f[e.v][e.u] - cfp
            max_f += cfp
            p, cfp = self.get_augmenting_path(s, t)
        return max_f
