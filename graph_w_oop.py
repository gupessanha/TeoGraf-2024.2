class Node:
    def __init__(self, value, adj=[]):
        # Inicializa um nó com um valor e uma lista de nós adjacentes
        self.value = value
        self.adj = adj
        
    def add_adj(self, node):
        # Adiciona um nó à lista de adjacentes
        self.adj.append(node)
        
    def __str__(self):
        # Retorna uma string representando o nó e seus adjacentes
        l = [i.value for i in self.adj]
        return str(self.value) + " -> " + str(l)
    

class Graph:
    """
    Classe que representa um grafo.
    Atributos:
    -----------
    nodes : list
        Lista de nós do grafo.
    Métodos:
    --------
    __init__(nodes=[]):
        Inicializa o grafo com uma lista de nós.
    add_node(node):
        Adiciona um nó ao grafo.
    print_nodes():
        Imprime todos os nós do grafo.
    add_edge(node1, node2):
        Adiciona uma aresta entre dois nós no grafo.
    load_from_file(file):
        Carrega um grafo a partir de um arquivo.
    __str__():
        Retorna uma representação em string do grafo.
    vertices():
        Retorna o número de vértices no grafo.
    edges():
        Retorna o número de arestas no grafo.
    degree(node):
        Retorna o grau de um nó específico.
    max_degree():
        Retorna o grau máximo de um nó no grafo.
    min_degree():
        Retorna o grau mínimo de um nó no grafo.
    adjacent_list():
        Imprime a lista de adjacência do grafo.
    """
    def __init__(self, nodes = []):
        # Inicializa o grafo com uma lista de nós
        self.nodes = nodes
        
    def add_node(self, node):
        # Adiciona um nó à lista de nós do grafo
        self.nodes.append(node)
    
    def print_nodes(self):
        # Imprime todos os nós do grafo
        for i in self.nodes:
            print(i)
            
    def add_edge(self, node1, node2):
        # Adiciona uma aresta entre dois nós no grafo
        # Verifica se a aresta já existe antes de adicionar
        if node2 not in node1.adj:
            node1.add_adj(node2)
        if node1 not in node2.adj:
            node2.add_adj(node1)
    
    def load_from_file(self, file):
        # Carrega um grafo a partir de um arquivo
        with open(file, "r") as f:
            for line in f:
                line = line.strip().split(" ")
                
                if len(line) != 2:
                    continue
                
                n1 = n2 = None
                for i in self.nodes:
                    if i.value == line[0]:
                        n1 = i
                    if i.value == line[1]:
                        n2 = i
                if n1 is None:
                    n1 = Node(line[0])
                    self.add_node(n1)
                if n2 is None: 
                    n2 = Node(line[1])
                    self.add_node(n2)
                self.add_edge(n1, n2)
        
    def __str__(self):
        # Retorna uma representação em string do grafo
        return "\n".join(str(i) for i in self.nodes)
    
    def vertices(self):
        # Retorna o número de vértices no grafo
        return len(self.nodes)
    
    def edges(self):
        # Retorna o número de arestas no grafo
        edges = sum(len(i.adj) for i in self.nodes)
        return edges // 2
    
    def degree(self, node):
        # Retorna o grau de um nó específico
        return len(node.adj)
    
    def max_degree(self):
        # Retorna o grau máximo de um nó no grafo
        max_degree = max(len(i.adj) for i in self.nodes)
        max_node = next(i for i in self.nodes if len(i.adj) == max_degree)
        return f"Vertice: {max_node.value}, Degree: {max_degree}"
    
    def min_degree(self):
        # Retorna o grau mínimo de um nó no grafo
        min_degree = min(len(i.adj) for i in self.nodes)
        min_node = next(i for i in self.nodes if len(i.adj) == min_degree)
        return f"Vertice: {min_node.value}, Degree: {min_degree}"
    
    def adjacent_list(self):
        # Imprime a lista de adjacência do grafo
        for i in self.nodes:
            print(i)
            
# Cria um grafo e carrega os dados a partir de um arquivo
grafo = Graph()
grafo.load_from_file("example_input.txt")
print(grafo)

# Descomente as linhas abaixo para testar outras funcionalidades
# print("=====================================\n")
# print("Vertices:", grafo.vertices())
# print("=====================================\n")
# print("Edges:", grafo.edges())
# print("=====================================\n")
# print("Max degree => " + grafo.max_degree())
# print("=====================================\n")
# print("Min degree => " + grafo.min_degree())
# print("=====================================\n")
# grafo.adjacent_list()
