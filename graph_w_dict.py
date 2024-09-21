import numpy as np

def read_graph_from_file(file_path):
    """
    Lê um grafo de um arquivo e retorna uma representação do grafo como um dicionário.
    O arquivo deve ter o seguinte formato:
    - A primeira linha contém um número inteiro que representa o número de vértices.
    - As linhas subsequentes contêm pares de inteiros que representam arestas entre vértices.
    Args:
        file_path (str): O caminho para o arquivo que contém a descrição do grafo.
    Returns:
        dict: Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes, ordenados em ordem crescente.
    """
    with open(file_path, 'r') as file:
        # Lê o número de vértices da primeira linha do arquivo
        num_vertices = int(file.readline().strip())
        
        # Inicializa um dicionário vazio para armazenar o grafo
        graph = dict()
        
        # Lê cada linha subsequente do arquivo para obter as arestas
        for line in file:
            u, v = map(int, line.strip().split())
            
            # Adiciona o vértice u ao grafo se ainda não estiver presente
            if u not in graph:
                graph[u] = []
                
            # Adiciona o vértice v ao grafo se ainda não estiver presente
            if v not in graph:
                graph[v] = []
        
            # Adiciona v à lista de adjacência de u e vice-versa
            graph[u].append(v)
            graph[v].append(u) 
            
        # Ordena as listas de adjacência para cada vértice
        for key in graph:
            graph[key] = sorted(graph[key])
        
    return graph

def get_max_degree(graph):
    """
    Calcula o vértice com o maior grau em um grafo representado por um dicionário.
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
    Returns:
        str: Uma string formatada indicando o vértice com o maior grau e o valor do grau.
    """
    # Encontra o vértice com o maior grau
    max_degree = max(graph, key=lambda k: len(graph[k]))
    
    return f"Max => V:{max_degree}, D:{len(graph[max_degree])}"
        
def get_min_degree(graph):
    """
    Retorna o vértice com o menor grau em um grafo e o valor do grau.
    Parâmetros:
    graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
    Retorna:
    str: Uma string no formato "Min => V:{vértice}, D:{grau}" indicando o vértice com o menor grau e o valor do grau.
    """
    # Encontra o vértice com o menor grau
    min_degree = min(graph, key=lambda k: len(graph[k]))
    
    return f"Min => V:{min_degree}, D:{len(graph[min_degree])}"

def get_adjacent_list(graph):
    """
    Imprime a lista de adjacência de um grafo.

    Args:
        graph (dict): Um dicionário onde as chaves são os nós do grafo e os valores são listas de nós adjacentes.

    Exemplo:
        graph = {
            'A': ['B', 'C'],
            'B': ['A', 'D'],
            'C': ['A', 'D'],
            'D': ['B', 'C']
        }
        get_adjacent_list(graph)
        # Saída:
        # A ['B', 'C']
        # B ['A', 'D']
        # C ['A', 'D']
        # D ['B', 'C']
    """
    # Imprime cada vértice e sua lista de adjacência
    for key in graph:
        print(key, graph[key])
        
def get_adjacent_matrix(graph):
    """
    Gera e imprime a matriz de adjacência de um grafo representado por um dicionário.

    Args:
        graph (dict): Um dicionário onde as chaves são os nós do grafo e os valores são listas de nós adjacentes.

    Returns:
        None
    """
    # Obtém as chaves do grafo
    keys_l = graph.keys()
    # Cria uma matriz de adjacência usando numpy
    matrix = np.array(graph[i] for i in keys_l)
    print(matrix)

def print_out(graph):
    """
    Imprime informações sobre o grafo fornecido.

    Parâmetros:
    graph (dict): Um dicionário representando o grafo onde as chaves são os vértices
                  e os valores são listas de vértices adjacentes.

    Saída:
    - Número de vértices no grafo.
    - Número de arestas no grafo.
    - Grau máximo do grafo.
    - Grau mínimo do grafo.
    - Lista de adjacência do grafo.
    - Matriz de adjacência do grafo.
    """
    # Imprime o número de vértices
    print("Vertices: ", len(graph))
    # Imprime o número de arestas (cada aresta é contada duas vezes)
    print("Edges: ", sum(len(graph[k]) for k in graph)//2)
    # Imprime o vértice com o maior grau
    print(get_max_degree(graph))
    # Imprime o vértice com o menor grau
    print(get_min_degree(graph))
    # Imprime a lista de adjacência
    get_adjacent_list(graph)
    # Imprime a matriz de adjacência
    get_adjacent_matrix(graph)
    
# Caminho para o arquivo de entrada
file_path = 'example_input.txt'
# Lê o grafo do arquivo
graph = read_graph_from_file(file_path)
# Imprime informações sobre o grafo
print_out(graph)