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

def get_num_vertices(graph):
    """
    Retorna o número de vértices em um grafo representado por um dicionário.
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
    Returns:
        int: O número de vértices no grafo.
    """
    return len(graph)

def get_num_edges(graph):
    """
    Retorna o número de arestas em um grafo representado por um dicionário.
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
    Returns:
        int: O número de arestas no grafo.
    """
    return sum(len(graph[k]) for k in graph)//2

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
    
    return f"Max => V:{max_degree}, Grau:{len(graph[max_degree])}"

def get_mean_degree(graph):
    """
    Calcula o grau médio de um grafo representado por um dicionário.
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
    Returns:
        float: O grau médio do grafo.
    """
    # Calcula o grau médio do grafo
    mean_degree = sum(len(graph[k]) for k in graph) / len(graph)
    
    return f"Med => {mean_degree:.2f}"
        
def get_min_degree(graph):
    """
    Retorna o vértice com o menor grau em um grafo e o valor do grau.
    Parâmetros:
    graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
    Retorna:
    str: Uma string no formato "Min => V:{vértice}, Grau:{grau}" indicando o vértice com o menor grau e o valor do grau.
    """
    # Encontra o vértice com o menor grau
    min_degree = min(graph, key=lambda k: len(graph[k]))
    
    return f"Min => V:{min_degree}, Grau:{len(graph[min_degree])}"

def get_median_degree(graph):
    """
    Calcula a mediana dos graus de um grafo representado por um dicionário.
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
    Returns:
        float: A mediana dos graus do grafo.
    """
    # Calcula a mediana dos graus do grafo
    degrees = [len(graph[k]) for k in graph]
    degrees.sort()
    n = len(degrees)
    if n % 2 == 0:
        median_degree = (degrees[n//2 - 1] + degrees[n//2]) / 2
    else:
        median_degree = degrees[n//2]
    
    return f"Med => {median_degree:.2f}"

def get_adjacent_list(graph):
    """
    Retorna a lista de adjacência de um grafo representado por um dicionário.

    Args:
        graph (dict): Um dicionário onde as chaves são os nós do grafo e os valores são listas de nós adjacentes.

    Exemplo:
        graph = {
            'A': ['B', 'C'],
            'B': ['A', 'D'],
            'C': ['A', 'D'],
            'D': ['B', 'C']
        }
        adj_list = get_adjacent_list(graph)
        # Saída:
        # [('A', ['B', 'C']), ('B', ['A', 'D']), ('C', ['A', 'D']), ('D', ['B', 'C'])]
    """
    adj_list = []
    for key in graph:
        adj_list.append((key, graph[key]))
    return adj_list
        
def get_adjacent_matrix(graph):
    """
    Retorna a matriz de adjacência de um grafo representado por um dicionário.

    Args:
        graph (dict): Um dicionário onde as chaves são os nós do grafo e os valores são listas de nós adjacentes.

    Returns:
        list: Uma matriz de adjacência representada como uma lista de listas.
    """
    nodes = list(graph.keys())
    size = len(nodes)
    matrix = [[0] * size for _ in range(size)]
    
    node_index = {node: idx for idx, node in enumerate(nodes)}
    
    for node, adjacents in graph.items():
        for adjacent in adjacents:
            matrix[node_index[node]][node_index[adjacent]] = 1
    
    return matrix
    

def print_out(graph):
    """
    Imprime informações sobre um grafo em um arquivo de saída.
    """
    text = f"Grafo\nNumero de vertices: {get_num_vertices(graph)}\nNumero de arestas: {get_num_edges(graph)}\nGrau Maximo: {get_max_degree(graph)}\nGrau Medio: {get_mean_degree(graph)}\nGrau Minimo: {get_min_degree(graph)}\nMediana dos Graus: {get_median_degree(graph)}\nLista de Adjacencia:{get_adjacent_list(graph)}\nMatriz de Adjacencia:\n"
    
    with open('output.txt', 'w') as file:
        file.write(text)
        
        matrix = get_adjacent_matrix(graph)
        for row in matrix:
            file.write(' '.join(map(str, row)) + '\n')
        
        
# Caminho para o arquivo de entrada
file_path = 'example_input.txt'
# Lê o grafo do arquivo
graph = read_graph_from_file(file_path)
# Imprime informações sobre o grafo
print_out(graph)