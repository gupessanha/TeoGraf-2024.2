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
    return '\n'.join(f"{key}: {', '.join(map(str, adjacents))}" for key, adjacents in adj_list)
        
def get_adjacent_matrix(graph):
    """
    Retorna a matriz de adjacência de um grafo representado por um dicionário, incluindo uma linha e coluna extra com os nomes dos vértices.

    Args:
        graph (dict): Um dicionário onde as chaves são os nós do grafo e os valores são listas de nós adjacentes.

    Returns:
        str: Uma string representando a matriz de adjacência com os nomes dos vértices.
    """
    nodes = list(graph.keys())
    size = len(nodes)
    matrix = [[0] * size for _ in range(size)]
    
    node_index = {node: idx for idx, node in enumerate(nodes)}
    
    for node, adjacents in graph.items():
        for adjacent in adjacents:
            matrix[node_index[node]][node_index[adjacent]] = 1
    
    # Adiciona a linha e coluna extra com os nomes dos vértices
    header = '  ' + ' '.join(map(str, nodes)) + '\n'
    matrix_str = header
    for i, node in enumerate(nodes):
        matrix_str += str(node) + ' ' + ' '.join(map(str, matrix[i])) + '\n'
    
    return matrix_str
    

def print_out(graph):
    """
    Imprime informações sobre um grafo em um arquivo de saída.
    """
    text = f"Grafo\nNumero de vertices: {get_num_vertices(graph)}\nNumero de arestas: {get_num_edges(graph)}\nGrau Maximo: {get_max_degree(graph)}\nGrau Medio: {get_mean_degree(graph)}\nGrau Minimo: {get_min_degree(graph)}\nMediana dos Graus: {get_median_degree(graph)}\nLista de Adjacencia:\n{get_adjacent_list(graph)}\nMatriz de Adjacencia:\n{get_adjacent_matrix(graph)}\n"
    
    with open('output.txt', 'w') as file:
        file.write(text)

def bfs(graph, initial_vertice):
    """
    Realiza a busca em largura (BFS) em um grafo a partir de um vértice inicial.
    Gera uma árvore de busca que contém o pai de cada vértice e o nível de cada vértice na árvore.
    O vértice inicial terá nível 0.

    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        initial_vertice (int): O vértice inicial a partir do qual a busca em largura será realizada.

    Returns:
        dict: Um dicionário que mapeia cada vértice ao seu pai e nível na árvore gerada pela BFS.
    """

    # 1. Define um fila vazia
    queue = []

    # 2. Marca o vertice inicial e o insere na fila
    queue.append(initial_vertice)
    visited = {initial_vertice: True}
    parent = {initial_vertice: None}
    level = {initial_vertice: 0}

    # 3. Enquanto a fila não estiver vazia
    while queue:
        # 4. Retira o vertice do inicio da fila
        current_vertice = queue.pop(0)
        # 5. Para todo vizinho do vertice, faz
        for neighbor in graph[current_vertice]:
            # 6. Se o vizinho não estiver marcado, marca o vizinho e o insere no fim da fila
            if neighbor not in visited:
                visited[neighbor] = True
                parent[neighbor] = current_vertice  # Define o pai do vértice
                level[neighbor] = level[current_vertice] + 1  # Calcula o nível do vizinho
                queue.append(neighbor)

    return parent, level

def bfs_tree_output(graph, initial_vertice):
    """
    Gera um arquivo de saída contendo a árvore de busca em largura (BFS).
    O arquivo inclui o pai de cada vértice e o nível de cada vértice na árvore.

    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        initial_vertice (int): O vértice inicial a partir do qual a busca em largura será realizada.
    """
    parent, level = bfs(graph, initial_vertice)
    
    # Escreve as informações da árvore em um arquivo de saída
    with open('bfs_output.txt', 'w') as file:
        file.write("Vértice, Pai, Nível\n")
        for vertice in sorted(parent.keys()):
            file.write(f"{vertice}, {parent[vertice]}, {level[vertice]}\n")

# def dfs(graph, initial_vertice):
    # 1. Marca o vertice inicial

    # 2. Para cada aresta incidente ao vertice inicial

        # 3. Se o vizinho não estiver marcado, faz a chamada recursiva
    

# Caminho para o arquivo de entrada
file_path = 'grafo_1.txt'
# Lê o grafo do arquivo
graph = read_graph_from_file(file_path)
# Imprime informações sobre o grafo
print_out(graph)
# Vértice inicial fornecido pelo usuário
initial_vertice = 1
# Gera a árvore de busca em largura
bfs_tree_output(graph, initial_vertice)