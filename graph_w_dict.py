import psutil
import os
import time
import random
import heapq

def read_weighted_graph_from_file(file_path):
    """
    Lê um grafo com pesos de um arquivo e retorna um dicionário de listas de adjacência.

    Args:
        file_path (str): O caminho para o arquivo que contém a descrição do grafo.
    Returns:
        dict: Um dicionário onde as chaves são vértices e os valores são listas de tuplas (vértice adjacente, peso), ordenados em ordem crescente dos vértices adjacentes.
    """
    with open(file_path, 'r') as file:
        # Lê o número de vértices da primeira linha do arquivo
        num_vertices = int(file.readline().strip())
        
        # Inicializa um dicionário vazio para armazenar o grafo
        graph = dict()
        
        # Lê cada linha subsequente do arquivo para obter as arestas
        for line in file:
            u, v, w = int(line.strip().split()[0]), int(line.strip().split()[1]), float(line.strip().split()[2])
            
            # Adiciona o vértice u ao grafo se ainda não estiver presente
            if u not in graph:
                graph[u] = []
                
            # Adiciona o vértice v ao grafo se ainda não estiver presente
            if v not in graph:
                graph[v] = []
        
            # Adiciona (v, w) à lista de adjacência de u e (u, w) à lista de adjacência de v
            graph[u].append((v, w))
            graph[v].append((u, w))
            
        # Ordena as listas de adjacência para cada vértice
        for key in graph:
            graph[key] = sorted(graph[key], key=lambda x: x[0])
    
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
    

def print_out(graph, output):
    """
    Imprime informações sobre um grafo em um arquivo de saída.
    """
    text = f"Grafo\nNumero de vertices: {get_num_vertices(graph)}\nNumero de arestas: {get_num_edges(graph)}\nGrau Maximo: {get_max_degree(graph)}\nGrau Medio: {get_mean_degree(graph)}\nGrau Minimo: {get_min_degree(graph)}\nMediana dos Graus: {get_median_degree(graph)}\nLista de Adjacencia:\n{get_adjacent_list(graph)}\nMatriz de Adjacencia:\n{get_adjacent_matrix(graph)}\n"
    
    with open(output, 'w') as file:
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

def print_out_bfs_tree(graph, initial_vertice, output):
    """
    Gera um arquivo de saída contendo a árvore de busca em largura (BFS).
    O arquivo inclui o pai de cada vértice e o nível de cada vértice na árvore.

    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        initial_vertice (int): O vértice inicial a partir do qual a busca em largura será realizada.
        output (str): O caminho para o arquivo onde os resultados serão salvos.
    """
    parent, level = bfs(graph, initial_vertice)
    
    # Escreve as informações da árvore em um arquivo de saída
    with open(output, 'w') as file:
        file.write("Vértice, Pai, Nível\n")
        for vertice in sorted(parent.keys()):
            file.write(f"{vertice}, {parent[vertice]}, {level[vertice]}\n")

def dfs(graph, initial_vertice):
    """
    Realiza a busca em profundidade (DFS) em um grafo a partir de um vértice inicial.
    Gera uma árvore de busca que contém o pai de cada vértice e o nível de cada vértice na árvore.
    O vértice inicial terá nível 0.

    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        initial_vertice (int): O vértice inicial a partir do qual a busca em profundidade será realizada.

    Returns:
        dict: Um dicionário que mapeia cada vértice ao seu pai e nível na árvore gerada pela DFS.
    """
    # 1. Define uma pilha com o vertice inicial
    stack = [initial_vertice]
    visited = {}
    parent = {initial_vertice: None}
    level = {initial_vertice: 0}

    # 2. Enquanto a pilha não estiver vazia
    while stack:
        # 3. Remove o vertice que está no topo da pilha
        current_vertice = stack.pop()
        # 4. Se ele não estiver marcado
        if current_vertice not in visited:
            # 5. Marca o vertice
            visited[current_vertice] = True
            # 6. Para cada vizinho desse vertice
            for neighbor in sorted(graph[current_vertice], reverse=True):
                # 7. Adiciona o vizinho no topo da pilha
                if neighbor not in visited:
                    stack.append(neighbor)
                    parent[neighbor] = current_vertice  # Define o pai do vizinho
                    level[neighbor] = level[current_vertice] + 1  # Define o nível do vizinho

    return parent, level

def print_out_dfs_tree(graph, initial_vertice, output):
    """
    Gera um arquivo de saída contendo a árvore de busca em profundidade (DFS).
    O arquivo inclui o pai de cada vértice e o nível de cada vértice na árvore.
    output (str): O caminho para o arquivo onde os resultados serão salvos.

    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        initial_vertice (int): O vértice inicial a partir do qual a busca em largura será realizada.
    """
    parent, level = dfs(graph, initial_vertice)
    
    # Escreve as informações da árvore em um arquivo de saída
    with open(output, 'w') as file:
        file.write("Vértice, Pai, Nível\n")
        for vertice in sorted(parent.keys()):
            file.write(f"{vertice}, {parent[vertice]}, {level[vertice]}\n")

def get_distance(graph, vertice_1, vertice_2):
    """
    Calcula a distância entre dois vértices de um grafo.

    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        vertice_1: O primeiro vértice para se calcular a distância
        vertice_2: O segundo vértice para se calcular a distância
    
    Returns:
    distance: A distância encontrada entre os dois vértices
    """
    parent, level = bfs(graph, vertice_1)
    return level.get(vertice_2)

def connected_components(graph):
    """
    Encontra todas as componentes conexas de um grafo.
    
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        
    Returns:
        components: Uma lista de componentes, onde cada componente é uma lista de vértices.
              As componentes são ordenadas em ordem decrescente de tamanho.
    """
    visited = set()  # Conjunto para armazenar os vértices visitados
    components = []  # Lista para armazenar as componentes

    def aux_dfs(vertice, component):
        """Função auxiliar que realiza uma DFS para encontrar todos os vértices de uma componente."""
        # 1. Define uma pilha com o vertice inicial
        stack = [vertice]

        # 2. Enquanto a pilha não estiver vazia
        while stack:
            # 3. Remove o vertice que está no topo da pilha
            v = stack.pop()
            # 4. Se ele não estiver marcado
            if v not in visited:
                # 5. Marca o vertice
                visited.add(v)
                component.append(v)
                # Adiciona os vizinhos não visitados à pilha
                stack.extend([neighbor for neighbor in graph[v] if neighbor not in visited])

    # Para cada vértice no grafo
    for vertice in graph:
        if vertice not in visited:
            component = []  # Componente atual
            aux_dfs(vertice, component)
            components.append(component)

    # Ordena as componentes em ordem decrescente de tamanho
    components.sort(key=len, reverse=True)

    return components

def print_out_connected_components(graph, output):
    """
    Escreve as componentes conexas de um grafo em um arquivo de saída, o número de componentes e o tamanho de cada componente.
    
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de vértices adjacentes.
        output (str): O caminho para o arquivo onde os resultados serão salvos.
    """
    components = connected_components(graph)
    
    with open(output, 'w') as f:
        f.write(f"Número de componentes conexas: {len(components)}\n")
        for i, component in enumerate(components, start=1):
            f.write(f"Componente {i} (Tamanho: {len(component)}): {component}\n")

def get_diameter(graph):
    max_diameter = 0

    # 1. Pegar as componentes conexas do grafo
    components = connected_components(graph)

    # 2. Para cada componente, encontrar a maior distância mínima
    for component in components:

        # 3. Faz uma BFS de um vértice qualquer
        initial_vertice = component[0]
        parent, level = bfs(graph, initial_vertice)
        farthest_vertice = max(level, key=level.get)

        # 4. Faz uma BFS a partir do vértice mais distante daquele, garantindo que estamos numa extremidade
        parent, level = bfs(graph, farthest_vertice)
        component_diameter = max(level.values())

        # 5. Guarda a maior distâcia
        max_diameter = max(max_diameter, component_diameter)

    return max_diameter

def print_memory():
    process = psutil.Process(os.getpid())
    # Converte o uso de memória de bytes para MB
    memory_in_mb = process.memory_info().rss / (1024 * 1024)
    print(f"Uso de memória: {memory_in_mb:.2f} MB")

def measure_average_time(graph):
    total_time = 0
    vertices = list(graph.keys())  # Lista de vértices do grafo
    
    for _ in range(100):
        initial_vertice = random.choice(vertices)  # Escolhe um vértice aleatório
        start_time = time.time()  # Início da medição
        bfs(graph, initial_vertice)  # Executa a BFS
        end_time = time.time()  # Fim da medição
        
        total_time += (end_time - start_time)  # Acumula o tempo de execução
    
    average_time = total_time / 100  # Calcula o tempo médio
    return average_time

def dijkstra(graph, initial_vertice):
    """
    Realiza o algoritmo de Dijkstra para encontrar os menores caminhos a partir de um vértice inicial.
    Retorna um dicionário com os menores caminhos e as distâncias associadas.

    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de tuplas (vértice adjacente, peso).
        initial_vertice (int): O vértice inicial a partir do qual os menores caminhos serão calculados.

    Returns:
        dict: Um dicionário que mapeia cada vértice ao seu pai e à distância do vértice inicial.
        
    """
    # Verifica se existem pesos negativos
    for vertice in graph:
        for neighbor, weight in graph[vertice]:
            if weight < 0:
                raise ValueError("O grafo contém pesos negativos.")
    
    # Inicializa as estruturas de dados
    distance = {v: float('inf') for v in graph}
    parent = {v: None for v in graph}
    path = {v: [] for v in graph}
    distance[initial_vertice] = 0
    visited = set()
    
    # Enquanto houver vértices não visitados
    while visited != set(graph):
        # Escolhe o vértice não visitado com menor distância
        current_vertice = min((v for v in graph if v not in visited), key=lambda v: distance[v])
        visited.add(current_vertice)
        
        # Atualiza as distâncias dos vizinhos do vértice atual
        for neighbor, weight in graph[current_vertice]:
            if neighbor not in visited and distance[current_vertice] + weight < distance[neighbor]:
                distance[neighbor] = distance[current_vertice] + weight
                parent[neighbor] = current_vertice
                path[neighbor] = path[current_vertice] + [current_vertice]
    
    # Adiciona o vértice final ao caminho
    for vertice in path:
        path[vertice].append(vertice)
                
    return parent, distance, path

def heap_dijkstra(graph, initial_vertice):
    """
    Realiza o algoritmo de Dijkstra utilizando uma fila de prioridade (heap) para encontrar os menores caminhos a partir de um vértice inicial.
    
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de tuplas (vértice adjacente, peso).
        initial_vertice (int): O vértice inicial a partir do qual os menores caminhos serão calculados.
    """
    # Verifica se existem pesos negativos
    for vertice in graph:
        for neighbor, weight in graph[vertice]:
            if weight < 0:
                raise ValueError("O grafo contém pesos negativos.")
    
    # Inicializa as estruturas de dados
    distance = {v: float('inf') for v in graph}
    parent = {v: None for v in graph}
    path = {v: [] for v in graph}
    distance[initial_vertice] = 0
    visited = set()
    heap = [(0, initial_vertice)]
    
    # Enquanto houver vértices não visitados
    while visited != set(graph):
        # Escolhe o vértice não visitado com menor distância
        current_distance, current_vertice = heapq.heappop(heap)
        visited.add(current_vertice)
        
        # Atualiza as distâncias dos vizinhos do vértice atual
        for neighbor, weight in graph[current_vertice]:
            if neighbor not in visited and current_distance + weight < distance[neighbor]:
                distance[neighbor] = current_distance + weight
                parent[neighbor] = current_vertice
                path[neighbor] = path[current_vertice] + [current_vertice]
                heapq.heappush(heap, (distance[neighbor], neighbor))
                
    # Adiciona o vértice final ao caminho
    for vertice in path:
        path[vertice].append(vertice)
        
    return parent, distance, path

def print_out_dijkstra(graph, initial_vertice, output, heap=False):
    """
    Gera um arquivo de saída contendo os menores caminhos a partir de um vértice inicial.
    O arquivo inclui o pai de cada vértice, a distância de cada vértice ao vértice inicial e o caminho.
    
    Args:
        graph (dict): Um dicionário onde as chaves são vértices e os valores são listas de tuplas (vértice adjacente, peso).
        initial_vertice (int): O vértice inicial a partir do qual os menores caminhos serão calculados.
        output (str): O caminho para o arquivo onde os resultados serão salvos.
    """
    if heap == False:
        parent, distance, path = dijkstra(graph, initial_vertice)
    else:
        parent, distance, path = heap_dijkstra(graph, initial_vertice)
    
    with open(output, 'w') as file:
        file.write("Vértice, Pai, Distância, Caminho\n")
        for vertice in sorted(parent.keys()):
            caminho = ' -> '.join(map(str, path[vertice]))
            file.write(f"{vertice}, {parent[vertice]}, {distance[vertice]}, {caminho}; custo: {distance[vertice]}\n")
            
def compare_dijkstra_algorithms(graph, initial_vertice):
    # Executa o algoritmo de Dijkstra sem heap
    start_time = time.time()
    print_out_dijkstra(graph, initial_vertice, 'outputs/dijkstra_output.txt', heap=False)
    end_time = time.time()
    time_without_heap = end_time - start_time

    # Executa o algoritmo de Dijkstra com heap
    start_time = time.time()
    print_out_dijkstra(graph, initial_vertice, 'outputs/dijkstra_heap_output.txt', heap=True)
    end_time = time.time()
    time_with_heap = end_time - start_time

    return time_without_heap, time_with_heap

            
            

# -------------------------------------------------------------------------------------------------------------------------------------

    
    
    
# Caminho para o arquivo de entrada
#file_path = 'inputs/grafo_c_peso_0.txt'
file_path = 'inputs/grafo_W_1.txt'

# Lê o grafo do arquivo
graph = read_weighted_graph_from_file(file_path)
#print(graph)

#print_out_dijkstra(graph, 1, 'outputs/dijkstra_output.txt')
# Representando o grafo com matriz de adjacências
# get_adjacent_matrix(graph)
# print_memory()  # Medir o uso de memória
# input("Pressione Enter para continuar...") # Pausando para verificar o uso de memória

# Representando o grafo com lista de adjacências
# get_adjacent_list(graph)
# print_memory()  # Medir o uso de memória
# input("Pressione Enter para continuar...") # Pausando para verificar o uso de memória

# Imprime informações sobre o grafo
# print_out(graph, 'outputs/output.txt')

# Vértice inicial fornecido pelo usuário
initial_vertice = 1

# Mostra o resultado da busca em largura
# print(bfs(graph, initial_vertice))

# Gera a árvore de busca em largura
# print_out_bfs_tree(graph, initial_vertice, 'outputs/bfs_output.txt')

# Gera a árvore de busca em profundidade
# print_out_dfs_tree(graph, initial_vertice, 'outputs/dfs_output.txt')

# Mostra a distância entre dois vértices
# print(get_distance(graph, 5194, 2450))

# Mostra o diâmetro de um grafo
# print(get_diameter(graph))

# Mostra os componentes conexos
# print(connected_components(graph))

# Gera informações sobre componentes conexas
# print_out_connected_components(graph, 'outputs/connected_components_output.txt')

# Medindo tempo médio de execução
#print(measure_average_time(graph))


    
# Compara o desempenho do algoritmo de Dijkstra com e sem heap


print(compare_dijkstra_algorithms(graph, initial_vertice))