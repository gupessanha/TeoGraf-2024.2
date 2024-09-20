import numpy as np

def read_graph_from_file(file_path):
    with open(file_path, 'r') as file:
        num_vertices = int(file.readline().strip())
        
        graph = dict()
        
        for line in file:
            u, v = map(int, line.strip().split())
            
            if u not in graph:
                graph[u] = []
                
            if v not in graph:
                graph[v] = []
        
            graph[u].append(v)
            graph[v].append(u) 
            
        for key in graph:
            graph[key] = sorted(graph[key])
        
    return graph


def get_max_degree(graph):
    max_degree = 0
    for key in graph:
        max_degree = max(max_degree, len(graph[key]))
        max_key = key
    return f"Max => V:{max_key}, Degree:{max_degree}"

def get_min_degree(graph):
    min_degree = 100000000
    for key in graph:
        min_degree = min(min_degree, len(graph[key]))
        min_key = key
    return f"Min => V:{min_key}, Degree:{min_degree}"

def get_adjacent_list(graph):
    for key in graph:
        print(key, graph[key])
        
def get_adjacent_matrix(graph):
    keys_l = graph.keys()
    matrix = np.array(graph[i] for i in keys_l)
    print(matrix)


file_path = 'example_input.txt'
graph = read_graph_from_file(file_path)

print(get_max_degree(graph))
print("====================================")
print(get_min_degree(graph))
print("====================================")
print(get_adjacent_list(graph))
print("====================================")
get_adjacent_matrix(graph)