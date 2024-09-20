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
    max_degree = max(graph, key=lambda k: len(graph[k]))
    
    return f"Max => V:{max_degree}, D:{len(graph[max_degree])}"
        
def get_min_degree(graph):
    min_degree = min(graph, key=lambda k: len(graph[k]))
    
    return f"Min => V:{min_degree}, D:{len(graph[min_degree])}"

def get_adjacent_list(graph):
    for key in graph:
        print(key, graph[key])
        
def get_adjacent_matrix(graph):
    keys_l = graph.keys()
    matrix = np.array(graph[i] for i in keys_l)
    print(matrix)


file_path = 'example_input.txt'
graph = read_graph_from_file(file_path)

print(graph)
print(get_adjacent_list(graph))
print("====================================")
get_adjacent_matrix(graph)
print("======================================")