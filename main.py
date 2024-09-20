class Node:
    def __init__(self, value, adj = []):
        self.value = value
        self.adj = adj
        
    def add_adj(self, node):
        self.adj.append(node)
        
    def __str__(self):
        l = []
        for i in self.adj:
            l.append(i.value)
        return str(self.value) + " -> " + str(l)

class Graph:
    def __init__(self, nodes = []):
        self.nodes = nodes
        
    def add_node(self, node):
        self.nodes.append(node)
        
    def add_edge(self, node1, node2):
        node1.add_adj(node2)
        node2.add_adj(node1)
    
    def load_from_file(self, file):
        f = open(file, "r")
        for line in f:
            line = line.strip().split(" ")
            
            if len(line) != 2:
                continue
            else:
                n1 = Node(line[0])
                n2 = Node(line[1])
                self.add_node(n1)
                self.add_node(n2)
                self.add_edge(n1, n2)
        f.close()
        
    def __str__(self):
        s = ""
        for i in self.nodes:
            s += str(i) + "\n"
        return s
    
    def vertices(self):
        return len(self.nodes)
    
    def edges(self):
        edges = 0
        for i in self.nodes:
            edges += len(i.adj)
        return edges//2
    
    def degree(self, node):
        return len(node.adj)
    
    def max_degree(self):
        max = 0
        for i in self.nodes:
            if len(i.adj) > max:
                max = len(i.adj)
        return f"Vertice: {i.value}, Degree: {max}"
    
    def min_degree(self):
        min = 100000000
        for i in self.nodes:
            if len(i.adj) < min:
                min = len(i.adj)
        return f"Vertice: {i.value}, Degree: {min}"
    
grafo = Graph()
grafo.load_from_file("example_input.txt")
print(grafo)
print("=====================================\n")
print("Vertices:", grafo.vertices())
print("=====================================\n")
print("Edges:", grafo.edges())
print("=====================================\n")
print("Max degree => " + grafo.max_degree())
print("=====================================\n")
print("Min degree => " + grafo.min_degree())