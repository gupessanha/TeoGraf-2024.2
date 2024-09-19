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
    
A = Node("A")
B = Node("B")
C = Node("C")
D = Node("D")
E = Node("E")

A.add_adj(B)
A.add_adj(C)

print(A)
