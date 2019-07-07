from collections import defaultdict
from queue import PriorityQueue

class Graph:
    def __init__(self):
        self.graph = defaultdict(list) 
        self.weights = defaultdict(list) 

    def addEdge(self,u,v,w): 
        self.graph[u].append(v)
        self.weights[u].append(w)

    def getDist(self,u,v):
        for i in range(len(self.graph[u])):
            if(self.graph[u][i] == v):
                return self.weights[u][i]

    def neighbors(self, node):
        return self.graph[node]

    def get_cost(self, from_node, to_node):
    	for i in range(len(self.graph[from_node])):
    		if(self.graph[from_node][i] == to_node):
    			return self.weights[from_node][i]


from math import radians, cos, sin, asin, sqrt

def haversine(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371
    return c * r


def read_nodes():
	nodes = defaultdict(list)
	with open("nodes.txt", "r") as file:
		node = file.readline()
		while node:
			node = node.split()
			nodes[node[0]].append(node[1])
			nodes[node[0]].append(node[2])
			node = file.readline()
	file.close()
	return nodes
	
def read_edges():
	edges = []
	with open("edges.txt", "r") as file:
		edge = file.readline()
		while edge:
			edge = edge.split()
			edges.append(edge)
			edge = file.readline()
	file.close()
	return edges

def findItem(theList, item):
   return [ind for ind in range(len(theList)) if item == theList[ind][0]]

def backtrace(parent, parentDist, start, end):
    cost = 0
    path = [end]
    while path[-1] != start:
        cost += parentDist[path[-1]]
        path.append(parent[path[-1]])
    path.reverse()
    return path, cost

def ucs(start, goal):

    edges = read_edges()
    nodes = read_nodes()

    g = Graph()

    for i in range(len(edges)): 
        dist = haversine( float(nodes[edges[i][0]][1]), float(nodes[edges[i][0]][0]), float(nodes[edges[i][1]][1]), float(nodes[edges[i][1]][0]) )
        g.addEdge(edges[i][0], edges[i][1], dist)
        g.addEdge(edges[i][1], edges[i][0], dist)

    count = 0
    parent = {}
    parentDist = {}
    visited = set()
    queue = PriorityQueue()
    queue.put((0, start))
    pre_node = start
    node = start

    while queue:
        pre_node = node
        cost, node = queue.get()
        if node not in visited:
	        visited.add(node)

	        count += 1

	        if node == goal:
	            path, cost = backtrace(parent, parentDist, start, goal)
	            print("visited:", count)
	            print("path:", len(path))
	            print("distance:", cost)
	            print(', '.join(path))
	            return
	        for i in g.neighbors(node):
	            if i not in visited:
	                total_cost = cost + g.get_cost(node, i)
	                if(parentDist.get(i) == None or parentDist.get(i) > total_cost ):
		                queue.put((total_cost, i))
		                parent[i] = node
		                parentDist[i] = g.getDist(node, i)


ucs('156241214', '5926559296')