from collections import defaultdict 
  
def backtrace(parent, start, end):
    path = [end]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

from math import radians, cos, sin, asin, sqrt

def haversine(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371
    return c * r

def distance(start, end, edges, nodes):
	return haversine( float(nodes[start][1]), float(nodes[start][0]), float(nodes[end][1]), float(nodes[end][0]) )

def get_travel_distance(path, edges, nodes):
	cost = 0
	for i in range(len(path)-1):
		cost += distance(path[i], path[i+1], edges, nodes)
	return cost

class Graph: 
  
    def __init__(self): 
        self.graph = defaultdict(list) 
  
    def addEdge(self,u,v): 
        self.graph[u].append(v) 
  
    def BFS_func(self, s, end, visited, edges, nodes): 
        start = s
        queue = [] 
        parent = {}
        count = 0
  
        queue.append(s) 
        visited[s] = True
  
        while queue: 
            if s == end:
                path = backtrace(parent, start, end)
                print("visited:", count)
                print("path:", len(path))
                cost = get_travel_distance(path, edges, nodes)
                print("distance:", cost)
                print(', '.join(path))
                break

            s = queue.pop(0) 
            count += 1
             
            for i in self.graph[s]: 
                if visited[i] == False: 
                    parent[i] = s
                    queue.append(i) 
                    visited[i] = True
					
				
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



def bfs(start, end):
	edges = read_edges()
	nodes = read_nodes()
	  
	visited = {}

	g = Graph()
	for i in range(len(edges)): 
		g.addEdge(edges[i][0], edges[i][1])
		g.addEdge(edges[i][1], edges[i][0])
		visited[edges[i][0]] = False
		visited[edges[i][1]] = False

	g.BFS_func(start, end, visited, edges, nodes)
	  
bfs('25920714', '25922522')