from collections import defaultdict
from queue import PriorityQueue
 
class AStarGraph(object):

	def __init__(self):
		self.graph = defaultdict(list) 
		self.weights = defaultdict(list)
		self.dist = defaultdict(list)
 
	def addEdge(self,u,v,w,d): 
		self.graph[u].append(v)
		self.weights[u].append(w)
		self.dist[u].append(d)

	def getDist(self,u,v):
		for i in range(len(self.graph[u])):
			if(self.graph[u][i] == v):
				return self.dist[u][i]

	def heuristic(self, start, goal, nodes):
		dist = haversine( float(nodes[start][1]), float(nodes[start][0]), float(nodes[goal][1]), float(nodes[goal][0]) ) / 80
		return dist
 
	def get_vertex_neighbours(self, pos):
		n = []
		for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
			x2 = pos[0] + dx
			y2 = pos[1] + dy
			if x2 < 0 or x2 > 7 or y2 < 0 or y2 > 7:
				continue
			n.append((x2, y2))
		return n
 
	def move_cost(self, from_node, to_node):
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

def findItem(theList, item):
   return [ind for ind in range(len(theList)) if item == theList[ind][0]]

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

 
def AStarSearch(start, end):

	edges = read_edges()
	nodes = read_nodes()
	graph = AStarGraph()
	for i in range(len(edges)): 
		dist = haversine( float(nodes[edges[i][0]][1]), float(nodes[edges[i][0]][0]), float(nodes[edges[i][1]][1]), float(nodes[edges[i][1]][0]) )
		max_speed = edges[i][2]
		time = dist / int(max_speed)
		if(edges[i][3]):
			graph.addEdge(edges[i][0], edges[i][1], time, dist)
		else:
			graph.addEdge(edges[i][0], edges[i][1], time, dist)
			graph.addEdge(edges[i][1], edges[i][0], time, dist)

	count = 0
 
	G = {}
	F = {}
 
	G[start] = 0 
	F[start] = graph.heuristic(start, end, nodes)
 
	closedVertices = set()
	openVertices = set([start])
	cameFrom = {}
	cameFromDist = {}
	cost = 0

	cameFromDist[start] = '0'
 
	while len(openVertices) > 0:
		current = None
		currentFscore = None
		for pos in openVertices:
			if current is None or F[pos] < currentFscore:
				currentFscore = F[pos]
				current = pos
		count += 1

		if current == end:
			path = [current]
			cost = float(cameFromDist[current])
			while current in cameFrom:
				current = cameFrom[current]
				cost += float(cameFromDist[current])
				path.append(current)
			path.reverse()
			print("visited:", count)
			print("path:", len(path))
			print("distance:", cost)
			print(', '.join(path))
			return path, F[end]
 
		openVertices.remove(current)
		closedVertices.add(current)

 
		for neighbour in graph.graph[current]:
			if neighbour in closedVertices: 
				continue
			candidateG = G[current] + graph.move_cost(current, neighbour)
 
			if neighbour not in openVertices:
				openVertices.add(neighbour)
			elif candidateG >= G[neighbour]:
				continue
 
			cameFrom[neighbour] = current
			cameFromDist[neighbour] = graph.getDist(current, neighbour)
			G[neighbour] = candidateG
			H = graph.heuristic(neighbour, end, nodes)
			F[neighbour] = G[neighbour] + H
 
 

AStarSearch('25920714', '25922522')

