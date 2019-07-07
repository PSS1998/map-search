from collections import defaultdict 

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
  
    def DLS(self,src,target,maxDepth,path,count,visited): 
        count.add(src)
  
        if src == target :
            return True
  
        if maxDepth <= 0 : return False
  
        for i in self.graph[src]: 
                if(self.DLS(i,target,maxDepth-1,path,count,visited)): 
                    path.append(i)
                    return True
        return False
  
    def IDDFS(self,src, target, maxDepth, edges, nodes, visited): 
        count = set()
        path = []
        for i in range(maxDepth):
            if (self.DLS(src, target, i, path, count, visited)):
                path.append(src)
                path = list(reversed(path))
                print("visited:", len(count))
                print("path:", len(path))
                cost = get_travel_distance(path, edges, nodes)
                print("distance:", cost)
                print(', '.join(path))
                return True
        return False

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
  

def id(start, end):
    edges = read_edges()
    nodes = read_nodes()

    visited = {}

    g = Graph () 
    for i in range(len(edges)): 
        g.addEdge(edges[i][0], edges[i][1])
        g.addEdge(edges[i][1], edges[i][0])
        visited[edges[i][0]] = False
        visited[edges[i][1]] = False
      
    target = end; maxDepth = 200; src = start
      
    g.IDDFS(src, target, maxDepth, edges, nodes, visited)
      


id('156241214', '5926559296')