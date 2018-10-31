## Python code for Floyd Warshall algorithm  ## Reference: GeeksforGeeks

import pdb

# Number of vertices in the graph 
V = 4 

# Define infinity as the large enough value. This value will be 
# used for vertices not connected to each other 
INF = 99999

# Solves all pair shortest path via Floyd Warshall Algorithm 
def floydWarshall(graph):

	""" dist[][] will be the output matrix that will finally 
		have the shortest distances between every pair of vertices """
	"""	Initializing the solution matrix same as input graph matrix 
		OR we can say that the initial values of shortest distances 
		are based on shortest paths considering no intermediate vertices
	"""

	dist = map(lambda i: map(lambda j: j, i), graph)

	for k in range(V):

		# pick all vertices as source one by one 
		for i in range(V):

			# pick all vertices as destination for above picked source 
			for j in range(V):

				dist[i][j] = min(dist[i][j], dist[i][k]+dist[k][j])

	printSolution(dist)

def printSolution(dist):

	for i in range(V):
		for j in range(V):
			if (dist[i][j] == INF):
				print "%7s" %("INF"),
			else:
				print "%7d\t" %(dist[i][j]),
			if j == V-1:
				print ""


## Directed graph - Adjacency list 
graph = [[0, 5, INF, 10], [INF, 0, 3, INF], [INF, INF, 0, 1], [INF, INF, INF, 0]]
floydWarshall(graph);