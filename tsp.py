## Code dynamic programming solution for one agent 
## (1) Find test example  (2) Create graph 

V = 4
INF = 9999

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

	return dist 

def printSolution(dist):

	for i in range(V):
		for j in range(V):
			if (dist[i][j] == INF):
				print "%7s" %("INF"),
			else:
				print "%7d\t" %(dist[i][j]),
			if j == V-1:
				print ""

graph = [[0, 10, 15, 20], [10, 0, 35, 25], [15, 35, 0, 30], [20, 25, 30, 0]]
dist = floydWarshall(graph)
printSolution(dist)