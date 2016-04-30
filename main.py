#!/usr/bin/env python
"""
An example using Graph as a weighted network.
"""
# Author: Aric Hagberg (hagberg@lanl.gov)
try:
    import matplotlib.pyplot as plt
except:
    raise

import networkx as nx
import Queue as Q
from networkx.algorithms import floyd_warshall_predecessor_and_distance
import math

def schedule(paths,costs):
	q = Q.PriorityQueue()
	init_pos = tuple(0 for key in paths) 
	#The data structure for priority queue is (cost,current,parent)
	#parent is used to get the final path in C(EZ) if any 
	q.put((0,init_pos,init_pos))
	visited = [(0,init_pos,init_pos)]
	goal = tuple(len(key)-1 for key in paths)

	while not q.empty():
		u = q.get() 
		#print "pop: %s" % str(u)

		if u[1] == goal:
			return get_path(visited,goal)
		#all possible neighbors
		all_neighbors=[(u[0]+costs[i][u[1][i]+j-1 if j==1 else u[1][i]+j],u[1][:i]+(u[1][i]+j,)+u[1][i+1:],u[1]) for i in xrange(len(paths)) for j in [-1,1] if (u[1][i]+j>=0 and u[1][i]+j<len(paths[i])) ]
		#filter visited neighbors out
		new_neighbors=[(i,j,k) for (i,j,k) in all_neighbors if j not in [x[1] for x in visited] ]	

		#removing collisions(illegals)
		for i in xrange(len(new_neighbors)):
			b = True
			for j in xrange(len(new_neighbors[i][1])):
				for k in range(j+1,len(new_neighbors[i][1])):
					if paths[j][new_neighbors[i][1][j]]==paths[k][new_neighbors[i][1][k]] :
						b = False 
						j = len(new_neighbors[i][1])
						k = len(new_neighbors[i][1])
			if b:
				q.put(new_neighbors[i])	
				visited.append(new_neighbors[i])



	return []

def get_path(visited,goal):
	path = [goal]
	curr = goal
	start = tuple(0 for k in range(len(goal)))
	while curr != start:
		curr = [(j,k) for (i,j,k) in visited if j==curr][0][1]
		path.append(curr)
	path.reverse()
	return path

def ezsolver(graph,starts,goals):
    (predecessor,distance) = floyd_warshall_predecessor_and_distance(graph, weight='weight')

    return#a sequence of states should be returned

G=nx.Graph()

G.add_edge(1,2,weight=5)
G.add_edge(1,3,weight=2)
G.add_edge(2,3,weight=2)
G.add_edge(3,4,weight=1)
G.add_edge(2,4,weight=3)
G.add_edge(2,5,weight=4)
G.add_edge(4,5,weight=2)
 
(predecessor,distance) = floyd_warshall_predecessor_and_distance(G, weight='weight')

# print(predecessor[1])
# esmall=[(u,v) for (u,v,d) in G.edges(data=True) if d['weight'] <=0.5]

# pos=nx.spring_layout(G,k=10/math.sqrt(G.order()),scale=2,iterations=100) # positions for all nodes
pos={1:[1,1],2:[2,1],3:[1,2],4:[2,2],5:[3,1.5]}

paths=[[1,3,4,5],[2,3,1]]
costs=[[2,1,2],[2,2]]

print "paths: %s" % paths
print "costs: %s" % costs

print "answer: %s" % schedule(paths,costs)
# b = tuple(value[0] for key,value in pos.iteritems())
# print b


# nodes
nx.draw_networkx_nodes(G,pos,node_size=700,node_color='c')

# edges
nx.draw_networkx_edges(G,pos,
                    width=6)
# nx.draw_networkx_edges(G,pos,edgelist=esmall,
#                     width=6,alpha=0.5,edge_color='b',style='dashed')

# labels
nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
labels = nx.get_edge_attributes(G,'weight')
nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)

plt.axis('off')
# plt.savefig("weighted_graph.png") # save as png
plt.show() # display