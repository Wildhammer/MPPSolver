try:
    import matplotlib.pyplot as plt
except:
    raise

import networkx as nx
import Queue as Q
from networkx.algorithms import floyd_warshall_predecessor_and_distance
import math

def schedule(paths,costs):
	cumulative_cost = [[sum(k[i:]) if i!=len(k) else 0 for i in xrange(len(k)+1)] for k in costs]
	# print "cumulative_cost: %s" % cumulative_cost
	q = Q.PriorityQueue()
	init_pos = tuple(0 for key in paths) 
	#The data structure for priority queue is (f,current,parent,cost)
	#parent is used to get the final path in C(EZ) if any 
	init_h = sum([i[0] for i in cumulative_cost])
	q.put((init_h,init_pos,init_pos,0))
	visited = [(init_h,init_pos,init_pos,0)]
	goal = tuple(len(key)-1 for key in paths)

	while not q.empty():
		u = q.get() 
		#print "pop: %s" % str(u)

		if u[1] == goal:
			return get_path_in_cez(visited,goal)
		#all possible neighbors
		all_neighbors=[(u[3]+costs[i][u[1][i]+j-1 if j==1 else u[1][i]+j]+sum([cumulative_cost[i][u[1][i]+j] if c==i else cumulative_cost[c][u[1][c]] for c in xrange(len(costs))]),u[1][:i]+(u[1][i]+j,)+u[1][i+1:],u[1],u[3]+costs[i][u[1][i]+j-1 if j==1 else u[1][i]+j]) for i in xrange(len(paths)) for j in [-1,1] if (u[1][i]+j>=0 and u[1][i]+j<len(paths[i])) ]
		#filtering out visited neighbors
		new_neighbors=[(i,j,k,l) for (i,j,k,l) in all_neighbors if j not in [x[1] for x in visited] ]	

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

def get_path_in_cez(visited,goal):
	path = [goal]
	curr = goal
	start = tuple(0 for k in range(len(goal)))
	while curr != start:
		curr = [(j,k) for (i,j,k,l) in visited if j==curr][0][1]
		path.append(curr)
	path.reverse()
	return path

def get_path_in_floyd(start,goal,predecessor,distance):
	path = [goal]
	cost = []
	curr = goal
	while curr != start:
		temp = predecessor[start][curr]
		path.append(temp)
		cost.append(distance[curr][temp])			
		curr = temp
	path.reverse()
	cost.reverse()
	return (path,cost)

def ezsolver(starts,goals,predecessor,distance):
	res = [get_path_in_floyd(starts[i],goals[i],predecessor,distance) for i in xrange(len(starts))]
	# print("res: ",res)
	return schedule([i[0] for i in res],[i[1] for i in res])
	 


G=nx.Graph()

G.add_edge(1,2,weight=5)
G.add_edge(1,3,weight=2)
G.add_edge(2,3,weight=9)
G.add_edge(3,4,weight=1)
G.add_edge(2,4,weight=1)
G.add_edge(2,5,weight=4)
G.add_edge(4,5,weight=2)

# pos=nx.spring_layout(G,k=10/math.sqrt(G.order()),scale=2,iterations=100) # positions for all nodes
pos={1:[1,1],2:[2,1],3:[1,2],4:[2,2],5:[3,1.5]}

(predecessor,distance) = floyd_warshall_predecessor_and_distance(G, weight='weight')
print "answer: %s" % ezsolver([1,5],[5,3],predecessor,distance)



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