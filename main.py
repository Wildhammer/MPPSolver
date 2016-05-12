import matplotlib.pyplot as plt
import networkx as nx
import Queue as Q
from networkx.algorithms import floyd_warshall_predecessor_and_distance
import math
import random
from heapq import heappush
from heapq import heappop
import sys
import signal


def get_key(pose):
	return ",".join("{0}".format(n) for n in pose)

def schedule(paths,costs):
	cumulative_cost = [[sum(k[i:]) if i!=len(k) else 0 for i in xrange(len(k)+1)] for k in costs]
	# print "cumulative_cost: %s" % cumulative_cost
	q = Q.PriorityQueue()
	init_pos = tuple(0 for key in paths) 
	#The data structure for priority queue is (f,current,parent,cost)
	#parent is used to get the final path in C(EZ) if any 
	init_h = sum([i[0] for i in cumulative_cost])
	q.put((init_h,init_pos,init_pos,0))
	visited = {get_key(init_pos):(init_h,init_pos,init_pos,0)}#hashtable instead of array. use python built-in dict which is hashtable
	goal = tuple(len(key)-1 for key in paths)

	while not q.empty():
		u = q.get() 
		#print "pop: %s" % str(u)

		if u[1] == goal:
			return get_path_in_cez(visited,goal,paths)
		#all possible neighbors
		all_neighbors=[(u[3]+costs[i][u[1][i]+j-1 if j==1 else u[1][i]+j]+sum([cumulative_cost[i][u[1][i]+j] if c==i else cumulative_cost[c][u[1][c]] for c in xrange(len(costs))]),u[1][:i]+(u[1][i]+j,)+u[1][i+1:],u[1],u[3]+costs[i][u[1][i]+j-1 if j==1 else u[1][i]+j]) for i in xrange(len(paths)) for j in [-1,1] if (u[1][i]+j>=0 and u[1][i]+j<len(paths[i])) ]
		#filtering out visited neighbors
		new_neighbors=[(i,j,k,l) for (i,j,k,l) in all_neighbors if get_key(j) not in visited.keys()  ]	

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
				visited[get_key(new_neighbors[i][1])] = new_neighbors[i]

	return []

def get_path_in_cez(visited,goal,paths):
	path = [goal]
	curr = goal
	start = tuple(0 for k in range(len(goal)))
	while curr != start:
		curr = visited[get_key(curr)][2]
		# curr = [(j,k) for (i,j,k,l) in visited if j==curr][0][1]
		path.append(curr)
	path.reverse()
	return [tuple(paths[i][a[i]] for i in range(len(a))) for a in path]

def get_path_in_floyd(start,goal):
	global distance
	global predecessor
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

def ezsolver(starts,goals):
	global distance
	global predecessor
	res = [get_path_in_floyd(starts[i],goals[i]) for i in xrange(len(starts))]
	return schedule([i[0] for i in res],[i[1] for i in res])

def rand_conf(n,G,visited,max_size):
	c = [a for a in G.nodes()]
	if max_size == len(visited):
		print >> sys.stderr, 'No more samples!'
	while True:
		a = tuple(random.sample(c,n))
		if a not in visited:
			return a

def NEAR(point):
	global distance
	near = []
	for n in tree.nodes():
		heappush(near,(total_distance(tree.node[n]['pose'],point),tree.node[n]['pose']))
	return near

tree = nx.Graph()
def RRT(q_init,q_goal,conf_space):
	global distance
	global predecessor
	visited = {q_init,q_goal}
	global tree
	tree.add_node(get_key(q_init),pose=q_init,parent=None)
	__n__ = len(list(conf_space.nodes()))
	__k__ = len(q_goal)
	num_of_states = 1
	for i in range(__n__-__k__+1,__n__+1):
		num_of_states *= i
	while len(visited)<=num_of_states:
		q_rand = rand_conf(__k__,conf_space,visited,num_of_states)
		near = NEAR(q_rand)
		while True:
			nearest = heappop(near)
			path = ezsolver(nearest[1],q_rand)
			if len(path) != 0:
				tree.add_node(get_key(q_rand),pose=q_rand,parent=nearest[1])
				tree.add_edge(get_key(q_rand),get_key(nearest[1]),weight=nearest[0],path=path)
				visited |= set([q_rand])
				break
			elif len(near) == 0:
				break

	return 

def total_distance(starts,goals):
	return sum([distance[starts[i]][goals[i]] for i in xrange(len(starts))])

def get_path_before_ez(pose):
	path = []
	curr = pose
	while tree.node[get_key(curr)]['parent']!=None:
		path.append(tree.get_edge_data(get_key(curr),get_key(tree.node[get_key(curr)]['parent']))['path'])
		curr = tree.node[get_key(curr)]['parent']
	return list(reversed(path))

def BFS(start,end):
	global tree
	q = Q.PriorityQueue()
	q.put((0,start,tree.node[get_key(start)]['parent']))
	while not q.empty():
		(cost,pose,parent) = q.get();
		path = ezsolver(pose,end)
		if(len(path) is not 0):
			# print "pose: %s" % (pose,)
			# print "get_path_before_ez: %s" % get_path_before_ez(pose)
			# print "path: %s" % path
			res = get_path_before_ez(pose)
			res.append(path)
			# print "res: %s" % res
			sol = [a[i] for a in res for i in range(len(a)-1)]
			sol.append(end)
			return sol
		for a in list(tree.neighbors(get_key(pose))):
			q.put((cost+tree.get_edge_data(get_key(pose),a)['weight'],tree.node[a]['pose'],pose))
	return []

def get_solution_cost(solution):
	global tree
	global distance
	num_of_robots = len(solution[0])
	# for i in range(len(solution)-1):
	# 	for j in range(num_of_robots):
	# 		print "dis: %s" % distance[solution[i][j]][solution[i+1][j]]
	return sum([distance[solution[i][j]][solution[i+1][j]] for i in range(len(solution)-1) for j in range(num_of_robots)])
	# return 0

G=nx.Graph()
G.add_edge(1,2,weight=5)
G.add_edge(1,3,weight=2)
G.add_edge(2,3,weight=9)
G.add_edge(3,4,weight=1)
G.add_edge(2,4,weight=1)
G.add_edge(2,5,weight=4)
G.add_edge(4,5,weight=2)
home = (1,3)
destination = (2,4)
(predecessor,distance) = floyd_warshall_predecessor_and_distance(G, weight='weight')

def run_program():
	global G
	global home
	global destination
	# pos=nx.spring_layout(G,k=10/math.sqrt(G.order()),scale=2,iterations=100) # positions for all nodes
	pos={1:[1,1],2:[2,1],3:[1,2],4:[2,2],5:[3,1.5]}

	plt.figure(1)
	# nodes
	nx.draw_networkx_nodes(G,pos,node_size=700,node_color='c')

	# edges
	nx.draw_networkx_edges(G,pos,width=6)
	# nx.draw_networkx_edges(G,pos,edgelist=esmall,
	#                     width=6,alpha=0.5,edge_color='b',style='dashed')

	# labels
	nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
	labels = nx.get_edge_attributes(G,'weight')
	nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)

	plt.axis('off')
	# plt.savefig("weighted_graph.png") # save as png



	# starts = [[1,5],[2,3],[4,5]]
	# goals = [1,4]
	# print "answer: %s" % [ezsolver(key,goals) for key in starts]
	# print "total_distance: %s" % [total_distance(key,goals) for key in starts]

	RRT(home,destination,G)


def exit_gracefully(signum, frame):
    # restore the original signal handler as otherwise evil things will happen
    # in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
    signal.signal(signal.SIGINT, original_sigint)

    try:
        if raw_input("\nReally quit? (y/n)> ").lower().startswith('y'):
			sol = BFS(home,destination)
			print "Solution: %s" % sol
			print "Cost: %s" % get_solution_cost(sol)
			pos=nx.spring_layout(tree,k=50,scale=10,iterations=10) # positions for all nodes
			plt.figure(2)
			nx.draw_networkx_nodes(tree,pos,node_size=1000,node_color='c')
			nx.draw_networkx_edges(tree,pos,width=6)
			nx.draw_networkx_labels(tree,pos,font_size=20,font_family='sans-serif')
			labels = nx.get_edge_attributes(tree,'weight')
			nx.draw_networkx_edge_labels(tree,pos,edge_labels=labels)
			plt.axis('off')
			plt.show()
			sys.exit(1)
    except KeyboardInterrupt:
        print("Ok ok, quitting")
        sys.exit(1)

    # restore the exit gracefully handler here    
    signal.signal(signal.SIGINT, exit_gracefully)

if __name__ == '__main__':
    # store the original SIGINT handler
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    run_program()