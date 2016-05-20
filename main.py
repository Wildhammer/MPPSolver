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

#Just creates keys for configurations by concatenation
def get_key(pose):
	return ",".join("{0}".format(n) for n in pose)

#The infrastructure of EZ-Solver is this. It searches through C_EZ
#using A-star search algorithm. Note that it doesn't construct C_EZ at once,
#because the memory it needs could be ridiculously large!
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
		all_neighbors=[(u[3]+costs[i][u[1][i]+j-1 if j==1 else u[1][i]+j]+sum([cumulative_cost[i][u[1][i]+j] if c==i else cumulative_cost[c][u[1][c]] for c in range(len(costs))]),u[1][:i]+(u[1][i]+j,)+u[1][i+1:],u[1],u[3]+costs[i][u[1][i]+j-1 if j==1 else u[1][i]+j]) for i in range(len(paths)) for j in [-1,1] if (u[1][i]+j>=0 and u[1][i]+j<len(paths[i])) ]
		#filtering out visited neighbors
		new_neighbors=[(i,j,k,l) for (i,j,k,l) in all_neighbors if get_key(j) not in visited.keys()  ]	

		#removing collisions(illegals)
		for i in range(len(new_neighbors)):
			b = True
			for j in range(len(new_neighbors[i][1])):
				for k in range(j+1,len(new_neighbors[i][1])):
					if paths[j][new_neighbors[i][1][j]]==paths[k][new_neighbors[i][1][k]] :
						b = False 
						break
						# j = len(new_neighbors[i][1])
						# k = len(new_neighbors[i][1])
				if b is False:
					break	
			if b:
				q.put(new_neighbors[i])	
				visited[get_key(new_neighbors[i][1])] = new_neighbors[i]

	return []

#Once the goal configuration is found in C_EZ, it iteratively builds up
#the path using each node's parent
def get_path_in_cez(visited,goal,paths):
	path = [goal]
	curr = goal
	start = tuple(0 for k in range(len(goal)))
	while curr != start:
		curr = visited[get_key(curr)][2]
		path.append(curr)
	path.reverse()
	return [tuple(paths[i][a[i]] for i in range(len(a))) for a in path]

#To retrieve the optimal path and the cost from floyd's solutions
def get_path_in_floyd(start,goal):
	global distance, predecessor
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

#EZ-Solver function which passes optimal paths and their costs to schedule function
def ezsolver(starts,goals):
	global distance, predecessor
	res = [get_path_in_floyd(starts[i],goals[i]) for i in xrange(len(starts))]
	return schedule([i[0] for i in res],[i[1] for i in res])

#Generate a random configuration in joint-configuration space
def rand_conf(n,G,visited,max_size):
	c = [a for a in G.nodes()]
	if max_size == len(visited):
		print >> sys.stderr, 'No more samples!'
	while True:
		a = tuple(random.sample(c,n))
		if a not in visited:
			return a

#This would return a priority queue on distance 
def NEAR(point):
	global distance
	near = []
	for n in tree.nodes():
		heappush(near,(total_distance(tree.node[n]['pose'],point),tree.node[n]['pose']))
	return near

#This function returns nearest nodes considering the cost from root
def NEAR_OPTIMAL(point):
	global distance
	all_near = NEAR(point)
	temp = list(all_near)
	near = []
	while len(all_near)>0 and all_near[0][0]<radius:
		next = heappop(all_near)
		heappush(near,(next[0]+tree.node[get_key(next[1])]['cost'],next[1]))
	return [near,temp]

def RRT_star(q_init,q_goal,conf_space):
	global distance, predecessor, tree, radius
	visited = {q_init,q_goal}
	tree.add_node(get_key(q_init),pose=q_init,parent=None,cost=0)
	__n__ = len(list(conf_space.nodes()))
	__k__ = len(q_goal)
	num_of_states = 1
	for i in range(__n__-__k__+1,__n__+1):
		num_of_states *= i
	while len(visited)<=num_of_states:
		radius = max(int(((gamma/zeta)*(math.log(len(visited))/len(visited)))**(1.0/dimension)),eta)
		# print "radius: %s" % radius
		q_rand = rand_conf(__k__,conf_space,visited,num_of_states)
		all_near = NEAR_OPTIMAL(q_rand)
		while len(all_near[0])>0:
			nearest = heappop(all_near[0])
			path = ezsolver(nearest[1],q_rand)
			if len(path) != 0:
				# print "sample: %s" % (q_rand,)
				# print "nearest: %s" % (nearest,)

				tree.add_node(get_key(q_rand),pose=q_rand,parent=nearest[1],cost=nearest[0])
				tree.add_edge(get_key(q_rand),get_key(nearest[1]),weight=total_distance(nearest[1],q_rand),path=path)
				visited |= set([q_rand]) 
				# near = []
				# while len(temp)>0 and temp[0][0]<radius:
				# 	near.append(heappop(temp))

				# print "optimal_near: %s" % all_near[0]			

				# pos=nx.spring_layout(tree,k=50,scale=10,iterations=10) # positions for all nodes
				# plt.figure(2)
				# nx.draw_networkx_nodes(tree,pos,node_size=1000,node_color='c')
				# nx.draw_networkx_edges(tree,pos,width=6)
				# nx.draw_networkx_labels(tree,pos,font_size=20,font_family='sans-serif')
				# edge_labels = nx.get_edge_attributes(tree,'weight')
				# nx.draw_networkx_edge_labels(tree,pos,edge_labels=edge_labels)
				# node_labels = nx.get_node_attributes(tree,'cost')
				# nx.draw(tree,pos,labels=node_labels)
				# plt.axis('off')
				# plt.show()
							
				#This part stands for rewiring the tree
				for n in all_near[1]:
					if n[0] > radius:
						break
					if n[1] != nearest:
						path = ezsolver(n[1],q_rand)	
						if len(path) != 0:
							costp = n[0]+tree.node[get_key(q_rand)]['cost']
							if costp<tree.node[get_key(n[1])]['cost']:	
								# print "node: %s" % tree.node[get_key(n[1])]
								tree.remove_edge(get_key(n[1]),get_key(tree.node[get_key(n[1])]['parent']))
								tree.add_edge(get_key(n[1]),get_key(q_rand),weight=n[0],path=path)
								tree.node[get_key(n[1])]['cost'] = costp
								tree.node[get_key(n[1])]['parent'] = q_rand

				break





#RRT never gets out unless interrupted by Ctrl+c. It then prints the best possible answer so far
tree = nx.Graph()
def RRT(q_init,q_goal,conf_space):
	global distance, predecessor, tree
	visited = {q_init,q_goal}
	tree.add_node(get_key(q_init),pose=q_init,parent=None)
	__n__ = len(list(conf_space.nodes()))#P(n,k)
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

#Distance between two configurations in joint-configuration space
def total_distance(starts,goals):
	return sum([distance[starts[i]][goals[i]] for i in xrange(len(starts))])

#From some point on EZ-Solver can solve the problem so we are going to need 
#the solution from start until this point
def get_path_before_ez(pose):
	path = []
	curr = pose
	while tree.node[get_key(curr)]['parent']!=None:
		path.append(tree.get_edge_data(get_key(curr),get_key(tree.node[get_key(curr)]['parent']))['path'])
		curr = tree.node[get_key(curr)]['parent']
	return list(reversed(path))

#BFS is used to find the path from start to end configurations in RRT result
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

#This computes the solution cost
def get_solution_cost(solution):
	global tree, distance
	num_of_robots = len(solution[0])
	return sum([distance[solution[i][j]][solution[i+1][j]] for i in range(len(solution)-1) for j in range(num_of_robots)])

G=nx.Graph()
# ############Example 1##############
# G.add_edge(1,2,weight=5)
# G.add_edge(1,3,weight=2)
# G.add_edge(2,3,weight=9)
# G.add_edge(3,4,weight=1)
# G.add_edge(2,4,weight=1)
# G.add_edge(2,5,weight=4)
# G.add_edge(4,5,weight=2)
# home = (1,3)
# destination = (2,4)
# pos={1:[1,1],2:[2,1],3:[1,2],4:[2,2],5:[3,1.5]}
###################################
############Example 2##############
G.add_edge(1,2,weight=1)
G.add_edge(1,3,weight=1)
G.add_edge(3,2,weight=1)
G.add_edge(3,4,weight=1)
G.add_edge(4,5,weight=1)
G.add_edge(5,6,weight=1)
G.add_edge(6,7,weight=1)
G.add_edge(7,8,weight=1)
G.add_edge(8,9,weight=1)
G.add_edge(9,10,weight=1)
G.add_edge(9,11,weight=1)
G.add_edge(10,11,weight=1)
home = (5,6,7)
destination = (10,2,4)
pos = nx.spring_layout(G)
###################################

zeta = 1.0 
gamma = 1000.0
eta = 10
dimension = len(home)
radius = 10 
(predecessor,distance) = floyd_warshall_predecessor_and_distance(G, weight='weight')

def run_program():
	global G,home,destination
	


	plt.figure(1)
	# nodes
	nx.draw_networkx_nodes(G,pos,node_size=700,node_color='c')

	# edges
	nx.draw_networkx_edges(G,pos,width=6)

	# labels
	nx.draw_networkx_labels(G,pos,font_size=20,font_family='sans-serif')
	labels = nx.get_edge_attributes(G,'weight')
	nx.draw_networkx_edge_labels(G,pos,edge_labels=labels)

	plt.axis('off')

	# starts = [[1,5],[2,3],[4,5]]
	# goals = [1,4]
	# print "answer: %s" % ezsolver(home,destination)

	# RRT(home,destination,G)
	RRT_star(home,destination,G)


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