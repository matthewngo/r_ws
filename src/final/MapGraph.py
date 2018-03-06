from PIL import Image
import math
import numpy as np
import rospy
from nav_msgs.srv import GetMap

import Utils


""" 
CONSTANTS
"""
CAR_WIDTH = 0.3 #m
CAR_LENGTH = 0.5 #m
PIXELS_TO_METERS = 0.02
MAP_CENTER_OFFSET = None


""" 
HELPER METHODS 
"""
def px(meters):
	return int(round(1.0 / PIXELS_TO_METERS * meters))

def m(pixels):
	return PIXELS_TO_METERS * pixels

def dist(p1, p2):
	return math.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2)


""" 
HYPERPARAMETERS
"""
FILTER_RADIUS = 2 #px
FILTER_DENS = 5 #px
BUFFER_RADIUS_M = (0.6 * CAR_LENGTH) #m
BUFFER_RADIUS_PX = px(BUFFER_RADIUS_M) #px
INITIAL_NODE_COVERAGE = 0.0005
RED_RADIUS_M = 0.2 + BUFFER_RADIUS_M #m
RED_RADIUS_PX = px(RED_RADIUS_M) #px

DEBUG_USE_IMAGES = True


class MapGraph:
	def __init__(self):
		print "start"

		#set variables
		self.map_msg = rospy.ServiceProxy('static_map', GetMap)().map
		self.h = self.map_msg.info.height
		self.w = self.map_msg.info.width
		self.dim = (self.h, self.w)

		# matrix has 3 values: -1 (unknown), 0 (empty), 100 (filled)
		self.original_map_mat = np.reshape(self.map_msg.data, self.dim)
		self.original_map_img = Image.new('L', self.dim)
		for i in range(self.h):
			for j in range(self.w):
				self.original_map_img.putpixel((i,j), self.original_map_mat[i, j])

		# for x in range(self.h):
		#     print self.original_map_mat[x,x]

		# matrix has 2 values: 1 (allowed), 0 (not allowed)
		self.processed_map_mat = np.ones(self.dim, dtype=np.int32)
		self.processed_map_img = Image.new('L', self.dim)

		self.nodes = {}
		self.named_nodes = {}

		# self.original_map_img.show()

		print "initialized"

	def process_map(self):
		ignore = []
		# for i in range(self.h):
		# 	for j in range(self.w):
		# 		ct = 0
		# 		for i2 in range(-FILTER_RADIUS, FILTER_RADIUS+1):
		# 			for j2 in range(-FILTER_RADIUS, FILTER_RADIUS+1):
		# 				if not self.inbounds(i+i2, j+j2):
		# 					continue
		# 				if self.original_map_mat[i+i2, j+j2] == 255: #?
		# 					ct += 1
		# 		if ct < FILTER_DENS:
		# 			ignore.append((i, j))
		# print "outliers found: " + str(len(ignore))

		for i in range(self.h):
			for j in range(self.w):
				if (i % 400 == 0) and (j == 0):
					print "  " + str(i * 100.0 / self.h) + "%"
				if (i, j) in ignore:
					continue
				if self.original_map_mat[i, j] == 100:
					for i2 in range(-BUFFER_RADIUS_PX, BUFFER_RADIUS_PX+1):
						for j2 in range(-BUFFER_RADIUS_PX, BUFFER_RADIUS_PX+1):
							if not self.inbounds(i+i2, j+j2):
								continue
							if dist((i,j), (i+i2,j+j2)) <= BUFFER_RADIUS_PX:
								self.processed_map_mat[i+i2, j+j2] = 0
				elif self.original_map_mat[i, j] == -1:
					self.processed_map_mat[i, j] = 0
		print "boundaries processed"

		for i in range(self.h):
			for j in range(self.w):
				if (i % 200 == 0) and (j == 0):
					print "  " + str(i * 100.0 / self.h) + "%"
				self.processed_map_img.putpixel((i,j), self.processed_map_mat[i,j]*50 + 50 - self.original_map_mat[i,j])
		# self.processed_map_img.show()

		print "processed map image done"

	def inbounds(self, i, j):
		return (i >= 0) and (i < self.h) and (j >= 0) and (j < self.w)

	def connected(self, p1, p2):
		pxl = Utils.pixels_between(p1, p2)
		for p in pxl:
			if self.processed_map_mat[p[0],p[1]] == 0:
				return False
		return True

	def init_nodes(self, dens=INITIAL_NODE_COVERAGE):
		allowed = np.nonzero(self.processed_map_mat)
		num = int(math.floor(dens*len(allowed[0])))
		#selected = np.random.choice(len(allowed[0]), num, replace=False)
		# print len(allowed[0])
		stride = int(math.floor(1.0 / dens))
		print "making " + str(num) + " nodes"
		# for s in selected:
		# 	p = (allowed[0][s], allowed[1][s])
		# 	n = Node(p)
		# 	self.nodes[n.id] = n
		for i in range(num):
			p = (allowed[0][i*stride], allowed[1][i*stride])
			n = Node(p)
			self.add(n)
		print "nodes initialized"

	def connectify(self, nodes=None):
		if nodes == None:
			for id1, n1 in self.nodes.iteritems():
				for id2, n2 in self.nodes.iteritems():
					if id1 <= id2:
						continue
					if self.connected(n1.pos, n2.pos):
						n1.add_edge(n2)
						n2.add_edge(n1)
		else:
			for n1 in nodes:
				id1 = n1.id
				for id2, n2 in self.nodes.iteritems():
					if id1 <= id2:
						continue
					if self.connected(n1.pos, n2.pos):
						n1.add_edge(n2)
						n2.add_edge(n1)
						
		print "connectified"

	# Only call this once
	def draw(self, show=True):
		for id1, n1 in self.nodes.iteritems():
			for id2 in n1.edges:
				if id1 <= id2:
					continue
				n2 = self.nodes[id2]
				self.processed_map_img.putpixel(n1.pos, 255)
				self.processed_map_img.putpixel(n2.pos, 255)

				px = Utils.pixels_between(n1.pos, n2.pos)
				for p in px:
					if self.processed_map_img.getpixel(p) != 255:
						self.processed_map_img.putpixel(p, 175)

		if show:
			self.processed_map_img.show()

	# Only call this once
	def draw_path(self, start, finish):
		self.draw(False)

		self.processed_map_img = self.processed_map_img.convert('RGB')

		path = self.search(start, finish)
		print path
		for i in range(len(path)-1):
			n1 = self.nodes[path[i]]
			n2 = self.nodes[path[i+1]]

			self.processed_map_img.putpixel(n1.pos, (255,0,0))
			self.processed_map_img.putpixel(n2.pos, (255,0,0))

			px = Utils.pixels_between(n1.pos, n2.pos)
			for p in px:
				self.processed_map_img.putpixel(p, (255,0,0))

		self.processed_map_img.show()

	def add(self, node):
		if node.name != None:
			self.named_nodes[node.name] = node
		self.nodes[node.id] = node

	def rem(self, node):
		if node.name != None:
			self.named_nodes.pop(node.name)

		self.nodes.pop(node.id)

		for nid in node.edges:
			self.nodes[oth].remove_edge(node)

	def search(self, start, finish):
		# Dijkstra, our old friend
		dist = {}
		dist[start.id] = 0
		prev = {}
		prev[start.id] = None
		unvisited = Utils.PriorityQueue()
		visited = {}
		visited[start.id] = True

		for nid, n in self.nodes.iteritems():
			if nid != start.id:
				dist[nid] = 99999999
				prev[nid] = None
				visited[nid] = False

			#unvisited.push(nid, dist[nid])
		unvisited.push(start.id, 0)

		while not unvisited.isEmpty():
			curr = unvisited.pop()
			visited[curr] = True
			# print curr
			if curr == finish.id:
				break
			else:
				for id2 in self.nodes[curr].edges:
					# print id2
					if visited[id2] == True:
						continue
					alt = dist[curr] + self.nodes[curr].edges[id2]
					if alt < dist[id2]:
						dist[id2] = alt
						prev[id2] = curr
						unvisited.push(id2, alt)

		ret = []
		c = finish.id
		while prev[c] != None:
			ret.append(c)
			c = prev[c]
		return ret



"""        
	def add_blue_and_red(self, filename):
		expand the not allowed areas by red_radius around red points
		remove any nodes within that radius
		check all nodes for connectedness under new allowed area

		add blue nodes, named B0, B1, B2...
		for each blue node, connect to all applicable nodes

	def write_to_file(self, filename):
		write map and graph to file

	def read_from_file(self, filename):
		read map and graph from file

"""

class Node:
	counter = 0

	def __init__(self, pos, name=None):
		self.pos = pos     # (x,y) tuple
		self.name = name   # string
		self.edges = {}    # dict of (other id, dist)
		self.id = Node.counter  # unique int
		Node.counter += 1

	def add_edge(self, other):
		self.edges[other.id] = dist(self.pos, other.pos)

	def remove_edge(self, other):
		self.edges.pop(other.id)


g = MapGraph()
g.process_map()
g.init_nodes()
g.connectify()
#g.draw()

b0 = Node((775,775), "B0")
b1 = Node((1000,1000), "B1")
g.add(b0)
g.add(b1)
g.connectify([b0, b1])

# path = g.search(b0, b1)
# for n in path:
# 	print g.nodes[n].pos

# path = g.search(g.nodes[0], g.nodes[230])
# print path
# for n in path:
#  	print g.nodes[n].pos

# g.draw()

print g.search(g.nodes[0], g.nodes[100])
print g.search(g.nodes[2], g.nodes[150])
print g.search(g.nodes[6], g.nodes[129])

g.draw_path(g.nodes[2], g.nodes[150])