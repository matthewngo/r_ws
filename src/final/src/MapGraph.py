from PIL import Image
import math
import numpy as np
import rospy
from nav_msgs.srv import GetMap
import itertools
import Utils
import sys
import csv


""" 
CONSTANTS
"""
CAR_WIDTH = 0.3 #m
CAR_LENGTH = 0.5 #m
PIXELS_TO_METERS = 0.02
MAP_CENTER_OFFSET = None
BLUE_NODE_NAMES = ["B0", "B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9"]


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
FILTER_RADIUS = 5 #px
FILTER_DENS = 5 #px
BUFFER_RADIUS_M = (0.7 * CAR_LENGTH) #m
BUFFER_RADIUS_PX = px(BUFFER_RADIUS_M) #px
INITIAL_NODE_COVERAGE = 0.0015
RED_RADIUS_M = 0.3 + BUFFER_RADIUS_M #m
RED_RADIUS_PX = px(RED_RADIUS_M) #px
INFILL_RADIUS = 50 #px

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
		# self.original_map_img = Image.new('L', self.dim)
		# for i in range(self.h):
		# 	for j in range(self.w):
		# 		self.original_map_img.putpixel((i,j), self.original_map_mat[i, j])

		# matrix has 2 values: 1 (allowed), 0 (not allowed)
		self.processed_map_mat = np.ones(self.dim, dtype=np.int32)
		self.processed_map_img = Image.new('L', (self.dim[1],self.dim[0]))

		self.nodes = {}
		self.named_nodes = {}

		self.plan = []

		# self.original_map_img.show()

		print "initialized " + str(self.dim)

	#red_pixels is a list of (x,y) tuples for no-go locations
	def process_map(self, red_pixels):
		ignore = []
		for i in range(self.h):
			for j in range(self.w):
				if (i % 400 == 0) and (j == 0):
					print "  " + str(i * 100.0 / self.h) + "%"
				ct = 0
				if self.original_map_mat[i, j] != 100:
					continue

				for i2 in range(-FILTER_RADIUS, FILTER_RADIUS+1):
					if ct >= FILTER_DENS:
						break
					for j2 in range(-FILTER_RADIUS, FILTER_RADIUS+1):
						if ct >= FILTER_DENS:
							break
						if not self.inbounds(i+i2, j+j2):
							continue
						if self.original_map_mat[i+i2, j+j2] == 100:
							ct += 1

				if ct < FILTER_DENS:
					ignore.append((i, j))
		print "outliers found: " + str(len(ignore))

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

		for rx1,ry1 in red_pixels:
			rx,ry = self.reverse_imgpx((rx1,ry1))
			for i2 in range(-BUFFER_RADIUS_PX, BUFFER_RADIUS_PX+1):
				for j2 in range(-BUFFER_RADIUS_PX, BUFFER_RADIUS_PX+1):
					if not self.inbounds(rx+i2, ry+j2):
						continue
					if dist((rx,ry), (rx+i2,ry+j2)) <= BUFFER_RADIUS_PX:
						self.processed_map_mat[rx+i2, ry+j2] = 0
		print "red points processed: " + str(len(red_pixels))

		for i in range(self.h):
			for j in range(self.w):
				if (i % 200 == 0) and (j == 0):
					print "  " + str(i * 100.0 / self.h) + "%"
				self.setpx((i,j), self.processed_map_mat[i,j]*50 + 50 - self.original_map_mat[i,j])
				# self.processed_map_img.putpixel((i,j), self.processed_map_mat[i,j]*50 + 50 - self.original_map_mat[i,j])
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

	def imgpx(self, pos):
		return (pos[1], self.h-pos[0]-1)

	def reverse_imgpx(self, pos):
		return self.imgpx(self.imgpx(self.imgpx(pos)))

	def setpx(self, pos, val):
		self.processed_map_img.putpixel(self.imgpx(pos), val)

	# Only call this once
	def draw(self, show=True):
		for id1, n1 in self.nodes.iteritems():
			for id2 in n1.edges:
				if id1 <= id2:
					continue
				n2 = self.nodes[id2]
				self.setpx(n1.pos, 255)
				self.setpx(n2.pos, 255)
				# self.processed_map_img.putpixel(n1.pos, 255)
				# self.processed_map_img.putpixel(n2.pos, 255)

				px = Utils.pixels_between(n1.pos, n2.pos)
				for p in px:
					if self.processed_map_img.getpixel(self.imgpx(p)) != 255:
						# self.processed_map_img.putpixel(p, 175)
						self.setpx(p, 175)

		if show:
			self.processed_map_img.show()

	# Only call this once
	def draw_path(self, start, finish):
		self.draw(False)

		self.processed_map_img = self.processed_map_img.convert('RGB')

		path = self.search(start, finish)
		# print path
		# for node in path:
		# 	print self.nodes[node].pos
		for i in range(len(path)-1):
			n1 = self.nodes[path[i]]
			n2 = self.nodes[path[i+1]]

			self.setpx(n1.pos, (255,0,0))
			self.setpx(n2.pos, (255,0,0))
			# self.processed_map_img.putpixel(n1.pos, (255,0,0))
			# self.processed_map_img.putpixel(n2.pos, (255,0,0))

			px = Utils.pixels_between(n1.pos, n2.pos)
			for p in px:
				self.setpx(p, (255,0,0))
				# self.processed_map_img.putpixel(p, (255,0,0))

		# self.processed_map_img.putpixel(start.pos, (0,255,0))
		# self.processed_map_img.putpixel(finish.pos, (0,255,0))

		self.processed_map_img.show()

	# Only call this once, AFTER find_path
	def draw_plan(self):
		self.draw(False)
		nodes = []
		for leg in self.plan:
			for n in leg:
				nodes.append(n)

		self.processed_map_img = self.processed_map_img.convert('RGB')
		for i in range(len(nodes)-1):
			n1 = self.nodes[nodes[i]]
			n2 = self.nodes[nodes[i+1]]
			self.setpx(n1.pos, (255,0,0))
			px = Utils.pixels_between(n1.pos, n2.pos)
			for p in px:
				self.setpx(p, (255,0,0))

		for b in BLUE_NODE_NAMES:
			n = self.named_nodes[b]
			for i in range(-2,3):
				for j in range(-2,3):
					self.setpx((n.pos[0]+i,n.pos[1]+j), (0,0,255))

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
		closed = set()
		fringe = Utils.PriorityQueue()
		fringe.push((start.id, [start.id], 0), 0 + Utils.mHeuristic(start.pos, finish.pos))
		while True:
			if fringe.isEmpty():
				return []
			node, path, back_cost = fringe.pop()
			#print node, finish.id
			if node == finish.id:
				#print path
				return path
			if node not in closed:
				closed.add(node)
				for child_id, child_cost in self.nodes[node].edges.iteritems():
					fringe.push((child_id, path + [child_id], back_cost + child_cost), back_cost + Utils.mHeuristic(self.nodes[child_id].pos, finish.pos))

	# blue_pixels is a list of (x,y) tuples for goal locations
	def add_blue(self, blue_pixels):
		bnodes = []
		global BLUE_NODE_NAMES
		BLUE_NODE_NAMES = BLUE_NODE_NAMES[:len(blue_pixels)]
		for i in range(len(blue_pixels)):
			pos = self.reverse_imgpx(blue_pixels[i])
			if self.processed_map_mat[pos[0],pos[1]] == 0:
				print "  d'oh!"
				mindist = 99999999
				minpos = None
				for i2 in range(-BUFFER_RADIUS_PX, BUFFER_RADIUS_PX+1):
					for j2 in range(-BUFFER_RADIUS_PX, BUFFER_RADIUS_PX+1):
						altpos = (pos[0]+i2,pos[1]+j2)
						if self.processed_map_mat[altpos[0],altpos[1]] == 0:
							continue
						if dist(pos, altpos) < mindist:
							mindist = dist(pos, altpos)
							minpos = altpos
				pos = minpos

			b = Node(pos, BLUE_NODE_NAMES[i])
			bnodes.append(b)
			self.add(b)
		self.connectify(bnodes)
		print "added blue nodes"

	# MUST add blue nodes first!
	# start is an (x,y) tuple
	def find_path(self, start):
		s = Node(self.reverse_imgpx(start), "START")
		self.add(s)
		self.connectify([s])

		size = len(BLUE_NODE_NAMES)+1
		costs = np.zeros((size, size))
		paths = [[None]*size for _ in range(size)]

		for i in range(size-1):
			p = self.search(self.named_nodes["START"], self.named_nodes[BLUE_NODE_NAMES[i]])
			costs[0,i+1] = self.cost_of_path(p)
			paths[0][i+1] = p
			costs[i+1,0] = self.cost_of_path(p)
			paths[i+1][0] = p

		for i1 in range(size-1):
			for i2 in range(size-1):
				p = self.search(self.named_nodes[BLUE_NODE_NAMES[i1]], self.named_nodes[BLUE_NODE_NAMES[i2]])
				costs[i1+1,i2+1] = self.cost_of_path(p)
				paths[i1+1][i2+1] = p
				costs[i2+1,i1+1] = self.cost_of_path(p)
				paths[i2+1][i1+1] = p

		lowest_cost = 99999999
		lowest_cost_path = None
		idx = range(1, size)
		for path in itertools.permutations(idx):
			cost = costs[0,path[0]]
			for i in range(len(path)-1):
				cost += costs[path[i],path[i+1]]
			if cost < lowest_cost:
				lowest_cost = cost
				lowest_cost_path = path

		self.plan.append(paths[0][lowest_cost_path[0]])
		for i in range(len(lowest_cost_path)-1):
			self.plan.append(paths[lowest_cost_path[i]][lowest_cost_path[i+1]])
		for x in range(1,len(self.plan)):
			self.plan[x].reverse()
		print self.plan

	def infill(self):
		print "infilling"
		newnodes = []
		for i in range(self.h):
			for j in range(self.w):
				if (i % 200 == 0) and (j == 0):
					print "  " + str(i * 100.0 / self.h) + "%"
				if self.processed_map_mat[i,j] == 0:
					continue

				nearest = None
				neardist = INFILL_RADIUS
				for nid, n in self.nodes.iteritems():
					d = dist((i,j), n.pos)
					if (d <= neardist) and (self.connected((i,j), n.pos)):
						nearest = n
						neardist = d
				if nearest == None:
					newnode = Node((i,j))
					self.add(newnode)
					newnodes.append(newnode)
		print "infill done: " + str(len(newnodes))
		self.connectify(newnodes)

	# Writes the plan found to the specified filename
	# Each line is a separate leg of the path (i.e. from start to a blue node or from one blue node to the next)
	# Each line is written as a list of (x,y) coordinates in map-space
	def write_to_file(self, filename):
		f = open(filename, 'w')
		for n in self.plan:
			f.write(self.path_to_string(n) + "\n")
		f.close()

	# where the path is a list of node IDs, i.e. as outputted by search()
	def cost_of_path(self, path):
		cost = 0
		for i in range(len(path)-1):
			cost += self.nodes[path[i]].edges[path[i+1]]
		return cost

	# where the path is a list of node IDs, i.e. as outputted by search()
	def path_to_string(self, path):
		ret = []
		for n in path:
			ret.append(self.nodes[n].pos)
		return str(ret)



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


def points_from_file(filename):
	ret = []
	with open(filename, 'rb') as csvfile:
		read = csv.reader(csvfile, delimiter=',', quotechar='|')
		firstrow = True
		for row in read:
			if firstrow:
				firstrow = False
				continue
			ret.append((int(row[0]),int(row[1])))
	return ret

# reds=[
# (2115,1415),
# (2160,1370),
# (2050,1310),
# (1400,1780),
# (1520,1830)
# ]
# blues=[
# (1070,2165),
# (1500,1715),
# (1900,1105),
# (2250,1515),
# (2430,1375)
# ]

# b0 = Node((600,600), "B0")
# b1 = Node((2350,2350), "B1")
# g.add(b0)
# g.add(b1)
# g.connectify([b0, b1])

# g.draw_path(b0, b1)
# print g.search(b0,b1)

# ARGV given RED, BLUE, START

g = MapGraph()
if len(sys.argv == 4):
	g.process_map(points_from_file(sys.argv[1]))
else:
	g.process_map([])

g.init_nodes()
g.connectify()
g.infill()

if len(sys.argv == 4):
	g.add_blue(points_from_file(sys.argv[2]))
	g.find_path(points_from_file(sys.argv[3])[0])
	g.draw_plan()
else:
	g.draw()

g.write_to_file("out.txt")
g.processed_map_mat.save("out.np")