#!/usr/bin/env python
# -*- coding: utf-8 -*-

import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import path_planning.Constants as const
from path_planning.Point import Point as pt
import random
from numpy import absolute, arctan, pi, sqrt


ANGLE_THRESHOLD = 30  # Angle threshold for charge possibility
SQUARE_RESERVE = 1  # (SQUARE_RESERVE*2)^2 == square of charging region

MAX_COORDINATE = 1024
MAX_X, MAX_Y = 30000, 30000
k = MAX_X/MAX_COORDINATE  # Коэффициент преобразования из hmap в point и наоборот

ENERGY_THRESHOLD = gc_const.LOWER_LIMIT_BATTERY
ENERGY_RESOURCE = 100
MAX_ENERGY_RESOURCE = gc_const.HIGH_LIMIT_BATTERY

# Point: x, y, z
# Hmap: y, x


def eval_charge_points(path, workpoints, energy_resource, distance):
	# Evaluating charging points on the path
	# Input:
	#	   path - trajectory of robot
	#	   hmap - height map
	# Output:
	#	   ch_p - array of tuples (charging_point, distance)

	ch_p = []
	total_distance = distance
	path_full = len(path)
	i = 1
	last_vect = path[0].get_dir_vector_between_points(path[1])
	while i < path_full:
		point = path[i]
		last_point = path[i-1]
		last_point.last_vect = last_vect

		part_distance = eval_distance(point, last_point)  # m
		total_distance += part_distance
		energy_resource -= part_distance*gc_const.MOVE_CHARGE_LOSS_COEF
		if energy_resource < ENERGY_THRESHOLD:
			ch_p.append((last_point, distance))
			#gc.spawn_sdf_model(new_p, gc_const.GREEN_VERTICE_PATH, str(i) + str(random.random()))
			energy_resource = MAX_ENERGY_RESOURCE
			i += 1

		if point in workpoints:
			energy_resource -= gc_const.TASK_ENERGY_COST
			if energy_resource < ENERGY_THRESHOLD:
				while True:
					energy_resource += MAX_ENERGY_RESOURCE - ENERGY_THRESHOLD
					ch_p.append((point, distance))
					if energy_resource >= ENERGY_THRESHOLD:
						break
		last_vect = last_point.get_dir_vector_between_points(point)
		i += 1

	return ch_p, energy_resource, total_distance


def eval_distance(p1, p2):
	distance = sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)
	return distance


def make_rectangle(point, hmap):
	# Calculating coordinates of square around center of charging point

	# Input:
	#	   point - vertex on the path
	#	   hmap - height map
	# Output:
	#	   rectangle - dictionary with coordinates of each vertex on each edgge of rectangle

	h, w, z = convert_point_to_hmap_vert(point)
	rectangle = {}
	w_range = [w - SQUARE_RESERVE, w + SQUARE_RESERVE]
	h_range = [h - SQUARE_RESERVE, h + SQUARE_RESERVE]
	# Make y-edges
	for x in w_range:
		for y in range(h_range[0], h_range[1] + 1):
			# (x, y) are reversed in hmap
			point = hmap[str(float(y)), str(float(x))]
			ww, hh, zz = convert_point_to_hmap_vert(point)
			rectangle[(x, y)] = zz
	# Make x-edges
	for y in h_range:
		for x in range(w_range[0], w_range[1] + 1):
			point = hmap[str(float(y)), str(float(x))]
			ww, hh, zz = convert_point_to_hmap_vert(point)
			rectangle[(x, y)] = zz

	return rectangle



def angle_with_ground(rectangle, hmap):
	# Calculating angle between rectangle and ground plane

	# Input:
	#	   rectangle (dict)
	#	   hmap
	# Output:
	#	   angle (float)

	(min_x, max_x, min_y, max_y) = min_max_coords(rectangle, hmap)
	# Define the average 'z' (av_z) of each edge of rectangle
	av_z = {'x': [], 'y': []}
	w_range = [min_x, max_x]
	h_range = [min_y, max_y]
	i = 0
	z = 0
	for x in w_range:
		for y in range(h_range[0], h_range[1] + 1):
			z += rectangle[(x, y)]
			i += 1
		# Average z
		z = z / i
		av_z['y'].append(z)
	i = 0
	z = 0
	for y in h_range:
		for x in range(w_range[0], w_range[1] + 1):
			z += rectangle[(x, y)]
			i += 1
		z = z / i
		av_z['x'].append(z)
	# Define 2 angles (2 axises)
	max_heigt_x = max(av_z['x'][0], av_z['x'][1])
	min_heigt_x = min(av_z['x'][0], av_z['x'][1])
	max_heigt_y = max(av_z['y'][0], av_z['y'][1])
	min_heigt_y = min(av_z['y'][0], av_z['y'][1])
	a_x = max_x - min_x
	a_y = max_y - min_y
	h_x = max_heigt_x - min_heigt_x
	h_y = max_heigt_y - min_heigt_y
	angle_x = arctan(h_x / a_x) if h_x != 0 else 0
	angle_y = arctan(h_y / a_y) if h_y != 0 else 0
	angle = max(angle_x, angle_y)
	# Convert radians to degrees
	angle = angle * 180 / pi
	return angle


def min_max_coords(rectangle, hmap):
	# Calculating min and max coords (corners) of rectangle

	# Input:
	#	   rectangle
	# Output:
	#	   min_x, max_x, min_y, max_y (min_x, max_y -> left upper corner of rectangle)

	# 128 - maximum vertex num. To define minimum, we take 129 to compare.
	min_x = sqrt(len(hmap))
	max_x = -1
	min_y = sqrt(len(hmap))
	max_y = -1

	for coord in rectangle:
		x, y = coord
		min_x = min(min_x, x)
		max_x = max(max_x, x)
		min_y = min(min_y, y)
		max_y = max(max_y, y)

	return min_x, max_x, min_y, max_y


def convert_point_to_hmap_vert(point):
	# Convert coords to hmap format

	# Input:
	#	   point - 3 coordinates (class Point)
	# Output:
	#	   h, w, z - 3 coordinates (tuple)

	string = str(point).split()
	# w <-> x
	# h <-> y
	# Define quadrant (I, II, III, IV) to convert coordinates.
	x = float(string[0])
	y = float(string[1])
	z = float(string[2])

	if x > 0:
		w = int((MAX_X/2 + x)/k)
		if y > 0:
			# I quadrant
			h = int((MAX_Y/2 - y)/k)
		else:
			# IV quadrant
			h = int((MAX_Y/2 + absolute(y))/k)
	else:
		w = int((MAX_X/2 - absolute(x))/k)
		if y > 0:
			# II quadrant
			h = int((MAX_Y/2 - y)/k)
		else:
			# III quadrant
			h = int((MAX_Y/2 + absolute(y))/k)

	return h, w, z


def draw_charging_region(rectangle, hmap):
	# Draws rectangle of charging region in Gazebo

	# Input:
	#	   rectangle
	# Output:
	#	   -

	# Define max height for drawing charging region
	z = 0
	for coord in rectangle:
		z = max(rectangle[coord], z)

	(min_x, max_x, min_y, max_y) = min_max_coords(rectangle, hmap)
	x = min_x
	while x <= max_x:
		# Convert each coord from hmap to point
		point_x = float((x - MAX_X/2)/k)

		point_y = float((MAX_X/2 - min_y)/k)
		point = pt(point_x, point_y, z)
		gc.spawn_sdf_model(str(random.random()), gc_const.GREEN_VERTICE_PATH, point)

		point_y = float((MAX_X/2 - max_y)/k)
		point = pt(point_x, point_y, z)
		gc.spawn_sdf_model(str(random.random()), gc_const.GREEN_VERTICE_PATH, point)

		x += 0.25
	y = min_y
	while y <= max_y:
		point_y = float((MAX_X/2 - y)/k)

		point_x = float((min_x - MAX_X/2)/k)
		point = pt(point_x, point_y, z)
		gc.spawn_sdf_model(str(random.random()), gc_const.GREEN_VERTICE_PATH, point)

		point_x = float((max_x - MAX_X/2)/k)
		point = pt(point_x, point_y, z)
		gc.spawn_sdf_model(str(random.random()), gc_const.GREEN_VERTICE_PATH, point)

		y += 0.25
