#!/usr/bin/env python3

# To execute this script, the python module path must include the directory
# containing the Choreonoid python modules.
# Choreonoid python module directory usually exists at ${PREFIX}/lib/choreonoid-x.x/python.
# The python module path can be added by setting the environmental variable PYTHONPATH.

from math import *
import random
import cnoid.Util

name = "CapsuleTerrain"
capsule_diameter = 0.4
capsule_radius = capsule_diameter / 2.0
capsule_height_step = 0.12
capsule_color_r = 0.56
capsule_color_g = 0.56
capsule_color_b = 0.56
num_capsules_x = 12
num_capsules_y = 12
x0 = capsule_radius
y0 = capsule_radius

header = '''\
format: ChoreonoidBody
formatVersion: 1.0
angleUnit: degree
name: {name}

APPEARANCE: &APP
  material:
    diffuseColor: [ {r:.4}, {g:.4}, {b:.4} ]\
'''

print(header.format(
  name = name,
  r = capsule_color_r,
  g = capsule_color_g,
  b = capsule_color_b
  ))


line_header_description = '''
LINE{no}: &LINE{no}\
'''

capsule_description = '''\
  -
    type: Shape
    translation: [ {x:.7}, 0, {z:.7} ]
    rotation: [ [ {rx:.7}, {ry:.7}, {rz:.7}, {theta:.7} ], [ 1, 0, 0, 90 ] ]
    geometry: {{ type: Capsule, radius: {radius}, height: {height} }}
    appearance: *APP\
'''

def is_edge_distance(x, y, distance):
  return (x == distance or x == num_capsules_x - 1 - distance or
          y == distance or y == num_capsules_y - 1 - distance)

for i in range(num_capsules_y):

    print(line_header_description.format(no = i))

    for j in range(num_capsules_x):

        if is_edge_distance(i, j, 0):
            h = random.randint(0, 1)
        elif is_edge_distance(i, j, 1):
            h = random.randint(0, 2)
        else:
            h = random.randint(1, 3)

        roll = random.random() * 0.6 - 0.3
        pitch = random.random() * 0.6 - 0.3
        R = cnoid.Util.rotFromRpy([roll, pitch, 0.0])
        aa = cnoid.Util.AngleAxis(R)
        axis = aa.axis()
            
        print(capsule_description.format(
            x = capsule_diameter * j,
            z = h * capsule_height_step / 2.0,
            rx = axis[0],
            ry = axis[1],
            rz = axis[2],
            theta = degrees(aa.angle()),
            radius = capsule_radius,
            height = h * capsule_height_step
            ))

link_header_description = '''
links:
  - 
    name: LINE0
    jointType: fixed
    material: Ground
    convexRecompostiion: true
    elements:
      -
        type: Transform
        translation: [ {x0:.5}, {y0:.5}, 0 ]
        elements: *LINE0\
'''
        
link_description = '''\
  - 
    name: LINE{index}
    parent: LINE0
    jointType: fixed
    material: Ground
    convexRecompostiion: true
    translation: [ {x0:.5}, {y:.5}, 0 ]
    elements: *LINE{index}\
'''

print(link_header_description.format(x0 = x0, y0 = y0))

for i in range(1, num_capsules_y):
    print(link_description.format(
        index = i,
        x0 = x0,
        y = capsule_diameter * i + y0
        ))
