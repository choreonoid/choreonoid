#!/usr/bin/env python3

from math import *
import random

name = "TerrainB"
block_size = 0.4
block_size_z1 = 0.14
block_size_z2 = 0.247
block_color_r = 0.56
block_color_g = 0.56
block_color_b = 0.56
num_blocks_x = 12
num_blocks_y = 12
iniPos_x = 0.2
iniPos_y = 0.2

block_size2 = block_size/2.0

header = '''\
format: ChoreonoidBody
formatVersion: 1.0
angleUnit: degree
name: {name}

BLOCK: &BLOCK
  type: Shape
  appearance:
    material:
      diffuseColor: [ {r:.4}, {g:.4}, {b:.4} ]
  geometry:
    type: IndexedFaceSet
    coordinate: [ -{xy:.5}, -{xy:.5}, 0.0,
                   {xy:.5}, -{xy:.5}, 0.0,
                   {xy:.5},  {xy:.5}, 0.0,
                  -{xy:.5},  {xy:.5}, 0.0,
                  -{xy:.5}, -{xy:.5}, {z1:.5},
                   {xy:.5}, -{xy:.5}, {z2:.5},
                   {xy:.5},  {xy:.5}, {z2:.5},
                  -{xy:.5},  {xy:.5}, {z1:.5} ]
    coordIndex: [ 3, 2, 1, 0, -1,
                  4, 5, 6, 7, -1,
                  4, 0, 1, 5, -1,
                  5, 1, 2, 6, -1,
                  6, 2, 3, 7, -1,
                  7, 3, 0, 4, -1 ]
                  
BASE_BLOCK: &BASE_BLOCK
  type: Shape
  appearance:
    material:
      diffuseColor: [ {r:.4}, {g:.4}, {b:.4} ]
  geometry: {{ type: Box, size: [ {size:.5}, {size:.5}, {z1:.5} ] }}
'''

print(header.format(
    name = name,
    r = block_color_r,
    g = block_color_g,
    b = block_color_b,
    xy = block_size2,
    z1 = block_size_z1,
    z2 = block_size_z2,
    size = block_size
    ))

line_header_description = '''\
LINE{no}: &LINE{no}
'''

line_description_block = '''\
  -
    <<: *BLOCK
    rotation: [ 0, 0, 1, {rot} ]
    translation: [ {x:.7}, 0, {z:.7} ]
'''

line_description_base = '''\
  -
    <<: *BASE_BLOCK
    translation: [ {x:.7}, 0, {z:.7} ]
'''

for i in range(num_blocks_y):
    print(line_header_description.format(
        no = i))
    for j in range(num_blocks_x):
        if i==0 or i==num_blocks_y-1 or j==0 or j==num_blocks_x-1:
            z_ = 0 #random.randint(0,1)
        else:
            z_ = random.randint(0,2)
        print(line_description_block.format(
            rot = 90 * random.randint(0,3),
            x = block_size * j,
            z = z_ * block_size_z1
            ))
        if z_==2:
            print(line_description_base.format(
                x = block_size * j,
                z = block_size_z1 * 1.5
                ))
        
link_header_description = '''\
links:
  - 
    name: 0
    jointType: fixed
    material: Ground
    convexRecompostiion: true
    elements:
      -
        type: Transform
        translation: [ {ini_x:.5}, {ini_y:.5}, 0 ]
        elements: *LINE0
'''
        
link_description = '''\
  - 
    name: {index}
    parent: 0
    jointType: fixed
    material: Ground
    convexRecompostiion: true
    translation: [ {ini_x:.5}, {y:.5}, 0 ]
    elements: *LINE{index}
'''

print(link_header_description.format(
    ini_x = iniPos_x,
    ini_y = iniPos_y
    ))
for i in range(1, num_blocks_y):
    print(link_description.format(
        index = i,
        ini_x = iniPos_x,
        y = block_size * i + iniPos_y,
        ))
