#!/usr/bin/env python3

from math import *

header = '''\
TUBE: &TUBE
  type: Group
  elements:
'''

box_description = '''\
    - # Cylinder wall {index}
      type: Shape
      rotation: [ {axis}, {theta:.7} ]
      translation: [ {x:.5}, {y:.5}, {z:.5} ]
      geometry: {{ type: Box, size: [ {sx:.7}, {sy:.7}, {sz:.7} ] }}
      appearance: *TUBE_APPEARANCE
'''

def generate_tube(verticalAxisIndex, verticalTranslation, radius, height, widthRatio, thickness, resolution):

    print(header)

    t2 = 2.0 * pi / resolution / 2.0;
    width = widthRatio * 2.0 * (radius * tan(t2) + (thickness / 2.0) * tan(t2));

    for i in range(resolution):
        theta = 360.0 * i / resolution
        x = radius * cos(radians(theta))
        y = radius * sin(radians(theta))

        if verticalAxisIndex == 0: # X
            axis = '1, 0, 0'
            tx = verticalTranslation
            ty = x
            tz = y
            sx = height
            sy = thickness
            sz = width
        elif verticalAxisIndex == 1: # Y
            axis = '0, 1, 0'
            tx = x
            ty = verticalTranslation
            tz = y
            sx = thickness
            sy = height
            sz = width
        elif verticalAxisIndex == 2: # Z
            axis = '0, 0, 1'
            tx = x
            ty = y
            tz = verticalTranslation
            sx = thickness
            sy = width
            sz = height

        print(box_description.format(
            index = i + 1,
            axis = axis,
            theta = theta,
            x = tx,
            y = ty,
            z = tz,
            sx = sx,
            sy = sy,
            sz = sz
        ))
       

#generate_tube(
#    verticalAxisIndex = 0, verticalTranslation = 0.0305,
#    radius = 0.0235, height = 0.07, widthRatio = 0.84, thickness = 0.0125, resolution = 24)

#generate_tube(
#    verticalAxisIndex = 0, verticalTranslation = -0.035,
#    radius = 0.0235, height = 0.07, widthRatio = 0.84, thickness = 0.0125, resolution = 24)

# Nozzle Slot
#generate_tube(
#    verticalAxisIndex = 0, verticalTranslation = -0.025,
#    radius = 0.0235, height = 0.05, widthRatio = 0.84, thickness = 0.0125, resolution = 24)

# Ceiling Hole
#thickness = 0.032
#generate_tube(
#    verticalAxisIndex = 2, verticalTranslation = 0.0,
#    radius = 0.08 + thickness / 2, height = 0.091, widthRatio = 1.0, thickness = thickness, resolution = 24)

# Fire Circle
thickness = 0.1
generate_tube(
    verticalAxisIndex = 2, verticalTranslation = 0.0,
    radius = 5.0 + thickness / 2, height = 0.002, widthRatio = 1.0, thickness = thickness, resolution = 48)

