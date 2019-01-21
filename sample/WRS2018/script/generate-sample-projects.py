#!/usr/bin/env python

tasks = [ "T1M", "T1L", "T2", "T3", "T4", "T5", "T6" ]
robots = [ "AizuSpiderSS", "AizuSpiderSA", "DoubleArmV7S", "DoubleArmV7A", "WAREC1", "Quadcopter" ]
interfaces = [ "", "RTM", "ROS" ]

def get_simulators(robot):
    if robot == "AizuSpiderSS" or robot == "DoubleArmV7S":
        return '"AISTSimulator"'
    elif robot == "AizuSpiderSA" or robot == "DoubleArmV7A":
        return '"AGXSimulator"'
    else:
        return '[ "AGXSimulator", "AISTSimulator" ]'

def get_vision_sensors(robot):
    if robot == "AizuSpiderSS" or robot == "AizuSpiderSA":
        return "FRONT_CAMERA"
    elif  robot == "DoubleArmV7A" or robot == "DoubleArmV7S":
        return "FRAME_FRONT_CAMERA"
    else:
        return ""

def write_project_script(filename, task, robot, interface):

    print("Generating {}".format(filename))

    f = open(filename, mode = "w")

    f.write('import WRSUtil\n')
    f.write('WRSUtil.loadProject(\n    ')
    
    # View configuration
    if not interface:
        f.write('"MultiSceneViews", ')
    else:
        f.write('"SingleSceneView\", ')

    # Task
    f.write('"{}", '.format(task))

    # Simulators
    f.write('{}, '.format(get_simulators(robot)))

    # Robot
    f.write('"{}"'.format(robot))

    options = ""

    if robot == "Quadcopter":
        options += "enableMulticopterSimulation = True"

    if interface:
        if options:
            options += ", "
        options += "enableVisionSimulation = True, "
        #options += 'targetVisionSensors = "{}", '.format(get_vision_sensors(robot))
        options += 'remoteType = "{}"'.format(interface)

    if not options:
        f.write(')\n')
    else:
        f.write(',\n    {})\n'.format(options))

    f.close()

for task in tasks:
    for robot in robots:
        if task == "T1M" and "DoubleArmV7" in robot:
            continue
        if task == "T1L" and ("AizuSpider" in robot or robot == "WAREC1"):
            continue
        for interface in interfaces:
            if not interface:
                filename = "{}-{}.py".format(task, robot)
            else:
                if robot == "WAREC1":
                    continue
                filename = "{}-{}-{}.py".format(task, robot, interface)

            write_project_script(filename, task, robot, interface)
