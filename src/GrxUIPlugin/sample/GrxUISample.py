import os, sys, traceback
import cnoid.grxui
import time

def setup1():
  print "setup1"

def setup2():
  print "setup2"

def utility1():
  print "utility1"

def utility2():
  print "utility2"

def putMessage(message):
  print message

def wait(second):
  t = 0.0
  while t < second:
    print "Wait %f [s] ..." % 0.5
    time.sleep(0.5)
    t += 0.5
  print "Wait finished."


def inputValues(x, y, z):
  print "inputValues(%d, %d, %d)" % x % y % z

def inputStrings(s1, s2):
  print "inputStrings(\"%s\", \"%s\")" % s1 % s2

menu = [
  [ 
    '---------- Setup ----------', '#label',
    'Setup 1',                     'setup1()',
    'Setup 2',                     'setup2()',
    '--------- Utility ---------', '#label',
    'Utility 1',                   'utility1()',
    'Utility 2',                   'utility2()', 
  ],
  [
    'Finish this page',            'putMessage("This is the last button in the first page")',
    'Wait',                        'wait(10.0)',
    'Input strings',               'inputStrings("#T", "#T")',
    'Input values',                'inputValues(#D, #D, #D)',
    'Start',                       'putMessage("This is the first button in the first page")',
  ],
  [
    'Finish the second page',      'putMessage("This is the last button in the second page")',
    'Start the second page',       'putMessage("Pages can be added like this")',
  ],
]
    
cnoid.grxui.waitInputMenu(menu)


