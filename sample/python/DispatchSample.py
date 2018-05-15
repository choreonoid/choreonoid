from cnoid.Base import *
from time import sleep

def doSomething(i):
  print("do something %d" % i)

for i in range(10):
    callLater(lambda : doSomething(i))
    sleep(1.0)


