from cnoid.QtCore import *

class TimerSample:
    def __init__(self):
        self.timer = QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.doSomething)
        self.timer.start()
        self.counter = 0

    def doSomething(self):
        print("do something %d" % self.counter)
        self.counter += 1
        if self.counter == 10:
            self.timer.stop()

timerSample = TimerSample()
