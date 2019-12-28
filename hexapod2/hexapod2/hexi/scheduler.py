import time

class Scheduler(object):
    def __init__(self, sTickFunc, tickDiffFunc):
        self.sTickFunc = sTickFunc
        self.tickDiffFunc = tickDiffFunc
        self.callbacks = []

    def interval(self, seconds, callback):
        self.callbacks.append([seconds, callback, self.sTickFunc()+seconds])

    def run(self):
        while True:
            tick = self.sTickFunc()
            for i in self.callbacks:
                if(i[2] >= tick):
                    i[1](tick)
                    i[2] = tick+i[0]
            time.sleep(0)