class Runner:
    def __init__(self, hardware, controller, tickInterval, msTickFunc, tickDiffFunc):
        self.hardware = hardware
        self.controller = controller
        self.msTickFunc = msTickFunc
        self.tickDiffFunc = tickDiffFunc
        self.tickInterval = tickInterval
        

    def runLoop(self):
        lastTick = self.msTickFunc()
        while True:
            tick = self.msTickFunc()
            tickDiff = self.tickDiffFunc(tick, lastTick)
            if(tickDiff >= self.tickInterval):
                #self.controller.update(tickDiff)
                self.hardware.update(tickDiff)
                lastTick = tick