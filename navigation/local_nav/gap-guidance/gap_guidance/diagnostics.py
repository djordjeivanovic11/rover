import time
class FPSCounter:
    def __init__(self, window=100):
        self.window = window
        self.times  = []

    def tick(self):
        now = time.time()
        self.times.append(now)
        if len(self.times) > self.window:
            self.times.pop(0)

    @property
    def fps(self):
        if len(self.times) < 2:
            return 0.0
        return (len(self.times)-1)/(self.times[-1]-self.times[0])
