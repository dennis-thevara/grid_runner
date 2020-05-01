import json
import os

def get_data(source):
    with open(os.path.dirname(os.path.realpath(__file__))+source, 'r') as f:
        return json.load(f)

class Localization(object):
    def __init__(self):
        config = get_data("/2d_params.json")
        # Get world state and measurements:
        self.measurements = get_data("/2d_params.json")["measurements"]
        self.motions = get_data("/2d_params.json")["motions"]
        self.world = get_data("/2d_params.json")["world"]
        # Import params for localization:
        self.rows = len(self.world)
        self.cols = len(self.world[0])
        self.pHit = config["pHit"]
        self.pMiss = 1-self.pHit
        self.pExact = config["pExact"]
        self.pOvershoot = config["pOvershoot"]
        self.pUndershoot = config["pUndershoot"]
        # Initialize uniform probability distribution:
        self.P = [[1/(self.rows*self.cols) for c in range(self.cols)] for r in range(self.rows)]

    def show(self):
        rows = [
            '[' + ','.join(map(lambda x: '{0:.5f}'.format(x), r)) + ']' for r in self.P]
        print('[' + ',\n '.join(rows) + ']')

    def sense(self,Z):
        s = 0
        for r in range(self.rows):
            for c in range(self.cols):
                # Check if the measurement matches the world cell:
                hit = (self.world[r][c] == Z)
                # Assign probability to current cell using above binary flag and sensor probabilities:
                self.P[r][c] *= self.pHit*hit + self.pMiss*(1-hit)
                s += self.P[r][c]
        # Normalize prob dist using sum of all elements:
        self.P = [[self.P[r][c]/s for c in range(self.cols)] for r in range(self.rows)]

    def move(self,U):
        Ur,Uc = U
        Q = [[0 for c in range(self.cols)] for r in range(self.rows)]
        for r in range(self.rows):
            for c in range(self.cols):
                # Exact motion implies single step was executed correctly:
                s = self.P[(r-Ur)%self.rows][(c-Uc)%self.cols]*self.pExact
                # Undershoot implies no movement was made (-U+U) as this is a single step robot:
                s += self.P[(r-Ur+Ur)%self.rows][(c-Uc+Uc)%self.cols]*self.pUndershoot
                # Overshoot implies the step was made twice (-U-U):
                s += self.P[(r-Ur-Ur)%self.rows][(c-Uc-Uc)%self.cols]*self.pOvershoot
                Q[r][c] = s
        self.P = Q

    def localize(self):
        for i in range(len(self.measurements)):
            self.move(self.motions[i])
            self.sense(self.measurements[i])
        self.show()

if __name__ == "__main__":
    Localization().localize()

