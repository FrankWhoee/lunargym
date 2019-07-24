class mapline():

    def __init__(self, xo, x, yo, y):
        self.xo = xo
        self.x = x
        self.yo = yo
        self.y = y

        self.isVertical = (x - xo == 0)
        if self.isVertical:
            self.m = None
        else:
            self.m = (y - yo) / (x - xo)
            self.b = y - (self.m * x)



    def getY(self, x):
        return self.m * x + self.b

    def isFlat(self):
        return self.m == 0;

    def get_coordinates(self):
        return [self.xo, self.x, self.yo, self.y]
