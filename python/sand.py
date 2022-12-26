from matplotlib import pyplot as plt
from math import cos, sin, pi, sqrt

max_d=2776 ## max number of steps 
steps_in_rotation = 22417 

# this is a mockup of the Zen Garden hardware, just for testing out the 
# line drawing algorithm
class SandTable:
    def __init__(self):
        # start at top position
        self.r_step = 0
        self.d_step = 0 
        self.points = []

    def step(self, r, d) :    
        self.r_step += r
        self.d_step += d
        # distance of the magnet from the center is determined by both
        # motors.    The large/small gear = 220/20 = 11
        s = 1.0 * self.d_step / max_d + 1.0 * self.r_step / max_d / 11 
        if (s>1) :
            print("Hitting the endstop (outside)")
        if (s<0) :
            print("Hitting the endstop (inside)")
        y = cos(2.0 * pi * self.r_step / steps_in_rotation) * s
        x = sin(2.0 * pi * self.r_step / steps_in_rotation) * s
        self.points.append((x,y)) 
    
    def plot(self):
        plt.scatter(*zip(*self.points),0.01)
        plt.show()


table = SandTable()

"""
for i in range(1000):   # 1500 is roughly halfway the 2776 
    table.step(0,1)
for i in range(steps_in_rotation // 11):
    table.step(1,-1)     
    for j in range(10):
        table.step(1,0)

table.plot()
"""

def distance(p1,p2):
    (x1,y1) = p1 
    (x2,y2) = p2 
    return sqrt((x2-x1)**2 + (y2-y1)**2)

def distanceToLine(p0, p1, p2):
    (x0,y0) = p0 
    (x1,y1) = p1 
    (x2,y2) = p2 
    #return abs((y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1)/sqrt((y2-y1)**2+(x2-x1)**2)
    return abs((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1)) / sqrt((x2-x1)**2 + (y2-y1)**2)

# TableRouter Class will provide pole-coordinate interface by correcting for 
# the in/out spiraling behavior of the rotating magnet 
class TableRouter:
    def __init__(self, polartable):
        self.table = table 
        self.r = 0 
        self.d = 1 # this is the min value - it ensures we are not stuck in the center 

    def step(self, r, d) :    
        self.r += r
        self.d += d
        d2 = 0
        # every 11th step in either rotational direction, correct the  
        if ((self.r % 11) == 0):
            d2 = -r
        if abs(d2 + d)>1:    
            self.table.step(r,d)
            self.table.step(0,d2)
        else:
            self.table.step(r,d + d2)

    def getxy(self, r=0, d=0):
        #s = 1.0 * (self.d+d) / max_d + 1.0 * (self.r+r) / max_d / 11 
        s = 1.0 * (self.d+d) / max_d 
        y = cos(2.0 * pi * (self.r+r) / steps_in_rotation) * s
        x = sin(2.0 * pi * (self.r+r) / steps_in_rotation) * s
        return (x,y)

    def isInRange(self, d):
        new_d = self.d + d 
        #return True
        return ( new_d > 0 ) and (new_d < max_d)

    # line drawing routine 
    def stepOnLine(self, x1,y1, x2,y2):
        directions = [(0,1),(1,1),(1,0),(1,-1),(0,-1),(-1,-1),(-1,0),(-1,1)]  
        done = True
        best_dir = (0,0);
        distance_to_destination = distance(self.getxy(),(x2,y2))
        best_distance_to_line = 2;
        for (r,d) in directions:
            in_range = self.isInRange(d)
            (x0, y0) = self.getxy(r,d) 
            candidate_distance = distance((x0,y0),(x2,y2))   
            if in_range and (candidate_distance < distance_to_destination):
                distance_to_line = distanceToLine((x0,y0),(x1,y1),(x2,y2))
                if distance_to_line < best_distance_to_line:
                   best_dir = (r,d)
                   best_distance_to_line = distance_to_line
                done = False 
        if not(done): 
            self.step(*best_dir)
            print("r = {}, d = {}".format(self.r,self.d))
        return done

    def goto(self,x,y): 
        (x1,y1) = self.getxy()
        (x2,y2) = (x,y)  
        done = False
        while not(done): 
            done = self.stepOnLine(x1,y1,x2,y2)
          

controller = TableRouter(table)
controller.goto(0,-1)
controller.goto(0,1)
controller.goto(1,0)
controller.goto(-1,0)

table.plot()
