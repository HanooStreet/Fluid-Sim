import numpy as np
import sys

class gridPoint:
    grid = []
    gravity = np.array([0, -9.8], dtype=float)
    viscosityCoefficient = 0.3

    def __init__(self, density, velocity, x, y, xDim, yDim):
        self.density = density
        self.densityTemp = 0
        self.velocity = velocity
        self.acceleration = np.array([0,0], dtype=float)
        self.x = x
        self.y = y
        self.neighbours = []

    def is_solid(self):
        return False

    def initializeGrid(xDim, yDim, otherForce=np.array([0,0], dtype=float)):
        for i in range(xDim):
            row = []
            for j in range(yDim):
                row.append(gridPoint(10, np.array([0,0], dtype=float), i, j, xDim, yDim))
            gridPoint.grid.append(row)
        gridPoint.gravity += otherForce

        for i in range(xDim):
            for j in range(yDim):
                gridPoint.grid[i][j].initializeNeighbours(xDim, yDim, i, j)

        for i in range(xDim):
            gridPoint.grid[i][0].setSolid()
            gridPoint.grid[i][yDim-1].setSolid()
        for j in range(yDim):
            gridPoint.grid[0][j].setSolid()
            gridPoint.grid[xDim-1][j].setSolid()

    # I think we can allow the grid to index from 1 to len-2 because the border cells
    # will always be there, please confirm/deny
    
    def initializeNeighbours(self, xDim, yDim, x, y):
        if x > 0:
            self.neighbours.append(gridPoint.grid[x-1][y])
        if x+1 < xDim:
            self.neighbours.append(gridPoint.grid[x+1][y])
        if y > 0:
            self.neighbours.append(gridPoint.grid[x][y-1])
        if y+1 < yDim:
            self.neighbours.append(gridPoint.grid[x][y+1])

    def nextFrame():
        def iterateGrid(func):
            for i in range(1, len(gridPoint.grid)-1):
                for j in range(1, len(gridPoint.grid[0])-1):
                    func(gridPoint.grid[i][j])

        iterateGrid(lambda p: p.nextFrame1())
        iterateGrid(lambda p: p.nextFrame2())
        iterateGrid(lambda p: p.nextFrame3())

    def sign(x):
        if x > 0:
            return 1
        if x < 0:
            return -1
        return 0
    
    def setSolid(self):
        for n in self.neighbours:
            if not n.is_solid():
                n.neighbours.remove(self)
        gridPoint.grid[self.x][self.y] = gridPointSolid(self.x, self.y)

    def nextFrame1(self):
        self.acceleration = self.calcAcceleration()
        self.densityTemp = 0
    
    def nextFrame2(self):
        self.velocity += self.acceleration
        if  (self.velocity[0] + self.velocity[1]) == 0:
            return
        
        velX = self.velocity[0]
        velY = self.velocity[1]

        percentageX = velX / (velX + velY)

        # not sure if this implmentation is correct, but I wanted to make it so if a cell is going to push density to
        # a solid cell it just doesnt change the current density in that direction of velocity
        '''
        targetX = self.x + gridPoint.sign(velX)
        targetY = self.y

        if (gridPoint.grid[targetX][targetY].is_solid()):
            self.densityTemp += self.density * percentageX
        else :
            gridPoint.grid[self.x + gridPoint.sign(velX)][self.y].densityTemp += self.density * percentageX
        

        targetX = self.x
        targetY = self.y + gridPoint.sign(velY) 

        if (gridPoint.grid[targetX][targetY].is_solid()):
            self.densityTemp += self.density * percentageX
        else :
            gridPoint.grid[self.x + gridPoint.sign(velX)][self.y].densityTemp += self.density * percentageX

        gridPoint.grid[self.x][self.y + gridPoint.sign(velY)].densityTemp += self.density * (1-percentageX)
        '''
        # bradley changes here


    def nextFrame3(self):
        self.density = self.densityTemp
        
    # we probably need to make some changes to pressure gradient calculation so that shit
    # wont flow into the solids 
    def calcPressure(self):
        if self.density == 0:
            return 0
        return np.array([gridPoint.grid[self.x+1][self.y].density - gridPoint.grid[self.x-1][self.y].density, gridPoint.grid[self.x][self.y+1].density - gridPoint.grid[self.x][self.y-1].density]) / self.density
    
    def calcGravity(self):
        return gridPoint.gravity
    
    def calcViscosity(self):
        neighboursDensity = sum(n.density for n in self.neighbours)
        if neighboursDensity == 0:
            return np.array([0,0])
        v = gridPoint.viscosityCoefficient * (sum(n.velocity*n.density for n in self.neighbours) / neighboursDensity - self.velocity)
        
        return v

    def calcAcceleration(self):
        if (self.x == 1) and self.y == 1:
            print("frame", self.x, self.y)
            print("density  \t", self.density)
            print("velocity \t", self.velocity)
            print("pressure \t", -self.calcPressure())
            print("gravity  \t", self.calcGravity())
            print("viscocity\t", self.calcViscosity())
        return -self.calcPressure() + self.calcGravity() + self.calcViscosity()
    
    

class gridPointSolid(gridPoint):
    def __init__(self, x, y):
        self.velocity = np.array([0,0])
        self.x = x
        self.y = y
        self.density = 9999

    def is_solid(self):
        return True

    def setSolid(self):
        pass

    def nextFrame1(self):
        pass
    def nextFrame2(self):
        pass
    def nextFrame3(self):
        pass


# Du/Dt + (1/density) * grad(pressure) = gravity + viscosity * Laplacian(velocity)
# Du/Dt is the material Derivative = dU/dt + U * grad(U)