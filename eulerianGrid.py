import numpy as np

class gridPoint:
    grid = []
    gravity = np.array([0, -9.8], dtype=float)
    viscosityCoefficient = 1

    def __init__(self, density, velocity, x, y):
        self.density = density
        self.densityTemp = 0
        self.velocity = velocity
        self.acceleration = np.array([0,0], dtype=float)
        self.x = x
        self.y = y

    def initializeGrid(xDim, yDim, otherForce=np.array([0,0], dtype=float)):
        for i in range(xDim):
            row = []
            for j in range(yDim):
                row.append(gridPoint(10, np.array([0,0], dtype=float), i, j))
            gridPoint.grid.append(row)
        gridPoint.gravity += otherForce

    def nextFrame():
        for i in range(1, len(gridPoint.grid)-1):
            for j in range(1, len(gridPoint.grid[0])-1):
                gridPoint.grid[i][j].nextFrame1()
        for i in range(1,len(gridPoint.grid)-1):
            for j in range(1,len(gridPoint.grid[0])-1):
                gridPoint.grid[i][j].nextFrame2()
        for i in range(1,len(gridPoint.grid)-1):
            for j in range(1,len(gridPoint.grid[0])-1):
                gridPoint.grid[i][j].nextFrame3()

    def sign(x):
        if x > 0:
            return 1
        if x < 0:
            return -1
        return 0
    
    def nextFrame1(self):
        self.acceleration = self.calcAcceleration()
        self.densityTemp = 0
    
    def nextFrame2(self):
        self.velocity += self.acceleration
        if  (self.velocity[0] + self.velocity[1]) == 0:
            return
        percentageX = self.velocity[0] / (self.velocity[0] + self.velocity[1])
        gridPoint.grid[self.x + gridPoint.sign(self.velocity[0])][self.y].densityTemp += self.density * percentageX
        gridPoint.grid[self.x][self.y + gridPoint.sign(self.velocity[1])].densityTemp += self.density * (1-percentageX)
    
    def nextFrame3(self):
        self.density = self.densityTemp
        
    def calcPressureGradient(self):
        return np.array([gridPoint.grid[self.x+1][self.y].density - gridPoint.grid[self.x-1][self.y].density, gridPoint.grid[self.x][self.y+1].density - gridPoint.grid[self.x][self.y-1].density])

    def calcPressure(self):
        if self.density == 0:
            return 0
        return self.calcPressureGradient() / self.density
    
    def calcGravity(self):
        return gridPoint.gravity
    
    def calcViscosity(self):
        x = self.x
        y = self.y
        v = gridPoint.viscosityCoefficient * (gridPoint.grid[x+1][y].velocity + gridPoint.grid[x-1][y].velocity + gridPoint.grid[x][y+1].velocity + gridPoint.grid[x][y-1].velocity - 4*gridPoint.grid[x][y].velocity)
        return v

    def calcAcceleration(self):
        return -self.calcPressure() + self.calcGravity() + self.calcViscosity()

class gridPointSolid(gridPoint):
    def __init__(self, density, velocity, x, y):
        self.mass = 0
        self.velocity = 0
        self.acceleration = np.array([0,0])
        self.x = x
        self.y = y



# Du/Dt + (1/density) * grad(pressure) = gravity + viscosity * Laplacian(velocity)
# Du/Dt is the material Derivative = dU/dt + U * grad(U)