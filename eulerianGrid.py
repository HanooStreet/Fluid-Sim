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

    def is_solid(self):
        return False

    def initializeGrid(xDim, yDim, otherForce=np.array([0,0], dtype=float)):
        for i in range(xDim):
            row = []
            for j in range(yDim):
                row.append(gridPoint(10, np.array([0,0], dtype=float), i, j))
            gridPoint.grid.append(row)
        gridPoint.gravity += otherForce

        for i in range(xDim):
            gridPoint.grid[i][0].setSolid()
            gridPoint.grid[i][yDim-1].setSolid()
        for j in range(yDim):
            gridPoint.grid[0][j].setSolid()
            gridPoint.grid[xDim-1][j].setSolid()

    # I think we can allow the grid to index from 1 to len-2 because the border cells
    # will always be there, please confirm/deny

    def nextFrame():
        for i in range(1, len(gridPoint.grid)-2):
            for j in range(1, len(gridPoint.grid[0])-2):
                gridPoint.grid[i][j].nextFrame1()
        for i in range(1,len(gridPoint.grid)-2):
            for j in range(1,len(gridPoint.grid[0])-2):
                gridPoint.grid[i][j].nextFrame2()
        for i in range(1,len(gridPoint.grid)-2):
            for j in range(1,len(gridPoint.grid[0])-2):
                gridPoint.grid[i][j].nextFrame3()

    def sign(x):
        if x > 0:
            return 1
        if x < 0:
            return -1
        return 0
    
    def setSolid(self):
        gridPoint.grid[self.x][self.y] = gridPointSolid(self.density, np.array([0.0,0.0], dtype=float), self.x, self.y)

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
        # bradley changes here


    def nextFrame3(self):
        self.density = self.densityTemp
        
    # we probably need to make some changes to pressure gradient calculation so that shit
    # wont flow into the solids 
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
        self.velocity = np.array([0,0])
        self.acceleration = np.array([0,0])
        self.x = x
        self.y = y
        self.density = density
        self.densityTemp = 0

    def is_solid(self):
        return True

    def nextFrame1(self):
        pass
    def nextFrame2(self):
        pass
    def nextFrame3(self):
        pass


# Du/Dt + (1/density) * grad(pressure) = gravity + viscosity * Laplacian(velocity)
# Du/Dt is the material Derivative = dU/dt + U * grad(U)