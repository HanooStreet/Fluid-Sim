# ...existing code...
import numpy as np

class gridPoint:
    grid = []
    gravity = np.array([0, -9.8], dtype=float)
    viscosityCoefficient = 1

    def __init__(self, density, velocity, x, y):
        self.density = density
        self.densityTemp = 0
        self.velocity = np.array(velocity, dtype=float)
        self.acceleration = np.array([0,0], dtype=float)
        self.x = x
        self.y = y

    def is_solid(self):
        return False

    def initializeGrid(xDim, yDim, otherForce=np.array([0,0], dtype=float), border_solids=True):
        # reset grid
        gridPoint.grid = []
        for i in range(xDim):
            row = []
            for j in range(yDim):
                row.append(gridPoint(10.0, np.array([0.0,0.0], dtype=float), i, j))
            gridPoint.grid.append(row)
        gridPoint.gravity += otherForce

        # optionally create solid boundary cells
        if border_solids:
            for i in range(xDim):
                gridPoint.setSolid(i, 0)
                gridPoint.setSolid(i, yDim-1)
            for j in range(yDim):
                gridPoint.setSolid(0, j)
                gridPoint.setSolid(xDim-1, j)

    def setSolid(x, y, density=0.0):
        # replace grid cell with a solid cell (preserves x,y indices)
        gridPoint.grid[x][y] = gridPointSolid(density, np.array([0.0,0.0], dtype=float), x, y)

    # shouldnt it be -2? we want the grid to be surrounded by solids, right?
    def nextFrame():
        for i in range(1, len(gridPoint.grid)-1):
            for j in range(1, len(gridPoint.grid[0])-1):
                if not gridPoint.grid[i][j].is_solid():
                    gridPoint.grid[i][j].nextFrame1()
        for i in range(1,len(gridPoint.grid)-1):
            for j in range(1,len(gridPoint.grid[0])-1):
                if not gridPoint.grid[i][j].is_solid():
                    gridPoint.grid[i][j].nextFrame2()
        for i in range(1,len(gridPoint.grid)-1):
            for j in range(1,len(gridPoint.grid[0])-1):
                if not gridPoint.grid[i][j].is_solid():
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
        if (self.velocity[0] + self.velocity[1]) == 0:
            return
        percentageX = self.velocity[0] / (self.velocity[0] + self.velocity[1])
        # ensure indices in bounds and avoid moving into solid by counting solids as non-targets
        nx = self.x + gridPoint.sign(self.velocity[0])
        ny = self.y
        if 0 <= nx < len(gridPoint.grid) and 0 <= ny < len(gridPoint.grid[0]) and not gridPoint.grid[nx][ny].is_solid():
            gridPoint.grid[nx][ny].densityTemp += self.density * percentageX
        else:
            # if neighbor is solid or out of bounds, keep density (no flux)
            self.densityTemp += self.density * percentageX

        nx = self.x
        ny = self.y + gridPoint.sign(self.velocity[1])
        if 0 <= nx < len(gridPoint.grid) and 0 <= ny < len(gridPoint.grid[0]) and not gridPoint.grid[nx][ny].is_solid():
            gridPoint.grid[nx][ny].densityTemp += self.density * (1-percentageX)
        else:
            self.densityTemp += self.density * (1-percentageX)
    
    def nextFrame3(self):
        self.density = self.densityTemp
        
    def calcPressureGradient(self):
        # treat solid neighbors as having same density as self to prevent flow into solids
        x = self.x
        y = self.y
        def neighbor_density(nx, ny):
            if gridPoint.grid[nx][ny].is_solid():
                return self.density
            return gridPoint.grid[nx][ny].density
        gx = neighbor_density(x+1,y) - neighbor_density(x-1,y)
        gy = neighbor_density(x,y+1) - neighbor_density(x,y-1)
        return np.array([gx, gy], dtype=float)

    def calcPressure(self):
        if self.density == 0:
            return np.array([0.0, 0.0], dtype=float)
        return self.calcPressureGradient() / self.density
    
    def calcGravity(self):
        return gridPoint.gravity
    
    def calcViscosity(self):
        x = self.x
        y = self.y
        # neighbor velocities: gridPointSolid.velocity is zero so this naturally enforces no-slip
        v = gridPoint.viscosityCoefficient * (
            gridPoint.grid[x+1][y].velocity + gridPoint.grid[x-1][y].velocity +
            gridPoint.grid[x][y+1].velocity + gridPoint.grid[x][y-1].velocity -
            4*gridPoint.grid[x][y].velocity
        )
        return v

    def calcAcceleration(self):
        return -self.calcPressure() + self.calcGravity() + self.calcViscosity()

class gridPointSolid(gridPoint):
    def __init__(self, density, velocity, x, y):
        # solids keep zero velocity vector and zero acceleration
        self.density = density
        self.densityTemp = 0
        self.velocity = np.array([0.0, 0.0], dtype=float)
        self.acceleration = np.array([0.0, 0.0], dtype=float)
        self.x = x
        self.y = y

    def is_solid(self):
        return True

    # solids do not participate in the advancing steps
    def nextFrame1(self):
        pass
    def nextFrame2(self):
        pass
    def nextFrame3(self):
        pass

# Du/Dt + (1/density) * grad(pressure) = gravity + viscosity * Laplacian(velocity)
# Du/Dt is the material Derivative = dU/dt + U * grad(U)
# ...existing code...