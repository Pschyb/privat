"""Small, Python 3-safe wrapper around ``nav_msgs/OccupancyGrid``."""

import io

from PIL import Image


class MyGridMap:
    def __init__(self, grid):
        self.occupancyGrid = grid
        self.mapWidth = int(grid.info.width)
        self.mapHeight = int(grid.info.height)
        self.lethalCost = 100

    def getWidth(self):
        return self.mapWidth

    def getHeight(self):
        return self.mapHeight

    def getSize(self):
        return self.mapHeight * self.mapWidth

    def getResolution(self):
        return self.occupancyGrid.info.resolution

    def getOriginX(self):
        return self.occupancyGrid.info.origin.position.x

    def getOriginY(self):
        return self.occupancyGrid.info.origin.position.y

    def getOccupancyGrid(self):
        return self.occupancyGrid

    def getCoordinates(self, index):
        if index < 0 or index >= self.getSize():
            return -1, -1
        return index % self.mapWidth, index // self.mapWidth

    def getIndex(self, x, y):
        x = int(x)
        y = int(y)
        if not self._in_bounds(x, y):
            return -1
        return y * self.mapWidth + x

    def getData(self, x, y=None):
        if y is None:
            index = int(x)
            if index < 0 or index >= self.getSize():
                return -1
        else:
            index = self.getIndex(x, y)
            if index == -1:
                return -1
        return self.occupancyGrid.data[index]

    def isFree(self, x, y=None):
        value = self.getData(x, y)
        return 0 <= value < self.lethalCost

    def isFrontier(self, x, y=None):
        if y is None:
            x, y = self.getCoordinates(int(x))
        if x == -1:
            return False
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if (dx or dy) and self.getData(x + dx, y + dy) == -1:
                    return True
        return False

    def createPGM(self):
        stream = io.BytesIO()
        stream.write(f"P5\n{self.mapWidth} {self.mapHeight}\n255\n".encode())
        for y in range(self.mapHeight):
            for x in range(self.mapWidth):
                index = x + (self.mapHeight - y - 1) * self.mapWidth
                value = self.occupancyGrid.data[index]
                pixel = 0 if value == 100 else 254 if value >= 0 else 205
                stream.write(bytes([pixel]))
        stream.seek(0)
        with Image.open(stream) as source:
            return source.copy()

    def areNeighbour(self, index1, index2):
        x1, y1 = self.getCoordinates(index1)
        x2, y2 = self.getCoordinates(index2)
        if x1 == -1 or x2 == -1:
            return False
        return max(abs(x1 - x2), abs(y1 - y2)) <= 1

    def _in_bounds(self, x, y):
        return 0 <= x < self.mapWidth and 0 <= y < self.mapHeight
