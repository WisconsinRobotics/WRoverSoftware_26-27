import rasterio
import numpy as np


class TerrainMap:

    def __init__(self, dem_file):
        self.dem = rasterio.open(dem_file)

        self.elevation = self.dem.read(1)
        self.transform = self.dem.transform
        self.crs = self.dem.crs
        self.resolution = self.transform.a

        self.calculate_slope()

        self.maxIncline = 10

    def calculate_slope(self):
        dy, dx = np.gradient(
            self.elevation,
            self.resolution
        )

        self.slope = np.degrees(
            np.arctan(np.sqrt(dx**2 + dy**2))
        )

    def is_traversable(self, row, col):
        return self.slope[row, col] <= self.maxIncline

    def get_elevation(self, row, col):
        return self.elevation[row, col]