from pyproj import Transformer
import rasterio


class CoordinateConverter:

    def __init__(self, terrain):
        self.terrain = terrain

        self.to_map = Transformer.from_crs(
            "EPSG:4326",
            terrain.crs,
            always_xy=True
        )

        self.to_gps = Transformer.from_crs(
            terrain.crs,
            "EPSG:4326",
            always_xy=True
        )

    def gps_to_grid(self, latitude, longitude):

        x, y = self.to_map.transform(
            longitude,
            latitude
        )

        row, col = rasterio.transform.rowcol(
            self.terrain.transform,
            x,
            y
        )

        return row, col

    def grid_to_gps(self, row, col):

        x, y = rasterio.transform.xy(
            self.terrain.transform,
            row,
            col
        )

        longitude, latitude = self.to_gps.transform(x, y)

        return latitude, longitude