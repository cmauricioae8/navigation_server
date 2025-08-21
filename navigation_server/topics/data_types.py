class PointData:
    def __init__(self, x: float, y: float):
        self.x: float = x
        self.y: float = y

    def update(self, x: float, y: float):
        self.x = x
        self.y = y

    def to_dict(self) -> dict:
        return {
            "x": round(self.x, 2),
            "y": round(self.y, 2),
        }


class PoseData:
    def __init__(self, position_x: float, position_y: float, orientation: float):
        self.position_x: float = position_x
        self.position_y: float = position_y
        self.orientation: float = orientation

    def update(self, position_x: float, position_y: float, orientation: float):
        self.position_x = position_x
        self.position_y = position_y
        self.orientation = orientation

    def to_dict(self) -> dict:
        return {
            "position_x": round(self.position_x, 2),
            "position_y": round(self.position_y, 2),
            "orientation": round(self.orientation, 2),
        }
