class Position:
    __x = 0.0
    __y = 0.0
    __yaw = 0.0

    def __init__(self, x, y, yaw):
        self.__x = x
        self.__y = y
        self.__yaw = yaw

    def getX(self):
        return self.__x

    def getY(self):
        return self.__y

    def getYaw(self):
        return self.__yaw
