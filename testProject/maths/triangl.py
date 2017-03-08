def findVertex(cPoint: (float, float), centerDist: float, xDist: float, yDist: float, side: float ) -> float(2):
    delta = (centerDist ** 2) + (side ** 2)
    lower = 2 * side
    x = (delta - (xDist ** 2)) / lower
    y = (delta - (yDist ** 2)) / lower
    return (x + cPoint[0], y + cPoint[1])

def triLaterate(point1: (float, float), point2: (float, float), point3: (float,float), dist1: float, dist2: float, dist3: float) -> float(2):
    A = -2 * point1[0] + 2 * point2[0]
    B = -2 * point1[1] + 2 * point2[1]
    C = dist1 ** 2 - dist2 ** 2 - point1[0] ** 2 + point2[0] ** 2 - point1[1] ** 2 + point2[1] ** 2
    D = -2 * point2[0] + 2 * point3[0]
    E = -2 * point2[1] + 2 * point3[1]
    F = dist2 ** 2 - dist3 ** 2 - point2[0] ** 2 + point3[0] ** 2 - point2[1] ** 2 + point3[1] ** 2
    x = (C * D - F * A) / (B * D - E * A)
    y = (A * E - D * B) / (C * E - F * B)
    return (x, y)

