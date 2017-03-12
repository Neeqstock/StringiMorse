def findVertex(cPoint: (float, float), centerDist: float, xDist: float, yDist: float, side: float ) -> float(2):
    delta = (centerDist ** 2) + (side ** 2)
    lower = 2 * side
    x = (delta - (xDist ** 2)) / lower
    y = (delta - (yDist ** 2)) / lower
    return x + cPoint[0], y + cPoint[1]

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

def findVertex2(cPoint: (float, float), yPoint: (float, float), xPoint: (float, float), cDist: float, xDist: float, yDist: float, side: float) -> float(2):

    ex = [0.0, 0.0]
    ey = [0.0, 0.0]
    i = [0.0, 0.0]
    j = [0.0, 0.0]

    #ex = (X - c) / ‖X - c‖
    # d = ‖X - c‖
    d = ((xPoint[0] - cPoint[0])**2 + (xPoint[1] - cPoint[1])**2)**0.5
    ex[0] = (xPoint[0] - cPoint[0]) / d
    ex[1] = (xPoint[1] - cPoint[1]) / d
    #ex,x = (Xx - cx) / sqrt( (Xx - cx)2 + (Xy - cy)2)
    #ex,y = (Xy - cy) / sqrt((Xx - cx)2 + (Xy - cy)2)
    fex = (ex[0] ** 2 + ex[1] ** 2) ** 0.5

    print("ex = %s , %s" % (ex[0], ex[1]))
    print("dx = ",(xPoint[0] - cPoint[0])**2,", dy= ",(xPoint[1] - cPoint[1])**2, " = ", d)

    #i = ex(Y - c)
    i[0] = ex[0]*(yPoint[0] - cPoint[0])
    i[1] = ex[1]*(yPoint[1] - cPoint[1])
    fI = (i[0]**2 + i[1]**2)**0.5
    print("i = %s , %s = %s" % (i[0], i[1], fI))

    #ey = (Y - c - i · ex) / ‖Y - c - i · ex‖
    eyAbs = ((yPoint[0] - cPoint[0] - i[0]*ex[0])**2 + (yPoint[1] - cPoint[1] - i[1]*ex[1])**2)**0.5
    ey[0] = (yPoint[0] - cPoint[0] - i[0]*ex[0]) / eyAbs
    ey[1] = (yPoint[1] - cPoint[1] - i[1]*ex[1]) / eyAbs
    fey = (ey[0] ** 2 + ey[1] ** 2) ** 0.5
    print("ey = %s , %s" % (ey[0], ey[1]))

    #j = ey(Y - c)
    j[0] = ey[0] * (yPoint[0] - cPoint[0])
    j[1] = ey[1] * (yPoint[1] - cPoint[1])
    fj = (j[0]**2 + j[1]**2)**0.5
    print("j = %s , %s = %s" % (j[0], j[1], fj))

    #x = (cdist^2 - xdist^2 + d^2) / 2*d

    x = (cDist**2 - xDist**2 + d**2) / (2*d)

    print("x = ", x)

    #y = (cdist^2 - ydist^2 + i^2 + j^2) / 2*j - i*x / j

    y = ((cDist**2 - yDist**2 + fI**2 + fj**2) / (2*fj)) - ((fI*x) / fj)

    print("y = ", y)

    #return x*ex[0] + cPoint[0], y*ey[1] + cPoint[1]
    return x*fex + cPoint[0], y*fey + cPoint[1]