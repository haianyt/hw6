import numpy as np


def cross(v1, v2):
    return v1[0] * v2[1] - v1[1] * v2[0]


def isIntersect(edge1, edge2):
    A = edge1[0]
    B = edge1[1]
    C = edge2[0]
    D = edge2[1]
    A = np.array(A)
    B = np.array(B)
    C = np.array(C)
    D = np.array(D)

    AB = B - A
    AC = C - A
    AD = D - A

    flag1 = cross(AB,AC) * cross(AB,AD)

    CD = D - C
    CA = A - C
    CB = B - C

    flag2 = cross(CD,CA) * cross(CD,CB)

    if flag2<0 and flag2<0:
        return True
    else:
        return False

if __name__ == "__main__":
    print(isIntersect([(0,0),(1,1)],[(1,0),(0,10)]))