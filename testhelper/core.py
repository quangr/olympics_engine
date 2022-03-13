import math
def point2line(l1, l2, point):
    """
    :param l1: coord of line start point
    :param l2: coord of line end point
    :param point: coord of circle center
    :return:
    """

    l1l2 = [l2[0] - l1[0], l2[1]-l1[1]]
    l1c = [point[0]-l1[0], point[1]-l1[1]]

    cross_prod = abs(l1c[0]*l1l2[1] - l1c[1]*l1l2[0])

    l1l2_length = math.sqrt(l1l2[0]**2 + l1l2[1]**2)
    return cross_prod/l1l2_length

def cross_prod(v1, v2):
    return v1[0]*v2[1] - v1[1]*v2[0]

def line_intersect(line1, line2):       #[[x1,y1], [x2,y2]], https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect

    p = line1[0]
    r = [line1[1][0] - line1[0][0] , line1[1][1] - line1[0][1]]

    q = line2[0]
    s = [line2[1][0] - line2[0][0] , line2[1][1] - line2[0][1]]

    rs = cross_prod(r,s)
    if rs == 0:
        return False
    else:
        q_p = [q[0]-p[0], q[1]-p[1]]

        t = cross_prod(q_p, s)/rs
        u = cross_prod(q_p, r)/rs

        if 0<=t<=1 and 0<=u<=1:
            point = [p[0]+t*r[0], p[1]+t*r[1]]
            return True
        else:
            return False

def closest_point(l1, l2, point):
    """
    compute the coordinate of point on the line l1l2 closest to the given point, reference: https://en.wikipedia.org/wiki/Cramer%27s_rule
    :param l1: start pos
    :param l2: end pos
    :param point:
    :return:
    """
    A1 = l2[1] - l1[1]
    B1 = l1[0] - l2[0]
    C1 = (l2[1] - l1[1])*l1[0] + (l1[0] - l2[0])*l1[1]
    C2 = -B1 * point[0] + A1 * point[1]
    det = A1*A1 + B1*B1
    if det == 0:
        cx, cy = point
    else:
        cx = (A1*C1 - B1*C2)/det
        cy = (A1*C2 + B1*C1)/det

    return [cx, cy]
