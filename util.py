import math

import cadquery as cq


def get_center(p1, p2):
    p1x, p1y = p1
    p2x, p2y = p2

    mx = (p1x + p2x) / 2
    my = (p1y + p2y) / 2

    a = -(p1x - p2x) / (p1y - p2y)
    b = my - a * mx

    return b


def get_bottom(dent, diameter, height=0.0):
    r = -get_center((0, dent), (diameter / 2, height)) + dent
    result = (
        cq.Workplane("XY")
        .sphere(r)
        .translate((0, 0, -r + dent))
        .intersect(
            cq.Workplane("XY")
            .cylinder(2 * dent, (diameter / 2))
            .translate((0, 0, dent))
        )
    )

    return result


def knurl(obj, height, radius, cut_angle, cut_depth, angle, n, point_radius=None):
    length = 2 * cut_depth
    x1 = length * math.cos(math.radians(cut_angle / 2))
    y1 = length * math.sin(math.radians(cut_angle / 2))

    def doprofile(loc):
        s = (
            cq.Sketch()
            .push([(-cut_depth, 0)])
            .segment((0, 0), (x1, y1))
            .segment((x1, -y1))
            .close()
            .assemble()
            .reset()
        )
        if point_radius:
            s = s.vertices("<X").fillet(point_radius).reset()
        return s.moved(loc)

    s1 = cq.Sketch().parray(radius, 0, 360, n).each(doprofile)

    w = cq.Workplane()
    w1 = w.placeSketch(s1).twistExtrude(height, angle)
    w2 = w.placeSketch(s1).twistExtrude(height, -angle)
    tool = w1.union(w2, clean=True, tol=1e-2)

    return obj.cut(tool)


def cut_in_half(obj):
    return (
        obj.faces("<Z")
        .workplane()
        .transformed(rotate=cq.Vector(-90, 0, 0))
        .split(keepBottom=True)
    )


def meshify(base, offset=2, mesh_th=4, r=1):
    bbox = base.val().BoundingBox()

    base_frame = (
        base.faces(">Z")
        .wires()
        .toPending()
        .workplane()
        .offset2D(-offset)
        .cutBlind("next")
    )

    x = r * math.cos(math.radians(30))
    xb = x + mesh_th / 2
    rb = xb * math.cos(math.radians(30))
    yb = rb * math.sin(math.radians(30))
    d = rb + 2 * yb

    xn = (bbox.xmax - bbox.xmin) / (2 * d)
    yn = (bbox.ymax - bbox.ymin) / xb

    mesh_points = []

    for x in range(0, math.ceil(xn)):
        for y in range(0, math.ceil(yn)):
            x_s = 0 if (y % 2) == 0 else d
            mesh_points.append(
                (bbox.xmin + (x * 2 * d) + x_s, bbox.ymin + 0.5 + (y * xb))
            )

    mesh = (
        base.faces(">Z")
        .workplane()
        .pushPoints(mesh_points)
        .polygon(6, r * 2)
        .cutThruAll()
    )

    return mesh.union(base_frame)


def meshify2(base, offset=5):
    bbox = base.val().BoundingBox()

    width = bbox.xmax - bbox.xmin
    height = bbox.ymax - bbox.ymin

    r = 2
    mesh_th = 2

    base_frame = (
        base.faces(">Z")
        .wires()
        .toPending()
        .workplane()
        .offset2D(-offset)
        .cutBlind("next")
    )

    x = r * math.cos(math.radians(30))
    xb = x + mesh_th / 2
    rb = xb * math.cos(math.radians(30))
    yb = rb * math.sin(math.radians(30))
    d = rb + 2 * yb

    off_x = width / (2 * d)
    off_y = height / xb
    off_x = off_x - int(off_x)
    off_y = off_y - int(off_y)
    off_x *= 2 * d
    off_y *= xb

    xn = width / (2 * d)
    yn = height / xb

    mesh_points = []

    for x in range(0, math.ceil(xn)):
        for y in range(0, math.ceil(yn)):
            x_s = 0 if (y % 2) == 0 else d
            mesh_points.append(
                (
                    bbox.xmin + (x * 2 * d) + x_s + off_x / 2,
                    bbox.ymin + (y * xb) + off_y / 2,
                )
            )

    mesh = (
        base.faces(">Z")
        .workplane()
        .pushPoints(mesh_points)
        .polygon(6, r * 2)
        .cutThruAll()
    )

    return mesh.union(base_frame)


def gen_slot_data(w, h, nh, a=None):
    assert nh > 1

    if a:
        ac = math.tan(math.radians(a - 90))
    else:
        ac = -w / h
        a = 90 - math.degrees(-math.atan(ac))

    dy = h / (nh + 1)
    dx = w / (nh + 1)

    pts = []

    for i in range(nh):
        pts.append(((w / 2) - dx * (i + 1), (h / 2) - dy * (i + 1)))

    def find_on_x(pt, x):
        return (x, ac * (x - pt[0]) + pt[1])

    def find_on_y(pt, y):
        return (pt[0] + ((y - pt[1]) / ac), y)

    def width(pt):
        ps = [
            find_on_x(pt, w / 2),
            find_on_x(pt, -w / 2),
            find_on_y(pt, h / 2),
            find_on_y(pt, -h / 2),
        ]

        ps = list(
            filter(
                lambda p: (p[0] <= w / 2)
                and (p[1] <= h / 2)
                and (p[0] >= -w / 2)
                and (p[1] >= -h / 2),
                ps,
            )
        )

        d = math.dist(ps[0], ps[1])
        return (((ps[0][0] + ps[1][0]) / 2), ((ps[0][1] + ps[1][1]) / 2)), d

    wd = math.hypot(dx, dy) / 2
    wd *= math.cos(math.radians(a) - math.atan(h / w))

    return a, wd, map(width, pts)


def slots(w, h, n, a=None, ratio=1.0):
    sk = cq.Sketch()
    a, sw, data = gen_slot_data(w, h, n, a)

    for c, w in data:
        sk = sk.push([c]).slot(w, sw * ratio, a + 90)

    return sk
