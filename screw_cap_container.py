import os

import random
import math
import numpy as np

import cadquery as cq
from cadquery import Location as Loc, Vector as Vec

from cq_warehouse.thread import IsoThread

from util import get_center, get_bottom, knurl, cut_in_half


def find_closest_divisor(n, m):
    """Find the divisor of n closest to m"""
    divisors = np.array([i for i in range(1, int(np.sqrt(n) + 1)) if n % i == 0])
    divisions = n / divisors
    return divisions[np.argmin(np.abs(m - divisions))]


class ScrewCapContainer:
    innerDiameter: float
    wallThickness: float
    innerHeight: float
    capHeight: float
    capOffset: float
    dent: float
    threadPitch: float
    threadClearance: float
    turns: int
    knurledCap: bool

    def __init__(
        self,
        innerDiameter=40.0,
        wallThickness=2.0,
        innerHeight=50.0,
        capHeight=10.0,
        dent=5.0,
        threadClearance=0.6,
        turns=3,
        knurledCap=True,
    ):
        assert innerDiameter > 0
        assert wallThickness > 0
        assert innerHeight > 0
        assert capHeight > 0
        assert dent >= 0
        assert dent <= innerDiameter / 2

        assert capHeight - wallThickness < innerHeight + wallThickness

        self.innerDiameter = innerDiameter
        self.wallThickness = wallThickness
        self.innerHeight = innerHeight
        self.capHeight = capHeight
        self.dent = dent
        self.threadClearance = threadClearance
        self.turns = turns
        self.knurledCap = knurledCap
        self.threadPitch = (self.capHeight - self.wallThickness) / (self.turns + 1)
        self.capOffset = self.innerHeight + 2 * self.wallThickness - self.capHeight

    def make(self):
        if self.dent == 0.0:
            container = (
                cq.Workplane(origin=(0, 0, (self.innerHeight + self.wallThickness) / 2))
                .cylinder(
                    self.innerHeight + self.wallThickness,
                    self.innerDiameter / 2 + self.wallThickness,
                )
                .faces(">Z")
                .shell(-self.wallThickness)
            )
        else:
            container = (
                cq.Workplane("XY")
                .circle(self.innerDiameter / 2 + self.wallThickness)
                .circle(self.innerDiameter / 2)
                .extrude(self.innerHeight + self.wallThickness)
            )
            r = (
                -get_center(
                    (0, self.dent), (self.innerDiameter / 2, self.wallThickness)
                )
                + self.dent
            )
            b1 = get_bottom(self.dent, self.innerDiameter, self.wallThickness)
            b2 = get_bottom(self.dent - self.wallThickness, self.innerDiameter)
            container = container.union(b1.cut(b2))

        major_diam = self.innerDiameter + 3 * self.wallThickness

        container_thread = IsoThread(
            major_diameter=major_diam - (self.threadClearance / 2),
            pitch=self.threadPitch,
            length=self.capHeight - self.wallThickness,
            external=True,
            end_finishes=("fade", "fade"),
            hand="right",
        )

        major_diam += 2 * (
            (self.innerDiameter / 2) + self.wallThickness - container_thread.min_radius
        )
        container_thread = IsoThread(
            major_diameter=major_diam - (self.threadClearance / 2),
            pitch=self.threadPitch,
            length=self.capHeight - self.wallThickness,
            external=True,
            end_finishes=("fade", "fade"),
            hand="right",
        )

        cap_thread = IsoThread(
            major_diameter=major_diam + (self.threadClearance / 2),
            pitch=self.threadPitch,
            length=self.capHeight - self.wallThickness - self.threadPitch,
            external=False,
            end_finishes=("fade", "fade"),
            hand="right",
        )

        container_thread = container_thread.cq_object
        cap_thread = cap_thread.cq_object

        container = container.union(
            container_thread.translate(
                (0, 0, self.innerHeight + 2 * self.wallThickness - self.capHeight)
            )
        )

        cap = (
            cq.Workplane(origin=(0, 0, self.capHeight / 2))
            .cylinder(self.capHeight, major_diam / 2 + self.wallThickness)
            .faces("<Z")
            .shell(-self.wallThickness)
        )

        cap = (
            cap.union(cap_thread.translate((0, 0, self.threadPitch / 2)))
            .faces(">Z")
            .fillet(self.wallThickness)
        )

        if self.knurledCap:
            angle = 2 * math.degrees(math.asin(2 * 4 / self.innerDiameter))
            cap = knurl(
                cap,
                self.capHeight,
                major_diam / 2 + self.wallThickness,
                120,
                self.wallThickness / 3,
                40,
                360 // int(find_closest_divisor(360, angle)),
            )

        return container, cap


def create_stls():
    ds = list(range(15, 105, 5))
    wt = [1.5 if d < 50 else 2 for d in ds]
    hs = [15] + list(range(20, 110, 10))
    chs = [h / 3 if h < 50 else 10 for h in hs]
    n = len(ds) * len(hs)
    i = 1

    all = []
    pos = []

    for x, (d, t) in enumerate(zip(ds, wt)):
        for y, (h, ch) in enumerate(zip(hs, chs)):
            fn = "container_{}x{}_{}.stl".format(d, h, t)
            out_path = os.path.join("stl", fn)
            if not os.path.exists(out_path):
                config = ScrewCapContainer(
                    innerDiameter=d,
                    innerHeight=h,
                    wallThickness=t,
                    capHeight=ch,
                    dent=0.0,
                    knurledCap=True,
                )
                print("({}/{}) Making container: '{}' ({})".format(i, n, fn, ch))
                container, cap = config.make()

                all.append((container, cap))
                pos.append((x, y, config.innerDiameter, config.capOffset))

                i += 1

                stl = container.union(cap.translate((config.innerDiameter + 20, 0, 0)))
                cq.exporters.export(stl, os.path.join("stl", fn))
            else:
                print("{} already exists".format(out_path))

    return all, pos


config = ScrewCapContainer(wallThickness=1.5, knurledCap=False)
container, cap = config.make()

offset_base = config.innerDiameter + 20

split = [
    cut_in_half(container),
    cut_in_half(cap.translate((0, 0, config.capOffset))),
]
parts = [
    container.translate((1 * offset_base, 0, 0)),
    cap.translate((2 * offset_base, 0, 0)),
]

show_object(split)
show_object(parts)

cq.exporters.export(container.union(cap.translate((100, 0, 0))), "container.stl")

# all, pos = create_stls()
# for (container, cap), (x, y, innerDiameter, capOffset) in zip(all, pos):
#     base = (innerDiameter * 4 * x, 2 * innerDiameter * y, 0)
#     split = [
#         cut_in_half(container).translate(base),
#         cut_in_half(cap.translate((0, 0, capOffset))).translate(base),
#     ]
#     parts = [
#         container.translate((2 * innerDiameter + base[0], base[1], 0)),
#         cap.translate((4 * innerDiameter + base[0], base[1], 0)),
#     ]

#     show_object(split)
#     show_object(parts)
