from __points import POINTS

world_origin = POINTS[3]
world_second = POINTS[5]
local_origin = POINTS[21]
local_second = POINTS[23]

print("world_origin:", world_origin)
print("world_second:", world_second)
print("local_origin:", local_origin)
print("local_second:", local_second)

world_dx = world_second[3] - world_origin[3]
world_dy = world_second[4] - world_origin[4]
local_dx = local_second[3] - local_origin[3]
local_dy = local_second[4] - local_origin[4]

print("world_dx, world_dy:", world_dx, world_dy)
print("local_dx, local_dy:", local_dx, local_dy)

import math
world_len = math.hypot(world_dx, world_dy)
local_len = math.hypot(local_dx, local_dy)
print("world_len:", world_len, "| local_len:", local_len)

cross = local_dx * world_dy - local_dy * world_dx
dot   = local_dx * world_dx + local_dy * world_dy
theta = math.atan2(cross, dot)
print("theta (deg):", math.degrees(theta))