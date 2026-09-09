# __posecalc.py
# Computes fixture and placement poses using the primer's method.

import math
from robodk.robomath import *
from __points import POINTS


def get_point(key):
      """Returns just the (x, y, z) coordinates for a given point key."""
      entry = POINTS[key]
      return entry[3], entry[4], entry[5]


def compute_fixture_pose(fixture_name, world_second_key, local_second_key):
    fixture_points = [v for v in POINTS.values() if v[2] == fixture_name]

    world_origin = next(v for v in fixture_points if v[0] == "world" and v[1])
    local_origin = next(v for v in fixture_points if v[0] == "local" and v[1])

    world_second = POINTS[world_second_key]
    local_second = POINTS[local_second_key]

    tx, ty, tz = world_origin[3], world_origin[4], world_origin[5]

    world_dx = world_second[3] - world_origin[3]
    world_dy = world_second[4] - world_origin[4]
    local_dx = local_second[3] - local_origin[3]
    local_dy = local_second[4] - local_origin[4]

    cross = local_dx * world_dy - local_dy * world_dx
    dot   = local_dx * world_dx + local_dy * world_dy
    theta = math.atan2(cross, dot)

    return transl(tx, ty, tz) * rotz(theta)


def build_pose(rotation_columns, xyz):
      """
      Builds a 4x4 pose from three axis columns (each a length-3 list —
      where each child axis points, in parent coordinates) and an
      (x, y, z) translation. This is the "read the rotation matrix by
      inspection" method from the primer, Section 5.
      """
      col_x, col_y, col_z = rotation_columns
      pose = eye(4)
      for i in range(3):
            pose[i, 0] = col_x[i]
            pose[i, 1] = col_y[i]
            pose[i, 2] = col_z[i]
      pose[0, 3], pose[1, 3], pose[2, 3] = xyz
      return pose


def compute_placement_pose(fixture_name, point_key, axis_mapping, tool_point_key, tool_z_offset_deg):
      """
      Generic placement-pose calculator — usable by any task.

      fixture_name: which fixture's frame to chain from (e.g. "mazzer_scale")
      point_key: the POINTS key for the placement point, in that fixture's
                  local frame (e.g. 29 for the ball-bearing apex)
      axis_mapping: [col_x, col_y, col_z] — where each of the tool's own
                        axes points, in the fixture's local coordinates,
                        decided by inspection (see primer Section 5)

      Returns ᵁᴿT_placement — the final pose to send the robot's TCP to.
      """
      T_world_fixture = compute_fixture_pose(fixture_name)
      point_local = get_point(point_key)
      T_fixture_placement = build_pose(axis_mapping, point_local)
      T_world_target = T_world_fixture * T_fixture_placement

      # Account for the tool's own geometry: where its contact point sits
      # relative to the TCP, including the tool's mounting angle offset
      tool_point_local = get_point(tool_point_key)
      T_tool_offset = rotz(tool_z_offset_deg * pi/180) * transl(*tool_point_local)

      # Work backward: invert the tool's own offset to find where the
      # TCP itself needs to be, given where the contact point needs to be
      return T_world_target * invH(T_tool_offset)