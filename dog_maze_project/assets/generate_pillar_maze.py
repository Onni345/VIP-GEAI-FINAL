#!/usr/bin/env python3
"""
generate_pillar_maze.py
───────────────────────────────────────────────────────────────
For every 1 in MAZE_GRID ➜ place one square pillar geom.
For every 0            ➜ leave empty.
Produces maze.xml in the same folder.
"""

from pathlib import Path

# ───────────── EDIT YOUR GRID HERE ─────────────
MAZE_GRID = [
  [0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
  [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1],
  [1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1],
  [1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1],
  [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1],
  [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1],
  [1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
  [1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
  [1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1],
  [1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
  [1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1],
  [1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1],
  [1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1],
  [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1],
  [1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1],
  [1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
  [1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1],
  [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1],
  [1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1],
  [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1],
  [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1],
]


# ───────────────────────────────────────────────

CELL_SIZE      = 0.6   # metres between cell centres
PILLAR_WIDTH   = 0.6   # full pillar width  (x & y)  – set =CELL_SIZE for gap-free walls
WALL_HEIGHT    = 1.0   # pillar height (m)
OUT_FILE       = "maze.xml"


def rc_to_xy(r, c, rows, cols, cell=CELL_SIZE):
    """Row/col ➜ world-space (x,y) centred on 0,0."""
    x0 = -(cols * cell) / 2 + cell / 2
    y0 =  (rows * cell) / 2 - cell / 2
    return x0 + c * cell, y0 - r * cell


def make_pillar_geoms(grid):
    rows, cols = len(grid), len(grid[0])
    half_w = PILLAR_WIDTH / 2
    geoms = []
    for r in range(rows):
        for c in range(cols):
            if grid[r][c]:
                x, y = rc_to_xy(r, c, rows, cols)
                geoms.append(
                    f'    <geom name="p_{r}_{c}" type="box" '
                    f'pos="{x:.3f} {y:.3f} {WALL_HEIGHT/2}" '
                    f'size="{half_w:.3f} {half_w:.3f} {WALL_HEIGHT/2}" '
                    f'material="wall_mat"/>')
    return "\n".join(geoms)


def write_xml(geoms, fname=OUT_FILE):
    xml = f'''<?xml version="1.0"?>
<mujoco model="pillar_maze">
  <compiler angle="radian" coordinate="local"/>
  <option gravity="0 0 -9.81" timestep="0.01" integrator="RK4"/>

  <default>
    <geom contype="1" conaffinity="1" friction="1 0.1 0.1" rgba="0.45 0.45 0.6 1"/>
  </default>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="512"/>
    <texture name="floor_tex" type="2d" builtin="checker" rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3" width="512" height="512"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="6 6" reflectance="0.2"/>
    <material name="wall_mat"  rgba="0.55 0.6 0.75 1"/>
  </asset>

  <worldbody>
    <geom name="floor" type="plane" pos="0 0 -0.02" size="6 6 0.1" material="floor_mat"/>

{geoms}

    <light diffuse="0.9 0.9 0.9" pos="0 0 6" dir="0 0 -1"/>
  </worldbody>
</mujoco>
'''
    Path(fname).write_text(xml)
    print(f"✅  Wrote {fname}")


if __name__ == "__main__":
    write_xml(make_pillar_geoms(MAZE_GRID))
