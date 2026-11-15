#!/usr/bin/env python3
"""离线运动学 + 射线仿真：在进 Gazebo 之前先校验控制律能否走出迷宫。

用同一份 LeftHandCore，配合自写的 360° 射线模型和差速车运动学积分，
可以把"整定参数"这件事从每轮 2~3 分钟的 Gazebo 里搬到 0.5 秒的脚本里。
"""

import math
import sys

import micromouse_maze.maze_model as mm
from micromouse_maze.left_hand_core import LeftHandCore, make_sectors

SEED = 4118


def build_maze(seed=SEED, n=5):
    mm.set_grid(n)
    m = mm.Maze.perfect(seed)
    m.open_entry_exit()
    return m


def simulate(seed=SEED, n=5, dt=0.05, noise=0.0, verbose=True, **kwargs):
    m = build_maze(seed, n)
    segs = mm.wall_segments(m)
    angles = [(-math.pi + 2 * math.pi * k / 360) for k in range(360)]
    sectors = make_sectors(-math.pi, 2 * math.pi / 360, 360)

    core = LeftHandCore(
        cell=mm.CELL,
        entry=mm.cell_center(mm.ENTRY),
        exit_cell=mm.cell_center(mm.EXIT),
        half=mm.HALF,
        **kwargs)

    x, y = mm.cell_center(mm.ENTRY)
    yaw = math.pi / 2.0        # 朝北进迷宫
    path = [(x, y)]
    collisions = 0
    cells = []
    t = 0.0
    last_phase = core.phase
    min_clear = 9.9

    for step in range(int(300 / dt)):
        # 雷达数据在机器人坐标系下，射线方向需叠加当前航向
        ranges = mm.raycast(segs, x, y, [a + yaw for a in angles], max_range=8.0)
        if noise:
            import random
            rng = random.Random(step)
            ranges = [max(0.05, r + rng.gauss(0, noise)) for r in ranges]
        scan = {'ranges': ranges, 'sectors': sectors}
        v, w, info = core.step(scan, (x, y, yaw), dt)

        dmin = min(ranges)
        min_clear = min(min_clear, dmin)
        if dmin < 0.09:
            collisions += 1

        if info.startswith('GO_') or info.startswith('TURN_') or info == 'EXIT_START':
            cell = (round((x - mm.X0) / mm.CELL), round((y - mm.Y0) / mm.CELL))
            if not cells or cells[-1] != cell:
                cells.append(cell)

        # 运动学积分（一阶差速近似，与仿真车行为同量级）
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = math.atan2(math.sin(yaw + w * dt), math.cos(yaw + w * dt))
        t += dt
        path.append((x, y))

        if core.phase != last_phase:
            if verbose:
                print('  [%6.2fs] %-6s -> %-6s  (% .2f, % .2f)' %
                      (t, last_phase, core.phase, x, y))
            last_phase = core.phase
        if core.phase in (core.DONE, core.TIMEOUT):
            break

    ok = core.phase == core.DONE
    if verbose:
        for e in core.events:
            print('   ', e)
    return {
        'ok': ok, 'time': t, 'distance': core.distance, 'collisions': collisions,
        'min_clear': min_clear, 'path': path, 'cells': cells,
        'turns': core.turn_count, 'phase': core.phase, 'maze': m, 'core': core,
    }


def main():
    print('=== 离线仿真：左手法则走迷宫 ===')
    r = simulate()
    print('\n结果: %s  用时 %.1f s  里程 %.2f m  转向 %d 次  最小间距 %.2f m  碰撞帧 %d'
          % ('成功出迷宫' if r['ok'] else '失败', r['time'], r['distance'],
             r['turns'], r['min_clear'], r['collisions']))
    print('路径格数', len(r['cells']))
    return 0 if r['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
