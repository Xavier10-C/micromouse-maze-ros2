#!/usr/bin/env python3
"""按 maze_model 的网格布局生成 Gazebo .world（含墙体 collision / visual）。

用法：
    python3 gen_maze_world.py [输出路径] [--seed N] [--n N]

默认输出 worlds/maze.world，同时把布局元数据写到 worlds/maze_layout.json，
供报告里的布局示意图与统计表使用（墙段数、死胡同数、左手法则步数等）。
"""

import argparse
import json
import os
import sys

from micromouse_maze import maze_model as mm

DEFAULT_SEED = 4118     # 经离线左手法则校验通过的布局种子
DEFAULT_N = 5


def build(seed=DEFAULT_SEED, n=DEFAULT_N):
    mm.set_grid(n)
    maze = mm.Maze.perfect(seed)
    maze.open_entry_exit()
    return maze


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out', nargs='?', default=None, help='输出 .world 路径')
    ap.add_argument('--seed', type=int, default=DEFAULT_SEED)
    ap.add_argument('--n', type=int, default=DEFAULT_N)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    out = args.out or os.path.join(here, '..', 'worlds', 'maze.world')
    out = os.path.abspath(out)
    os.makedirs(os.path.dirname(out), exist_ok=True)

    maze = build(args.seed, args.n)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(mm.build_world_sdf(maze))

    path, cell, ok = mm.left_hand_path(maze)
    meta = mm.layout_json(maze, {
        'seed': args.seed,
        'left_hand_steps': len(path) - 1,
        'left_hand_solvable': ok,
        'cell_sequence': [[c[0], c[1], 'ENWS'[d]] for c, d in path],
        'entry_center': list(mm.cell_center(mm.ENTRY)),
        'exit_center': list(mm.cell_center(mm.EXIT)),
        'half': mm.HALF,
    })
    meta_path = os.path.join(os.path.dirname(out), 'maze_layout.json')
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)

    print('[gen_maze_world] 已生成 %s' % out)
    print('[gen_maze_world] %dx%d 格 / 单元 %.2f m / 墙厚 %.2f m / 墙高 %.2f m'
          % (mm.N, mm.N, mm.CELL, mm.THICK, mm.HEIGHT))
    print('[gen_maze_world] 内部墙 %d 段，死胡同 %d 个，左手法则 %d 步，可解=%s'
          % (maze.n_walls(), len(maze.dead_ends()), len(path) - 1, ok))
    print('[gen_maze_world] 布局元数据 -> %s' % meta_path)
    if not ok:
        print('[gen_maze_world] 警告：该布局左手法则无法出迷宫，请更换 seed',
              file=sys.stderr)


if __name__ == '__main__':
    main()
