#!/usr/bin/env python3
"""在随机迷宫里筛选"符合竞赛逻辑 + 左手法则可解且步数适中"的布局。"""
import random
import sys

from micromouse_maze.maze_model import (CELL, EXIT, ENTRY, N, Maze,
                                        left_hand_path)


def knock_random(maze, k, rng):
    """随机拆掉 k 段内墙，制造环路（真实竞赛迷宫的环）。"""
    cands = []
    for i in range(N):
        for j in range(N):
            for d in (0, 1):
                if maze.walls.get((i, j, d), True) and _safe(i, j, d):
                    cands.append((i, j, d))
    rng.shuffle(cands)
    done = 0
    for c in cands:
        if done >= k:
            break
        if maze.knock(c[:2], c[2]):
            done += 1
    return done


def _safe(i, j, d):
    # 不拆外墙（d=0 的东墙只有 i<N-1 才是内部墙，d=1 同理）
    if d == 0 and i == N - 1:
        return False
    if d == 1 and j == N - 1:
        return False
    return True


def score(maze):
    path, _cell, ok = left_hand_path(maze)
    steps = len(path) - 1
    if not ok:
        return None
    # 步数太少显得太简单，太多录像太长；目标 34~62 步
    if not (26 <= steps <= 52):
        return None
    de = len(maze.dead_ends())
    if de < 5:
        return None
    visits = len({p[0] for p in path})
    if visits < 13:
        return None
    return steps, de, maze.loops(), visits, path


def main():
    best = []
    for seed in range(1, 4000):
        rng = random.Random(seed)
        m = Maze.perfect(seed)
        knock_random(m, rng.choice([2, 3, 3, 4]), random.Random(seed * 7 + 1))
        m.open_entry_exit()
        s = score(m)
        if s is None:
            continue
        steps, de, loops, visits, path = s
        best.append((steps, seed, de, loops, visits))
    best.sort()
    print('candidates:', len(best))
    for b in best[:15]:
        print('steps=%3d seed=%4d dead_ends=%2d loops=%d visited=%2d' % b)
    if best:
        seed = best[0][1]
        rng = random.Random(seed)
        m = Maze.perfect(seed)
        knock_random(m, rng.choice([2, 3, 3, 4]), random.Random(seed * 7 + 1))
        m.open_entry_exit()
        path, _c, ok = left_hand_path(m); steps = len(path) - 1
        print('\n--- best seed', seed, '---')
        print(m.ascii([p[0] for p in path]))
        print('steps', steps, 'ok', ok)
        print('cells:', ' '.join('%s%s' % (c, 'NESW'[d]) for c, d in path))


if __name__ == '__main__':
    main()
