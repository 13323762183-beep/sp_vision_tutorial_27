#!/usr/bin/env python3
"""原版 SimRobot/二进制动力学 + 生产 C++ 控制器/A* 的闭环回归。

只通过控制器输出驱动 SimRobot；不修改仿真器，不发布 cmd_vel。
数据和日志输出到指定目录。时间为仿真步进时间，不是完整 BT 评测用时。
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/sp_nav_sim'))
from sp_nav_sim.sim.sim_map import load_sim_map
from sp_nav_sim.sim.sim_robot import SimRobot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', required=True)
    parser.add_argument('--params', default=str(ROOT / 'src/sp_nav_bringup/config/nav_params.yaml'))
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=240.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(Path(args.params).read_text())
    params = config['esdf_map_publisher']['ros__parameters']
    sim = yaml.safe_load((ROOT / 'src/sp_nav_sim/config/sim_robot.yaml').read_text())['sim_robot_node']['ros__parameters']
    _, occ, meta, _ = load_sim_map(str(ROOT / 'src' / sim['map_yaml']), sim['treat_unknown_as_occupied'])
    h, w = occ.shape
    distance = cv2.distanceTransform(np.where(occ, 0, 255).astype(np.uint8), cv2.DIST_L2, 5) * meta.resolution
    costs = np.minimum(99, np.rint(100 * (1 - np.exp(-params['w'] * np.maximum(0, params['d_safe'] - distance)**2))))
    costs[distance < params['robot_radius'] + params['margin']] = 100
    costs = np.flipud(costs).astype(int)
    common = dict(v_max=sim['v_max'], a_max=sim['a_max'], robot_radius=sim['robot_radius'],
                  map_occ=occ, meta=meta, map_w=w, map_h=h, sim_hz=30.0, scan_speed=sim['scan_speed'])
    robot = SimRobot(robot_id=1, init_pos=np.array([sim['init_x'], sim['init_y']]),
                     tf_child_frame_id='base_link', **common)
    robot.yaw = args.yaw
    peers = []
    for i, (x, y) in enumerate(zip(sim['init_xs'].split(','), sim['init_ys'].split(','))):
        peers.append(SimRobot(robot_id=i+2, init_pos=np.array([float(x), float(y)]),
                              tf_child_frame_id=f'base_link_{i+2}', **common))
    all_robots = [robot] + peers
    for r in all_robots:
        r.peers = [p for p in all_robots if p is not r]
    env = dict(os.environ, ROS_LOG_DIR=str(output / 'ros_logs'), ROS_HOME=str(output / 'ros_home'))
    log = (output / 'probe.log').open('w')
    proc = subprocess.Popen([args.probe, '--ros-args', '--params-file', args.params,
                             '--log-level', 'warn', '--disable-stdout-logs'],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                            text=True, env=env)

    def request(op, t):
        pos, vel, yaw = robot.snapshot()
        proc.stdin.write(f'{op} {t+1:.9f} {pos[0]} {pos[1]} {yaw} {vel[0]} {vel[1]}\n')
        proc.stdin.flush()
        prefix = 'PATH' if op == 'plan' else 'CMD'
        while True:
            line = proc.stdout.readline()
            if not line:
                raise RuntimeError('C++ probe exited; inspect probe.log')
            if line.startswith(prefix + ' '):
                return np.fromstring(line[len(prefix):], sep=' ')

    trajectory, reference, latest = [], None, None
    collision_steps = 0
    reached_time = None
    min_distance = float('inf')
    try:
        proc.stdin.write(f'{w} {h} {meta.resolution} {meta.origin[0]} {meta.origin[1]}\n')
        proc.stdin.write(' '.join(map(str, costs.ravel())) + '\n')
        proc.stdin.flush()
        for step in range(int(args.duration * 30)):
            t = step / 30.0
            if step % 3 == 0 and reached_time is None:
                result = request('plan', t)
                latest = result[1:].reshape(-1, 2)
                if len(latest) < 2:
                    raise RuntimeError(f'A* failed at t={t:.2f}')
                if reference is None:
                    reference = latest.copy()
                    np.savetxt(output / 'reference.csv', reference, delimiter=',', header='x,y')
            command = request('step', t)
            robot.on_cmd_vel(*command, int((t+1)*1e9))
            for r in all_robots:
                r.step(int((t+1)*1e9))
            pos, vel, yaw = robot.snapshot()
            collision_steps += int(robot._wall_contact)
            goal_distance = float(np.linalg.norm(pos - [14.1, 14.1]))
            min_distance = min(min_distance, goal_distance)
            a, b = reference[:-1], reference[1:]
            v = b-a
            u = np.clip(np.sum((pos-a)*v, axis=1)/np.maximum(np.sum(v*v, axis=1), 1e-12), 0, 1)
            error = float(np.min(np.linalg.norm(pos - (a+u[:, None]*v), axis=1)))
            trajectory.append([t, *pos, *vel, yaw, *command, error, goal_distance, int(robot._wall_contact)])
            if reached_time is None and goal_distance < .05:
                reached_time = t
            if step % 600 == 0:
                print(f't={t:.1f}s pos={pos.round(3)} goal={goal_distance:.3f} error={error:.3f} contacts={collision_steps}', flush=True)
            if reached_time is not None and t > reached_time + 3:
                break
    finally:
        proc.stdin.close()
        proc.wait(timeout=10)
        log.close()
    with (output / 'trajectory.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'x', 'y', 'vx', 'vy', 'yaw', 'cmd_x', 'cmd_y', 'path_error', 'goal_distance', 'contact'])
        writer.writerows(trajectory)
    rows = np.asarray(trajectory)
    summary = dict(reached=reached_time is not None, arrival_sim_seconds=reached_time,
                   final_error_m=float(rows[-1, 9]), min_goal_error_m=min_distance,
                   tracking_rmse_m=float(np.sqrt(np.mean(rows[:, 8]**2))),
                   tracking_max_m=float(rows[:, 8].max()), collision_steps=collision_steps,
                   final_speed_mps=float(np.linalg.norm(rows[-1, 3:5])),
                   samples=len(rows), initial_yaw=args.yaw,
                   scope='Production C++ A*/controller with unmodified SimRobot and binary plant; no DDS/BT/GUI')
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary['reached'] and collision_steps == 0 and summary['final_error_m'] < .05 else 1


if __name__ == '__main__':
    sys.exit(main())
