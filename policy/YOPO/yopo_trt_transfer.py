import os
import argparse
import time
import numpy as np
import torch
from torch2trt import torch2trt
from config.config import cfg
from policy.yopo_network import YopoNetwork


def parser():
    p = argparse.ArgumentParser()
    p.add_argument("--weight", type=str, required=True, help="path to trained .pth")
    p.add_argument("--dir", type=str, default='yopo_trt.pth', help="output TensorRT torch file")
    return p


if __name__ == "__main__":
    args = parser().parse_args()
    weight = args.weight

    print("Loading Network...")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    state_dict = torch.load(weight, weights_only=True)
    policy = YopoNetwork()
    policy.load_state_dict(state_dict)
    policy = policy.to(device)
    policy.eval()

    num_points = int(cfg.get("pointcloud_num_points", 4096))
    feature_dim = int(cfg.get("pointcloud_feature_dim", 4))
    sensor_input = np.zeros(shape=[1, num_points, feature_dim], dtype=np.float32)

    obs = np.zeros(shape=[1, 9, cfg["vertical_num"], cfg["horizon_num"]], dtype=np.float32)
    sensor_in = torch.from_numpy(sensor_input).to(device)
    obs_in = torch.from_numpy(obs).to(device)

    print("TensorRT Transfer...")
    model_trt = torch2trt(policy, [sensor_in, obs_in], fp16_mode=True)
    torch.save(model_trt.state_dict(), args.dir)

    print("Evaluation...")
    traj_trt, score_trt = model_trt(sensor_in, obs_in)
    traj, score = policy(sensor_in, obs_in)
    torch.cuda.synchronize()

    torch_start = time.time()
    traj, score = policy(sensor_in, obs_in)
    torch.cuda.synchronize()
    torch_end = time.time()

    trt_start = time.time()
    traj_trt, score_trt = model_trt(sensor_in, obs_in)
    torch.cuda.synchronize()
    trt_end = time.time()

    traj_error = torch.mean(torch.abs(traj - traj_trt))
    score_error = torch.mean(torch.abs(score - score_trt))

    print(f"Sensor: LiDAR point cloud, "
          f"Torch Latency: {1000 * (torch_end - torch_start):.3f} ms, "
          f"TensorRT Latency: {1000 * (trt_end - trt_start):.3f} ms, "
          f"Transfer Trajectory Error: {traj_error.item():.6f}, "
          f"Transfer Score Error: {score_error.item():.6f}")
