import torch
import numpy as np
from config.config import cfg
from policy.primitive import LatticePrimitive


class StateTransform:
    def __init__(self):
        self.lattice_primitive = LatticePrimitive.get_instance()
        self.goal_length = cfg['goal_length']

    def pred_to_endstate(self, endstate_pred: torch.Tensor) -> torch.Tensor:
        B, V, H = endstate_pred.shape[0], endstate_pred.shape[2], endstate_pred.shape[3]

        endstate_pred = endstate_pred.permute(0, 2, 3, 1).reshape(B, V * H, 9)

        yaw, pitch = self.lattice_primitive.getAngleLattice()
        yaw = yaw.flip(0)[None, :].expand(B, -1)
        pitch = pitch.flip(0)[None, :].expand(B, -1)
        Rbp = self.lattice_primitive.getRotation().flip(0)
        Rbp = Rbp[None, :, :, :].expand(B, -1, -1, -1)

        delta_yaw = endstate_pred[:, :, 0] * self.lattice_primitive.yaw_diff
        delta_pitch = endstate_pred[:, :, 1] * self.lattice_primitive.pitch_diff
        radio = (endstate_pred[:, :, 2] + 1.0) * self.lattice_primitive.radio_range

        cos_pitch = torch.cos(pitch + delta_pitch)
        endstate_x = cos_pitch * torch.cos(yaw + delta_yaw) * radio
        endstate_y = cos_pitch * torch.sin(yaw + delta_yaw) * radio
        endstate_z = torch.sin(pitch + delta_pitch) * radio
        endstate_p = torch.stack([endstate_x, endstate_y, endstate_z], dim=-1)

        endstate_vp = endstate_pred[:, :, 3:6] * self.lattice_primitive.vel_max
        endstate_ap = endstate_pred[:, :, 6:9] * self.lattice_primitive.acc_max

        endstate_vb = torch.matmul(Rbp, endstate_vp.unsqueeze(-1)).squeeze(-1)
        endstate_ab = torch.matmul(Rbp, endstate_ap.unsqueeze(-1)).squeeze(-1)

        endstate = torch.cat([endstate_p, endstate_vb, endstate_ab], dim=-1)

        endstate = endstate.permute(0, 2, 1).reshape(B, 9, V, H)
        return endstate

    def pred_to_endstate_cpu(self, endstate_pred: np.ndarray, lattice_id: torch.Tensor) -> np.ndarray:
        delta_yaw = endstate_pred[:, 0] * self.lattice_primitive.yaw_diff
        delta_pitch = endstate_pred[:, 1] * self.lattice_primitive.pitch_diff
        radio = (endstate_pred[:, 2] + 1.0) * self.lattice_primitive.radio_range

        yaw, pitch = self.lattice_primitive.getAngleLattice(lattice_id)
        yaw, pitch = yaw.cpu().numpy(), pitch.cpu().numpy()
        endstate_x = np.cos(pitch + delta_pitch) * np.cos(yaw + delta_yaw) * radio
        endstate_y = np.cos(pitch + delta_pitch) * np.sin(yaw + delta_yaw) * radio
        endstate_z = np.sin(pitch + delta_pitch) * radio
        endstate_p = np.stack((endstate_x, endstate_y, endstate_z), axis=1)

        endstate_vp = endstate_pred[:, 3:6] * self.lattice_primitive.vel_max
        endstate_ap = endstate_pred[:, 6:9] * self.lattice_primitive.acc_max

        Rpb = self.lattice_primitive.getRotation(lattice_id).cpu().numpy()
        endstate_vb = np.matmul(Rpb, endstate_vp[:, :, np.newaxis]).squeeze(-1)
        endstate_ab = np.matmul(Rpb, endstate_ap[:, :, np.newaxis]).squeeze(-1)

        return np.concatenate((endstate_p, endstate_vb, endstate_ab), axis=1)

    def prepare_input(self, obs):
        B, N = obs.shape[0], self.lattice_primitive.traj_num

        Rbp_all = self.lattice_primitive.getRotation().flip(0)

        obs = obs.view(B, 3, 3)

        obs_exp = obs[:, None, :, :].expand(B, N, 3, 3)
        Rbp_exp = Rbp_all[None, :, :, :].expand(B, N, 3, 3)

        transformed = torch.matmul(obs_exp, Rbp_exp)

        transformed_flat = transformed.view(B, N, 9)
        out = transformed_flat.permute(0, 2, 1).contiguous()
        out = out.view(B, 9, self.lattice_primitive.vertical_num, self.lattice_primitive.horizon_num)
        return out

    def unnormalize_obs(self, vel_acc):
        vel_acc[:, 0:3] = vel_acc[:, 0:3] * self.lattice_primitive.vel_max
        vel_acc[:, 3:6] = vel_acc[:, 3:6] * self.lattice_primitive.acc_max
        return vel_acc

    def normalize_obs(self, vel_acc_goal):
        vel_acc_goal[:, 0:3] = vel_acc_goal[:, 0:3] / self.lattice_primitive.vel_max
        vel_acc_goal[:, 3:6] = vel_acc_goal[:, 3:6] / self.lattice_primitive.acc_max

        goal_norm = vel_acc_goal[:, 6:9].norm(dim=1, keepdim=True)
        vel_acc_goal[:, 6:9] = vel_acc_goal[:, 6:9] / goal_norm.clamp(min=self.goal_length)
        return vel_acc_goal


def rotate_body2world(rot_wb, pos_b):
    pos_w = torch.matmul(rot_wb, pos_b.unsqueeze(-1)).squeeze(-1)
    return pos_w


def transform_body2world(rot_wb, t_w, pos_b):
    return rotate_body2world(rot_wb, pos_b) + t_w


def state_body2world(pos_w, rot_wb, pos_b, vel_b, acc_b):
    pos_b = transform_body2world(rot_wb, pos_w, pos_b)
    vel_b = rotate_body2world(rot_wb, vel_b)
    acc_b = rotate_body2world(rot_wb, acc_b)
    return pos_b, vel_b, acc_b


if __name__ == '__main__':
    CoordTransform = StateTransform()
