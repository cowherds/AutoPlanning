import torch
from torch import nn
import numpy as np
from config.config import cfg
from policy.models.backbone import YopoBackbone
from policy.models.head import YopoHead
from policy.state_transform import *


class YopoNetwork(nn.Module):

    def __init__(
            self,
            observation_dim=9,
            output_dim=10,
            hidden_state=64,
    ):
        super(YopoNetwork, self).__init__()
        self.state_transform = StateTransform()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        in_channels = int(cfg.get("pointcloud_feature_dim", 4))
        self.sensor_backbone = YopoBackbone(hidden_state, in_channels=in_channels)
        self.state_backbone = nn.Sequential()
        self.yopo_head = YopoHead(hidden_state + observation_dim, output_dim)

    def forward(self, sensor_input: torch.Tensor, obs: torch.Tensor) -> torch.Tensor:
        sensor_feature = self.sensor_backbone(sensor_input)
        obs_feature = self.state_backbone(obs)
        input_tensor = torch.cat((obs_feature, sensor_feature), 1)
        output = self.yopo_head(input_tensor)
        endstate = torch.tanh(output[:, :9])
        score = torch.nn.functional.softplus(output[:, 9])
        return endstate, score

    def inference(self, sensor_input: torch.Tensor, obs: torch.Tensor) -> torch.Tensor:
        obs = self.state_transform.normalize_obs(obs)
        obs = self.state_transform.prepare_input(obs)
        endstate_pred, score_pred = self.forward(sensor_input, obs)
        endstate = self.state_transform.pred_to_endstate(endstate_pred)
        return endstate, score_pred

    def print_grad(self, grad):
        print("grad of hook: ", grad)
