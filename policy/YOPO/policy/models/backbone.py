import time
import torch
import torch.nn as nn
from config.config import cfg


class PTv3Block(nn.Module):
    def __init__(self, hidden_dim: int, num_heads: int, mlp_ratio: float = 4.0):
        super().__init__()
        self.norm1 = nn.LayerNorm(hidden_dim)
        self.attn = nn.MultiheadAttention(hidden_dim, num_heads, batch_first=True)
        self.norm2 = nn.LayerNorm(hidden_dim)
        mlp_hidden = int(hidden_dim * mlp_ratio)
        self.mlp = nn.Sequential(
            nn.Linear(hidden_dim, mlp_hidden),
            nn.GELU(),
            nn.Linear(mlp_hidden, hidden_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x_norm = self.norm1(x)
        attn_out, _ = self.attn(x_norm, x_norm, x_norm, need_weights=False)
        x = x + attn_out
        x = x + self.mlp(self.norm2(x))
        return x


class PointTransformerV3Backbone(nn.Module):
    """
    PTv3-style lightweight point backbone:
    - point token embedding
    - stacked transformer encoder blocks
    - learnable lattice queries cross-attend to point tokens
    - output reshaped to [B, C, vertical_num, horizon_num] for YOPO head
    """

    def __init__(self, output_dim: int, in_channels: int = 4, num_layers: int = 4, num_heads: int = 8):
        super().__init__()
        self.vertical_num = int(cfg["vertical_num"])
        self.horizon_num = int(cfg["horizon_num"])
        self.traj_num = self.vertical_num * self.horizon_num

        self.point_embed = nn.Sequential(
            nn.Linear(in_channels, output_dim),
            nn.GELU(),
            nn.Linear(output_dim, output_dim),
        )
        self.pos_embed = nn.Sequential(
            nn.Linear(3, output_dim),
            nn.GELU(),
            nn.Linear(output_dim, output_dim),
        )
        self.blocks = nn.ModuleList([
            PTv3Block(output_dim, num_heads=num_heads) for _ in range(num_layers)
        ])
        self.query_tokens = nn.Parameter(torch.randn(1, self.traj_num, output_dim) * 0.02)
        self.query_norm = nn.LayerNorm(output_dim)
        self.cross_attn = nn.MultiheadAttention(output_dim, num_heads, batch_first=True)
        self.output_proj = nn.Linear(output_dim, output_dim)

    def forward(self, points: torch.Tensor) -> torch.Tensor:
        # points: [B, N, C], first three channels must be xyz
        xyz = points[..., :3]
        x = self.point_embed(points) + self.pos_embed(xyz)
        for block in self.blocks:
            x = block(x)

        query = self.query_tokens.expand(points.shape[0], -1, -1)
        q, _ = self.cross_attn(self.query_norm(query), x, x, need_weights=False)
        query = query + q
        query = self.output_proj(query)
        return query.transpose(1, 2).contiguous().view(
            points.shape[0], -1, self.vertical_num, self.horizon_num
        )


def YopoBackbone(output_dim, in_channels=4):
    return PointTransformerV3Backbone(output_dim, in_channels=in_channels)


if __name__ == '__main__':
    net = YopoBackbone(64, in_channels=4)
    input_ = torch.zeros((1, 4096, 4))
    start = time.time()
    output = net(input_)
    print(f"Output shape: {output.shape}, Time: {time.time() - start:.4f}s")
