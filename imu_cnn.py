import torch
import torch.nn as nn


class IMU_CNN(nn.Module):
    def __init__(self, input_size=200, num_classes=10):
        super().__init__()
        self.features = nn.Sequential(
            # 输入形状: (batch, 6, 200)
            nn.Conv1d(in_channels=6, out_channels=16, kernel_size=5, stride=1, padding=2),
            nn.BatchNorm1d(16),
            nn.ReLU(),
            nn.MaxPool1d(kernel_size=2),  # 输出: (16, 100)

            nn.Conv1d(16, 32, kernel_size=3, stride=1, padding=1),
            nn.BatchNorm1d(32),
            nn.ReLU(),
            nn.MaxPool1d(2),  # 输出: (32, 50)

            nn.Dropout(0.3)
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(32 * 50, 64),
            nn.ReLU(),
            nn.Linear(64, num_classes)
        )

    def forward(self, x):
        x = self.features(x)
        return self.classifier(x)
