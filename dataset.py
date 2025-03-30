import os
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader


class IMUDataset(Dataset):
    def __init__(self, data_dir, label_list=range(10), window_size=200):  # 默认加载0-9
        self.data = []
        self.labels = []
        for label in label_list:
            dir_path = os.path.join(data_dir, str(label))
            if not os.path.exists(dir_path):
                continue  # 跳过不存在的类别目录
            for file in os.listdir(dir_path):
                if file.endswith('.npy'):
                    data = np.load(os.path.join(dir_path, file))  # (200,6)
                    data = data.transpose(1, 0)  # 转为 (6,200)
                    self.data.append(torch.FloatTensor(data))
                    self.labels.append(label)  # 直接使用整数标签（0-9）

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return self.data[idx], self.labels[idx]  # 标签已经是0-9的整数
