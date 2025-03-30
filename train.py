import torch
import torch.optim as optim
import torch.nn as nn
from torch.utils.data import DataLoader
from dataset import IMUDataset
from imu_cnn import IMU_CNN

# 配置参数
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = IMU_CNN(num_classes=10).to(device)
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

train_dataset = IMUDataset(data_dir='dataset', label_list=range(10))
train_loader = DataLoader(train_dataset, batch_size=16, shuffle=True)

# 训练循环
for epoch in range(50):
    model.train()
    total_loss = 0.0
    for inputs, labels in train_loader:
        inputs = inputs.to(device)
        labels = torch.tensor(labels).to(device)  # 直接使用0-9的整数标签

        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()

    print(f'Epoch {epoch + 1}, Loss: {total_loss / len(train_loader):.4f}')

# 保存模型
torch.save(model.state_dict(), 'imu_cnn.pth')