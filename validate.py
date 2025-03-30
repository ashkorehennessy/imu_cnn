import numpy as np
from sklearn.metrics import confusion_matrix
import seaborn as sns
import matplotlib.pyplot as plt
import torch
from torch.utils.data import DataLoader
from dataset import IMUDataset
from imu_cnn import IMU_CNN
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')


def validate(model, data_dir):
    model.eval()
    test_dataset = IMUDataset(data_dir, label_list=range(10))  # 测试所有类别
    test_loader = DataLoader(test_dataset, batch_size=16)

    all_preds = []
    all_labels = []
    with torch.no_grad():
        for inputs, labels in test_loader:
            inputs = inputs.to(device)
            outputs = model(inputs)
            preds = torch.argmax(outputs, dim=1)
            all_preds.extend(preds.cpu().numpy())
            all_labels.extend(labels.numpy())  # 直接记录原始标签

    # 计算准确率
    accuracy = sum([1 if p == l else 0 for p, l in zip(all_preds, all_labels)]) / len(all_labels)
    print(f'Accuracy: {accuracy:.2f}')

    # 绘制10x10混淆矩阵
    cm = confusion_matrix(all_labels, all_preds)
    plt.figure(figsize=(12, 10))
    sns.heatmap(cm, annot=True, fmt='d',
                xticklabels=range(10),
                yticklabels=range(10))
    plt.xlabel('Predicted')
    plt.ylabel('True')
    plt.show()


model = IMU_CNN(num_classes=10).to(device)
model.load_state_dict(torch.load('imu_cnn.pth'))
validate(model, 'dataset')