import torch
from imu_cnn import IMU_CNN
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = IMU_CNN(num_classes=10).to(device)
model.load_state_dict(torch.load('imu_cnn.pth'))
# sets the module in eval node
model.eval()

test_data = torch.rand(1,6,200).to(device)

mod = torch.jit.trace(model, test_data)

mod.save("imu_cnn.pt")
