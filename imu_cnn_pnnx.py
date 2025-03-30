import os
import numpy as np
import tempfile, zipfile
import torch
import torch.nn as nn
import torch.nn.functional as F
try:
    import torchvision
    import torchaudio
except:
    pass

class Model(nn.Module):
    def __init__(self):
        super(Model, self).__init__()

        self.convbn1d_0 = nn.Conv1d(bias=True, dilation=(1,), groups=1, in_channels=6, kernel_size=(5,), out_channels=16, padding=(2,), padding_mode='zeros', stride=(1,))
        self.features_2 = nn.ReLU()
        self.features_3 = nn.MaxPool1d(ceil_mode=False, dilation=(1,), kernel_size=(2,), padding=(0,), return_indices=False, stride=(2,))
        self.convbn1d_1 = nn.Conv1d(bias=True, dilation=(1,), groups=1, in_channels=16, kernel_size=(3,), out_channels=32, padding=(1,), padding_mode='zeros', stride=(1,))
        self.features_6 = nn.ReLU()
        self.features_7 = nn.MaxPool1d(ceil_mode=False, dilation=(1,), kernel_size=(2,), padding=(0,), return_indices=False, stride=(2,))
        self.classifier_1 = nn.Linear(bias=True, in_features=1600, out_features=64)
        self.classifier_2 = nn.ReLU()
        self.classifier_3 = nn.Linear(bias=True, in_features=64, out_features=10)

        archive = zipfile.ZipFile('imu_cnn.pnnx.bin', 'r')
        self.convbn1d_0.bias = self.load_pnnx_bin_as_parameter(archive, 'convbn1d_0.bias', (16), 'float32')
        self.convbn1d_0.weight = self.load_pnnx_bin_as_parameter(archive, 'convbn1d_0.weight', (16,6,5), 'float32')
        self.convbn1d_1.bias = self.load_pnnx_bin_as_parameter(archive, 'convbn1d_1.bias', (32), 'float32')
        self.convbn1d_1.weight = self.load_pnnx_bin_as_parameter(archive, 'convbn1d_1.weight', (32,16,3), 'float32')
        self.classifier_1.bias = self.load_pnnx_bin_as_parameter(archive, 'classifier.1.bias', (64), 'float32')
        self.classifier_1.weight = self.load_pnnx_bin_as_parameter(archive, 'classifier.1.weight', (64,1600), 'float32')
        self.classifier_3.bias = self.load_pnnx_bin_as_parameter(archive, 'classifier.3.bias', (10), 'float32')
        self.classifier_3.weight = self.load_pnnx_bin_as_parameter(archive, 'classifier.3.weight', (10,64), 'float32')
        archive.close()

    def load_pnnx_bin_as_parameter(self, archive, key, shape, dtype, requires_grad=True):
        return nn.Parameter(self.load_pnnx_bin_as_tensor(archive, key, shape, dtype), requires_grad)

    def load_pnnx_bin_as_tensor(self, archive, key, shape, dtype):
        fd, tmppath = tempfile.mkstemp()
        with os.fdopen(fd, 'wb') as tmpf, archive.open(key) as keyfile:
            tmpf.write(keyfile.read())
        m = np.memmap(tmppath, dtype=dtype, mode='r', shape=shape).copy()
        os.remove(tmppath)
        return torch.from_numpy(m)

    def forward(self, v_0):
        v_1 = self.convbn1d_0(v_0)
        v_2 = self.features_2(v_1)
        v_3 = self.features_3(v_2)
        v_4 = self.convbn1d_1(v_3)
        v_5 = self.features_6(v_4)
        v_6 = self.features_7(v_5)
        v_7 = torch.flatten(input=v_6, end_dim=-1, start_dim=1)
        v_8 = self.classifier_1(v_7)
        v_9 = self.classifier_2(v_8)
        v_10 = self.classifier_3(v_9)
        return v_10

def export_torchscript():
    net = Model()
    net.float()
    net.eval()

    torch.manual_seed(0)
    v_0 = torch.rand(1, 6, 200, dtype=torch.float)

    mod = torch.jit.trace(net, v_0)
    mod.save("imu_cnn_pnnx.py.pt")

def export_onnx():
    net = Model()
    net.float()
    net.eval()

    torch.manual_seed(0)
    v_0 = torch.rand(1, 6, 200, dtype=torch.float)

    torch.onnx.export(net, v_0, "imu_cnn_pnnx.py.onnx", export_params=True, operator_export_type=torch.onnx.OperatorExportTypes.ONNX_ATEN_FALLBACK, opset_version=13, input_names=['in0'], output_names=['out0'])

def test_inference():
    net = Model()
    net.float()
    net.eval()

    torch.manual_seed(0)
    v_0 = torch.rand(1, 6, 200, dtype=torch.float)

    return net(v_0)

if __name__ == "__main__":
    print(test_inference())
