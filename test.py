from hobot_dnn_rdkx5 import pyeasy_dnn as dnn
model = dnn.load('/home/sunrise/vision_ws/best_640_480_bayese_640x640_nv12.bin')

# 打印模型输入输出信息
print("--- 输入信息 ---")
for i, tensor in enumerate(model[0].inputs):
    print(f"输入 {i}: 形状={tensor.properties.shape}, 类型={tensor.properties.dtype}")

print("--- 输出信息 ---")
for i, tensor in enumerate(model[0].outputs):
    print(f"输出 {i}: 形状={tensor.properties.shape}")