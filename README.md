# Orbbec

该项目封装了奥比中光（Orbbec）双目摄像头的驱动，便于用户以回调（callback）方式异步获取 RGB 与 深度（Depth） 数据。

**设计目标**：将彩色图像与深度信息实时传输给 AI 模型处理模块

## 编译说明
```bash
mkdir build
cd build
cmake ..
make
```

## 运行
```bash
./orbbec
```

## 用户 API 接口说明
- 使用 `rgb_cb` 异步回调方式获取 RGB 图像数据。
- 使用 `depth_cb` 异步回调方式获取深度图像数据。
- RGB 与 Depth 来自 同一帧，可有效减少视觉差带来的误差。

示例代码请参考 `example.cpp`。

## License
MIT License

Author: Jim 
