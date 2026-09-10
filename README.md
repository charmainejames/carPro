# carPro 智能车

基于 STC32G 单片机的智能车竞赛工程，包含基础巡线版本、完整智能车版本、赛道资料、硬件原理图与可直接烧录的 HEX 固件。

## 目录结构

```text
Projects/基础版/       PWM、电机和基础巡线实验工程
Projects/智能车版/     带 OLED、软件 I2C 等模块的完整智能车工程
Docs/                  校赛原理图、PCB 与赛道规则
Firmware/              从原工程中保留的可烧录 HEX
```

## 工程入口

- 基础版：`Projects/基础版/PWM.uvproj`
- 基础版另一配置：`Projects/基础版/PWM_1.uvproj`
- 智能车版：`Projects/智能车版/PWM.uvproj`

## 固件

- `Firmware/基础版_PWM.hex`
- `Firmware/基础版_PWM_1.hex`
- `Firmware/智能车版_PWM.hex`

三份固件从原有 `list` 编译目录中迁出并保留；其余 `.obj`、`.lst`、`.map`、`.lnp` 和构建日志已移除。

## 编译与测试

1. 使用支持 STC32G/C51 的 Keil 工具链打开对应 `.uvproj`。
2. 检查晶振、PWM 频率、电机方向和传感器引脚。
3. 执行 Rebuild，生成文件只保留在本地；确认可用的 HEX 再复制到 `Firmware`。

首次上电应架空驱动轮，先测试 PWM 和方向，再进行赛道测试。

## Git 管理约定

- `main` 保存可编译或已验证版本。
- 不提交 `list`、个人 `.uvgui`、`.uvopt`、备份文件和构建日志。
- 现有 `_1` 文件代表历史工程配置，未验证等价前不要直接删除或合并。
- 每次调参只改变一类参数，并在提交信息中记录实车效果。
