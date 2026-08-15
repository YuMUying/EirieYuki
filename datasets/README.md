# 数据集恢复与使用边界

GitHub 仓库只保存数据集说明、来源、许可和完整性清单，不提交原始图像、掩膜、MAT/RF 波形或批量派生数据。本机完整归档仍位于本目录下，正式代码通过项目根目录推导默认路径。

## 焊缝视觉数据

目标目录：`datasets/weld_vision/WES-Combined-Dataset`

数据组成、划分和处理参数见 [WES 数据说明](weld_vision/WES-Combined-Dataset/README.md)，许可见 [来源与许可](weld_vision/WES-Combined-Dataset/SOURCE_LICENSES.md)。完整本机数据包含 2,540 对监督样本和 2,056 张辅助未标注图像。辅助池不能作为像素级分割真值。

恢复数据时，应按来源许可重新取得原始集，并使用说明中固定的划分、分组和随机种子重建。不得把检测框自动当作分割掩膜，也不得在训练前混入验证集或测试集。

## 超声数据

目标目录：`datasets/ultrasonic/Ultrasonic_Weld_Imaging`

来源、轴顺序、材料域和候选探头信息见 [超声归档说明](ultrasonic/Ultrasonic_Weld_Imaging/README.md) 与 [来源记录](ultrasonic/Ultrasonic_Weld_Imaging/docs/SOURCES.md)。[排除来源](ultrasonic/Ultrasonic_Weld_Imaging/docs/EXCLUDED_SOURCES.md) 不进入正式算法验证。完整文件的相对路径、大小和 SHA-256 记录在 `MANIFEST.csv`，但清单本身不授予数据再分发权。

恢复后运行归档自带校验脚本，确认 MAT 变量、轴顺序、TOFD 图像配对、标签值和哈希。公开数据、半解析合成数据和目标储罐实测数据必须分别标注，不能将合成结果表述为现场检出率或法规能力。

## 版本化原则

- Git 版本化代码、配置、来源、许可、清单、模型卡和结构化评估结果。
- 原始数据与大批量派生数据保存在受控本地存储或对象存储，不进入普通 Git。
- 每次训练记录数据集版本/哈希、划分、模型版本、配置、随机种子和评估结果。
- 发布或商业使用前逐项复核上游许可，未知许可数据默认不再分发。
