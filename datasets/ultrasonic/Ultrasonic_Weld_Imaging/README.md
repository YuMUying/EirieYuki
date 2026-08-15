# 超声焊缝成像数据归档

本目录归档了用于独立超声焊缝成像系统原型开发的公开原始阵列通道数据、来源记录、目标材料启动参数，以及候选商用探头/楔块公开参数。归档日期为 2026-08-14。

## 归档内容

| 类别 | 数据 | 用途 | 关键限制 |
|---|---|---|---|
| 标定/算法基线 | 同一 1 mm 侧钻孔试件的 64 阵元 FMC、9/17/25 次 PWI | 读取器、TFM、PWI 相干叠加与发射次数对照 | 不是焊缝，也不是四个独立试件 |
| 真实焊缝 FMC | 316L 未熔合裂纹，128 阵元、5 MHz、100 MHz 采样 | 裂纹定位、TFM、相干成像 | 奥氏体不锈钢，不是目标碳钢 |
| 真实焊缝 FMC | 304SS MMA 焊缝中的 3 mm 侧钻孔，128 阵元、2.25 MHz、25 MHz 采样 | 强结构噪声抑制和已知反射体定位 | 奥氏体不锈钢 |
| 真实焊缝 FMC | Inconel 82/182 中心线粗糙裂纹，45 阵元、5 MHz、100 MHz 采样 | 异质焊缝和粗糙裂纹鲁棒性测试 | 镍基异种金属焊缝 |
| TOFD 图像分割 | Steel 20 人工缺陷实测图像 991 对、CIVA 仿真图像 605 对；混合训练/验证 1276/319 对 | TOFD D-scan 缺陷与结构信号的像素级分割 | 仅有 8 位 PNG 和掩膜，无原始 RF；CC BY-NC 4.0，不可商用 |
| TOFD 合成 A 扫 | 半解析模型生成 128 个独立工况、44,928 条 A 扫 | RF 读取、到达时间监督、预训练和算法测试 | 合成相对幅值数据，不是实测或认证仿真，不能证明检出率 |
| 目标材料配置 | Q245R、Q345R | 软件默认材料参数和接口设计 | 没有冒充这两个牌号的公开实测 RF 数据 |
| 候选采购配置 | Evident 5L64-A2 + SA2-N55S；5L64-I1 | 接触斜楔或水囊/轮式耦合头设计 | 到货后必须标定楔块、声程、阵元原点和工具坐标 |

原始 MAT 文件总计约 358 MB。新增 TOFD 发布快照包含 9572 张 PNG（该数字含原始集、混合集及训练/验证副本）和固定 commit 的源 ZIP。许可证、DOI/版本、原始元数据、文件轴顺序和材料域警告见每个数据目录的 `metadata.json` 以及 `docs/SOURCES.md`。

另有约 157 MB 的 TOFD 合成 RF 归档，位于 `derived/tofd_synthetic_dataset_v1/`。它按完整扫查工况划分为训练 90、验证 19、测试 19；归档中只含 MAT、CSV、JSON 和说明/校验文件，没有批量预览图。

## 统一数据接口

公开 TOFD 分割数据不是下述阵列波形接口。它是 `96 x 176` 的 8 位灰度 D-scan 图像及同尺寸像素掩膜，文件名按 `data_<id>.png` 与 `Label_<id>.png` 配对。掩膜标签定义见其 `metadata.json`。合成 TOFD 数据则保存为 `rf_int16[scan, sample]`，用 `double(rf_int16) / 32767` 恢复单位峰值 RF。

算法内部建议统一采用 SI 单位，并将 FMC 波形表示为：

```text
rf[tx, rx, sample]
```

PWI 表示为：

```text
rf[transmit_event, rx, sample]
angles_rad[transmit_event]
```

各来源不能直接按同一轴顺序读取：

- Figshare FMC/PWI 的 `exp_data.time_data` 是 `[sample, channel_pair]`，须用 `exp_data.tx` 和 `exp_data.rx` 展开。
- Strathclyde 316L 与 Inconel 的磁盘变量是 `[sample, tx, rx]`，Python 中转为 `array.transpose(1, 2, 0)`。
- Strathclyde 304SS 已是 `[tx, rx, sample]`。
- 每套公开数据必须使用自身 `metadata.json` 中的声速，不能用 Q245R/Q345R 默认值覆盖。

## 可先实现的算法版本

在没有实物和机器人位姿数据时，可完成一版可运行、可评价的二维算法：

1. 读取 MAT 与轴归一化，保留原始 RF 动态范围。
2. 去直流、时间零点、频带滤波、坏阵元屏蔽和可选增益补偿。
3. FMC 数据采用均匀介质直达声程的 DAS/TFM；PWI 数据采用按发射角计算声程的相干叠加。
4. 计算解析信号包络，归一化并输出 dB 图。
5. 增加 coherence factor、phase coherence factor 或短延迟相干作为可切换抑噪权重。
6. 用侧钻孔中心、裂纹尖端/走向真值评价定位误差、横向/轴向分辨率、SNR/CNR 和运行时间。
7. 将材料声速、阵元坐标、楔块声程、扫描位姿和图像网格全部作为外部配置，不写死在算法中。

这版算法可以验证数据链和成像主流程，但不能证明在 Q245R/Q345R 现场焊缝上的检出率。实物阶段至少还需补采：同批母材/焊缝声速与衰减、楔块延迟和折射角、标准试块 RF、无缺陷背景、多种人工/真实缺陷 RF、探头姿态与触发同步数据。

## 机械结构关联边界

成像核心与机械结构是“接口强相关、代码可解耦”：机械结构决定阵元空间坐标、楔块/水路声程、入射角、耦合稳定性、压力、扫描姿态和触发同步；这些量必须进入传播时间模型。把它们参数化后，滤波、波束合成、包络、相干权重和评价代码可以先在公开数据上开发。

理想轨迹接口示例在 `metadata/scan/ideal_linear_scan.json`。它只是机器人输出协议的起点，不是公开数据的真实扫描轨迹。

## 目录

```text
raw/calibration/       FMC/PWI 标定与算法基线数据
raw/weld/              真实焊缝 FMC 数据及原始来源元数据
raw/tofd/              TOFD D-scan 图像、像素掩膜、源 ZIP 与版本记录
derived/tofd_synthetic_dataset_v1/  TOFD 合成 RF、真值、配置和划分清单
metadata/materials/    Q245R/Q345R 启动参数与标准网页存档
metadata/products/     探头/楔块目录、网页存档和采购配置
metadata/scan/         理想扫描轨迹与机器人字段约定
docs/                  来源、排除项和参考文献
derived/validation/    自动校验报告
tools/                 归档校验工具
MANIFEST.csv           文件大小与 SHA-256 清单
```

## 完整性校验

系统 Python 安装 NumPy、SciPy 与 Pillow 后，在项目根目录运行：

```powershell
python -X utf8 "datasets\ultrasonic\Ultrasonic_Weld_Imaging\tools\validate_archive.py"
```

脚本会解析全部 JSON，核对 7 套 MAT 的变量、形状和类型，检查小型数据的时间轴与有限数值，并核验 TOFD 图像尺寸、格式、图像/掩膜配对、标签值及训练/验证划分，最后重新生成 `MANIFEST.csv` 和 `derived/validation/archive_validation.json`。大型 MAT 采用变量表校验，避免无意义地分配数百 MB 内存。

TOFD 合成集另用 MATLAB 逐工况校验，覆盖 MAT 字段、形状、类型、轴、采样率、归一化范围、CSV/JSON 一致性和零图片约束：

```matlab
cd(fullfile(project_root, 'datasets', 'ultrasonic', 'Ultrasonic_Weld_Imaging', 'tools', 'matlab_tofd'));
validate_tofd_dataset
```

## 许可与引用

FMC/PWI 公开数据以 CC BY 4.0 为主；新增 TOFD 图像集为 **CC BY-NC 4.0，禁止商业使用**。训练、发布或再分发时须按数据来源分别追踪许可，并引用各自 DOI 或仓库版本及作者。厂商目录、产品网页和国家标准页面仅作为选型与参数来源存档，不因放入本目录而变成开放数据许可证。详细来源见 `docs/SOURCES.md`。
