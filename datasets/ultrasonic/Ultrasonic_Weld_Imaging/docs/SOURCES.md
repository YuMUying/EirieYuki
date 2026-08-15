# 数据与参数来源

检索和归档日期：2026-08-14。下列“原始数据”均已实际下载到本目录，不是只保存链接。

## 原始 RF/FMC/PWI 数据

### Figshare / The Royal Society 实验阵列系列

论文：A. Velichko 与 A. J. Croxford, *Strategies for data acquisition using ultrasonic phased arrays*, DOI [10.1098/rspa.2018.0451](https://doi.org/10.1098/rspa.2018.0451)。四个记录均为 CC BY 4.0，且来自同一 1 mm 孔、20 mm 深度实验系列。

- FMC，DOI [10.6084/m9.figshare.7178630.v1](https://doi.org/10.6084/m9.figshare.7178630.v1)，64 x 64 通道对，512 点。
- 9 次 PWI，DOI [10.6084/m9.figshare.7178636.v1](https://doi.org/10.6084/m9.figshare.7178636.v1)，9 x 64 接收通道，512 点。
- 17 次 PWI，DOI [10.6084/m9.figshare.7178633.v1](https://doi.org/10.6084/m9.figshare.7178633.v1)，17 x 64 接收通道，512 点。
- 25 次 PWI，DOI [10.6084/m9.figshare.7178627.v1](https://doi.org/10.6084/m9.figshare.7178627.v1)，25 x 64 接收通道，512 点。

文件中记录 5 MHz、64 阵元、0.60 mm pitch、约 25 MHz 采样和 `ph_velocity = 6400 m/s`。它们适合验证 FMC/PWI 数据组织与成像数学，但不代表焊缝或目标压力容器材料。

### University of Strathclyde 焊缝 FMC

- K. M. M. Tant, *FMC dataset - Lack-of-fusion crack on welded 316L stainless steel plates*, DOI [10.15129/086404bd-eb69-429b-978c-2c35cdbfcf87](https://doi.org/10.15129/086404bd-eb69-429b-978c-2c35cdbfcf87)，CC BY 4.0。128 阵元、5 MHz、100 MHz 采样，316L 未熔合裂纹。
- K. M. M. Tant, *FMC dataset - 3mm side drilled hole 304ss MMA weld*, DOI [10.15129/60b6a5b8-e78e-4742-8414-aaba9399a9c8](https://doi.org/10.15129/60b6a5b8-e78e-4742-8414-aaba9399a9c8)，CC BY 4.0。128 阵元、2.25 MHz、25 MHz 采样，强噪声 304SS MMA 焊缝。
- K. M. M. Tant, *FMC dataset - Centreline crack in Inconel 82/182 weld*, DOI [10.15129/179e1b38-e701-443d-b995-a4449851330c](https://doi.org/10.15129/179e1b38-e701-443d-b995-a4449851330c)。仓库附件页将 MAT 与 XLSX 标为 CC BY 4.0，但 DataCite API 的 `rightsList` 为空；此差异已原样记录。45 阵元、5 MHz、100 MHz 采样，Inconel 82/182 中心线裂纹。

每个目录保留出版方原始 ODS/XLSX 元数据和 DataCite JSON。下载时当前网页附件端点受 Cloudflare 限制，归档文件取自 University of Strathclyde Pure 的官方 `ws/portalfiles/portal` 附件端点。

## TOFD D-scan 图像与像素掩膜

- Leonid Medvedev, *TOFD Defects Dataset*, GitHub release [`1.0`](https://github.com/leonidmedved/TOFD-dataset/releases/tag/1.0)，固定 commit [`c939ffa`](https://github.com/leonidmedved/TOFD-dataset/commit/c939ffaee2d280d00852119a8526a43ce17bbe3a)，CC BY-NC 4.0。
- 发布内容包括 991 对 Steel 20 人工缺陷实测图像/掩膜、605 对 CIVA 仿真图像/掩膜，以及由其中 1595 对组成的 1276/319 训练/验证划分。全部 PNG 均为 `96 x 176`、8 位灰度。
- 实测配置由上游 README 给出：25 mm 厚 Steel 20，5 MHz、直径 6 mm 探头，60 度入射，X/Y 扫描步长 0.5/0.2 mm，从试块两侧测量。
- 这是已成像的 TOFD D-scan 语义分割集，不含原始 RF/A-scan 数值，不能用于从波形重新形成 D-scan。许可证含 NonCommercial 限制，不可进入商业训练或产品数据链。
- 全量核验发现：CIVA 样本 ID `1460` 仅存在于原始 CIVA 目录，未进入混合集或训练/验证划分；README 声明的标签 ID `200` 在 release 1.0 的掩膜中没有实际出现。归档保留上游原样，不自行补样或重标。
- 固定 commit 的源 ZIP、原始 `README.md`、`README_RU.md`、`LICENSE`、本地 `metadata.json` 与 API 来源记录均已保存到 `raw/tofd/`。

公开仓储检索还覆盖了 Zenodo、Figshare、Kaggle、Hugging Face、Harvard Dataverse、DataCite、GitHub 与学术文献索引。检索结果中大量条目是论文、演示文稿或空仓库；截至归档日，没有发现第二套同时满足“公开下载、明确 TOFD 焊缝图像/波形、可核验许可、可用于训练”的独立数据发布。此结论是本次检索结果，不代表不存在未被索引、需申请或付费的数据。

## 目标材料

- 国家标准信息：GB/T 713.2-2023，*Steel plate, sheet and strip for pressure equipments - Part 2: Non-alloy and alloy steel with specified temperature properties*。国家标准全文公开系统页面已存档为 `metadata/materials/GBT713.2-2023_official_record.html`。
- 声速参考：[Evident Ultrasonic Material Velocity Reference](https://ims.evidentscientific.com/en/learn/ndt-tutorials/thickness-gauge/appendices-velocities)。本地网页存档列出 Steel 1020 纵波 5890 m/s、Steel 4340 纵波 5850 m/s。

本项目选择 Q245R 与 Q345R 作为两种常见铁磁性压力容器钢。配置中的纵波 5890 m/s、横波 3230 m/s、密度 7850 kg/m3 都只是软件启动值，不是 GB/T 713.2-2023 对牌号保证的超声参数。两种材料均需用同批试样复测。

## 候选采购探头与楔块

来源：Evident/Olympus *Phased Array Testing Catalog*，本地保存为 `metadata/products/Evident_PA_Probe_Catalog_EN.pdf`，并保存了文本提取版。

- 优先路线：`5L64-A2`，item `U8330072`，5 MHz、64 阵元、0.60 mm pitch、38.4 x 10 mm 有效孔径；配 `SA2-N55S`，钢中标称 55 度横波，推荐 40 至 70 度扫查。
- 备选路线：[`5L64-38.4X10-I1-P-2.5-OM`](https://ims.evidentscientific.com/en/products/phased-array-probes/u8330323)，item `U8330323`，水/Aqualene 匹配，适合后续设计轮式、水柱或水囊耦合头。

目录示例中的 Rexolite 楔块声速 2330 m/s 仅作为开发初值。`SA2-N55S` 的实际角度、声速、阵元高度、偏置和楔块延迟应以随货规格书和实测标定为准。

## 参考文献而非 RF 数据

- Dheeraj P R, *Single Sided Weld Inspection using Advanced Ultrasonic Methods*, DOI [10.6084/m9.figshare.11379633.v1](https://doi.org/10.6084/m9.figshare.11379633.v1)，本地 PDF 是方法参考，不含可训练的原始 RF 通道矩阵。
- Curtis J. Schroeder, *Inspection of Steel Bridge Welds Using Phased Array Ultrasonic Testing*, DOI [10.25394/PGS.7366943.v1](https://doi.org/10.25394/PGS.7366943.v1)，本地 PDF 是论文，不是原始 FMC 数据。

## 使用边界

本归档没有找到并声称拥有 Q245R 或 Q345R 焊缝的公开原始 FMC/RF 数据。现有不锈钢与 Inconel 数据可用于实现、调试和压力测试算法，但不能替代目标材料上的标定、训练和最终性能验证。
