# 已排除或降级的来源

## TOFD 检索中的论文、演示与空仓库

- Figshare `11380056`，*PRACTICAL APPLICATIONS OF ADVANCED NDT: PAUT & ToFD*：只有 CC BY 4.0 演示 PDF，不是数据集。
- University of Liverpool DOI [`10.17638/00006413`](https://doi.org/10.17638/00006413)，*Automatic detection, sizing and characterisation of weld defects using ultrasonic time-of-flight diffraction*：公开附件是学位论文 PDF，未发布可下载的训练图像或原始波形。
- GitHub `AIhuohuo9005/Weld-TOFD-dataset` 与 `yuekun695/MIFFM`：检索时仓库只有极短 README，没有数据文件。
- Crossref/OpenAlex 命中的 TOFD 缺陷识别、去噪、图像增强论文：作为潜在方法参考，但论文中的少量插图不能当作有明确样本边界和许可的数据集。

## Figshare 28725820

- 标题：*Original datasets for Fig 11, Fig 15 and Fig 18*
- DOI：[10.1371/journal.pone.0320970.s001](https://doi.org/10.1371/journal.pone.0320970.s001)
- 处理：排除，未保留附件。
- 原因：压缩包内只有拉伸应力-应变、应力-位移和显微硬度电子表格，没有超声 RF、FMC 或 PWI 原始通道数据。标题中的“Original datasets”不能作为存在超声波形的证据。

## 单侧焊缝检验论文与钢桥 PAUT 学位论文

- 处理：降级为 `docs/source_records` 中的方法参考。
- 原因：文件是论文/演示 PDF，不是原始 RF 通道数据。它们可支持机械布置、扫查策略和评价指标设计，但不能进入波束合成数据读取流程。

## 公共 304SS、316L 与 Inconel 焊缝数据

- 处理：保留为真实焊缝 FMC，同时标注材料域差异。
- 原因：这些数据确实包含原始通道 RF/FMC，但材料不是 Q245R 或 Q345R。不得将其重命名、混标或作为目标压力容器碳钢的实测数据。
