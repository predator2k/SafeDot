# SafeDot 阶段 0 探针实验结果

日期:2026-07-13/14。对应 docs/SafeDot.md 第 9.1 节的探针实验;本文档记录实际做法、完整实测矩阵与结论,并对照第 7 章的数量级估算修订预算。高频目标(0.8 / 0.6 ns)按项目要求作为一等公民纳入。

## 0 与设计文档的偏差(有意为之)

1. **目标模块**:文档 9.1 节点名的是组合逻辑版 `transdot_decomp_multiplier_w6_4lane_dp`;实际 FMA(经 `w6_direct_outputs`)例化的是 `transdot_decomp_multiplier_w6_4lane_dp_piped`,且后者的 DPA 结构已重构为"按符号取负 → 共享 50 位带符号压缩加(adder_4)→ 末端取幅值",不再有 5.2 节所述的逐级幅值比较器。探针以 piped 版为目标——它才是真实数据通路,且带 INT DP 输出与可选流水寄存器。
2. **综合流程**:按用户指示使用 cacheflex l2_spm 的 Synopsys DC-NXT 拓扑模式流程(GF12LP,SSPG 0.72 V / −40 °C,`compile_ultra -gate_clock -self_gating -spg -retime -no_autoungroup -no_boundary_optimization`),而非仓库 Genus 配置。流程副本在 `syn/`(按仓库既有 .gitignore 政策留在本地),环境变量参数化:`SAFEDOT_TOP`(base/mod3)、`SAFEDOT_CLK_PS`、`SAFEDOT_STAGES`(重定时种子寄存器组数)、`SAFEDOT_CMP`(影子比较流水级数);驱动脚本 `syn/run_stage0_syn.sh`(≤4 路并行,每路 8 核,逐 run 隔离工作目录)。
3. **综合后时序分析一律使用 PrimeTime**(`syn/pt_sta/`):载入 mapped 网表 + DC 输出的 SDC + DC 拓扑模式估算 SPEF(~99% 网络 RC 反标),复位端口 set_false_path(recovery 属实现层问题);**不重开 ddc**(丢失 SPG 布局时序,与 QoR 矛盾,已实测证伪)。
4. **输出负载**:块级探针将 l2_spm 的 1.8444 pF 输出负载改为 10 fF。
5. **VT 前提**:本流程映射结果为 100% RVT(SSC14R)——LVT/SLVT 在 target list 中但未被使用(threshold voltage group 报告证实)。全部绝对时序为 RVT-only 悲观值。

## 1 校验器实现(mod 3,最终形态 v3)

RTL:影子逻辑内联于 piped 乘法器(参数 `SAFEDOT_CHECK` + 宏 `SAFEDOT_CHECK_EN` 双守卫,关闭时对现有流程零影响,现有 21,510 向量 FMA 回归全数通过);支持文件 `src/safedot/safedot_mod3_{pkg,reduce,shiftneg}.sv`;A/B 顶层 `src/safedot/safedot_stage0_wrap.sv`(输入打拍,`pipe_en=1`,`SAFEDOT_EXTRA_STAGES` 个输出侧种子寄存器组供重定时切分)。

**v3 分级(最终)**:
- 发射拍:仅 (a) 对模块输入做操作数段/FP4 幅值残差提取,(b) tap 早期 lane 载荷(pp[i][i][7:0] 32b、pp3_res[0]/[3] 48b)与 shamt/sign/模式(~29b)进影子寄存器。**不 tap 压缩和、不 tap addend 符号位、不 tap 幅值取负输出**。
- 比较拍 B1:恒等式 4/5 的 lane 残差链(移出段提取、取负、零标志);同时从**已寄存的输出**取 final_sum 修正位(product_int_dp_q[49]/[48]/[0] 与 |[48:0] 零标志——mag[0]=fs[0]、mag[49]=fs49&fs_lo_z、FP4 支路 mag[49:48] 由 9 位端口界结构性为 0,幅值取负 CPA 完全不进影子锥)与三路输出提取残差、sign_out。
- 比较拍 B2(`SAFEDOT_CMP_STAGES=2` 时独立一拍,否则与 B1 同拍):lane 选择、影子域回绕计数 K(lane 符号 = sign & 移位非零,零标志由 shiftneg 导出——依赖打包器前导零不变式)、模 3 求和与修正、比较,报警寄存。报警滞后输出 1(cmp1)或 2(cmp2)拍。

打包器不变式(FMA 保证,RTL 头注释记录):fp8/int4-DP 下段载荷 4 位(pp[11:8]=0);fp16/int8-DP 下 12 位半字前导零(pp3_res<2^22,lane 符号可由 sign&非零导出)。

## 2 功能验证

- `tb/safedot/tb_safedot_stage0.sv`:**300,045 个无故障随机周期零误报**(scalar / SIMD-FP16 / SIMD-FP8 / DP-FP16 / DP-FP8 / DP-FP4 / 随机模式组合,操作数镜像打包器不变式),cmp1 与 cmp2 两种配置分别验证;**8/8 定向注入检出**,报警位与受损输出一一对应。
- 开发中被随机验证击中并修正的真实角例:恒等式 4 的取负零角例;50 位压缩和回绕计数 K;fp16 lane 符号导出对前导零不变式的依赖(违反时 ~2M 周期内即出误报——6.4 节风险的活教材);全一段残差为 0 导致注入被 ×0 掩蔽(注入方法学教训)。

## 3 综合结果(完整矩阵)

条件:GF12LP 7.5T,SSPG 0.72 V/−40 °C,100% RVT,setup 不确定度 50 ps,输入延迟 50 ps/INVX1 驱动,输出延迟 50 ps/10 fF。s*N* = N 组种子寄存器 + `ungroup -all -flatten` + `set_optimize_registers`(高频必需);c2 = 影子比较两拍。

| 设计 | 周期 (ps) | DC WNS (ps) | PT WNS (ps) | 面积 (µm²) | 判定 |
|---|---|---|---|---|---|
| base | 1800 | +0.7 | (+25.9 min ep) | 1572.2 | 收敛 |
| mod3 v3 | 1800 | +0.9 | +35.8(影子 min ep) | 2184.5 | 收敛,**+38.9%** |
| base | 1700 | −18.7 | — | 1793.4 | 差 ~19 ps |
| mod3 v3 | 1700 | −0.1 | +29.9(影子 min ep) | 2320.0 | 收敛,+29.4% |
| base s1 | 800 | +0.002 | **+1.3** | 1739.9 | **收敛** |
| mod3 v3 s1 | 800 | −17.4 | **+2.6** | 2601.1 | **收敛(PT)**,+49.5%* |
| base s4 | 600 | −0.2 | −15.2 | 2302.0 | 实际收敛(±15 ps 工具噪声带) |
| mod3 v3 s3/s4/s5 ×c2 | 600 | −257…−285 | −234.8 | 2982–3102 | **未收敛**,阻塞已定位(见 3.3) |

\* 800 ps 点的面积对比含流水化代价的不对称:mod3 的影子寄存器与提取树在重定时后与主通路寄存器混合计入;600 ps 点(未收敛)面积仅供参考。恐慌模式(650/900 ps 无种子)与 v1/v2 历史数据在 `syn/results/*_v1/` 与 git 历史,不再列表。

### 3.1 面积结论(对照第 7 章)

收敛点的 A/B 增量:**+29%(1700)~ +39%(1800/800)**,显著高于第 7 章"乘法器校验 ≈ 主乘法器 8–12%"。构成:
1. **探针放大项**:模块边界三组输出寄存器各自提取校验(~162 µm²);FPU 集成只在预舍入点设一个检查点,操作数提取树亦与 FPU 级共享。
2. **文档低估项**:lane 侧恒等式 5 移出段提取 ×6 lane(~200 µm²);乘法器内截断移位无 sticky 可共享。
3. **可优化项**:余数树逐级规范化(改冗余形式,省 15–20%);高频配置的影子/种子寄存器。

修订后的集成语境估计:**乘法器校验 ≈ 乘法器的 20–25%,折合 FPU 的 ~7–9%**(乘法器占版图 34.5%)。全机 8–12% 预算勉强可守、无余量;模 63 树面积 ×3,同预算大概率不可行 → 模数 Pareto 扫描(9.2 节)升级为必答题。

### 3.2 影子分级的演进(时序教训,均经 300k 向量回归)

| 版本 | 分级 | 影子状态 | 实测教训 |
|---|---|---|---|
| v1 | 残差数学在发射拍 | ~23b | 影子发射锥 = 主 pp 生成 + 影子尾巴,把墙推 ~100 ps(1700:−120.6,违例路径终点 sd_r_base_q) |
| v2 | 发射拍只 tap,数学在比较拍 | ~152b | 低频恢复零影响(1700 收敛);高频下 **async-reset tap 寄存器把被 tap 网(final_sum/mag/addend 符号)钉死在固定流水级**,重定时无法再切分(800:−55) |
| v2r | 影子寄存器全部去复位 | ~152b | **更差**(800:−277)——工具把影子寄存器重定时进坏位置;经验:async tap 优于 free tap |
| v3 | 修正位改读已寄存输出;K 影子域内导出;只留早期载荷 tap | ~152b | 800 收敛(PT +2.6);1700/1800 保持收敛;600 仍阻塞(见 3.3) |
| v3+c2 | 影子比较链自身两拍 | +~56b | 影子链每级 <450 ps;600 阻塞点不在影子链(数字不变) |

### 3.3 0.6 ns 的阻塞:已定位的工具级协同设计问题

600 ps 下 mod3 全部配置(s3/s4/s5 × c2)停在同一条 ~770 ps 路径;`final_resources` 报告定位:DC 数据通路提取把 **50 位 4:2 压缩加(adder_4,行 1851)与幅值取负(行 660)融合为单一 DP_OP 块**,该融合算子在 mod3 的约束下重定时后仍残留 ~770 ps 内部链;base(同一 RTL 算子)在 s4 下正常切到 505 ps。影子端点全部有裕量(PT),阻塞不是校验器逻辑深度,而是**早期载荷 tap(pp3_res[0]/[3])使被 tap 的 12×12 半字乘法器无法吸收重定时寄存器**,挤压其余级的调度空间,叠加算子融合后表现为不可切分段。

阶段 1 的修复候选(按侵入性排序):
1. 阻止该算子融合(表达式重写/中间模块边界/`set_dont_retime` 反向使用),让压缩加与取负各自成级;
2. 主通路在 final_sum 处显式加一级架构寄存器(高频配置),输出组延迟对齐随之调整;
3. fp16 lane 改用 5.2 节"分级检查点"变体消除 pp3_res tap(代价:重复 36 位移位器,~65 µm²/lane)。

**0.8 ns(1.25 GHz @RVT SS 0.72V/−40C)双方均收敛(PT 签核口径)**;0.6 ns 为 base-only 收敛,mod3 待阶段 1 上述修复。

## 4 复现命令

```bash
source ~/general.sh && export SAFEDOT_HOME=/mnt/ssd/mhnie/SafeDot
# 功能验证(cmp1 与 cmp2)
vcs -full64 -sverilog -timescale=1ns/1ps +define+SAFEDOT_CHECK_EN \
    -f $SAFEDOT_HOME/tb/safedot/stage0_tb.f -top tb_safedot_stage0 -o simv && ./simv
vcs -full64 -sverilog -timescale=1ns/1ps +define+SAFEDOT_CHECK_EN+SAFEDOT_CMP_STAGES=2 \
    -f $SAFEDOT_HOME/tb/safedot/stage0_tb.f -top tb_safedot_stage0 -o simv_c2 && ./simv_c2
# 现有回归(影子关闭;需先 git submodule update --init)
cd tb/sv_tb_new && SKIP_GEN=1 ./run_regression.sh 64
# A/B 综合(低频 / 高频)
cd syn && TOPS="safedot_stage0_base safedot_stage0_mod3" CLKS="1800 1700" ./run_stage0_syn.sh
TOPS="safedot_stage0_base safedot_stage0_mod3" CLKS=800 STAGES=1 ./run_stage0_syn.sh
TOPS=safedot_stage0_mod3 CLKS=600 STAGES=4 CMP=2 ./run_stage0_syn.sh
python3 summarize_stage0.py
# PrimeTime 综合后时序(网表+SDC+SPEF)
cd pt_sta && ./run_pt_sta.sh safedot_stage0_mod3_800ps_s1
python3 ../classify_endpoint_slack.py ../results/<tag>/pt/<top>.pt.endpoint_slack.rpt
```

## 5 阶段 0 结论

1. **可行性**:mod-3 余数校验器在真实多模式 DPA 乘法器上功能完备且经受了 6.4 节所警告的角例考验(取负零、回绕计数、打包器不变式依赖均被 300k 随机向量击中并修正);检错语义与结果正确性对齐。
2. **面积**:收敛点 +29–39%(隔离块),修订集成估计为乘法器的 20–25% ≈ FPU 的 7–9%;第 7 章分项表需按此修订,模数 Pareto 扫描升级为必答题。
3. **时序**:0.8 ns 双方收敛(PT 签核口径;RVT-only 前提);0.6 ns base 收敛、mod3 被一个已精确定位的"算子融合 × 载荷 tap"工具交互阻塞,修复路径明确。影子分级演进(v1→v3+c2)的每一步教训都有实测支撑,最终形态把主通路 tap 压缩到早期载荷 96b。
4. **判据**:实测面积超出估算 ~3×,但可解释、可优化、不动摇 residue 路线选型;**建议进入阶段 1**,首批工程项:输出检查点合并、冗余形式余数树、模数扫描、0.6 ns 的算子融合修复。

*(待补:PrimeTime PX 功耗对比、INT_DP_FMADD 的 FMA 级打通。)*
