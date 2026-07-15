# SafeDot 阶段 0 探针实验结果

日期:2026-07-13/14。对应 docs/SafeDot.md 第 9.1 节的探针实验;本文档记录实际做法、完整实测矩阵与结论,并对照第 7 章的数量级估算修订预算。高频目标(0.8 / 0.6 ns)按项目要求作为一等公民纳入。

## 0 与设计文档的偏差(有意为之)

1. **目标模块**:文档 9.1 节点名的是组合逻辑版 `transdot_decomp_multiplier_w6_4lane_dp`;实际 FMA(经 `w6_direct_outputs`)例化的是 `transdot_decomp_multiplier_w6_4lane_dp_piped`,且后者的 DPA 结构已重构为"按符号取负 → 共享 50 位带符号压缩加(adder_4)→ 末端取幅值",不再有 5.2 节所述的逐级幅值比较器。探针以 piped 版为目标——它才是真实数据通路,且带 INT DP 输出与可选流水寄存器。
2. **综合流程**:按用户指示使用 cacheflex l2_spm 的 Synopsys DC-NXT 拓扑模式流程(GF12LP,SSPG 0.72 V / −40 °C,`compile_ultra -gate_clock -self_gating -spg -retime -no_autoungroup -no_boundary_optimization`),而非仓库 Genus 配置。流程副本在 `syn/`(按仓库既有 .gitignore 政策留在本地),环境变量参数化:`SAFEDOT_TOP`(base/mod3)、`SAFEDOT_CLK_PS`、`SAFEDOT_STAGES`(重定时种子寄存器组数)、`SAFEDOT_CMP`(影子比较流水级数);驱动脚本 `syn/run_stage0_syn.sh`(≤4 路并行,每路 8 核,逐 run 隔离工作目录)。
3. **综合后时序分析一律使用 PrimeTime**(`syn/pt_sta/`):载入 mapped 网表 + DC 输出的 SDC + DC 拓扑模式估算 SPEF(~99% 网络 RC 反标),复位端口 set_false_path(recovery 属实现层问题);**不重开 ddc**(丢失 SPG 布局时序,与 QoR 矛盾,已实测证伪)。
4. **输出负载**:块级探针将 l2_spm 的 1.8444 pF 输出负载改为 10 fF。
5. **VT 前提**:本流程映射结果为 100% RVT(SSC14R)——LVT/SLVT 在 target list 中但未被使用(threshold voltage group 报告证实)。全部绝对时序为 RVT-only 悲观值。

## 1 校验器实现(mod 3,最终形态 v4)

RTL:影子逻辑内联于 piped 乘法器(参数 `SAFEDOT_CHECK` + 宏 `SAFEDOT_CHECK_EN` 双守卫,关闭时对现有流程零影响,现有 21,510 向量 FMA 回归全数通过);支持文件 `src/safedot/safedot_mod3_{pkg,reduce,shiftneg}.sv`;A/B 顶层 `src/safedot/safedot_stage0_wrap.sv`(输入打拍,`pipe_en=1`,`SAFEDOT_EXTRA_STAGES` 个输出侧种子寄存器组供重定时切分)。

**v3 分级(最终)**:
- 发射拍:仅 (a) 对模块输入做操作数段/FP4 幅值残差提取,(b) tap 早期 lane 载荷(pp[i][i][7:0] 32b、pp3_res[0]/[3] 48b)与 shamt/sign/模式(~29b)进影子寄存器。**不 tap 压缩和、不 tap addend 符号位、不 tap 幅值取负输出**。
- 比较拍 B1:恒等式 4/5 的 lane 残差链(移出段提取、取负、零标志);同时从**已寄存的输出**取 final_sum 修正位(product_int_dp_q[49]/[48]/[0] 与 |[48:0] 零标志——mag[0]=fs[0]、mag[49]=fs49&fs_lo_z、FP4 支路 mag[49:48] 由 9 位端口界结构性为 0,幅值取负 CPA 完全不进影子锥)与三路输出提取残差、sign_out。
- 比较拍 B2(`SAFEDOT_CMP_STAGES=2` 时独立一拍,否则与 B1 同拍):lane 选择、影子域回绕计数 K(lane 符号 = sign & 移位非零,零标志由 shiftneg 导出——依赖打包器前导零不变式)、模 3 求和与修正、比较,报警寄存。报警滞后输出 1(cmp1)或 2(cmp2)拍。

**v4 增量(阶段 1 首批工程项,在 v3 之上)**:
- **输出检查点合并**:三棵输出提取树 → 两棵。int 输出(final_sum 本体)恒查,作为规范算术检查点;两路 fp 输出只查**当拍架构上被消费的那一路**(dp 模式查 product_dp、scalar/simd 查 product_non_dp,以 48 位 2:1 mux 选入同一棵树,预测侧同步 mux)。未消费输出寄存器组内的故障架构上静默,首次被消费的拍即报警(消费点检查语义)。报警位 [1] 因合并恒为 0。
- **冗余形式余数树**:`safedot_mod3_reduce` 内部节点保持冗余数字域({0..3},3≡0,对去规范化的 EAC 加法封闭——a+b≤6 折叠后 s+c≤3),叶子退化为导线,每节点的规范化消失,仅树根规范化一次;操作数侧树与 shiftneg 弃段树输出原始数字(下游 pkg 辅助函数全部内部规范化)。`+define+SAFEDOT_TREE_LEGACY` 恢复 v3 逐节点规范化树(A/B 参照,亦是 0.8 ns 收敛配置,见 3.4)。

打包器不变式(FMA 保证,RTL 头注释记录):fp8/int4-DP 下段载荷 4 位(pp[11:8]=0);fp16/int8-DP 下 12 位半字前导零(pp3_res<2^22,lane 符号可由 sign&非零导出)。

## 2 功能验证

- `tb/safedot/tb_safedot_stage0.sv`:**300,045 个无故障随机周期零误报**(scalar / SIMD-FP16 / SIMD-FP8 / DP-FP16 / DP-FP8 / DP-FP4 / 随机模式组合,操作数镜像打包器不变式),v4 起共 4 种配置分别验证(冗余树/传统树 × cmp1/cmp2);**9/9 定向注入检出**(v4 新增注入 9:dp 模式下损毁 product_dp_q,覆盖合并 mux 的另一分支),报警位与受损输出一一对应。
- 开发中被随机验证击中并修正的真实角例:恒等式 4 的取负零角例;50 位压缩和回绕计数 K;fp16 lane 符号导出对前导零不变式的依赖(违反时 ~2M 周期内即出误报——6.4 节风险的活教材);全一段残差为 0 导致注入被 ×0 掩蔽(注入方法学教训)。
- **FMA 级打通(实弹检验打包器不变式)**:`+define+SAFEDOT_FMA_CHECK` 翻转 piped 乘法器的参数默认值即可在 FMA 内启用影子(接口零改动),`tb/safedot/safedot_alarm_bind.sv` 以 bind 挂入报警监视器;现有 21,510 向量回归(全格式、特殊值、INT DP)**全数通过且影子零报警(107,545 拍受检)**——两条打包器不变式经真实操作数打包器验证成立。

## 3 综合结果(完整矩阵)

条件:GF12LP 7.5T,SSPG 0.72 V/−40 °C,100% RVT,setup 不确定度 50 ps,输入延迟 50 ps/INVX1 驱动,输出延迟 50 ps/10 fF。s*N* = N 组种子寄存器 + `ungroup -all -flatten` + `set_optimize_registers`(高频必需);c2 = 影子比较两拍。**判定口径(项目决定,2026-07-15 起):综合仅映射 RVT,绝对时序悲观,~100 ps 以内的违例按可接受处理**(数值仍如实列出);超出该带的点(如 mod3@600、FPU@1800)仍记未收敛。

| 设计 | 周期 (ps) | DC WNS (ps) | PT WNS (ps) | 面积 (µm²) | 判定 |
|---|---|---|---|---|---|
| base | 1800 | +0.7 | (+25.9 min ep) | 1572.2 | 收敛 |
| mod3 v3 | 1800 | +0.9 | +35.8(影子 min ep) | 2184.5 | 收敛,**+38.9%** |
| base | 1700 | −18.7 | — | 1793.4 | 差 ~19 ps |
| mod3 v3 | 1700 | −0.1 | +29.9(影子 min ep) | 2320.0 | 收敛,+29.4% |
| base s1 | 800 | +0.002 | **+1.3** | 1739.9 | **收敛** |
| mod3 v3 s1 | 800 | −17.4 | **+2.6** | 2601.1 | **收敛(PT)**,+49.5%* |
| mod3 v4a | 1800 | +0.7 | 无违例 | 2124.6 | 收敛,+35.1% |
| mod3 v4b | 1800 | +0.5 | 无违例(影子 min +33.3) | 2099.5 | 收敛,**+33.5%** |
| mod3 v4a s1 | 800 | −22.3 | **+1.9** | 2599.2 | **收敛(PT)**,+55.6%* |
| mod3 v4b s1 | 800 | −33.0 | −14.2 | 2640.8 | **可接受**(~100 ps 口径;s2:−22.8、s1+c2:−20.6) |
| base s4 | 600 | −0.2 | −15.2 | 2302.0 | 实际收敛(±15 ps 工具噪声带) |
| mod3 v3 s3/s4/s5 ×c2 | 600 | −257…−285 | −234.8 | 2982–3102 | **未收敛**,阻塞已定位(见 3.3) |

\* 800 ps 点的面积对比含流水化代价的不对称:mod3 的影子寄存器与提取树在重定时后与主通路寄存器混合计入;600 ps 点(未收敛)面积仅供参考。恐慌模式(650/900 ps 无种子)与 v1/v2 历史数据在 `syn/results/*_v1/` 与 git 历史,不再列表。

### 3.1 面积结论(对照第 7 章)

收敛点的 A/B 增量:**+29%(1700)~ +39%(1800/800)**,显著高于第 7 章"乘法器校验 ≈ 主乘法器 8–12%"。构成:
1. **探针放大项**:模块边界三组输出寄存器各自提取校验(~162 µm²)——**v4 已合并为"恒查 int + 消费路 fp"两棵树,实测 −60 µm²**(见 3.4)。
2. **文档低估项**:lane 侧恒等式 5 移出段提取 ×6 lane(~200 µm²);乘法器内截断移位无 sticky 可共享。
3. **可优化项**:余数树逐级规范化——**v4 已改冗余形式,实测再 −25 µm²**(见 3.4;0.8 ns 点例外,用传统树);高频配置的影子/种子寄存器(未回收)。

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

修复候选的实测排除与存留:
1. ~~阻止算子融合~~——**已实验排除**:XOR-mask+进位形式的取负确实切断了融合(final_resources 证实 DP_OP 只剩压缩加),但 QoR 全面变差(mod3@600:−257→−308;mod3@800:−17→−94)——融合后的数据通路实现本身更优,~770 ps 段的根源不在融合;并行会话的 dont_touch 变体(_dt)同样无效(−255)。表达式已回退为原始形式。
2. **存留(阶段 1 路径)**:主通路在 final_sum 处显式加一级架构寄存器(高频配置),输出组延迟对齐随之调整,影子链相应加一级对齐;
3. 备选:fp16 lane 改用 5.2 节"分级检查点"变体消除 pp3_res tap(代价:重复 36 位移位器,~65 µm²/lane)。

**0.8 ns(1.25 GHz @RVT SS 0.72V/−40C)双方均收敛(PT 签核口径)**;0.6 ns 为 base-only 收敛,mod3 待上述修复——**该项已按项目决定暂缓**(2026-07-14),候选 2(final_sum 架构寄存器)留作后续独立事项。

### 3.4 阶段 1 首批工程项实测:v4 面积回收 + FPU 级集成测量

**v4 面积回收(隔离块,1800 ps 干净面积点,DC+PT 双收敛)**:

| 形态 | 面积 (µm²) | 校验器增量 vs base | 相对 v3 |
|---|---|---|---|
| v3(三棵输出树,逐节点规范化) | 2184.5 | +612.3(+38.9%) | — |
| v4a(输出检查点合并) | 2124.6 | +552.4(+35.1%) | −59.9 |
| v4b(v4a + 冗余形式余数树) | 2099.5 | **+527.3(+33.5%)** | **−85.0(校验器 −13.9%)** |

- 合并实测 −60 µm²(省一棵 48 位树 + 一组预测比较,换 48 位 2:1 mux);冗余树再 −25 µm²(shadow 子模块实例 342.9 → 303.0 µm²)。两项低于结构推算上限,因 DC 布尔优化已吸收部分逐节点规范化逻辑。
- **0.8 ns 高频点的取舍**:v4a 严格收敛(PT +1.9,与 v3 +2.6 同水平);v4b 最好 −14.2 ps(系统性差 ~15–20 ps——冗余树去掉的逐节点规范化恰好是重定时可利用的切分裕量),**按 ~100 ps 口径同样可接受(2026-07-15 起)。工程结论:冗余树为无条件默认;`+define+SAFEDOT_TREE_LEGACY` 仅作 A/B 参照与严格收敛需要时的备选**。800 ps 点面积仍受重定时混计影响(见表注),面积结论以 1800 ps 点为准。

**FPU 级集成 A/B(transdot_fpu_top,回归所用 4 级 ADDMUL-only 全格式配置;第 7 章"整合后 ≈ FPU 7–9%"估算的实测检验)**:

| 设计 | 周期 (ps) | DC WNS (ps) | PT | FPU 面积 (µm²) | 乘法器核 (µm²) |
|---|---|---|---|---|---|
| FPU base | 2100 | +3.0 | 无违例 | 5306.5 | 1591.6 |
| FPU + mod3 v4b | 2100 | +2.9 | 无违例 | 5826.4 | 2104.4 |
| FPU base | 1800 | −254.9 | −238.1 | 5787.1 | 1807.2 |
| FPU + mod3 v4b | 1800 | −232.7 | −218.6 | 6335.9 | 2369.5 |

- **收敛点(2100 ps,PT 双方无违例):校验器 = FPU 的 +9.8%(+519.9 µm²),= 集成乘法器的 +32.2%**(乘法器核 1591.6 → 2104.4);1800 ps 违例点 A/B 一致(+9.5%)。增量几乎全部落在乘法器作用域内(FPU 级 Δ ≈ 乘法器级 Δ),其余 FPU 逻辑不受扰动;含影子侧 FPU 的 WNS 反而更优 ~20 ps——**墙是 FPU 自身数据通路,校验器不是限制因素**。
- FPU 深度实测 ~2040 ps(PT,RVT-only SS 0.72V/−40C);乘法器占 FPU 面积 30.0%,与论文版图口径(34.5%)相符。
- 影子子模块面积 FPU 级 294.7–299.4 µm² ≈ 隔离块 303.0 µm²,实例化一致;报警锥在 FMA 未连接 `safedot_alarm_o` 的情况下由 `-no_boundary_optimization` 保全(网表证实 alarm 寄存器与两棵提取树在位),集成时接到顶层即可。
- 对照第 7 章与 3.1 节:修订估计"乘法器的 20–25% ≈ FPU 的 7–9%"实测为**乘法器的 32% ≈ FPU 的 9.8%**,略超上限;注意分母是 ADDMUL-only FPU,含 div/sqrt/conv 的全 FPU 百分比会进一步稀释。

### 3.5 模数参数化与 mod 3/15/63 Pareto 实测(阶段 1,2026-07-14/15)

校验器已按 K 位数字全参数化(模数 m=2^K−1,`+define+SAFEDOT_K`,默认 2):残差原语迁入 `src/safedot/safedot_res.svh`(SV package 不可参数化,以 localparam+include 进入各参数化作用域),K=2 与既有 v4 位级等价(3 配置 × 300k + 9/9 注入复验)。K-通式化揪出三处 mod-3 恒等式的"隐形退化"并显式化:移位恒等式中载荷的 2^LO 场内放置(LO 偶数时 mod 3 不可见,K=6 随机验证 24k 周期内即以误报暴露——参数化本身就是对推导的测试)、lane 帧放置旋转(0/12/24)、符号扩展常数 C_EXT=2^(48%K)+2^(49%K)(仅 K=2 为零),后者与 2^50 回绕折并为单常数 CD。K=4/6 各 300k 周期零误报、9/9 注入;FMA 级回归在 K=2 与 K=6 影子实弹下均 21,510/21,510、零报警。

**Pareto 实测(隔离块,1800 ps;K≥4 需 c2 两拍比较——单拍影子链装不下 4×4/6×6 数字乘,cmp1 下 WNS −607/−770 全在影子锥内,主通路不受扰)**:

| 模数 | 配置 | 面积 (µm²) | 校验器增量 | 相对 mod3 | 随机错误逃逸率 | 时序 (DC/PT) |
|---|---|---|---|---|---|---|
| mod 3 (K=2) | cmp1 | 2099.5 | +527.3(+33.5%) | 1× | ~33% | **收敛**(PT 无违例) |
| mod 15 (K=4) | cmp2 | 2591.0 | +1018.8(+64.8%) | 1.93× | ~6.7% | **收敛**(DC +0.7,PT 无违例) |
| mod 63 (K=6) | cmp2 | 3200.2 | +1628.0(+103.5%) | 3.09× | ~1.6% | PT −63.9,**可接受**(~100 ps 口径;严格收敛才需三拍比较) |

- 检错语义随模数的取舍:单比特错误三档均 100% 检出;双比特错误 mod 3 约半数混叠逃逸而 mod 63 同向对全检出/异向仅 1/6 逃逸;多比特/突发(时序错误的现实形态)按 ~1/m 逃逸。
- **DMR 基线锚点(9.2 节参照实现,`safedot_stage0_dmr`:共享输入寄存器 + 双乘法器核 + 全宽输出比较,报警滞后 1 拍同 cmp1)**:1800 ps 实测 **+1545.7 µm²(+98.3%),DC/PT 双收敛**(1700:+90.0%,PT −30.5,口径内可接受)。对照结论:**mod 3 = DMR 成本的 34%**(以 ~33% 多比特混叠为代价);mod 15 = 66%;**mod 63(+103.5%)成本已超过 DMR 而覆盖率更低(~1.6% 逃逸 vs DMR 仅共模盲区)——在乘法器块粒度上被 DMR 支配**,残差路线的价值区间在低成本端。DMR 的对价:动态功耗同样翻倍、无法定位共模设计错误、对角点比较器本身无保护。RPR 锚点尚未实现(多模式 DPA 的降精度副本需先定义各模式的截断语义与容差比较——留待界定)。
- 按"校验器增量局限于乘法器作用域"外推集成占比:mod 3 实测 9.8% FPU → mod 15 ≈ 19% → mod 63 ≈ 30%。**第 7 章的预算判断证实:8–12% FPU 预算内仅 mod 3 可行;mod 63 树面积 ×3 的预测与实测(3.09×)偏差 <3%**;mod 15 以约 2× 校验器成本换 5 倍混叠压缩(33%→6.7%),是预算翻倍情形下的中间选项。
- K≥4 的报警延迟为输出后 2 拍(c2);1800 ps 下 cmp1 只有 K=2 可行。双模数并行(3∥5,近似 mod-15 覆盖)未实测,留作候选。

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
# v4 变体 A/B(VTAG 隔离结果目录;EXTRA_DEFS 注入宏;0.8 ns 收敛配置用传统树)
TOPS=safedot_stage0_mod3 CLKS=1800 VTAG=v4b ./run_stage0_syn.sh
TOPS=safedot_stage0_mod3 CLKS=800 STAGES=1 VTAG=v4a EXTRA_DEFS="+SAFEDOT_TREE_LEGACY" ./run_stage0_syn.sh
# 模数 Pareto(K≥4 需两拍比较)
TOPS=safedot_stage0_mod3 CLKS=1800 CMP=2 VTAG=k4 EXTRA_DEFS="+SAFEDOT_K=4" ./run_stage0_syn.sh
TOPS=safedot_stage0_mod3 CLKS=1800 CMP=2 VTAG=k6 EXTRA_DEFS="+SAFEDOT_K=6" ./run_stage0_syn.sh
# FPU 级集成 A/B(transdot_fpu_top,SAFEDOT_FL 换填 FPU 文件列表)
TDEF="+TRANSDOT_ENABLE+USE_TRANSDOT_MULTIPLIER+USE_TRANSDOT_EXPONENT_DATAPATH+USE_TRANSDOT_ADDEND_DATAPATH+USE_TRANSDOT_NORMALIZE_DATAPATH+FP4_INCLUDED"
TOPS=transdot_fpu_top SAFEDOT_FL=safedot_fpu_syn.f CLKS=2100 VTAG=fpu0 EXTRA_DEFS="$TDEF" ./run_stage0_syn.sh
TOPS=transdot_fpu_top SAFEDOT_FL=safedot_fpu_syn.f CLKS=2100 VTAG=fpusd EXTRA_DEFS="$TDEF+SAFEDOT_FMA_CHECK" ./run_stage0_syn.sh
python3 summarize_stage0.py
# PrimeTime 综合后时序(网表+SDC+SPEF)
cd pt_sta && ./run_pt_sta.sh safedot_stage0_mod3_800ps_s1
python3 ../classify_endpoint_slack.py ../results/<tag>/pt/<top>.pt.endpoint_slack.rpt
```

## 5 阶段 0 结论

1. **可行性**:mod-3 余数校验器在真实多模式 DPA 乘法器上功能完备且经受了 6.4 节所警告的角例考验(取负零、回绕计数、打包器不变式依赖均被 300k 随机向量击中并修正);检错语义与结果正确性对齐。
2. **面积**:v4(输出检查点合并 + 冗余形式余数树)把隔离块增量从 +38.9% 压到 **+33.5%**(1800 收敛点,校验器 −13.9%);**FPU 级集成实测(2100 ps 收敛点,PT 无违例):+9.8% FPU / +32% 集成乘法器**——第 7 章估算的实测替代,略超修订估计上限(分母为 ADDMUL-only FPU,全 FPU 会稀释)。模数 Pareto 扫描仍是必答题。
3. **时序**:0.8 ns 双方收敛(PT 签核口径;RVT-only 前提;v4 用 `SAFEDOT_TREE_LEGACY` 配置,冗余树在该点系统性差 ~15 ps);FPU 级墙 ~2040 ps 由 FPU 自身数据通路决定,校验器非限制因素(影子侧 WNS 反而更优)。0.6 ns 事项**按项目决定暂缓**。影子分级演进(v1→v4)的每一步教训都有实测支撑。
4. **判据**:实测面积超出原估算,但经 v4 回收后可解释、可继续优化、不动摇 residue 路线选型;**阶段 1 已开工**,已完成:输出检查点合并、冗余形式余数树、FPU 级集成测量、**模数参数化与 mod 3/15/63 Pareto 实测(3.5 节)——预算内仅 mod 3 可行,mod 63 ×3 预测实测证实,mod 15 为预算翻倍时的中间选项**。后续:9.2 节其余矩阵维度(仅控制奇偶档、累加器随行余数档、RPR/DMR 基线锚点)、双模数并行候选、加数通路影子(第 5 章链条的下一环)。

*(功耗对比由独立的 pt_pwr 流程承担(建设中,非本文档范围);INT_DP_FMADD 的 FMA 级打通已完成,见第 2 节。)*
