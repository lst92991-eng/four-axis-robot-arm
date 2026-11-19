# 第一课实物图片来源

## 采用的照片

- `nema17-exterior-oomlout.jpg`
  - 原作者：oomlout。
  - 原始照片：[Nema 17 Stepper Motor](https://commons.wikimedia.org/wiki/File:Nema_17_Stepper_Motor.jpg)。
  - 许可：[CC BY-SA 2.0](https://creativecommons.org/licenses/by-sa/2.0/)。
  - 采用完整原图，无裁剪、重绘或内容修改。讲义按比例缩放显示，作者与许可在图注中以普通文字保留，来源链接保存在本文件。
- `hybrid-stepper-inside-robocore.jpg`
  - 来源：[RoboCore — Controlando motores: Motor de passo](https://www.robocore.net/tutoriais/controlando-motor-de-passo)。
  - 原图：https://s3-sa-east-1.amazonaws.com/robocore-tutoriais/97/MOTOR2.0.jpg
  - 采用完整原图，无裁剪、重绘或内容修改。照片原有 ROTOR 与 ESTATOR 标注保留，正文通过中间和右侧的位置说明对应的中文名称。
  - 该页面未给出开放许可，不能把该照片标成 CC 或项目自有图片。

两张照片是同类电机的外观与拆解示例，不声称是同一台电机拆解前后。Word 内部为兼容渲染使用 PNG 编码封装照片，已检查解码后的像素逐字节一致，原始 JPEG 保留在此目录。

`hybrid-stepper-open-sleepsfortheweak.png` 为筛选时保留的候选照片，未用于当前讲义。来源为 [Wikimedia Commons](https://commons.wikimedia.org/wiki/File:Stepper_BipolarHybrid.png)，原作者 Sleepsfortheweak，CC BY-SA 3.0。

## 电磁学与通电顺序配图

- `long-wire-right-hand-rule-mikerun.svg` 为 [MikeRun 的直导线右手定则原图](https://commons.wikimedia.org/wiki/File:Long-wire-right-hand-rule.svg)，[CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/)。
- `current-field-handrule.png` 在上述原图外侧增加中文说明，图注已注明改编来源。该组合图及新增标注同样采用 CC BY-SA 4.0。
- `solenoid-current-and-field.png` 为项目绘制的螺旋线圈及端面方向图。I 箭头沿绕线，B 箭头表示线圈内部磁场，观察端固定为右端。几何方向检查见 `output/lesson01-foundations/physics-check.json`。
- `four-state-cycle.png` 为项目的四状态图，状态与配套动画对应。
- `right-hand-grip-public-domain.svg` 为未采用的候选手势，原作者 Schorschi2、Wizard191 等，来源 [Wikimedia Commons](https://commons.wikimedia.org/wiki/File:Right-hand_grip_rule.svg)，原作者释放为公有领域。

## 当前新增的磁极吸斥图

`magnet-poles.png` 为项目绘制的简洁对照图：N/N 相对时，两磁铁受力向外；N/S 相对时，两磁铁受力向内。箭头表示力，不能当作磁场方向。完整 N/S 标注保留，正文补充 S/S 相对也相斥。生成源与检查位于 `output/lesson01-narrative/`。

## 已暂缓的基础概念配图

`concept-steps.png`、`concept-motion.png`、`concept-transmission.png`、`concept-winding.png`、`concept-driver.png`、`concept-torque.png`、`concept-feedback.png` 为项目绘制的计算或概念说明图。

- 步距与齿数关系参考 [Oriental Motor 两相与五相说明](https://www.orientalmotor.com/stepper-motors/technology/2-phase-vs-5-phase-stepper-motors.html)。
- 细分、分辨率与精度参考 [Analog Devices](https://www.analog.com/en/resources/analog-dialogue/articles/mastering-precision-understanding-microstepping.html) 与 [MPS](https://www.monolithicpower.com/en/learning/resources/why-microstepping-isnt-as-good-as-you-think)。
- 保持、起停及转速—转矩条件参考 [Oriental Motor 转矩曲线说明](https://www.orientalmotor.com/stepper-motors/technology/speed-torque-curves-for-stepping-motors.html)。
- 驱动电压与绕组电流的区别参考 [Analog Devices 的电机供电说明](https://ez.analog.com/motor-control-hardware-platforms2/a/documents/DO19238/why-do-i-need-to-use-a-stepper-motor-supply-voltage-higher-than-the-rated-phase-voltage)。
- 数值例题由所列参数计算，检查见 `output/lesson01-concepts/calculation-check.json`。转矩趋势图没有使用伪造的实测数值，不能用于本机选型。

Word 视频能力核对依据为 [Microsoft 官方说明](https://support.microsoft.com/en-us/word/insert-an-online-video-in-word)。当前讲义未嵌入视频，也未使用旧的自制运动演示。

## 磁铁模型及已撤下的结构过渡

- `magnet-model-steps.png`：项目新绘的连续四步图，条形磁铁转子与八边形定子，通电改变后再显示转子到位。用于先观察动作，后文再解释磁极作用。
- `model-to-teeth.png`：历史结构过渡图，已按用户最新要求从讲义删除，连同对应整段文字一并撤下。保留源文件仅供历史追溯。
- 原图生成源在 `output/lesson01-model/make_models.cjs`，方向和结构检查在同目录 `model-check.json`。
- 轴向磁化、两段错齿及定子主极结构核对 [Oriental Motor Stepper Motor Basics](https://www.orientalmotor.com/stepper-motors/technology/stepper-motor-basics.html)、[官方两相结构说明](https://www.orientalmotor.com.sg/sg/tech/reference/ap/stepping-motor-02) 和 [Stepper Motor Structure](https://www.orientalmotor.com.sg/sg/tech/reference/stepping-motor01)。正文只讲当前需要的结构关系，未引用原文段落或套用截图公式。

## 历史绕组电流反向图（原向转子侧为 S）

`winding-current-reversal.png` 为项目绘制，放大四步模型中右侧绕组。两图固定相同视角和绕线几何；原向前侧电流向下、内部 B 向右、转子侧 S，反向则电流向上、B 向左、转子侧 N。蓝箭头沿铜线标电流 I，绿箭头在铁芯内标 B，正反两图的极性对应。

生成源与方向核验见 `output/lesson01-winding/make_winding.cjs`、`winding-check.json`。该图替换旧的通用线圈图，使右手定则直接回到模型部件。

## 当前视频流程重构配图

- 当前流程来源为用户指定的 [爱上半导体步进电机视频](https://www.bilibili.com/zh-Hans/video/BV1cN4y197TG/)，已实际核对各段关键帧和字幕。观察记录见 `output/lesson01-video-flow/source-review.md`。
- `flow-magnet-four-positions.png`：项目绘制，外侧磁铁依次从上、右、下、左靠近，中心条形磁铁随之转动，直箭头和弧箭头含义分别说明。
- `flow-two-magnets-half-step.png`：项目绘制，先上方单块作用，再等距的上方与右侧共同作用，最后只留右侧，解释中间位置。
- `flow-replace-magnets-with-coils.png`：项目绘制，把右侧磁铁替换为固定铁芯与线圈，对照前后相同的转子位置。
- `flow-winding-current-reversal.png`：项目重绘同一右侧绕组。原向转子侧为 N，与当前线圈替换图一致。该图为当前版本，上方旧 `winding-current-reversal.png` 保留作历史记录。
- 生成脚本和检查在 `output/lesson01-video-flow/`。新流程图未复制原视频画面或品牌；文字为重新组织的教学说明。

## 完整原理新增配图

`complete-` 前缀的 11 张图片为本轮项目绘制：结构与齿距、对齐位置与单步放大、两相连接与电流反向、四次换相、转子两段与半齿距、前后视图配合及少量磁极的同轴模型。

源代码为 `output/lesson01-complete/make_complete_figures.cjs`。所有连续状态使用同一个齿相位模型，验证见 `geometry-check.json`。来源与覆盖范围见 `../视频内容对照.md`。

主要技术核对：[Oriental Motor Stepper Motor Basics](https://www.orientalmotor.com/stepper-motors/technology/stepper-motor-basics.html)、[两相与五相结构说明](https://www.orientalmotor.com.sg/sg/tech/reference/ap/stepping-motor-02)。48 个定子齿是当前绘图模型的具体排列；后端视图编号按观察方向镜像。

## 齿距与组间错位

- `stator-tooth-spacing.png` 用作图 10。上半为假想的 50 齿对齐，下半展示实际分组的接缝。用 `output/lesson01-tooth-spacing/edit_lesson.py` 绘制，尺寸和角度见 `structural-check.json`。
- 1.5 的磁针实验史实取自奥斯特的报告。 [1820 年实验报告](https://tekstnet.dk/books/oersted-hc_experimenta.en/001/)
- 50 齿、7.2° 齿距及四分之一齿距的步距与制造商资料核对。八组各六齿及组间多留的角度按本课图示计算。 [Oriental Motor 技术说明](https://www.orientalmotor.com/stepper-motors/technology/stepper-motor-basics.html)
