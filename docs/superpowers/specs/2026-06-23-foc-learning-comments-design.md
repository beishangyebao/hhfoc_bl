# FOC 工程教学型注释设计

## 目标

为本工程的核心控制代码和必要的 STM32 外设衔接代码补充中文教学型注释，使读者主要通过函数头注释和函数内流程注释，就能理解：

- 每个模块解决什么问题；
- 每个函数在整个控制链中的位置；
- 参数、返回值及关键变量的物理意义和单位；
- 数据从传感器采集到闭环控制、再到 PWM 输出的流向；
- 初始化顺序、1 ms 调度方式和中断回调关系；
- SPWM、SVPWM、位置环、速度环、Iq 电流环之间的关系。

本次工作只增加或改进注释，不修改函数签名、表达式、控制参数或运行逻辑。发现疑似缺陷时，只在注释中客观说明当前代码的实际行为和阅读注意点，不借机修复。`hhPID` 和 `hhLowPassFilter` 两个模块已有用户自写注释，本次不修改它们。

## 注释风格

### 函数头

每个函数采用中文 Doxygen 风格，按实际需要包含：

- `@brief`：一句话说明函数职责；
- “所在流程”：说明调用者和下游函数；
- `@param`：参数含义、单位、有效范围；
- `@return`：返回值含义和单位；
- `@note`：调用周期、前置条件、副作用、共享状态和容易误解的地方。

### 函数内部

按“步骤 1、步骤 2……”或功能段落解释算法流程，重点解释：

- 为什么执行这一步；
- 输入数据来自哪里；
- 变换前后量的物理意义；
- 结果被哪个下游环节使用；
- 电机 M0/M1 与 ADC、定时器通道、GPIO 引脚的对应关系。

不对明显的赋值、返回语句做机械逐行翻译，以免真正关键流程被注释淹没。

### 头文件

头文件补充：

- 模块总体职责和上下游依赖；
- 枚举、结构体及每个成员的意义和单位；
- 全局对象的用途；
- 每个公开 API 的函数头注释。

## 文件范围

### 第一组：用户指定的 10 个核心文件

- `Core/hhAS5600/hhAS5600.c`
- `Core/hhAS5600/hhAS5600.h`
- `Core/hhFoc/hhFoc.c`
- `Core/hhFoc/hhFoc.h`
- `Core/hhPwm/hhPwm.c`
- `Core/hhPwm/hhPwm.h`
- `Core/hhSerial/hhSerial.c`
- `Core/hhSerial/hhSerial.h`
- `Core/hhGetIq/hhGetIq.c`
- `Core/hhGetIq/hhGetIq.h`

### 第二组：理解工程入口、硬件映射和调度必须阅读的 14 个文件

- `Core/Src/main.c`
- `Core/Inc/main.h`
- `Core/Src/adc.c`
- `Core/Inc/adc.h`
- `Core/Src/tim.c`
- `Core/Inc/tim.h`
- `Core/Src/gpio.c`
- `Core/Inc/gpio.h`
- `Core/Src/usart.c`
- `Core/Inc/usart.h`
- `Core/Src/dma.c`
- `Core/Inc/dma.h`
- `Core/Src/stm32f1xx_it.c`
- `Core/Inc/stm32f1xx_it.h`

### 第三组：建议阅读但本次不改注释的 4 个文件

- `Core/hhPID/hhPID.c`
- `Core/hhPID/hhPID.h`
- `Core/hhLowPassFilter/hhLowPassFilter.c`
- `Core/hhLowPassFilter/hhLowPassFilter.h`

这 4 个文件仍是学习闭环控制时需要看的支撑模块，但注释由用户自行维护，本次保持原样。


CubeMX 生成文件中的新增说明优先放在 `USER CODE` 区域；必须解释生成配置时，可在对应初始化代码附近增加注释，并明确这些注释在重新生成工程后可能被覆盖。

## 不修改的文件

- `Drivers/`：STM32 HAL 和 CMSIS 通用库，不属于本工程控制逻辑，学习时只需按调用点查阅。
- `25.SvpwmCloseLoopPosWithVelWithIq.ioc`：作为引脚和外设配置依据，只读不改；文件首行明确要求不要手工修改。
- `Debug/`：编译产物和自动生成的 Makefile，不作为源码学习重点。
- 启动文件和系统库文件：除非排查启动或链接问题，否则不需要作为本工程 FOC 学习主线。

## 需要讲清楚的主数据流

1. `TIM4` 每 1 ms 产生中断。
2. HAL 中断处理进入 `HAL_TIM_PeriodElapsedCallback()`，设置 `Flag_1ms`。
3. `main()` 主循环看到标志后调用 `UpdateAllSensor()`。
4. AS5600 软件 I2C 读取机械角度，展开多圈角度并计算速度、滤波速度。
5. ADC1 连续扫描四个通道，DMA 循环更新两台电机的两相电流采样值。
6. `UpdatePhaseCurrent()` 将 ADC 值换算为相电流，再做 Clarke/Park 变换得到 `Iq`。
7. 位置环输出目标速度，速度环输出目标 `Iq`，Iq 环输出目标 `Uq`。
8. `setPhaseVoltage()` 根据电角度完成逆 Park 变换，并走 SPWM 或 SVPWM 分支。
9. `hhPwm` 把占空比或 CCR 比较值写入 TIM2/TIM3 通道，最终输出六路 PWM。
10. USART1 接收外部目标值，并把控制量或传感器量输出到上位机。

## 验证标准

- 24 个目标文件均有清晰的模块说明。
- 所有自定义函数和对理解流程关键的 CubeMX 初始化/中断函数均有函数头说明。
- 复杂函数内部按算法阶段分段注释，尤其是 `setPhaseVoltage()`、AS5600 更新、电流变换和三级串级闭环。
- 注释明确常用单位：弧度、弧度每秒、安培、伏特、秒、ADC 计数值和定时器 CCR。
- 注释明确 M0/M1 与 ADC 通道、PWM 通道、AS5600 GPIO 的映射。
- 对修改前后文件去除空白和注释后进行比较，确认可执行代码未变化。
- 使用现有 STM32CubeIDE Makefile 或可用工具执行一次构建；若当前生成目录或工具链不可移植，则至少完成语法级和预处理级检查，并报告限制。

## 最终交付

- 完成上述 24 个文件的教学型注释。
- 给出“建议阅读顺序”，从 `main.c` 到三级闭环、传感器、PWM 和底层外设。
- 单独列出阅读时应特别留意的当前实现行为或潜在问题，但不在本次注释任务中修改它们。

## 自审结果

- 无待定项或占位内容。
- 范围覆盖用户指定文件及理解工程所需的全部直接依赖。
- 注释任务与代码修复任务边界明确。
- 自动生成文件的注释保留风险和 `.ioc` 只读要求已明确。
