# PMSM的 Sensorless FOC



- 基于观测器估计转子位置、转速，设计无感FOC的电流、速度双闭环的调速系统。
- 通过Simulink对Sensorless FOC建模，仿真，代码生成。
- 基于Simulink搭建上位机，观察电流ia/ib, Id/iq, 位置theta，速度speed。
- 在STM32G4 嵌入式平台上调试，测试。

## 一.  PMSM/BLDC的无感FOC原理

###  1.1 磁链观测器

​     磁链观测器（Magnetic [Flux](https://so.csdn.net/so/search?q=Flux&spm=1001.2101.3001.7020) Observer）是一种在电机控制系统中常用的技术方法，尤其在VESC（Vedder电子速度控制器）中得到了广泛的应用。它的作用是实时观测电机内部的磁链，并将其转化为电压信号。alph-beta轴的磁链\psi_\alpha、\psi__\beta，的方程为：
$$
\psi_{\alpha} =\int{(V_{\alpha}-I_{\alpha}R)dt}-(L_s.I_{\alpha})\\
\psi_{\beta}= \int{(V_{\beta}-I_{\beta}R)dt}-(L_s.I_{\beta})
$$
设定
$$
V_{\alpha}-I_{\alpha}R = v_1(t)\\
V_{\beta}-I_{\beta}R = v_2(t)
$$
alph-beta轴的磁链\psi_\alpha、\psi__\beta的积分项的Laplace表示为：
$$
L(\int{v_1(t)}dt)=\frac{1}{s}L(v_1(t)) = \frac{1}{s}v_1(s)\\
L(\int{v_2(t)}dt)=\frac{1}{s}L(v_2(t)) = \frac{1}{s}v_2(s)
$$
这里引入截止频率为omega_c的LPF来对积分项处理，这样能消除DC offset
$$
L(\int{v_1(t)}dt)=\frac{1}{s+\omega_c}v_1(s)\\
L(\int{v_2(t)}dt) = \frac{1}{s+\omega_c}v_2(s)
$$
如前所述，基于 LPF 的积分器使该模块能够消除 Ψα 和 Ψβ 中可用的任何直流分量。
$$
\psi_{\alpha} = \frac{L_r}{L_m}(\int{(V_{\alpha}-I_{\alpha}R)dt}-\sigma L_s.I_{\alpha})\\
\psi_{\beta} = \frac{L_r}{L_m}(\int{(V_{\beta}-I_{\beta}R)dt}-\sigma L_s.I_{\beta})
$$
转子位置估算、转矩、磁通为：
$$
\sigma=1 - \frac{L_m^2}{L_r.L_s}\\
\theta_e = tan^{-1}\frac{\psi_\beta}{\psi_\alpha}\\
T_e = \frac{3}{2}.P.\frac{L_m}{L_r}(\psi_{\alpha}I_{\beta}-\psi_{\beta}I_{\alpha})\\
\psi = \sqrt{\psi_{\alpha}^2+\psi_{\beta}^2}
$$


### 1.2 开环转闭环的切换策略

​    I/F控制，开环拖动电机运行稳态后，拖动电角度theta_e_re与电机电角度theta_e之间纯在相位差，theta_L。 在切换到采用磁链观测器的无位置传感器闭环算法时，容易引起电流冲击和转速波动，甚至导致切换失败。

论文“A Simple Startup Strategy Based on Current Regulation for Back-EMF-Based Sensorless Control of PMSM”提到了一种简单的IF切换闭环的策略，通过拖动电角度和观测器估计电角度之间的平滑过渡，而角度是角速度的积分，进一步转矩Te决定了角速度w的增减。
$$
T_e = \frac{3}{2}n_p i_q^*cos{\theta_L}(\psi_{m}+(L_d-L_q)i_q^* sin{\theta_L})\approx K_T i_q^*cos{\theta_L}
$$
转矩方程
$$
T_e - T_L = J\frac{\Omega_r}{dt}= \frac{J}{n_p}\frac{\omega_r}{dt}
$$
功率角theta_L
$$
\frac{d\theta_L}{dt} = \omega_r - \omega_e
$$
在I/F开环拖动电机转速稳定后，通过线性衰减iq\* ,让电机的拖动电角度theta_re趋近于观测器电角度theta_hat，iq\* 衰减公式为：
$$
i_q^* = i_{q0}^* - K_i(t - t_0)
$$
​       I/F过渡的电机转速，电角度相位差，转矩电流iq\* 的变化过程如下图所示，在2.5s左右，iq\* 开始衰减，电角度相位差衰减逐渐衰减到接近0，在此过程中，电机转速保持恒定。



![](.\img\02\Iq_decay_set_02.png)

当相位差小于阈值(>0), 即可停止iq\*衰减。



## 二.  PMSM/BLDC的无感FOC建模



​          通过simulink 对控制系统进行建模，分为全局变量、低频任务500Hz（计算速度环PI， 按键读取，给定速度信号处理）, 高频任务10kHz（电流环），ADC采样完成触发、仿真模型、串口读取指令。

![](.\img\02\Sensorless_FOC_SYS_02.png)



​         这里将电机的控制状态分为4个阶段，第0阶段，Enable=0,电机停止转动；第1阶段，设置theta=0, Iq\* =初始值，电机对齐；第2阶段，电机处于|/F开环拖动，先线性增加拖动角速度，达到设定值后电机保持匀速转动，然后线性衰减Iq\* ,使得拖动电角度和观测器电角度的相位差小于设定阈值时，转矩电流iq\* 停止衰减，切换到下一阶段；第3阶段，通过观测器估计的电角度和速度进行闭环FOC控制，通过EnCloseLoop状态变量，来判断电机是否处于闭环状态。

![](.\img\02\FOC_Change_Strategy_01.png)

​       第2阶段，主要通过几个积分器来实现斜坡函数，当电机的钻速达到预定的转速时，这里需要线性递减iq\*, 同时计算拖动电角度和观测器电角度的相位差，直到切换到下一阶段。

![](.\img\02\Iq_ramp_decay_code_02.png)



几种状态之间的切换关系如下图所示：

![](.\img\02\stateflow_foc_02.png)

​      电流计算以及设置过流保护阈值

![](.\img\02\Current_read_02.png)



电流环PI计算，以及SVPWM调制， 信号监控，pwm信号输出等。

![](.\img\02\CurrentLoop_SVPWM_Gen_02.png)

## 三  PMSM/BLDC的仿真



仿真bldc模型

![](.\img\02\Sim_Plant_02.png)