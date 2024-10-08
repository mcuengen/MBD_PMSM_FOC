# PMSM的I/F开环控制

## 一.  电流频率比I/F控制原理

​      PMSM/BLDC的恒压频比V/F控制是保持电机的电压和频率之比固定，即磁通为常数，既不需要转速闭环控制，也不需要进行电流采样，是一种完全的开环控制方式。VF控制有两个明显的不足：不具备负载转矩匹配能力，转速容易产生振荡；最佳V/F曲线的整定比较困难，容易引起电机过电流。

​        低速阶段一般采用高频注入法进行磁链位置的估计，利用转子自身结构的凸极效应或者人为造成的电感饱和特性来追踪转子磁链位置。采用高频注入法须额外注入高频信号， 会造成高频噪声，而且算法较为复杂，对于控制芯片的性能要求较高。而I/F控制开环启动具有易于实现的优势。

![IF_PMSM_SCH_01](./assets/a/IF_PMSM_SCH_01.png)

​                                                                              I/F控制系统框图

​        将位置角发生器给定的旋转坐标系定义为虚拟同步坐标系dq-v，以电机转子为基准的坐标系为dq坐标系，两坐标系的相位差为$$/theta_L$$。为了使转子更容易被拖入同步，dq-v坐标系的初始位置设定为落后dq坐标系90°。



![IF_PMSM_VECTOR_01](./assets/a/IF_PMSM_VECTOR_01.png)

​         这时候d轴与q-v轴重合，电流id\*=id=0，iq\*=iq=常数，电流给定矢量iq\*从d轴开始逐渐加速旋转，电磁转矩牵引永磁体转子平滑的加速转动，并保持一定相位差。电磁转矩为：
$$
T_e = \frac{3}{2}n_pi_q^*cos(\theta_L)[\psi_f+(L_d-L_q)i_q^*sin{\theta_L}]
$$


​        对于表贴式电机而言，由于直轴和交轴电感相等，即*Ld=Lq*，电磁转矩可以表示为：
$$
T_e=\frac{3}{2}n_p i_q^* cos(\theta_L)\psi_f  \,.
$$
​      iq\*为正值时，电磁转矩为正，电机正向旋转；iq\*为负值时，电磁转矩为负，电机反向旋转。iq\*为定值时，q轴上的转矩电流为iq\*cosθL，相位差θL.决定了电磁转矩的大小。其转矩平衡方程式可以表示为：

​                                              
$$
T_e - T_L = \frac{J}{n_p}.\frac{d\omega_{re}}{dt} = \frac{J}{n_p}.\frac{d^2\theta_{re}}{dt}
$$
​     只要电机提供足够大的电磁转矩保证上式能够平衡，电机就不会失步。。以电机正转为例：当电机进入稳态时，电磁转矩Te与负载转矩TL相等，若负载转矩突然变大，则转子的转速下降，使虚拟同步坐标系与转子同步坐标系的相位差0.变小，进而导致电磁转矩Te的提高，达到新的平衡。负载转矩突然变小的分析过程可以此类推。此为I/F控制的“转矩—功角自平衡”特性。

​        由以上分析可知，电流给定iq\*幅值越大，稳态下的相位差（也越大，越接近π/2。相位差0.越大则电机的电磁转矩储备越大，电机抵抗负载扰动的能力越强。电流给定幅值越小，稳态下的相位差theta_L.也越小，越接近0, 则储备功率越小，当theta_L<0 , 电机失步。



## 二.  I/F控制系统Matlab/Simulink仿真分析



###      2.1 整体组成

​       基于Simulink的I/F控制模型主要分为全局变量、按键使能、ADC采样完成触发、控制环（电流环）、PMSM\BLDC仿真模型、上位机数据监控。设置ADC采样频率为10kHz, 每次ADC触发控制环运行，控制环完成运算输出控制逆变器的PWM信号，下一个采样周期继续重复。使能按键控制电机的转/停，使用Variant 变体控制仿真信号和硬件信号的条件自动切换。 通过信号线表明各个模块的信号传递，对于多次使用的变量，这里使用data storage memory设置全局变量。

![IF_PMSM_OpenLoop_01](./assets/a/IF_PMSM_OpenLoop_01.png)



### 2.2   I/F控制算法

​        这里将I/F 控制算法设计成4个阶段，第0阶段，Enable=0, 电机处于停止状态；第1阶段，Enable=1, 电机d轴和给定q-v轴对齐; 第2阶段，给定Iq\* >0， 转速W_re 从0逐渐增加，电机从0开始转动；第3阶段，给定Iq\* >0 ， 转速 W_re 恒定，电机的转速趋近于给定转速。为了方便，这里将2、3合并一起。该部分主要输出位置theta、q轴电流Iq\*。

![IF_PMSM_Strategy_01](./assets/a/IF_PMSM_Strategy_01.png)



​       为了实现转速W_re 从0逐渐增加到指定值，这里使用饱和积分器来实现，输入时间系数能调节斜坡的斜率，1 、5表示5s上升1，速度积分得到位置，Position Generator生成开环拖动位置。

![IF_PMSM_IFGEN_01](./assets/a/IF_PMSM_IFGEN_01.png)

​      这里使用Stateflow对不同状态的离散系统进行建模，通过信号箭头和条件来构建不同状态的转换，当Enable=0, 状态全部转换为Motor_state=0, 在1->2->3之间，通过时间段计数完成切换。

![IF_PMSM_Stateflow_01](./assets/a/IF_PMSM_Stateflow_01.png)

###  2.3 其他模块

![IF_PMSM_IFPI_01](./assets/a/IF_PMSM_IFPI_01.png)

​                                                                    Id Iq的PI Controller 

![IF_PMSM_IFPWM_01](./assets/a/IF_PMSM_IFPWM_01.png)

通过电流ia、ib计算dq轴调制电压Ud 、Uq, 首先经过Clark变换将ab->alph/beta, 再经过Park变换将alph\beta->dq，经过Idq Controller计算Ud、Uq，经过inverse Park 变换，将dq->alph/beta，在生成SVPWM调制信号。

### 2.4 仿真结果

下图为电机转速曲线，可以看出电机在初始时刻保持禁止，随着拖动，电机转速匀速提升，最后稳定在2000rpm左右。

![IF_PMSM_speed_plot_01](./assets/a/IF_PMSM_speed_plot_01.png)

下图为给定电角度theta，和电机的实际电角度Pos_PU，POS_PU滞后theta一个固定的相位，电机达到功角平衡状态。

![IF_PMSM_Theta_01](./assets/a/IF_PMSM_Theta_01.png)



## 三.  I/F控制系统STM32实验

### 3.1 硬件平台

​       主控MCU采用STM32G474RET6，三相逆变器使用IHM07M1，电源采样24V开关电源。

![pmsm_IF_hardware_01](./assets/a/pmsm_IF_hardware_01.jpg)

主控板通过usb与电脑上位机相连，上位机这里采样simulink开发的模型，通过界面交互向mcu发送指令，使用scope可视化展示mcu处理的数据。



![pmsm_IF_exphost_01](./assets/a/pmsm_IF_exphost_01.jpg)

3.2 STM32G4配置

​       CUBEMX配置STM32G474RET6的资源，包括定时器pwm输出，ADC采样通道，button按键，逆变器使能输出，UART端口。

![CUBEMX_IOC_01](./assets/a/CUBEMX_IOC_01.png)

需要在Simulink模型的硬件设置进行配置，需要根据mcu的使用情况对ioc文件，端口，adc等资源进行设置。

![SW_CONFIG_01](./assets/a/SW_CONFIG_01.png)



### 3.3 实验数据

为便于对信号进行处理，这里采样标幺的处理。

电流ia, ib 的数据如下，接近正玄波。

![CURRENT_IAB_01](./assets/a/CURRENT_IAB_01.png)

Id Iq 的波形，黄色为Id, 蓝色为Iq, id\* = 0, iq\* = 0.25。

![CURRENT_Idq_01](./assets/a/CURRENT_Idq_01.png)



Iq跟踪效果如下

![CURRENT_Iq_ref_01](./assets/a/CURRENT_Iq_ref_01.png)
