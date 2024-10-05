# 电磁循迹小车赛后总结

>   MCU选用：STM32F103C8T6
>
>   编程语言：C语言
>
>   开发工具：MDK Keil，CubeMX
>
>   所用开发库：HAL库
>
>   适用读者: 初步接触STM32,没有制作智能车的实战经验等嵌入式入门读者.

[TOC]

## 一.小车架构

### 1.硬件部分

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/%E7%A1%AC%E4%BB%B6%E9%83%A8%E5%88%86-165270141424418.png" style="zoom:40%;" />

>   在硬件部分我只列举了做一个简易电磁车最基本的部件，蓝色字体是我们所选用的部件。
>
>   在上面所列出的小车底板、控制板、和电磁杆中，控制板和电磁杆是由我的队员用EDA软件画板，开板		再焊接制成的，其余一些模块都是通过购买所得。

### 2.软件部分

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/%E8%BD%AF%E4%BB%B6%E9%83%A8%E5%88%86.png" style="zoom:40%;" />

### 3.控制逻辑

<img src="E:/%E5%8D%9A%E5%AE%A2/%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6/Markdowm%E7%B4%A0%E6%9D%90/%E6%8E%A7%E5%88%B6%E9%80%BB%E8%BE%91.png" style="zoom:40%;" />

## 二.关键模块说明

### 1.电磁杆（电磁传感器）

>   下面有关电磁传感器的内容摘自 ***王盼宝《智能车制作-从元器件、机电系统、控制算法到完整的智能车设计》**清华大学出版社出版*，如有侵权请联系我删除。

#### Ⅰ.电感传感器原理

根据电磁学相关知识，我们知道在导线中通人变化的电流（如按止弦规律变化的电流）.则导线周围会产生变化的磁场，且磁场与电流的变化规律具有一致性，如果在此磁场中置一个电感，则该电感上会产生感应电动势，且该感应电动势的大小和通过线圈回路的磁通量的变化率成正比。由于在导线周围不同位置，磁感应强度的大小和方向不同，所以不同位置上的电感产生的感应电动势也应该不同。据此，则可以确定电感的大致位置。

#### Ⅱ.磁传感器信号处理电路

确定使用电感作为检测导线的传感器，但是其感应信号较微弱，且混有杂波，所以要进行信号处理。要进行以下三个步骤才能得到较为理想的信号：信号的滤波，信号的放大，信号的检波。

##### 1）信号的滤波

比赛选择20kHz的交变电磁场作为路径导航信号，在频谱上可以有效地避开周围其他磁场的干扰，因此信号放大需要进行选频放大，使得20kHz的信号能够有效地放大，并且去除其他干扰信号的影响。使用LC串联谐振电路实现选频电路（带通电路），具体电路如下图所示。

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/5A9FECD994D1EEF620A84219BDB22C0E.jpg" style="zoom: 27%;" />

<center><p>LC谐振电路</p></center>


​																		

其中，$E$是感应线圈中的感应电动势，$L$是感应线圈的电感值，$R_{0}$主要是电感的内阻，$C$是谐振电容。电路谐振频率为：
$$
f =\frac{1}{2π\sqrt{LC}}
$$
已知感应电动势的频率$f=20kHz$，感应线圈电感为$L=10mH$，可以计算出谐振电容的容量为$C=6.33×10^{-9}F$。通常在市场上可以购买到的标称电容与上述容值最为接近的电容为$6.8nF$，所以在实际电路中选用$6.8nF$的电容作为谐振电容。

##### 2）信号的放大

第一步处理后的电压波形已经是较为规整的20kHz正弦波，但是幅值较小，随着距离衰减很快，不利于电压采样，所以要进行放大，官方给出了如下图所示的参考方案，即用三极管进行放大，但是用三极管放大有一个不可避免的缺点就是温漂较大，而且在实际应用中静电现象严重。

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/%E5%85%B1%E5%B0%84%E4%B8%89%E6%9E%81%E7%AE%A1%E6%94%BE%E5%A4%A7%E7%94%B5%E8%B7%AF.jpg" style="zoom:27%;" />
<center><p>共射三极管放大电路</p></center>

因此我们放弃三级管放大的方案，而是采用集成运放进行信号的放大处理，集成运放较三极管优势是准确、受温度影响很小、可靠性高。集成运放放大电路可构成同相比例运算电路和反相比例运算电路，在实际中使用反相比例运算电路。由于运放使用单电源供电，因此在同相端加$V_{cc}/2$（典型值）的基准电位，基准电位由两个阻值相等的电阻分压得到。

##### 3）信号的检波

测量放大后的感应电动势的幅值 $E$ 可以有多种方法。最简单的方法是使用二极管检 波电路将交变的电压信号检波形成直流信号，然后再通过单片机的AD采集获得正比于感 应电压幅值的数值。

我们采用的是竞赛组委会给出的第一种方案，即使用两个二极管进行倍压检波。倍压检波电路可以获得正比于交流电压信号峰-峰值的直流信号。为了能够获得更大的动态范围，倍压检波电路中的二极管推荐使用肖特基二极管或者锗二极管。由于这类二极管的开启电压一般在0.1～0.3V，小于普通的硅二极管（0.5～0.7V），可以增加输出信号的动态范 围和增加整体电路的灵敏度。这里选用常见的肖特基二极管1N5817。

最终确定下来的电路方案如下图所示：

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/%E6%9C%80%E7%BB%88%E6%96%B9%E6%A1%88%E7%94%B5%E8%B7%AF.jpg" style="zoom:40%;" />
<center><p>最终方案电路</p></center>

#### Ⅲ.磁传感器的布局原理及改进

对于直导线，当装有小车的中轴线对称的两个线圈的小车沿其直线行驶，即两个线圈的位置关于导线对称时，则两个线圈中感应出来的电动势大小应相同且方向亦相同。若小车偏离直导线，即两个线圈关于导线不对称时，则通过两个线圈的磁通量是不一样的。这时，距离导线较近的线圈中感应出的电动势应大于距离导线较远的那个线圈中的。根据这两个不对称的信号的差值，即可调整小车的方向，引导其沿直线行驶。

对于弧形导线，即路径的转弯处，由于弧线两侧的磁力线密度不同，则当载有线圈的小车行驶至此处时，两边的线圈感应出的电动势是不同的。具体的情况是，弧线内侧线圈的感应电动势大于弧线外侧线圈的，据此信号可以引导小车拐弯。

另外，当小车驶离导线偏远致使两个线圈处于导线的一侧时，两个线圈中感应电动势也是不平衡的。距离导线较近的线圈中感应出的电动势大于距离导线较远的线圈。由此，可以引导小车重新回到导线上。

由于磁感线的闭合性和方向性，通过两线圈的磁通量的变化方向具有一致性，即产生的感应电动势方向相同，所以由以上分析，比较两个线圈中产生的感应电动势大小即可判断小车相对于导线的位置，进而做出调整，引导小车大致循线行驶。

采用双水平线圈检测方案，在边缘情况下，其单调性发生变化，这样就存在一个定位不清的区域（如下图箭头所指）。同一个差值，会对应多个位置，不利于定位。另外，受单个线圈感应电动势的最大距离限制，两个线圈的检测广度很有限。

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/%E5%8F%8C%E7%BA%BF%E5%9C%88%E5%B7%AE%E5%80%BC%E6%B3%95%E6%9C%89%E5%AE%9A%E4%BD%8D%E4%B8%8D%E6%B8%85%E5%8C%BA%E5%9F%9F.jpg" style="zoom:50%;" />
<center><p>双线圈差值法有定位不清区域</p></center>

现提出一种优化方案，5个垂直放置的电感按一字排布，每个电感相距约为5cm（见下图），这样覆盖赛道范围约为25cm。三个一字排布的电感可以大大提高检测密度和广度，向前有两个电感，可以提高前瞻，改善小车入弯状态和路径，两个45°的电感，可以改善入弯和出弯的姿态。

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/%E7%94%B5%E6%84%9F%E6%8E%92%E5%B8%83%E6%A3%80%E6%B5%8B%E6%96%B9%E6%A1%88.jpg" style="zoom:50%;" />
<center><p>电感排布检测方案</p></center>

#### Ⅳ．谐振电路的改进

按照组委会推荐的10mH＋6.8nF组成的谐振电路，其谐振的峰值频率为19.6kHz，并不是信号发生器的20kHz。当谐振频率与外界激励频率不匹配时，将不会输出最大的谐振电压。另外，一般电感和电容均有±20％的误差，谐振频率将会随机分布在16kHz～24kHz之间，这会对传感器的对称性造成极大的影响。因此需要对组成谐振电路的电感电容进行匹配，使得其谐振频率恰好为20kHz，同时挑选电感电容对，使得对称位置的输出电压一致。

电磁杆的具体制作可以参考我队友的博客：

[智能车：这是你要找的电磁杆吗？_悟黎678的博客-CSDN博客](https://blog.csdn.net/qq_62774677/article/details/124547276?utm_source=app&app_version=5.0.1&code=app_1562916241&uLinkId=usr1mkqgl919blen)

### 2.L298N电机驱动模块

L298N模块的介绍可以参考以下博主的博客

[【STM32小案例 04 】STM32简单使用L298N电机驱动模块 控制直流电机正反转_我也不知道取什么好的博客-CSDN博客_stm32电机控制](https://blog.csdn.net/teavamc/article/details/77429519)

[stm32单片机+驱动L298N控制直流电机调速_薄情书生的博客-CSDN博客_l298n控制电机转速](https://blog.csdn.net/weixin_53402301/article/details/119420281)

[L298N电机驱动模块详解_魏波-的博客-CSDN博客_l298n电机驱动模块](https://blog.csdn.net/weibo1230123/article/details/80793905)

[基于L298N的STM32的直流电机PWM调速控制_Hu.先森的博客-CSDN博客_l298n电机驱动模块pwm调速](https://blog.csdn.net/weixin_38679101/article/details/84075905)

### 3.HC-05蓝牙模块

**在此次比赛中我们主要使用蓝牙模块来返回小车调试过程中的赛道信息，以及通过蓝牙模块给小车发送相关指令以控制小车电机的启动与停止和更改小车控制算法的一些参数。**

相关资料可以参考以下博主的博客：

[STM32控制HC-05蓝牙模块进行通信_Zach_z的博客-CSDN博客_hc05 stm32](https://blog.csdn.net/Zach_z/article/details/72784369)****

[【常用模块】HC-05蓝牙串口通信模块使用详解（实例：手机蓝牙控制STM32单片机）_Yngz_Miao的博客-CSDN博客_stm32蓝牙串口通信](https://blog.csdn.net/qq_38410730/article/details/80368485)

### 4.直流减速电机

可以参考以下博主的博客：

[直流电机的原理及驱动_star-air的博客-CSDN博客_直流电机驱动](https://blog.csdn.net/qq_41262681/article/details/95319321)

[stm32、直流减速电机（接线、编码器代码详解）_wzyannn的博客-CSDN博客_stm32与编码器接线](https://blog.csdn.net/m0_51951121/article/details/121759468)

### 5.模拟舵机

可以参考以下博主的博客：

[单片机——SG90舵机工作原理_掏一淘哆啦A梦的奇妙口袋的博客-CSDN博客_sg90舵机原理图](https://blog.csdn.net/qq_41873236/article/details/116353829)

[stm32之MG995舵机+原理+程序+详解_-electronic-engineer的博客-CSDN博客_mg995舵机](https://blog.csdn.net/qq_45941706/article/details/108951250)

[STM32控制舵机讲解，从入门到放弃。_KING_阿飞的博客-CSDN博客_stm32控制舵机](https://blog.csdn.net/qq_42866708/article/details/113355329)

### 6.干簧管

相关原理可以参考如下链接：

[什么是干簧管（一）|什么是磁簧开关|干簧管原理视频|干簧管工作原理-产品知识-资讯-深圳华壬电子 (chinahuaren.com)](http://www.chinahuaren.com/shen-me-shi-gan-huang-guan-yi/582.html)

**在此次比赛中我们需要用的干簧管来检测车库外所防止的磁铁以实现入库操作。**

**干簧管的使用较为简单，可以把他看作一个开关使用，从MUC上分配处一个GPIO口给干黄管的一端，另一端接地，通过捕捉该GPIO口的下降沿触发外部中断即可。**

### 7.OLED屏幕

**OLED的屏幕主要用于显示ADC采集到的赛道值用以确定特征位置如环岛、Y叉等处的特征标志值方便对算法的相关判断阈值进行更改。同时我们也通过OLED屏幕显示当前小车的PID参数以方便记录。**

OLED的相关使用方法可参考以下博主的博客：

[STM32复习笔记（九）OLED的介绍和使用方法_Sumjess的博客-CSDN博客_oledshowstring函数什么意思](https://blog.csdn.net/qq_38351824/article/details/82621675)

[12. STM32——硬件IIC驱动OLED屏幕显示_ZCY(Yinyuer1)的博客-CSDN博客_oled屏幕](https://blog.csdn.net/weixin_46105931/article/details/120786031)

### 8.Wifi模块-ESP8266

**该模块可以完成同蓝牙模块的相同操作，同时搭配Vofa+软件的话可以实现可视化调参。**

相关使用方法可参考以下博主的博客：

[esp8266介绍和使用_世界著名CV工程师的博客-CSDN博客_esp8266介绍](https://blog.csdn.net/weixin_44178250/article/details/97614719)

[烂大街的ESP8266该怎么玩！ - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/166536234)

## 三.CubeMX配置

CubeMX如何配置的总体介绍可以参考以下博主的博客:

[cubemx代码生成详解 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/377844152)

### 1.引脚预览

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/CubeMX%E9%85%8D%E7%BD%AE.png" style="zoom:50%;" />

### 2.ADC

ADC我们配置了6路通道，ADC1五路，ADC2一路。最初方案我们配置了七个通道，ADC1五路，ADC2两路，我们最初也只是使用了五个电感，所以只用了ADC1的五路通道，ADC2多出来的两路当时是备用。但是在调车过程中我们发现原先中间的电感旋转角度无法兼顾环岛与Y叉标志值的检测（可能是我们的设计存在问题），因此我们在中间多加了一路电感和原先的中路电感垂直，用于标志值的检测，并关闭了ADC２的一路通道方便代码编写。

*ADC1*:

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/ADC%E9%85%8D%E7%BD%AE1%20(2).png" style="zoom:50%;" />

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/ADC%E9%85%8D%E7%BD%AE2.png" style="zoom: 50%;" />

*ADC２*:

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/ADC%E9%85%8D%E7%BD%AE3.png" style="zoom:50%;" />

ADC的相关配置可参考以下博主的博客:

[STM32 ADC详细篇（基于HAL库） - 东小东 - 博客园 (cnblogs.com)](https://www.cnblogs.com/dongxiaodong/p/14355843.html)

### 3.TIM

>   TIM1用于输出控制电机两轮转速的PWM波

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/TIM1_1.png" style="zoom:50%;" />

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/TIM1_2.png" style="zoom:50%;" />

>   TIM2用于输出控制舵机的PWM波

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/TIM2.png" style="zoom:50%;" />

>   TIM3中断用于控制算法的执行

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/TIM3.png" style="zoom:50%;" />

>   TIM4中断用于一些需要计时函数的执行

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/TIM4.png" style="zoom:50%;" />

TIM的相关配置可以参考以下博主的博客:

[STM32CubeMX之定时器TIM_while(1)的博客-CSDN博客_stm32cubemx tim](https://blog.csdn.net/qq_42900996/article/details/110259963)

[STM32对HAL库的PWM控制 - 无乐不作丶 - 博客园 (cnblogs.com)](https://www.cnblogs.com/zjx123/p/11871699.html)

### 4.I2C

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/I2C.png" style="zoom:50%;" />

### 5.USART

我们配置了两个USART,比赛中只用到一个USART1

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/USART_1.png" style="zoom:50%;" />

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/USART_2.png" style="zoom:50%;" />

### 6.NVIC

**在NVIC的配置中要根据自己的控制逻辑来配置他们的中断优先级**

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/NVIC.png" style="zoom:50%;" />

## 四.小车代码编写

### 1.基础框架的搭建

在写代码之前可以像我上面一样**画出整个代码的架构和控制逻辑的思维导图**(*所用软件:uTools插件-知犀思维导图|uTools官网下载:[uTools官网 - 新一代效率工具平台](https://u.tools/))*.然后**根据代码的整体架构编写相关独立函数先初步搭建出基础框架,如相关变量的结构体,外围功能的函数等**:

```c
/*---------------------- 相关结构体定义 -------------------------- */
/**
	*@name		MotorDriver
	*@type		struct array
	*@about 	电机控制
	*@param
	*		- MotorDriver[0]		电机1
	*		- MotorDriver[1]		电机2
	*		--
	*		- Onoff			电机启动与停止标志位
	*		- pwmrate		传给Tim -> CRR的值，可改变PWM占空比
	*		- IN1			L298n IN1逻辑控制引脚
	*		- IN2			L298n IN2逻辑控制引脚
	*/
typedef struct{
	
	_Bool	OnOff;
	_Bool	IN1;
	_Bool 	IN2;
	float	pwmrate;
	
}MotorDriver;

/**
	*@name		MotDiff
	*@type		struct
	*@about 	电机差速控制
	*@param		
	*		--
	*		-	Param			增幅系数
	*		-	pwmSwitch		电机差速跟随转向控制开关
	*/
struct{
	
	_Bool		pwmSwitch;
	uint16_t	basepwmvalue;
	float 		Param;
	
}MotDiff;

/**
	*@name		SteMotDriver
	*@type		struct
	*@about 	舵机控制
	*@param
	*		- pwmrateTemp		pwm控制过渡值
	*		- pwmrateFnal		pwm控制最终值
	*		- Min				舵机中值
	*		- angle				舵机目前角度
	*/
typedef struct{
	
	uint8_t		angle;
	float		pwmrateTemp;	
	float   	pwmrateFnal;
	float		Min;
    
}SteMotDriver;


/**
	*@name		ADCData
	*@type		struct
	*@about 	ADC数据
	*@param
	*		- origanlData[]		ADC采集到的原始数据
	*		- filterData[]		滤波处理后的数据
	*		- IDUC_L			左电感
	*		- IDUC_R			右电感
	*		- IDUC_M			中电感
	*		- IDUC_LM			左中电感
	*		- IDUC_RM			右中电感
	*		- Error				差值
	*/
typedef struct{
	
	__IO	uint16_t		orignalData[5];
	__IO	float			filterData[5];
	__IO	float			IDUC_L;
	__IO	float			IDUC_R;
	__IO	float			IDUC_M;
	__IO	float			IDUC_LM;
	__IO	float			IDUC_RM;
	__IO	float			IDUC_Ex;
	__IO	float			Error;
	
}ADCData;


/**
	*@name	PoorCmpAnd
	*@type	struct
	*@about 	差比和加权算法系数
	*@param
	*		--
	*		- paramA			控制系数A
	*		- paramB			控制系数B
	*		- paramC			控制系数C
	*		- paramP			比例系数P
 	*/
typedef struct{
	
	uint8_t	flag;			//控制算法转换
	float paramA;
	float paramB;
	float paramC;
	float paramP;
	float paramL;
	
}PoorCmpAnd;


/**
	*@name		Switch
	*@type		struct
	*@about 	函数开关
	*@param
	*		- 		
	*		- 		
	*/
struct{
	
	_Bool ONOF1;		//环岛
	_Bool ONOF2;		//十字
	_Bool ONOF3;		//Y形
	_Bool ONOF4;		//标志值捕获
	uint8_t ONOF5;		//滤波
	
}Switch = {1, 0, 1, 0, 1};

/**
	*@name	Kalmam
	*@type	struct
	*@about Kalman Filter 
	*@param
	*	- symStateNow			系统实时状态			   	  X(k)
	* 	- symStatePostFore		系统上次预测状态			 X(k|k-1)
	* 	- symStatePostBest		系统上次最优状态			 X(k-1|k-1)
	* 	- covNow				本次系统状态协方差			P(k|k)
	* 	- covPostFore			上次预测状态协方差			P(k|k-1)
	*	- covPostBest			上次最优状态协方差			P(k-1|k-1)
	*	- symControl			系统控制量				   U(k)
	*	- symParmA				系统参数A					A
	*	- symParmB				系统参数B					B
	* 	- errorMes				k时刻测量值				   Z(k)
	* 	- mesParm				测量系统的参数				  H
	* 	- pcesNoise				过程噪声					W(k)
	* 	- mesNoise				测量噪声					V(k)
	*	- transposeA			A的转置矩阵					A'
	*	- transposeQ			W(k)的转置矩阵				Q	
	*	- transposeR			V(k)的转置矩阵				R
	*	- transposeH			H的转置矩阵					H'
	*	- gain					卡尔曼增益					Kg
	*/
typedef struct{
	__IO	float symStateNow[5];
	__IO	float symStatePostFore[5];
	__IO	float symStatePostBest[5];
	__IO	float covNow[5];
	__IO	float covPostFore[5];
	__IO	float covPostBest[5];
			float symControl;
			float symParmA;
			float symParmB;
	__IO	float errorMes;
			float mesParm;
			float pcesNoise;
			float mesNoise;	
			float transposeA;	
			float transposeQ;
			float transposeR;
			float transposeH;
	__IO	float gain[5];
}Kalman;


/**
	*@name		TyPID
	*@type		struct
	*@about 	PID控制系数
	*@param
	*		--
	*		- Err					自变量
	*		- ErrLastValue[]		前几次Err值
	*		- ErrLastSum			Err累加值
	*		- Proportion			比例
	*		- Integral				积分
	*		- Differential			微分
	*		- Integra_Max			积分限幅值
	*/
typedef struct{
	
	int 	Proportion;
	int 	Integral;
	int 	Differential;
	float   Err;
	float	ErrLastValue[3];
	float	ErrLastSum;
	float	Integral_Max;
	float 	k;
	float 	b;
	
}TyPID;

/**
	*@breif	全局标志位
	*/
struct Flag{
	uint16_t 	A;		//环岛
	uint16_t 	B;		//环形
	uint16_t	C;		//Y形
	uint16_t	D;		//捕获sign
	uint16_t	G;		//干簧管
	uint16_t	S;		//出库与入库
	uint16_t	T;		//出库
	uint16_t	K;		//Y形消抖
	uint16_t	W;		//入库消抖
}Flag = {0, 0, 0, 0, 0, 0, 0, 0};

/**
	*@breif	中断读秒
	*/
struct	ITReadTimes{
	int	Tim1;		//环岛
	int	Tim2;		//入库
	int Tim3;		//Y形
	int	Tim4;
	int Tim5;
	int	Tim6;
	int	Tim7;
	int Tim8;
}ITRT = {270, 45, 100, 35, 30, 30, 30, 30};

/**
	*@breif	赛道特征判断值
	*/
struct JudgeValue{
	uint16_t	jm1;		//环岛
	uint16_t	jm2;		//环岛
	uint16_t	jm3;		//十字
	uint16_t	jm4;		//十字
	uint16_t	jm5;		//十字
	uint16_t	jm6;		//Y形
	uint16_t	jm7;		//Y形
	uint16_t	jm8;		//捕获
}JDVL = {3800, 500, 2000, 100, 100, 10, 100, 2000};

/*------------------------ 相关变量定义 --------------------------- */

Kalman iducKalman;			//卡尔曼滤波
PoorCmpAnd PCA;				//差比和差加权算法系数
MotorDriver Motor[2];		//电机
SteMotDriver SteMot;		//舵机
ADCData adcData;			//ADC采样
TyPID PID;					//舵机PID
TyPID MotorPID;				//电机PID（闲置）

uint8_t	huandaoK = 20;		//环岛中值电感增益压幅值

```

```c
/**
	*@funcname		bsp_LED_FeedBack()
	*@brief 		LED程序测试闪烁反馈函数
	*/
void bsp_LED_FeedBack(void)
{
	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
	HAL_Delay(100);
	HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
}
```

```c
/**
	*@funcname		fputc()
	*@brief 		串口输出重定向
	*/
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

```

```c
/**
	*@funcname		bsp_Usart_Receive()
	*@brief 		串口接收数据函数
	*/
void bsp_Usart_Receive(void)
{
	if(recv_end_flag ==1)			
		{	
			bsp_Usart_Operate(rx_buffer);
			for(uint8_t i=0;i<rx_len;i++)
			{
				rx_buffer[i]=0;
			}
			rx_len=0;
			recv_end_flag=0;
		}
		HAL_UART_Receive_DMA(&huart1,rx_buffer,200);	
}

```

STM32串口的应用可以参考该博主博客:

[STM32 HAL库之串口详细篇（基于HAL库） - 东小东 - 博客园 (cnblogs.com)](https://www.cnblogs.com/dongxiaodong/p/14275284.html)

```c
/**
	*@funcname		bsp_OLED_Display()
	*@brief 			OLED屏显函数
	*@count
	*		--
	*		-	PID数值
	*		-	ADC原始采集值
	*		-	Err
	*/
void bsp_OLED_Display(void)
{
	bsp_PID_Control();
	bsp_SteMot_PwmSet(SteMot.pwmrateTemp);
	
	OLED_ShowString(0, 0, (uint8_t *)"P:", 12);
	OLED_ShowNum(15, 0, PID.Proportion, 3, 12);
	OLED_ShowString(40, 0, (uint8_t *)"I:", 12);
	OLED_ShowNum(55, 0, PID.Integral, 3, 12);
	OLED_ShowString(80, 0, (uint8_t *)"D:", 12);
	OLED_ShowNum(95, 0, PID.Differential, 3, 12);	
	
	OLED_ShowString(0, 2, (uint8_t *)"ADC OrValue:", 12);
	OLED_ShowNum(0, 3, adcData.orignalData[0], 4, 12);
	OLED_ShowNum(50, 3, adcData.orignalData[1], 4, 12);
	OLED_ShowNum(100, 3, adcData.orignalData[2], 4, 12);
	OLED_ShowNum(0, 4, adcData.orignalData[3], 4, 12);
	OLED_ShowNum(50, 4, adcData.orignalData[4], 4, 12);
	OLED_ShowNum(100, 4, adcData.IDUC_Ex, 4, 12);
	
	OLED_ShowString(0, 5, (uint8_t *)"PwmRate:", 12);
	OLED_ShowUnFloat(60, 5, SteMot.pwmrateFnal, 7, 2, 12);
	
	OLED_ShowString(0, 6, (uint8_t *)"A:", 12);
	OLED_ShowNum(30, 6, Flag.A, 4, 12);
	OLED_ShowString(60, 6, (uint8_t *)"B:", 12);
	OLED_ShowNum(100, 6, Flag.B, 4, 12);
	OLED_ShowString(0, 7, (uint8_t *)"C:", 12);
	OLED_ShowNum(30, 7, Flag.C, 4, 12);
	OLED_ShowString(60, 7, (uint8_t *)"D:", 12);
	OLED_ShowNum(100, 7, Flag.D, 4, 12);
	
}

```

```c
/**
	*@funcname		bsp_Usart_CallBack()
	*@brief 		串口参数返回
	*/
void bsp_Usart_CallBack(void)
{
	printf("\n");
	printf("\nPID - P: %d", PID.Proportion);
	printf("\nPID - I: %d", PID.Integral);
	printf("\nPID - D: %d", PID.Differential);
	printf("\nPID - Pb: %.2f", PID.b);
	printf("\nPID - Pk: %.2f", PID.k);
	printf("\n");
	printf("\nPCA - A: %.2f", PCA.paramA);
	printf("\nPCA - B: %.2f", PCA.paramB);
	printf("\nPCA - C: %.2f", PCA.paramC);
	printf("\nPCA - P: %.2f", PCA.paramP);
	printf("\nPCA - L: %.2f", PCA.paramL);
	printf("\n");
	printf("\nSpeed: %.2f%%", MotDiff.basepwmvalue/100.0);
	if (MotDiff.pwmSwitch == 1)
	{
		printf("\nMotDiff: ON");
		printf("\nMotDiff Param: %.2f", MotDiff.Param);
	}
	else
		printf("\nMotDiff: OFF");
	if (Switch.ONOF1 == 1)
	{
		printf("\nhuandaoK; %d", huandaoK);
		printf("\nJ1: %d", JDVL.jm1);
		printf("\nJ2: %d", JDVL.jm2);
	}
	if (Switch.ONOF2 == 1)
	{
		printf("\nJ3: %d", JDVL.jm3);
		printf("\nJ4: %d", JDVL.jm4);
		printf("\nJ5: %d", JDVL.jm5);
	}
	if (Switch.ONOF3 == 1)
	{
		printf("\nJ6: %d", JDVL.jm6);
		printf("\nJ7: %d", JDVL.jm7);
	}
	if (Switch.ONOF4 == 1)
		printf("\nJ8: %d", JDVL.jm8);
	
	printf("\nFlag G:%d", Flag.G);
	printf("\nITRT Tim2: %d", ITRT.Tim2);
	printf("\nITRT Tim1: %d", ITRT.Tim1);
	printf("\nITRT Tim3: %d", ITRT.Tim3);
	printf("\nErr: %.3f", adcData.Error);
	
	printf("\n");
}
```

**在这一步,你也可以只定义出结构体与函数的名称,留到后面再完善也可,只需初步搭建出代码框架.如:**

```c
/* 舵机控制 */
typedef struct{
    
}SteMotDriver;

```

```c
/**
	*@funcname		bsp_LED_FeedBack()
	*@brief 		LED程序测试闪烁反馈函数
	*/
void bsp_LED_FeedBack(void)
{

}
```

### 2.滤波算法

滤波算法的使用是为了在一定程度上滤除部分噪声，使得最终得到的值无限逼近实际值，在电磁循迹车上是为了使得电磁杆采集值经过滤波处理后可以比较真实的反应赛道实际情况。

刚开始我们选择的滤波算法是卡尔曼滤波，他的结构体在上面可以看到。但是由于对卡尔曼滤波的不甚了解，我们无法对其参数进行调整，所以在最后改用了一阶αβ滤波算法。由于赛道较为简单，速度较慢，在比赛中我们也遇到了不用滤波算法直接对电感采集值归一化后传给PID控制的小组。事实也证明，**并非越复杂越高级的算法就越好，在实际运用中，只有最适合算法的没有最好的算法，所谓大道至简**。

在开头的架构我也列出了一些常见的滤波算法，具体算法的思想可以参考以下博主的博客：

[十大滤波算法总结_无刷电机控制的博客-CSDN博客_滤波算法](https://blog.csdn.net/richardgann/article/details/78780040)

```c
/**
	*@funcname		bsp_ArBi_Filter()
	*@brief 			一阶(αβ)滤波
	*/
float bsp_ArBi_Filter(uint16_t value, uint8_t i)
{
	static uint16_t ArBi_lastValue[5] = {0, 0, 0, 0, 0};
	float result;
	
	result = 0.80 * value + (1 - 0.80) * ArBi_lastValue[i];
	ArBi_lastValue[i] = result;
	
	return result;
}

```

### 3.归一化算法

我们采用的是南通大学原创的差比和差加权算法，其数学模型如下：
$$
Err=\frac{A·(L-R)+B·(LM-RM)}{A·(L+R)+C·|LM-RM|}·P
$$
详细介绍可参考以下链接：

[智能车电感差比和差加权算法研究_卓晴的博客-CSDN博客_差比和](https://blog.csdn.net/zhuoqingjoking97298/article/details/108993827)

该算法有四个参数要调，具体如何调参可以参考我队友的博客：



```c
/**
	*@funcname		bsp_PCA_Init()
	*@brief 		PCA(差比和加权算法系数)初始化
	*@param
	*			--	未调参值
	*				- A			1
	*				- B			1
	*				- C			1
	*				- P			0.5
	*/
void bsp_PCA_Init(void)
{
	PCA.paramA = 1.90;
	PCA.paramB = 6.75;
	PCA.paramC = 9.65;
	PCA.paramP = 1.18;
	PCA.paramL = 1;
}
```



```c
/**
	*@funcname		bsp_ADCValue_PoorCmpAnd()
	*@brief 		差比和差加权算法
	*/
float bsp_ADCValue_PoorCmpAnd(ADCData value)
{
	float Err;
	
	/* 差比和差加权 */
	if(PCA.flag == 0)
	{
		Err = (
						(	PCA.paramA * (value.IDUC_L - value.IDUC_R) +
							PCA.paramB * (value.IDUC_LM - value.IDUC_RM)) /
						(
							(	PCA.paramA * (value.IDUC_L + value.IDUC_R)) +
							(	PCA.paramC * (fabs((double)(value.IDUC_LM - value.IDUC_RM))))
						)
					) * PCA.paramP;
						
	}
	
	return Err;
}
```

### 4.PID控制算法

PID的具体原理可以参考以下博主的博客：

[PID应用详解 - -零 - 博客园 (cnblogs.com)](https://www.cnblogs.com/-wenli/p/11141370.html)

[一文读懂PID控制算法（抛弃公式，从原理上真正理解PID控制）_确定有穷自动机的博客-CSDN博客_pid](https://blog.csdn.net/qq_25352981/article/details/81007075?ops_request_misc=%7B%22request%5Fid%22%3A%22164829226216780264025224)

在实际调车中，我们一般只用PD进行控制，具体调参表现为：

>   P增大，小车对差值响应幅度更大，过大会导致小车行驶过程中扭动。

>   D增大，可以一定程度上减小小车扭动，D过大后对P的抑制作用逐渐减弱。

附PID调参口诀：

<img src="%E7%94%B5%E7%A3%81%E5%BE%AA%E8%BF%B9%E5%B0%8F%E8%BD%A6%E8%B5%9B%E5%90%8E%E6%80%BB%E7%BB%93.assets/PID%E8%B0%83%E5%8F%82%E5%8F%A3%E8%AF%80.jpg" style="zoom: 80%;" />

关于PID我们对P通过一个关于差值Err的线性函数对其实现动态改变，以适应直道与弯道的不同需求，满足公式：
$$
P = k·|Err|+b
$$
因此在实际调参中是对$k$与$b$值进行调整。

```c
/**
	*@funcname		bsp_PID_Init()
	*@brief 		PID初始化
	*@param
	*			--(未调参值)
	*			- P			1
	*			- I			1
	*			- D			1
	*/
void bsp_PID_Init(void)
{
	PID.Proportion = 30;
	PID.Integral = 0;
	PID.Differential = 80;
	PID.Integral_Max = 0;
	PID.b = 55;
	PID.k = 178;
	
	PID.ErrLastSum = 0;
	PID.ErrLastValue[0] = 0.0;
	PID.ErrLastValue[1] = 0.0;
	PID.ErrLastValue[2] = 0.0;
}

```

```c
/**
	*@funcname		bsp_PID_Core()
	*@brief 		PID核心算法(位置式)
	*/
float bsp_PID_Core(float error)
{
	PID.Err = error;
	float PwmRate;		
	
	PID.ErrLastValue[2] = PID.ErrLastValue[1];
	PID.ErrLastValue[1] = PID.ErrLastValue[0];
	PID.ErrLastValue[0] = error;
	
	/* 积分限幅 */
	if (
			((PID.ErrLastSum + error) < PID.Integral_Max) &&
			((PID.ErrLastSum + error) > -PID.Integral_Max)
		 )
	{
		/* err值累加 */
		PID.ErrLastSum += error;			
	}
	else if (PID.ErrLastSum > 0)		
	{
		/* 正向限幅 */
		PID.ErrLastSum = PID.Integral_Max;
	}
	else if (PID.ErrLastSum < 0)		
	{
		/* 反向限幅 */
		PID.ErrLastSum = -PID.Integral_Max;
	}
	
	PID.Proportion = PID.k * fabs(adcData.Error)+ PID.b;
	/* Core */
	PwmRate = PID.Proportion * (error) + PID.Integral * PID.ErrLastSum + 
						PID.Differential * ((PID.ErrLastValue[0] - PID.ErrLastValue[1]) -
					(	PID.ErrLastValue[1] - PID.ErrLastValue[2]));
	
	return PwmRate;
}


/**
	*@funcname		bsp_PID_Control()
	*@brief 		PID控制函数
	*/
void bsp_PID_Control(void)
{
	bsp_ADC_Operate();
	SteMot.pwmrateTemp = bsp_PID_Core(adcData.Error);
	bsp_SteMot_PwmSet(SteMot.pwmrateTemp);

}

```

### 5.赛道特征值响应函数

该函数主要是要在实际调参过程中在赛道的环岛，Y叉等特征处通过采集确定可以识别出该处的特征值，在函数中通过条件判断，一旦小车电磁杆识别到特征值直接通过对舵机进行固定转角实现过弯。

```c
/**
	*@funcname		bsp_CycleIn()
	*@brief 		环岛判断
	*/
float bsp_CycleIn(float value)
{
	float result = value;
	
	if (Switch.ONOF1 == 1)
	{
		if((adcData.IDUC_LM + adcData.IDUC_RM) > 5000 && adcData.IDUC_M > 3000)
		{
					Flag.A = 1;
		}
	}
	
	if (Flag.A == 1)
	{
		result -= adcData.IDUC_Ex/15;
	}
		return result;
}



/**
	*@funcname		bsp_Cross()
	*@brief 		十字，环形
	*/
float bsp_Cross(float	value)
{	
	float result = value;
	
	if (Switch.ONOF2 == 1)
	{
		if (adcData.IDUC_M > JDVL.jm3)
		{
			if (adcData.IDUC_L - adcData.IDUC_R < JDVL.jm4)
			{
				if (adcData.IDUC_LM - adcData.IDUC_RM < JDVL.jm5)
				{
					Flag.B++;
				}
			}
		}
	}
	return result;
}

/**
	*@funcname		bsp_Yshape()
	*@brief 		Y形
	*/
float bsp_Yshape(float value)
{
	float result = value;
	
	if (Switch.ONOF3 == 1)
	{
		if (adcData.IDUC_M < 10)
		{
			if (adcData.IDUC_Ex < 120 && adcData.IDUC_M < 120 && ITRT.Tim3 == 100)
			{				
					Flag.C++;
					Flag.K = 1;			
			}
		}
	}
	
	if(Flag.C == 1)
	{
		result -= 120;
	}
	if(Flag.C == 3)
	{
		result += 170;
	}
	return result;
}
```

基本的控制类算法就如上所示了。

### 6.如何方便调参

蓝牙模块与Wifi模块的运用可以很大程度上提高调参的效率。

可以通过编写相关的蓝牙指令**实现在小车上电过程中直接对小车参数进行调整，从而避免反复烧录程序的麻烦。**

```c
/**
	*@funcname		bsp_Usart_Operate()
	*@brief 		串口指令
	*@oder
	*		--
	*		-		1			P:value			PID_P = value
	*		-		2			I:value			PID_I = value
	*		-		3			D:value			PID_D = value
	*		-		4			A:value			PCA_A = value
	*		-		5			B:value			PCA_B = value
	*		-		6			C:value			PCA_C = value
	*		-		7			p:value			PCA_P = value
	*		-		24			L:value			PCA_L = value
	*		-		8			M:value			SteMot.Min = value
	*		-		9			--	
	*							-	G:1			电机启动
	*							-	G:0			电机关闭
	*		-		10			--
	*							-	S:1			电机差速跟随控制启动
	*							-	S:0			电机差速跟随控制关闭
	*		-		11			M:value			电机占空比（速度）	
	*		-		15			Pb:value		PID_Pb = value
	*		-		16			Pk:value		PID_Pk = value
	*		-		19			s1:value		开断环岛判断
	*		-		20			s2:value		开断十字判断
	*		-		21			s3:value		开断Y形判断
	*		-		22			s4:value		开断标识值捕获
	*		-		23			s5:value		切换滤波算法
	*		-		17			W:value			切换归一化算法
	*		-		18			h:value			环岛中值电感增益值削减度
	*		-		25			U:Any			串口返回参数值
	*		--
	*/
void bsp_Usart_Operate(uint8_t *str)
{
	float value;
	uint8_t oder = 0;
	
	if			((*str == 'P') && (sscanf((const char *)str, "P:%f", &value) == 1))		oder = 1;
	else if ((*str == 'I') && (sscanf((const char *)str, "I:%f", &value) == 1))		oder = 2;
	else if ((*str == 'D') && (sscanf((const char *)str, "D:%f", &value) == 1))		oder = 3;
	else if ((*str == 'A') && (sscanf((const char *)str, "A:%f", &value) == 1))		oder = 4;
	else if ((*str == 'B') && (sscanf((const char *)str, "B:%f", &value) == 1))		oder = 5;
	else if ((*str == 'C') && (sscanf((const char *)str, "C:%f", &value) == 1))		oder = 6;
	else if ((*str == 'L') && (sscanf((const char *)str, "L:%f", &value) == 1))		oder = 24;
	else if ((*str == 'p') && (sscanf((const char *)str, "p:%f", &value) == 1))		oder = 7;
	else if ((*str == 'F') && (sscanf((const char *)str, "F:%f", &value) == 1))		oder = 8;
	else if ((*str == 'G') && (sscanf((const char *)str, "G:%f", &value) == 1))		oder = 9;
	else if ((*str == 'S') && (sscanf((const char *)str, "S:%f", &value) == 1))		oder = 10;
	else if ((*str == 'M') && (sscanf((const char *)str, "M:%f", &value) == 1))		oder = 11;
	
	else if ((*str == 'P') && (*(str+1) == 'k') && (sscanf((const char *)str, "Pk:%f", &value) == 1))		oder = 15;
	else if ((*str == 'P') && (*(str+1) == 'b') && (sscanf((const char *)str, "Pb:%f", &value) == 1))		oder = 16;
	else if ((*str == 's') && (*(str+1) == '1') && (sscanf((const char *)str, "s1:%f", &value) == 1))		oder = 19;
	else if ((*str == 's') && (*(str+1) == '2') && (sscanf((const char *)str, "s2:%f", &value) == 1))		oder = 20;
	else if ((*str == 's') && (*(str+1) == '3') && (sscanf((const char *)str, "s3:%f", &value) == 1))		oder = 21;
	else if ((*str == 's') && (*(str+1) == '4') && (sscanf((const char *)str, "s4:%f", &value) == 1))		oder = 22;
	else if ((*str == 's') && (*(str+1) == '5') && (sscanf((const char *)str, "s5:%f", &value) == 1))		oder = 23;
		
	else if ((*str == 'W') && (sscanf((const char *)str, "W:%f", &value) == 1))		oder = 17;
	else if ((*str == 'h') && (sscanf((const char *)str, "h:%f", &value) == 1))		oder = 18;
	else if ((*str == 'U') && (sscanf((const char *)str, "U:%f", &value) == 1))		oder = 25;
	
	else if ((*str == 'J') && (*(str+1) == '1') && (sscanf((const char *)str, "J1:%f", &value) == 1))		oder = 26;
	else if ((*str == 'J') && (*(str+1) == '2') && (sscanf((const char *)str, "J2:%f", &value) == 1))		oder = 27;
	else if ((*str == 'J') && (*(str+1) == '3') && (sscanf((const char *)str, "J3:%f", &value) == 1))		oder = 28;
	else if ((*str == 'J') && (*(str+1) == '4') && (sscanf((const char *)str, "J4:%f", &value) == 1))		oder = 29;
	else if ((*str == 'J') && (*(str+1) == '5') && (sscanf((const char *)str, "J5:%f", &value) == 1))		oder = 30;
	else if ((*str == 'J') && (*(str+1) == '6') && (sscanf((const char *)str, "J6:%f", &value) == 1))		oder = 31;
	else if ((*str == 'J') && (*(str+1) == '7') && (sscanf((const char *)str, "J7:%f", &value) == 1))		oder = 32;
	else if ((*str == 'J') && (*(str+1) == '8') && (sscanf((const char *)str, "J8:%f", &value) == 1))		oder = 33;
	
	else		printf("\nvalue set fail!");
	
	switch(oder)
	{
		case 1: 
						PID.Proportion = (int)value;	
						printf("\nSuccessful set the value of P: %d", PID.Proportion);
						break;
		case 2: 
						PID.Integral = (int)value;	
						printf("\nSuccessful set the value of I: %d", PID.Integral); 
						break;
		case 3:
						PID.Differential = (int)value;	
						printf("\nSuccessful set the value of D: %d", PID.Differential); 
						break;
		case 4:
						PCA.paramA = value;
						printf("\nSuccessful set the value of PCA-A: %f", PCA.paramA); 
						break;
		case 5:
						PCA.paramB = value;
						printf("\nSuccessful set the value of PCA-B: %f", PCA.paramB); 
						break;
		case 6:
						PCA.paramC = value;
						printf("\nSuccessful set the value of PCA-C: %f", PCA.paramC); 
						break;
		case 7:
						PCA.paramP = value;
						printf("\nSuccessful set the value of PCA-P: %f", PCA.paramP); 
						break;
		case 24:
						PCA.paramL = value;
						printf("\nSuccessful set the value of PCA-L: %f", PCA.paramL); 
						break;
		case 8:
						MotDiff.Param = value;
						printf("\nSuccessful set the value of SteMin: %f", MotDiff.Param); 
						break;
		case 9:
						if(value == 1)
						{
							Motor[0].OnOff = 1;
							printf("\nCar Motor ON."); 
						}
						else if (value == 0)
						{
							Motor[0].OnOff = 0;
							printf("\nCar Motor OFF.");
						}
						else
							printf("Err!!");
						break;
		case 10:
						if(value == 1)
						{
								MotDiff.pwmSwitch = 1;
								printf("\nCar Motor Diff Foller ON."); 
						}
						else if (value == 0)
						{
								MotDiff.pwmSwitch = 0;
								printf("\nCar Motor Diff Follor OFF.");
						}
						else
							printf("Err!!");
						break;

		case 11:
						MotDiff.basepwmvalue = (uint16_t)value;
						printf("\nMotor Speed: %.2f%%", (MotDiff.basepwmvalue/100.0)); 
						break;
		case 15:
						PID.k = value;
						printf("\nPID Pk: %.2f", PID.k); 
						break;
		case 16:
						PID.b = value;
						printf("\nPID Pb: %.2f", PID.b); 
						break;
		case 17:
						PCA.flag = value;
						printf("\nSuccessful set the value of pac flag: %f", (float)PCA.flag); 
						break;
		case 18:
						huandaoK = (uint8_t)value;
						printf("\nSuccess! %d", huandaoK);
						break;
		case 19:
						Switch.ONOF1 = (_Bool)value;
						printf("\nSuccess! ONOF1: %d", Switch.ONOF1);
						break;
		case 20:
						Switch.ONOF2 = (_Bool)value;
						printf("\nSuccess! ONOF2: %d", Switch.ONOF2);
						break;
		case 21:
						Switch.ONOF3 = (_Bool)value;
						printf("\nSuccess! ONOF3: %d", Switch.ONOF3);
						break;
		case 22:
						Switch.ONOF4 = (_Bool)value;
						printf("\nSuccess! ONOF4: %d", Switch.ONOF4);
						break;
		case 23:
						Switch.ONOF5 = (uint8_t)value;
						printf("\nSuccess! ONOF5: %d", Switch.ONOF5);
						break;
		case 25:
						bsp_Usart_CallBack();
						break;
		
		case 26:
						JDVL.jm1 = (uint16_t)value;
						printf("\nJ1: %d", JDVL.jm1);
						break;
		case 27:
						JDVL.jm2 = (uint16_t)value;
						printf("\nJ2: %d", JDVL.jm2);
						break;
		case 28:
						JDVL.jm3 = (uint16_t)value;
						printf("\nJ3: %d", JDVL.jm3);
						break;
		case 29:
						JDVL.jm4 = (uint16_t)value;
						printf("\nJ4: %d", JDVL.jm4);
						break;
		case 30:
						JDVL.jm5 = (uint16_t)value;
						printf("\nJ5: %d", JDVL.jm5);
						break;
		case 31:
						JDVL.jm6 = (uint16_t)value;
						printf("\nJ6: %d", JDVL.jm6);
						break;
		case 32:
						JDVL.jm7 = (uint16_t)value;
						printf("\nJ7: %d", JDVL.jm7);
						break;
		case 33:
						JDVL.jm8 = (uint16_t)value;
						printf("\nJ8: %d", JDVL.jm8);
						break;
		default:	break;
	}

}
```

### 7.其他代码

```c
/**
	*@funcname		bsp_Motor_PwmSet()
	*@brief 		电机PWM控制函数(调速)
	*/
void bsp_Motor_PwmSet(MotorDriver *MD)
{
	char K = 0;
	
	if(MotDiff.pwmSwitch==1)
	{
		K = adcData.Error > 0 ? 1 : -1;
		MD[0].pwmrate += K * (MotDiff.Param * (SteMot.pwmrateFnal - SteMot.Min) /
																					(STEPWMMAX - SteMot.Min));
		MD[1].pwmrate -= K * (MotDiff.Param * (SteMot.pwmrateFnal - SteMot.Min) /
																					(STEPWMMAX - SteMot.Min));
	}
	
	Motor[0].pwmrate = MotDiff.basepwmvalue;
	Motor[1].pwmrate = MotDiff.basepwmvalue;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, (uint16_t)MD[0].pwmrate);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, (uint16_t)MD[1].pwmrate);
}

```

```c
/**
	*@funcname		bsp_SteMot_PwmSet()
	*@brief 		舵机PWM设置函数(转向)
	*/
void bsp_SteMot_PwmSet(float value)
{
	float pwmrate = value;
	
	/* 中值增减 */
	pwmrate += SteMot.Min;
	
	/* 环岛判断 */
	pwmrate = bsp_CycleIn(pwmrate);
	
	/* 十字判断 */
	pwmrate = bsp_Cross(pwmrate);
	
	/* Y形判断 */
	pwmrate = bsp_Yshape(pwmrate);
	
	/* 出库判断 */
	if (Flag.T == 0)
	{
		pwmrate += 220;
	}
	/* 入库 */
		if (Flag.S == 1 && ITRT.Tim2 != 0)
	{
		pwmrate += 230;
	}
	
	/* 标志值捕获 */
	bsp_SignJudge();
	
	if (Flag.S == 1 && ITRT.Tim2 > 0)
		pwmrate += 150;
	if (Flag.S == 1 && ITRT.Tim2 <= 0)
		Motor->OnOff = 0;
	
	/* 限幅 */
	if(pwmrate < STEPWMMIN)
		pwmrate = STEPWMMIN;
	else if (pwmrate > STEPWMMAX)
		pwmrate = STEPWMMAX;
	
	/* 终值更新 */
	SteMot.pwmrateFnal=pwmrate;
	
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, (uint16_t)SteMot.pwmrateFnal);
}
```

```c
/**
	*@brief 	定时器中断回调函数
	*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		HAL_ADC_Start(&hadc2);
		adcData.IDUC_Ex = HAL_ADC_GetValue(&hadc2);
		bsp_PID_Control();
		bsp_Motor_PwmSet(Motor);
	}
	if(htim == &htim4)
	{
		if (Flag.S == 1)
		{
			ITRT.Tim2--;
		}
		if (Flag.K == 1)
		{
			ITRT.Tim3--;
			if(ITRT.Tim3 <= 0)
			{
				Flag.K = 0;
				ITRT.Tim3 = 100;
			}
		}
		if (Flag.A == 1)
		{
			ITRT.Tim4--;
			if (ITRT.Tim4 <= 0)
			{
				Flag.A = 0;
				ITRT.Tim4 = 35;
			}
		}
		if (Flag.T == 0)
		{
			ITRT.Tim5--;
			if (ITRT.Tim5 <= 0)
			{
				Flag.T = 1;
			}
		}
		if (Flag.C == 1)
		{
			ITRT.Tim6--;
			if (ITRT.Tim6 <= 0)
			{
				Flag.C = 2;
				ITRT.Tim6= 30;
			}
		}		
		if(Flag.C == 3)
		{
			ITRT.Tim7--;
			if(ITRT.Tim7<=0)
			{
				Flag.C = 0;
				ITRT.Tim7 = 30;
			}
		}
		if (Flag.W == 1)
		{
			ITRT.Tim8--;
			if (ITRT.Tim8 <= 0)
			{
				Flag.W = 0;
				ITRT.Tim8 = 30;
			}
		}
	}
}
```

```c
/**
  * @brief  外部中断回调函数
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if (GPIO_Pin == GPIO_PIN_1 && Flag.W == 0)
	{
		Flag.G++;
		Flag.W = 1;
		bsp_OutAndInbound();
		while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1));
	}
	else if (GPIO_Pin == Key1_Pin)
	{
		Motor->OnOff = !(Motor->OnOff);
		while(!HAL_GPIO_ReadPin(GPIOA, Key1_Pin));
	}
}

```

## 五.总结

#### 1.在开始做车之前，不要急着动手，先理清楚思路，确定好方案，在开始着手。

#### 2.大道至简，并非最复杂的算法就是最好的算法，只有最适合的才是最好的。

#### 3.在调参时，不要随机乱调参，要先搞清楚每个参数的作用，在一点一点逐渐调参。

>   博主也是第一次做车，第一次参赛，所以在代码编写和算法理解上还有些生疏，如果那里有错误，欢迎大家指正。