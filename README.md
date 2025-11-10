# SICK转接板使用说明
## 一、参数表

<table>   <tr><td colspan="2" align="center" style="background-color: #f5f5f5;"><b>总体参数</b></td></tr>   <tr>     <td style="width: 100px; padding: 4px;">产品名称</td>     <td style="width: 200px; padding: 4px;">SICK转接模块</td>   </tr>   <tr>     <td style="width: 100px; padding: 4px;">主控</td>     <td style="width: 200px; padding: 4px;">STM32F103C8T6</td>   </tr>   <tr>     <td style="width: 100px; padding: 4px;">ADC芯片</td>     <td style="width: 200px; padding: 4px;">AD7606</td>   </tr>   <tr><td colspan="2" align="center" style="background-color: #f5f5f5;"><b>电源参数</b></td></tr>   <tr>     <td style="width: 100px; padding: 4px;">最大输入电压</td>     <td style="width: 200px; padding: 4px;">5.25V</td>   </tr>   <tr>     <td style="width: 100px; padding: 4px;">最小输入电压</td>     <td style="width: 200px; padding: 4px;">4.75V</td>   </tr>   <tr>     <td style="width: 100px; padding: 4px;">VCC电压</td>     <td style="width: 200px; padding: 4px;">3.3V</td>   </tr>   <tr><td colspan="2" align="center" style="background-color: #f5f5f5;"><b>板载资源</b></td></tr>   <tr>     <td style="width: 100px; padding: 4px;">UART串口</td>     <td style="width: 200px; padding: 4px;">1个(外部通信，波特率115200)</td>   </tr>   <tr>     <td style="width: 100px; padding: 4px;">USB接口</td>     <td style="width: 200px; padding: 4px;">1个(供电+通信，12Mhz)</td>   </tr> </table>



## 二、引脚定义

![image-20251110202544791](C:\Users\HH\AppData\Roaming\Typora\typora-user-images\image-20251110202544791.png)

**使用步骤：**

1. 将USB与工控机连接，串口 Tx 与大疆A板的 Rx 连接
2. 保证工控机、大疆A板、SICK转接板三者共地
3. 将SICK模拟输出端接入 CH1 ~ CH8
4. 读取数据即可



## 三、通讯协议

数据包由 ***帧头、帧类型、有效数据长度、有效数据、帧尾*** 组成

- **帧头：**数据包除帧头帧尾外的字节相加，以 uint8_t 型存储
- **帧类型：**0x55
- **有效数据长度：**20字节
- **有效数据：**32字节，对应8个float型数据，分别为8个通道的模拟量
- **帧尾：**同帧头
