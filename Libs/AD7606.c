#include "main.h"
#include "AD7606.h"
#include "delay.h"
#include "usbd_cdc_if.h"
#include "tim.h"
#include "usart.h"
#include "math.h"

/*
	RST: PA6
	ConvstA: PB0
	ConvstB: PA7

	OSI2: PB1
	OSI1: PB2
	OSI0: PB10

	BUSY: PA3
	CS: PA4
	RD/SCK: PD15

	DB7: PA12
*/

// AD7606 模拟SPI引脚定义（示例）
#define AD7606_SCK_GPIO_Port GPIOA
#define AD7606_SCK_Pin GPIO_PIN_5

#define AD7606_DB7_GPIO_Port GPIOA
#define AD7606_DB7_Pin GPIO_PIN_2 // AD7606串行输出数据

#define AD7606_CS_GPIO_Port GPIOA
#define AD7606_CS_Pin GPIO_PIN_4

#define AD7606_RST_GPIO_Port GPIOA
#define AD7606_RST_Pin GPIO_PIN_6

#define AD7606_BUSY_GPIO_Port GPIOA
#define AD7606_BUSY_Pin GPIO_PIN_3

#define AD7606_CONVSTA_GPIO_Port GPIOB
#define AD7606_CONVSTA_Pin GPIO_PIN_0 // 转换启动

#define AD7606_CONVSTB_GPIO_Port GPIOA
#define AD7606_CONVSTB_Pin GPIO_PIN_7 // 转换启动

#define AD7606_OS2_GPIO_Port GPIOB
#define AD7606_OS2_Pin GPIO_PIN_1 // OS2

#define AD7606_OS1_GPIO_Port GPIOB
#define AD7606_OS1_Pin GPIO_PIN_2 // OS1

#define AD7606_OS0_GPIO_Port GPIOB
#define AD7606_OS0_Pin GPIO_PIN_10 // OS0

#define AD_OS2_0 HAL_GPIO_WritePin(AD7606_OS2_GPIO_Port, AD7606_OS2_Pin, GPIO_PIN_RESET)
#define AD_OS2_1 HAL_GPIO_WritePin(AD7606_OS2_GPIO_Port, AD7606_OS2_Pin, GPIO_PIN_SET)
#define AD_OS1_0 HAL_GPIO_WritePin(AD7606_OS1_GPIO_Port, AD7606_OS1_Pin, GPIO_PIN_RESET)
#define AD_OS1_1 HAL_GPIO_WritePin(AD7606_OS1_GPIO_Port, AD7606_OS1_Pin, GPIO_PIN_SET)
#define AD_OS0_0 HAL_GPIO_WritePin(AD7606_OS0_GPIO_Port, AD7606_OS0_Pin, GPIO_PIN_RESET)
#define AD_OS0_1 HAL_GPIO_WritePin(AD7606_OS0_GPIO_Port, AD7606_OS0_Pin, GPIO_PIN_SET)
#define AD_CS_0 HAL_GPIO_WritePin(AD7606_CS_GPIO_Port, AD7606_CS_Pin, GPIO_PIN_RESET)
#define AD_CS_1 HAL_GPIO_WritePin(AD7606_CS_GPIO_Port, AD7606_CS_Pin, GPIO_PIN_SET)
#define AD_RST_0 HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_RESET)
#define AD_RST_1 HAL_GPIO_WritePin(AD7606_RST_GPIO_Port, AD7606_RST_Pin, GPIO_PIN_SET)
#define AD_CONVSTA_0 HAL_GPIO_WritePin(AD7606_CONVSTA_GPIO_Port, AD7606_CONVSTA_Pin, GPIO_PIN_RESET)
#define AD_CONVSTA_1 HAL_GPIO_WritePin(AD7606_CONVSTA_GPIO_Port, AD7606_CONVSTA_Pin, GPIO_PIN_SET)
#define AD_CONVSTB_0 HAL_GPIO_WritePin(AD7606_CONVSTB_GPIO_Port, AD7606_CONVSTB_Pin, GPIO_PIN_RESET)
#define AD_CONVSTB_1 HAL_GPIO_WritePin(AD7606_CONVSTB_GPIO_Port, AD7606_CONVSTB_Pin, GPIO_PIN_SET)
#define AD_SCK_0 HAL_GPIO_WritePin(AD7606_SCK_GPIO_Port, AD7606_SCK_Pin, GPIO_PIN_RESET)
#define AD_SCK_1 HAL_GPIO_WritePin(AD7606_SCK_GPIO_Port, AD7606_SCK_Pin, GPIO_PIN_SET)
#define READ_AD_BUSY HAL_GPIO_ReadPin(AD7606_BUSY_GPIO_Port, AD7606_BUSY_Pin)

// 数据包

int16_t CH_data_buf[8]; // AD7606通道数据缓存
float sick_data[8];		// 转换成电压值的缓存

/* AD7606是高电平复位，要求最小脉宽50ns */
void ADReset(void)
{
	AD_RST_0;

	AD_RST_1;
	AD_RST_1;
	AD_RST_1;
	AD_RST_1;

	AD_RST_0;
}

/*
*********************************************************************************************************
*	函 数 名: ad7606_SetOS
*	功能说明: 设置过采样模式（数字滤波，硬件求平均值)
*	形    参：_ucMode : 0-6  0表示无过采样，1表示2倍，2表示4倍，3表示8倍，4表示16倍
*				5表示32倍，6表示64倍
*	返 回 值: 无
*********************************************************************************************************
*/
void ADSetOs(uint8_t os)
{
	switch (os)
	{
	case 0:
		AD_OS2_0;
		AD_OS1_0;
		AD_OS0_0;
		break;
	case 1:
		AD_OS2_0;
		AD_OS1_0;
		AD_OS0_1;
		break;
	case 2:
		AD_OS2_0;
		AD_OS1_1;
		AD_OS0_0;
		break;
	case 3:
		AD_OS2_0;
		AD_OS1_1;
		AD_OS0_1;
		break;
	case 4:
		AD_OS2_1;
		AD_OS1_0;
		AD_OS0_0;
		break;
	case 5:
		AD_OS2_1;
		AD_OS1_0;
		AD_OS0_1;
		break;
	case 6:
		AD_OS2_1;
		AD_OS1_1;
		AD_OS0_0;
		break;
	default:
		AD_OS2_0;
		AD_OS1_0;
		AD_OS0_0;
		break;
	}
}

void ADStartConv(void)
{
	AD_CONVSTA_0;
	AD_CONVSTA_0;
	AD_CONVSTA_0;
	AD_CONVSTA_0;
	AD_CONVSTA_1;
	// CONVST A/CONVST B 上升沿之间最大容许时间：0.5ms
	AD_CONVSTB_0;
	AD_CONVSTB_0;
	AD_CONVSTB_0;
	AD_CONVSTB_0;
	AD_CONVSTB_1;
}

// convst到busy的时间最大为40ns

void ADInit()
{
	ADSetOs(0); // 无过采样
	ADReset();
	AD_CONVSTA_1;
	AD_CONVSTB_1;
	AD_CS_1;
	AD_SCK_1;
}

// 禁止Delay!!!时序会有大问题
void SpiReadData(int16_t *tar_data)
{
	uint8_t i, j;
	// AD7606通道数据缓存，需要用 16*8 个SCLK周期读取（SCLK下降沿读取）

	for (i = 0; i < 8; i++)
	{
		uint16_t CH_data = 0;
		AD_CS_0; // 片选使能
				 //		delay_us(3);
		for (j = 0; j < 16; j++)
		{
			AD_SCK_0;																							  // 拉低SCLK
																												  //			delay_us(4);
			CH_data = ((uint16_t)(HAL_GPIO_ReadPin(AD7606_DB7_GPIO_Port, AD7606_DB7_Pin)) << (15 - j)) + CH_data; // 在SCLK下降沿读取数据
			AD_SCK_1;																							  // 重新拉高SCLK
																												  //			delay_us(4);
		}
		AD_CS_1;			   // 重新拉高CS
		tar_data[i] = CH_data; // 存储通道i的数据，每个通道两个字节，带正负
	}
}

// 计算帧头校验
uint8_t calculateFrameHead(uint8_t *data, int length)
{
	uint8_t sum = 0;
	for (int i = 0; i < length; i++)
	{
		sum += data[i];
	}
	return sum;
}

/**
 * @brief 将float数组转换为字节数组（小端）
 * @param f: 输入float数组
 * @param bytes: 输出字节数组（长度需为4×n）
 * @param count: float数组元素个数
 */
void floatToBytes(float *f, uint8_t *bytes, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++)
	{
		uint32_t temp = *(uint32_t *)&f[i];	   // 共用体或指针转换（避免类型转换警告）
		bytes[i * 4 + 0] = (temp >> 0) & 0xFF; // 低字节
		bytes[i * 4 + 1] = (temp >> 8) & 0xFF;
		bytes[i * 4 + 2] = (temp >> 16) & 0xFF;
		bytes[i * 4 + 3] = (temp >> 24) & 0xFF; // 高字节
	}
}

/* 低通滤波参数 */
#define SAMPLE_FREQ  1000.0f        // 采样频率 1000Hz (1ms定时器)
#define CUTOFF_FREQ  5.0f          // 截止频率(带宽)，单位Hz，根据需要调整
static float sick_data_filtered[8] = {0};  // 滤波状态值

/* ========== 标定表（分段线性插值） ========== */
/* 输入：传感器电压值（((float)CH_data_buf / 32768.0f) * 10.0f） */
static const float calib_x[7] = {
    0.0195f, 0.3052f, 0.6039f, 0.8606f,
    1.1465f, 1.4160f, 3.3386f
};
/* 输出：距离值（米） */
static const float calib_y[7] = {
    0.081f,  0.288f,  0.501f,  0.686f,
    0.885f,  1.085f,  2.446f
};
#define CALIB_POINTS 7

/**
 * @brief 分段线性插值查表
 * @param x: 输入值（传感器电压）
 * @return 对应的距离值（米）
 */
static float LinearInterp(float x)
{
    /* 边界处理：低于最小输入则钳位到最小输出 */
    if (x <= calib_x[0]) return calib_y[0];
    /* 边界处理：高于最大输入则钳位到最大输出 */
    if (x >= calib_x[CALIB_POINTS - 1]) return calib_y[CALIB_POINTS - 1];

    /* 查找 x 所在区间 */
    for (uint8_t i = 0; i < CALIB_POINTS - 1; i++)
    {
        if (x < calib_x[i + 1])
        {
            /* 线性插值: y = y0 + (y1 - y0) * (x - x0) / (x1 - x0) */
            float t = (x - calib_x[i]) / (calib_x[i + 1] - calib_x[i]);
            return calib_y[i] + t * (calib_y[i + 1] - calib_y[i]);
        }
    }

    return calib_y[CALIB_POINTS - 1];
}

/* 一阶低通滤波 - 基于截止频率 */
static void lowpass_filter(float *raw, float *filtered, uint8_t len)
{
    /* 根据采样频率和截止频率计算alpha系数 */
    float omega = 2.0f * 3.14159265f * CUTOFF_FREQ / SAMPLE_FREQ;
    float alpha = 1.0f - expf(-omega);   // 脉冲响应不变法

    for (uint8_t i = 0; i < len; i++)
    {
        filtered[i] = alpha * raw[i] + (1.0f - alpha) * filtered[i];
    }
}

// 读取通道数据
void GetAdcData()
{
	ADStartConv();

	while (READ_AD_BUSY == 1)
	{
		HAL_Delay(1);
	}

	delay_us(1);

	SpiReadData(CH_data_buf);

	/* 将AD原始值转为电压，再通过标定表插值得到距离 */
	for (uint8_t i = 0; i < 8; i++)
	{
		float voltage = ((float)CH_data_buf[i] / 32768.0f) * 10.0f;
		sick_data[i] = LinearInterp(voltage);
	}

	/* ———— 低通滤波 ———— */
    lowpass_filter(sick_data, sick_data_filtered, 8);
    /* 将滤波结果写回sick_data供后续发送 */
    for (uint8_t i = 0; i < 8; i++)
    {
        sick_data[i] = sick_data_filtered[i];
    }
}

uint8_t tx_buf[36] = {0}; // 1(头) + 1(帧类型) 1(数据长度) + 32(数据) + 1(尾) = 36字节
uint8_t frame_head = 0;
uint8_t frame_type = 0x55;
uint8_t data_len = 32;
uint8_t frame_tail = 0;
uint8_t data_bytes[32]; // 8个float数据转成字节，共32字节

// #define BUF_SIZE 512
// #define FRAME_SIZE 36

// typedef struct {
//     uint8_t data[BUF_SIZE];
//     volatile uint16_t head;  // 写入位置
//     volatile uint16_t tail;  // 读取位置
// } ring_buffer_t;

// ring_buffer_t rb = {0};
// uint8_t dma_tx_buf[FRAME_SIZE];
// volatile uint8_t dma_busy = 0;

uint8_t need_send = 0;
uint8_t uart_tx_busy = 0;
uint8_t usb_tx_busy = 0;
// 定时器回调里发数据，用 USB CDC 发给视觉上位机，串口发给电控下位机
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		//		uint8_t temp_buf[FRAME_SIZE];
		// 10ms定时器
		GetAdcData(); // 读取通道采样数据

		// float数据转换成字节数组
		//		floatToBytes(sick_data, &tx_buf[3], 8); // 8个，但是发现工控机那边处理不了这么大的数据包，先发4个通道的
		floatToBytes(sick_data, &tx_buf[3], 4); // 4个通道

		// 组装数据帧
		tx_buf[1] = frame_type;
		tx_buf[2] = data_len;
		//		memcpy(&temp_buf[3], &temp_buf[3], 32); // 8个float数据，共32字节
		//		frame_head = calculateFrameHead(&tx_buf[1], 34);
		frame_head = calculateFrameHead(&tx_buf[1], 18);
		frame_tail = frame_head;
		tx_buf[0] = frame_head;
		//		tx_buf[35] = frame_tail;
		tx_buf[19] = frame_tail;

		// usb发送局部缓冲区
		// CDC_Transmit_FS(tx_buf, 20); // 发送很快，应该不用考虑tx_buf被下一个定时器中断重新覆盖的情况


//		if (uart_tx_busy == 0)
//		{ 
//			// 仅当DMA空闲时才触发发送
//			uart_tx_busy = 1;
			HAL_UART_Transmit_DMA(&huart1, tx_buf, 20);
//		}

		//		need_send = 1;
		//		//放入环形缓冲区
		//		uint16_t next_head = (rb.head + 1) % BUF_SIZE;
		//        if (next_head != rb.tail) // 缓冲区没满
		//		{
		//            memcpy(&rb.data[rb.head * FRAME_SIZE], temp_buf, FRAME_SIZE);
		//            rb.head = next_head;
		//        }
		// 		// 通过USB CDC发送
		// //		CDC_Transmit_FS(tx_buf, 36);

		// 		// 同时通过串口发送
		// 		HAL_UART_Transmit_DMA(&huart1, tx_buf, 36);
	}
}





void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		uart_tx_busy = 0; // DMA发送完成
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		uart_tx_busy = 0; // 出错也要释放标志
	}
}