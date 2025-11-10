#include "main.h"
#include "AD7606.h"
#include "delay.h"
#include "usbd_cdc_if.h"
#include "tim.h"
#include "usart.h"

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

int16_t CH_data_buf[8];		// AD7606通道数据缓存
float sick_data[8];	// 转换成电压值的缓存

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

//禁止Delay!!!时序会有大问题
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
			AD_SCK_0; // 拉低SCLK
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
        uint32_t temp = *(uint32_t *)&f[i];  // 共用体或指针转换（避免类型转换警告）
        bytes[i*4 + 0] = (temp >> 0)  & 0xFF;  // 低字节
        bytes[i*4 + 1] = (temp >> 8)  & 0xFF;
        bytes[i*4 + 2] = (temp >> 16) & 0xFF;
        bytes[i*4 + 3] = (temp >> 24) & 0xFF;  // 高字节
    }
}

// 读取通道数据,手动改参吧，没有必要做封装
void GetAdcData()
{
	float sick_param[8] = {0};//神秘参数，每增加一个sick，自己调

	sick_param[0] = 0.7199;
	sick_param[1] = 0.7199;
	sick_param[2] = 0.7199;
	sick_param[3] = 0.7199;
	sick_param[4] = 0.7199;
	sick_param[5] = 0.7199;
	sick_param[6] = 0.7199;
	sick_param[7] = 0.7199;

	ADStartConv();

	while (READ_AD_BUSY == 1)
	{
		HAL_Delay(1);
	}

	delay_us(1);

	SpiReadData(CH_data_buf);

	sick_data[0] = (float)CH_data_buf[0] / 32768.0f * 10 * sick_param[0]; // 得到读取的电压值
	sick_data[1] = (float)CH_data_buf[1] / 32768.0f * 10 * sick_param[1];
	sick_data[2] = (float)CH_data_buf[2] / 32768.0f * 10 * sick_param[2];
	sick_data[3] = (float)CH_data_buf[3] / 32768.0f * 10 * sick_param[3];
	sick_data[4] = (float)CH_data_buf[4] / 32768.0f * 10 * sick_param[4];
	sick_data[5] = (float)CH_data_buf[5] / 32768.0f * 10 * sick_param[5];
	sick_data[6] = (float)CH_data_buf[6] / 32768.0f * 10 * sick_param[6];
	sick_data[7] = (float)CH_data_buf[7] / 32768.0f * 10 * sick_param[7];
}

uint8_t tx_buf[36] = {0}; // 1(头) + 1(帧类型) 1(数据长度) + 32(数据) + 1(尾) = 36字节
uint8_t frame_head = 0;
uint8_t frame_type = 0x55;
uint8_t data_len = 32;
uint8_t frame_tail = 0;
uint8_t data_bytes[32]; // 8个float数据转成字节，共32字节

// 定时器回调里发数据，用 USB CDC 发给视觉上位机，串口发给电控下位机
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1)
	{
		// 10ms定时器
		GetAdcData();//读取通道采样数据

		// float数据转换成字节数组
		floatToBytes(sick_data, data_bytes, 8); // 8个

		// 组装数据帧
		tx_buf[1] = frame_type;
		tx_buf[2] = data_len;
		memcpy(&tx_buf[3], data_bytes, 32); // 8个float数据，共32字节
		frame_head = calculateFrameHead(&tx_buf[1], 34);
		frame_tail = frame_head;
		tx_buf[0] = frame_head;
		tx_buf[35] = frame_tail;

		// 通过USB CDC发送
		CDC_Transmit_FS(tx_buf, 36);

		// 同时通过串口发送
		HAL_UART_Transmit_DMA(&huart1, tx_buf, 36);
	}
}