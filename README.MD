简介
---
本项目是将MPU6050的DMP5.1固件库移植到stm32。

所使用的IDE是stm32cubeIDE，挺好用的。虽然牺牲了一定的芯片性能，但是的确是将整个写代码（主要是配置各个库、寄存器之类的）的流程给优化了。

移植时使用了stm32的硬件I2C1

项目的代码上，是对很多人的代码进行了复制粘贴，后续我再增加一些代码来源的说明。

DMP的姿态融合的确牛逼，甚至翻转180度都可以很跟手。效果可以看一下 ![video](./效果.mp4)

使用说明
---
MPU6050模块的INT引脚接到你的单片机上

在main函数中初始化

然后在中断回调函数中读取DMP的值，通过串口发送给上位机

```cpp
#include "MPU6050.h"

int main(void)
{
    ...

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  DMP_Init();

  while (1)
  {
      //led跑马灯
	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	  HAL_Delay(1000);
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_5)
    {
//    	printf("interrupt--\r\n");

  	  float Pitch, Roll, Yaw;

  	  uint8_t ret = Read_DMP(&Pitch, &Roll, &Yaw);

//  	  printf("Read_DMP Return is %d \r\n", ret);
//  	  printf("Pitch:%f, Roll:%f, Yaw:%f \r\n",Pitch, Roll, Yaw);

  	  printf("%f,%f,%f\r\n", Pitch, Roll, Yaw);

    }
}
```