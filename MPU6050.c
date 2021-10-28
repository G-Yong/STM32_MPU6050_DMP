/*
 * MPU6050.c
 *
 *  Created on: Oct 26, 2021
 *      Author: Administrator
 */

#include "MPU6050.h"

#include <stdio.h>

 u8 IIC_Write_Byte(u8 reg, u8 value)
 {
	 return IIC_Write_Len(MPU_ADDR, reg, 1, &value);
 }

 u8 IIC_Read_Byte(u8 reg)
 {
	 u8 value = 0;

	 IIC_Read_Len(MPU_ADDR, reg, 1, &value);

	 return value;
 }

 //IIC连续写
 u8 IIC_Write_Len(u8 addr,u8 reg, u8 len, u8 *buf)
 {
	 addr = (addr << 1) | 0;

	 u8 data[10] = {0};
	 data[0] = reg;
	 for(int i = 0; i < len; i++)
	 {
		 data[1 + i] = buf[i];
	 }

	 if(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)addr, (uint8_t*)data, len + 1, 10000)!= HAL_OK)
	 {
		 printf("++write len byte fail\r\n");
		 return 1;
	 }

	 return 0;
 }

 //IIC连续读
 u8 IIC_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
 {
	 u8 tmp  = (addr << 1) | 0;
	 if(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)tmp, (uint8_t*)&reg, 1, 10000)!= HAL_OK)
	 {
		 printf("++write byte fail\r\n");
		 return 1;
	 }

	 tmp  = (addr << 1) | 1;
	 if(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)tmp, (uint8_t *)buf, len, 10000) != HAL_OK)
	 {
		 printf("--read byte fail\r\n");
	 }

 }


//初始化MPU6050
//返回值: 0,成功
//        其他,错误代码
u8 MPU_Init(void)
{
    u8 res = 0;

    IIC_Write_Byte(MPU_PWR_MGMT1_REG,0X80);//复位MPU6050

    HAL_Delay(100);

    IIC_Write_Byte(MPU_PWR_MGMT1_REG,0X00);//唤醒MPU6050

    MPU_Set_Gyro_Fsr(3); //陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0); //加速度传感器 ±2g
    MPU_Set_Rate(50); //设置采样率50HZ
    IIC_Write_Byte(MPU_INT_EN_REG,0X00); //关闭所有中断
    IIC_Write_Byte(MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
    IIC_Write_Byte(MPU_FIFO_EN_REG,0X00);//关闭FIFO
    IIC_Write_Byte(MPU_INTBP_CFG_REG,0X80);//INT引脚低电平有效

    res=IIC_Read_Byte(MPU_DEVICE_ID_REG);

    printf("the addr is:%d\r\n", res);

    if(res==MPU_ADDR)//器件ID正确
    {
        IIC_Write_Byte(MPU_PWR_MGMT1_REG,0X01);//设置CLKSEL,PLL X 轴为参考
        IIC_Write_Byte(MPU_PWR_MGMT2_REG,0X00);//加速度陀螺仪都工作
        MPU_Set_Rate(50); //设置采样率为50HZ
    }
    else
    {
    	return 1;
    }

    return 0;
}

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
    return IIC_Write_Byte(MPU_GYRO_CFG_REG, fsr<<3);//设置陀螺仪满量程范围
}

//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
    return IIC_Write_Byte(MPU_ACCEL_CFG_REG, fsr<<3);//设置加速度传感器满量程范围
}

//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_LPF(u16 lpf)
{
    u8 data=0;
    if(lpf>=188) data=1;
    else if(lpf>=98) data=2;
    else if(lpf>=42) data=2;
    else if(lpf>=42) data=3;
    else if(lpf>=20) data=4;
    else if(lpf>=10) data=5;
    else data=6;
    return IIC_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器
}

//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败
u8 MPU_Set_Rate(u16 rate)
{
    u8 data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    data=IIC_Write_Byte(MPU_SAMPLE_RATE_REG, data);  //设置数字低通滤波器
    return MPU_Set_LPF(rate/2); //自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2];
    short raw;
    float temp;
    IIC_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw=((u16)buf[0]<<8)|buf[1];
    temp=36.53+((double)raw)/340;

    return temp*100;;
}

//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
// 其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;
    res=IIC_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if(res==0)
    {
        *gx=((u16)buf[0]<<8)|buf[1];
        *gy=((u16)buf[2]<<8)|buf[3];
        *gz=((u16)buf[4]<<8)|buf[5];
    }
    return res;
}

//得到加速度值(原始值)
//ax,ay,az:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;
    res=IIC_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if(res==0)
    {
        *ax=((u16)buf[0]<<8)|buf[1];
        *ay=((u16)buf[2]<<8)|buf[3];
        *az=((u16)buf[4]<<8)|buf[5];
    }
    return res;;
}

/*--------------------以下为使用板载固件DMP的相关代码，这个融合得到的姿态角效果很好*/


/**
  * @brief  获取当前毫秒值
  * @param  存储最新毫秒值的变量
  * @retval 无
  */
int get_tick_count(unsigned long *count)
{
   count[0] = HAL_GetTick();
	return 0;
}


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */ //100
#define DEFAULT_MPU_HZ  (50)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};

#define BYTE u_char
/* Send data to the Python client application.
 * Data is formatted as follows:
 * packet[0]    = $
 * packet[1]    = packet type (see packet_type_e)
 * packet[2+]   = data
 */
void send_packet(char packet_type, void *data)
{

	#define MAX_BUF_LENGTH  (18)
    char buf[MAX_BUF_LENGTH], length;

    memset(buf, 0, MAX_BUF_LENGTH);
    buf[0] = '$';
    buf[1] = packet_type;

    if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO) {
        short *sdata = (short*)data;
        buf[2] = (char)(sdata[0] >> 8);
        buf[3] = (char)sdata[0];
        buf[4] = (char)(sdata[1] >> 8);
        buf[5] = (char)sdata[1];
        buf[6] = (char)(sdata[2] >> 8);
        buf[7] = (char)sdata[2];
        length = 8;
    } else if (packet_type == PACKET_TYPE_QUAT) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        buf[10] = (char)(ldata[2] >> 24);
        buf[11] = (char)(ldata[2] >> 16);
        buf[12] = (char)(ldata[2] >> 8);
        buf[13] = (char)ldata[2];
        buf[14] = (char)(ldata[3] >> 24);
        buf[15] = (char)(ldata[3] >> 16);
        buf[16] = (char)(ldata[3] >> 8);
        buf[17] = (char)ldata[3];
        length = 18;
    } else if (packet_type == PACKET_TYPE_TAP) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        length = 4;
    } else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
        buf[2] = ((char*)data)[0];
        length = 3;
    } else if (packet_type == PACKET_TYPE_PEDO) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        length = 10;
    } else if (packet_type == PACKET_TYPE_MISC) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        buf[4] = ((char*)data)[2];
        buf[5] = ((char*)data)[3];
        length = 6;
    }

//    cdcSendDataWaitTilDone((BYTE*)buf, length, CDC0_INTFNUM, 100);
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}
static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static inline void run_self_test(void)
{
    int result;
    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }

    /* Report results. */
    test_packet[0] = 't';
    test_packet[1] = result;
    send_packet(PACKET_TYPE_MISC, test_packet);
}


#define INT_EXIT_LPM0 12

void DMP_Init(void)
{
    struct int_param_s int_param;

	/* Set up gyro.
	 * Every function preceded by mpu_ is a driver function and can be found
	 * in inv_mpu.h.
	 */
	int_param.cb = gyro_data_ready_cb;
	int_param.pin = 16;
	int_param.lp_exit = INT_EXIT_LPM0;
	int_param.active_low = 1;

	int result = mpu_init(&int_param);

	printf("mpu_init, %d\r\n", result);

	if(result == 0)     //mpu初始化
	{
		if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))    //设置需要的传感器
			printf("mpu_set_sensor complete ......\r\n");

		if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) //设置fifo
			printf("mpu_configure_fifo complete ......\r\n");

		if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))              //设置采集样率
			printf("mpu_set_sample_rate complete ......\r\n");


		unsigned short gyro_rate = 0, gyro_fsr = 0;
		unsigned char accel_fsr = 0;

	    mpu_get_sample_rate(&gyro_rate);
	    mpu_get_gyro_fsr(&gyro_fsr);
	    mpu_get_accel_fsr(&accel_fsr);

	    printf("gyro_rate %d, gyro_fsr %d, accel_fsr %d\r\n", gyro_rate, gyro_fsr, accel_fsr);



		if(!dmp_load_motion_driver_firmware())                //加载dmp固件
			printf("dmp_load_motion_driver_firmware complete ......\r\n");

		if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
			printf("dmp_set_orientation complete ......\r\n"); //设置陀螺仪方向

		if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
				DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
				DMP_FEATURE_GYRO_CAL))
			printf("dmp_enable_feature complete ......\r\n");

		if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))    //设置速率
			printf("dmp_set_fifo_rate complete ......\r\n");

		run_self_test();                          //自检(非必要)

		if(!mpu_set_dmp_state(1))                 //使能
			printf("mpu_set_dmp_state complete ......\r\n");
	}

}

#define q30 1073741824.0

uint8_t Read_DMP(float* Pitch,float* Roll,float* Yaw)
{
	short gyro[3], accel[3], sensors;
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	unsigned char more;
	long quat[4];

	int ret = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

//	printf("dmp_read_fifo, %d\r\n", ret);

	if(ret)
	{
		return 1;
	}

	if (sensors & INV_WXYZ_QUAT)
	{
		q0=quat[0] / q30;
		q1=quat[1] / q30;
		q2=quat[2] / q30;
		q3=quat[3] / q30;
		*Pitch = (float)asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
		*Roll =  (float)atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
		*Yaw =   (float)atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
		return 0;
	}
	else
	{
		return 2;
	}

}






