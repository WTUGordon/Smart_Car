/******************************************************
 * 龙邱库移植到野火库。
 * by          Gordon
 * 时间        2019/3/3
 * 文件名      8700_2100.c
 * 原文件名    8700_2100.c
 * 内容        陀螺仪加速度计数据获取
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#include "include.h"


void Init2100(void)
{
         ///////FXAS21002//////////////////////////////////////////////////////////////////////////////////////////
      // write 0000 0000 = 0x00 to CTRL_REG1 to place FXOS21002 in Standby
      // [7]: ZR_cond=0
      // [6]: RST=0
      // [5]: ST=0 self test disabled
      // [4-2]: DR[2-0]=000 for 800Hz
      // [1-0]: Active=0, Ready=0 for Standby mode
      IIC_WriteByteToSlave( FXAS21002_ADDR, FXAS21002_CTRL_REG1, 0x00); //设置采样率800Hz
      // write 0000 0000 = 0x00 to CTRL_REG0 to configure range and filters
      // [7-6]: BW[1-0]=00, LPF disabled
      // [5]: SPIW=0 4 wire SPI (irrelevant)
      // [4-3]: SEL[1-0]=00 for 10Hz HPF at 200Hz ODR
      // [2]: HPF_EN=0 disable HPF
      // [1-0]: FS[1-0]=00 for 1600dps (TBD CHANGE TO 2000dps when final trimmed parts available)
      IIC_WriteByteToSlave(FXAS21002_ADDR, FXAS21002_CTRL_REG0, 0x00);   //陀螺仪传感器,±2000dps      °/s

      //delayms(100);

      // write 0000 0001 = 0x01 to CTRL_REG1 to configure 800Hz ODR and enter Active mode
      // [7]: ZR_cond=0
      // [6]: RST=0
      // [5]: ST=0 self test disabled
      // [4-2]: DR[2-0]=000 for 800Hz ODR
      // [1-0]: Active=1, Ready=0 for Active mode
      IIC_WriteByteToSlave(FXAS21002_ADDR, FXAS21002_CTRL_REG1, 0x03);   //陀螺仪工作
//        i2c_write_reg(I2C1,SlaveAddress2100,0x0d,0x02);
//        i2c_write_reg(I2C1,SlaveAddress2100,CTRL_REG1_2100,0x02);

}

void Init8700(void)
{
  uint8_t val;
  IIC_ReadByteFromSlave(FXOS8700_ADDR, FXOS8700_CTRL_REG1, &val);  //读CTRL1寄存器
  IIC_WriteByteToSlave(FXOS8700_ADDR, FXOS8700_CTRL_REG1, val & (uint8_t)~ACTIVE_MASK);   //使8700处于待机模式
  IIC_WriteByteToSlave(FXOS8700_ADDR, F_SETUP_REG,F_MODE_DISABLED);    //关FIFO
  //IIC_WriteByteToSlave(FXOS8700_ADDR, FXOS8700_M_CTRL_REG2, MOD_HIGH_RES);
 // IIC_WriteByteToSlave( FXOS8700_ADDR, M_CTRL_REG1, (M_RST_MASK | M_OSR_MASK | M_HMS_MASK));   //混合模式，加计和地磁计同时使用
 // IIC_WriteByteToSlave(FXOS8700_ADDR, M_CTRL_REG2, M_HYB_AUTOINC_MASK);
  IIC_WriteByteToSlave(FXOS8700_ADDR, XYZ_DATA_CFG_REG, FULL_SCALE_2G);       //加计 正负2g模式
  IIC_WriteByteToSlave(FXOS8700_ADDR, FXOS8700_CTRL_REG1, (HYB_DATA_RATE_200HZ | ACTIVE_MASK));       //设置数据输出频率 200hz 并且激活FX8700

     //  I2C_WriteAddr(I2C1,SlaveAddress8700,0x0f,0x33);
//i2c_write_reg(I2C1,SlaveAddress8700,CTRL_REG1_8700,0x05);

}


void Get_Acc(int16 *ax,int16 *ay,int16 *az)
{
  uint8 acc_buf[6];
  IIC_ReadMultByteFromSlave(FXOS8700_ADDR,0x01,6,acc_buf);

  *ax = ((int16_t)((uint16_t)acc_buf[0]<<8 | (uint16_t)acc_buf[1]));  //加速度计14位的，低两位影响不大，直接按16位的用
  *ay = ((int16_t)((uint16_t)acc_buf[2]<<8 | (uint16_t)acc_buf[3]));
  *az = ((int16_t)((uint16_t)acc_buf[4]<<8 | (uint16_t)acc_buf[5]));
}

void Get_Gyro(int16 *gx,int16 *gy,int16 *gz)
{
  uint8 gyr_buf[6];
  IIC_ReadMultByteFromSlave(FXAS21002_ADDR,0x01,6,gyr_buf);

  *gx = (int16_t)((uint16_t)gyr_buf[0]<<8 | (uint16_t)gyr_buf[1]);
  *gy = (int16_t)((uint16_t)gyr_buf[2]<<8 | (uint16_t)gyr_buf[3]);
  *gz = (int16_t)((uint16_t)gyr_buf[4]<<8 | (uint16_t)gyr_buf[5]);
}

void Get_Offset(void)
{
	static uint16  n = 0;
	static float XF = 0, YF = 0, ZF = 0;
    int16 GyroX,GyroY,GyroZ;
	Get_Gyro(&GyroX,&GyroY,&GyroZ);
	XF += GyroX;
	YF += GyroY;
	ZF += GyroZ;
	n++;

	if (n == 1000)
	{
		GYRO_Real.Xoffset = XF * 0.001f;
		GYRO_Real.Yoffset = YF * 0.001f;
		GYRO_Real.Zoffset = ZF * 0.001f;
		GYRO_Real.GyrooffsetOK = 1;
		XF = 0;
		YF = 0;
		ZF = 0;
		n = 0;
	}
	return;
}