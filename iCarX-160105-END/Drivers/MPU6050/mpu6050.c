#include "mpu6050.h"
//#include "sys.h"
//#include "delay.h"
//#include "usart.h" 
#include "RTC.h"

extern ECUstat car_state;
extern uint8_t mpu6050_buf[];
extern uint8_t ID_CFG[];
extern uint16_t Accela_this_times;
extern uint16_t Brake_this_times;
extern uint32_t Accela_all_times;
extern uint32_t Brake_all_times;
	

//初始化MPU6050
//返回值:0,成功
//    其他,错误代码
u8 MPU_Init(void)
{ 
	u8 res;
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    DelayMs(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO 
	MPU_Write_Byte(MPU_MOTION_DET_REG,0X01);   //运动检测阈值
	//MPU_Write_Byte(MPU_MDETECT_CTRL_REG,0x00);  //这句语句不能有，否则不能产生运动中断
	MPU_Write_Byte(MPU6050_RA_MOT_DUR,0X01);
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效 
	MPU_Write_Byte(MPU_INT_EN_REG,0X40);	//打开运动中断
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else return 1;
	return 0;
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
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
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		IIC_Send_Byte(buf[i]);	//发送数据
		if(IIC_Wait_Ack())		//等待ACK
		{
			IIC_Stop();	 
			return 1;		 
		}		
	}    
    IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	IIC_Start(); 
	IIC_Send_Byte((addr<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((addr<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	while(len)
	{
		if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(IIC_Wait_Ack())	//等待应答
	{
		IIC_Stop();		 
		return 1;		
	}
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答 
	IIC_Send_Byte(data);//发送数据
	if(IIC_Wait_Ack())	//等待ACK
	{
		IIC_Stop();	 
		return 1;		 
	}		 
    IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    IIC_Start(); 
	IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	IIC_Wait_Ack();		//等待应答 
    IIC_Send_Byte(reg);	//写寄存器地址
    IIC_Wait_Ack();		//等待应答
    IIC_Start();
	IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    IIC_Wait_Ack();		//等待应答 
	res=IIC_Read_Byte(0);//读取数据,发送nACK 
    IIC_Stop();			//产生一个停止条件 
	return res;		
}

//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[24] ={"$AT 16 "};

  mem_copy(tbuf, "$AT 16 ");
  tbuf[7] = 0;
  tbuf[8] = 12;	
	tbuf[9]=(aacx>>8)&0XFF;
	tbuf[10]=aacx&0XFF;
	tbuf[11]=(aacy>>8)&0XFF;
	tbuf[12]=aacy&0XFF;
	tbuf[13]=(aacz>>8)&0XFF;
	tbuf[14]=aacz&0XFF; 
	tbuf[15]=(gyrox>>8)&0XFF;
	tbuf[16]=gyrox&0XFF;
	tbuf[17]=(gyroy>>8)&0XFF;
	tbuf[18]=gyroy&0XFF;
	tbuf[19]=(gyroz>>8)&0XFF;
	tbuf[20]=gyroz&0XFF;
	tbuf[21]='\n';
	SCI_Transmit(1,22,tbuf);	
}	

//发送熄火碰撞检测数据
void mpu6050_send_idledata(void)
{
	RTC_timer timempu;
	uint8_t i,Int_status = 0;
	short aacx = 0,aacy = 0,aacz = 0;		//加速度传感器原始数据
	short gyrox = 0,gyroy = 0,gyroz = 0;	//陀螺仪原始数据
	
	Int_status = MPU_Read_Byte(MPU_INT_STA_REG);
	
	if((Int_status & 0x40)!=0)   //运动中断产生
	{
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
		mem_copy(mpu6050_buf, "$AT 16 ");
		mpu6050_buf[7] = 0;
		mpu6050_buf[8] = 20;
		for(i=9;i<13;i++)
				mpu6050_buf[i] = ID_CFG[i-9];
		get_time(&timempu);			
		mpu6050_buf[13] = timempu.hour;
		mpu6050_buf[14] = timempu.minute;
		mpu6050_buf[15] = timempu.second;
		mpu6050_buf[16] = 1;
		mpu6050_buf[17] = (aacx >> 8) & 0xFF;
		mpu6050_buf[18] = aacx & 0xFF;
		mpu6050_buf[19] = (aacy >> 8) & 0xFF;
		mpu6050_buf[20] = aacy & 0xFF;
		mpu6050_buf[21] = (aacz >> 8) & 0xFF;
		mpu6050_buf[22] = aacz & 0xFF;
		mpu6050_buf[23] = (gyrox >> 8) & 0xFF;
		mpu6050_buf[24] = gyrox & 0xFF;
		mpu6050_buf[25] = (gyroy >> 8) & 0xFF;
		mpu6050_buf[26] = gyroy & 0xFF;
		mpu6050_buf[27] = (gyroz >> 8) & 0xFF;
		mpu6050_buf[28] = gyroz & 0xFF;
		mpu6050_buf[29] = '\n';
		SCI_Transmit(1,30,mpu6050_buf);		
	}
}


//急加急减判断
void mpu6050_judge(void)
{
	short aacx = 0,aacy = 0,aacz = 0;		//加速度传感器原始数据
	short ax = 0,ay = 0,az = 0;
	uint8_t jbuf[15] = {"$AT 17 "};
	
	if(car_state > IDLE)
	{
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
		ax = 2*98*aacx/32768;   //乘以10
		ay = 2*98*aacy/32768;
		az = 2*98*aacz/32768; 
		if(ax<0)                //判断x轴方向的加速度
		{
			ax = -ax;
			if(ax > 98)
			{
				Brake_this_times++;
				Brake_all_times++;
			}
		}
		else
		{
			if(ax > 98)
			{
				Accela_this_times++;
				Accela_all_times++;
			}
		}
		if(ay<0)                     //判断y轴方向的加速度
		{
			ay = -ay;
			if(ay > 98)
			{
				Brake_this_times++;
				Brake_all_times++;
			}
		}
		else
		{
			if(ay > 98)
			{
				Accela_this_times++;
				Accela_all_times++;
			}
		}
		if(az<0)     //判断z轴方向的加速度
		{
			az = -az;
			if(az > 98)
			{
				Brake_this_times++;
				Brake_all_times++;
			}
		}
		else
		{
			if(az > 98)
			{
				Accela_this_times++;
				Accela_all_times++;
			}
		}
//		jbuf[7] = 0;
//		jbuf[8] = 4;
//		jbuf[9] = (Accela_this_times >> 8)&0xFF;
//		jbuf[10] = Accela_this_times & 0xFF;
//		jbuf[11] = (Brake_this_times >> 8)&0xFF;
//		jbuf[12] = Brake_this_times & 0xFF;
//		jbuf[13] = '\n';
//		SCI_Transmit(1,14,jbuf);
	}
	else if(car_state < 1)
	{
		Accela_this_times = 0;
		Brake_this_times = 0;
	}
}


