#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "usart3.h"
#include "sim900a.h"
#include "mpu6050.h"  
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "led.h"
#include "key.h"
#include "string.h"	
#include "gps.h" 

u8 USART1_TX_BUF[USART2_MAX_RECV_LEN]; 					//串口1,发送缓存区
nmea_msg gpsx; 											//GPS信息

char lon_str_end[15];//精度
char lat_str_end[15];//纬度


//显示GPS定位信息 
void Gps_Msg_Show(void)
{
 	float tp;		   
	tp=gpsx.longitude;
  if(tp>1)	
	sprintf(lon_str_end,"%.6f",tp=(tp/100000));	//得到经度字符串
	printf("\r\n经度：%s\r\n",lon_str_end); 	   
	tp=gpsx.latitude;	
	if(tp>1)
	sprintf(lat_str_end,"%.6f",tp=(tp/100000));	//得到纬度字符串
	printf("\r\n纬度：%s\r\n\r\n",lat_str_end); 	 
}

int main(void)
{
	u16 i,rxlen;
	u16 lenx,t;
	u8 key=0XFF;
	u8 upload=0;
 static	float pitch,roll,yaw; 		//欧拉角
//	short aacx,aacy,aacz;		//加速度传感器原始数据
//	short gyrox,gyroy,gyroz;	//陀螺仪原始数据	
  static float P=0,R=0,Y=0;
	
	delay_init(); //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组为组2：2位抢占优先级，2位响应优先级
	uart_init(115200);	 	//串口初始化为115200	
  usart2_init(38400);		//初始化串口3波特率为38400
	usart3_init(115200);		//初始化串口3
	sim900a_gprs_test();   //初始化gprs参数
	
	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键 
  MPU_Init();					//初始化MPU6050	

	delay_ms(1500);
  LED0=1;	

	while(mpu_dmp_init())//检测MPU6050
	{
		LED0=!LED0;
		delay_ms(200);
	}
	
	if(Ublox_Cfg_Rate(1000,1)!=0)	//设置定位信息更新速度为1000ms,顺便判断GPS模块是否在位. 
	{
		LED1=0;
		while((Ublox_Cfg_Rate(1000,1)!=0)&&key)	//持续判断,直到可以检查到NEO-6M,且数据保存成功
		{
			usart2_init(9600);			//初始化串口3波特率为9600(EEPROM没有保存数据的时候,波特率为9600.)
	  		Ublox_Cfg_Prt(38400);			//重新设置模块的波特率为38400
			usart2_init(38400);			//初始化串口3波特率为38400
			Ublox_Cfg_Tp(1000000,100000,1);	//设置PPS为1秒钟输出1次,脉冲宽度为100ms	    
			key=Ublox_Cfg_Cfg_Save();		//保存配置  
		}	  					 
		delay_ms(500);
		LED1=1;
	}
	LED0=1;
	  
	while (1)
	{
		delay_ms(1);		
		if(USART2_RX_STA&0X8000)		//接收到一次数据了
		{
			rxlen=USART2_RX_STA&0X7FFF;	//得到数据长度
			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART2_RX_BUF[i];	   
 			USART2_RX_STA=0;		   	//启动下一次接收
			USART1_TX_BUF[i]=0;			//自动添加结束符
			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//分析字符串
			Gps_Msg_Show();				//显示信息	
			if(upload)printf("\r\n%s\r\n",USART1_TX_BUF);//发送接收到的数据到串口1
 		}
    if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			printf("pitch:%f  roll:%f  yaw:%f\r\n",pitch ,roll ,yaw );
			printf("P:%f  R:%f  Y:%f\r\n",P ,R ,Y );
			printf("pitch1:%f  roll1:%f  yaw1:%f\r\n\r\n",(P-pitch) ,(R-roll) ,(Y-yaw));
//			delay_ms(100);
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			
				if(pitch<-3 || pitch>3 ||roll<-3 || roll>3||yaw<-30 || yaw>30)
				{
						printf("pitch2:%f  roll2:%f  yaw2:%f\r\n\r\n",(P-pitch) ,(R-roll) ,(Y-yaw));
					if(P&&((((P-pitch)>2)||((P-pitch)<-2) || ((R-roll)>2)||((R-roll)<-2))))
					{
						printf("Enter \r\n");
						printf("pitch3:%f  roll3:%f  yaw3:%f\r\n\r\n",(P-pitch) ,(R-roll) ,(Y-yaw));
						LED1=1;
						LED0=1;
						sim900a_gprs_test();
						delay_ms(300);
						LED1=1;
						LED0=1;
												
						P=pitch;
			      R=roll;
			      Y=yaw;
					}				
				}
				else if(P&&((fabs(P-pitch)<2) || (fabs(R-roll)<2)||(fabs(Y-yaw)<2)))
				{
					t++;//一次自加大概10us
					if(t==50000)//为了实现低功耗
					{
						LED0=0;
						LED1=0;
					}
				}	
     if((lenx%100)==0)	
		 {
			P=pitch;
			R=roll;
			Y=yaw;
		 }
		}			
		if((lenx%200)==0)LED1=!LED1; 	    				 
		lenx++;	
		if(lenx==65535) lenx=0;

	}
}

