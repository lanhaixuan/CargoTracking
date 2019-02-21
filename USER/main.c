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

u8 USART1_TX_BUF[USART2_MAX_RECV_LEN]; 					//����1,���ͻ�����
nmea_msg gpsx; 											//GPS��Ϣ

char lon_str_end[15];//����
char lat_str_end[15];//γ��


//��ʾGPS��λ��Ϣ 
void Gps_Msg_Show(void)
{
 	float tp;		   
	tp=gpsx.longitude;
  if(tp>1)	
	sprintf(lon_str_end,"%.6f",tp=(tp/100000));	//�õ������ַ���
	printf("\r\n���ȣ�%s\r\n",lon_str_end); 	   
	tp=gpsx.latitude;	
	if(tp>1)
	sprintf(lat_str_end,"%.6f",tp=(tp/100000));	//�õ�γ���ַ���
	printf("\r\nγ�ȣ�%s\r\n\r\n",lat_str_end); 	 
}

int main(void)
{
	u16 i,rxlen;
	u16 lenx,t;
	u8 key=0XFF;
	u8 upload=0;
 static	float pitch,roll,yaw; 		//ŷ����
//	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
//	short gyrox,gyroy,gyroz;	//������ԭʼ����	
  static float P=0,R=0,Y=0;
	
	delay_init(); //��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����Ϊ��2��2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200	
  usart2_init(38400);		//��ʼ������3������Ϊ38400
	usart3_init(115200);		//��ʼ������3
	sim900a_gprs_test();   //��ʼ��gprs����
	
	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������ 
  MPU_Init();					//��ʼ��MPU6050	

	delay_ms(1500);
  LED0=1;	

	while(mpu_dmp_init())//���MPU6050
	{
		LED0=!LED0;
		delay_ms(200);
	}
	
	if(Ublox_Cfg_Rate(1000,1)!=0)	//���ö�λ��Ϣ�����ٶ�Ϊ1000ms,˳���ж�GPSģ���Ƿ���λ. 
	{
		LED1=0;
		while((Ublox_Cfg_Rate(1000,1)!=0)&&key)	//�����ж�,ֱ�����Լ�鵽NEO-6M,�����ݱ���ɹ�
		{
			usart2_init(9600);			//��ʼ������3������Ϊ9600(EEPROMû�б������ݵ�ʱ��,������Ϊ9600.)
	  		Ublox_Cfg_Prt(38400);			//��������ģ��Ĳ�����Ϊ38400
			usart2_init(38400);			//��ʼ������3������Ϊ38400
			Ublox_Cfg_Tp(1000000,100000,1);	//����PPSΪ1�������1��,������Ϊ100ms	    
			key=Ublox_Cfg_Cfg_Save();		//��������  
		}	  					 
		delay_ms(500);
		LED1=1;
	}
	LED0=1;
	  
	while (1)
	{
		delay_ms(1);		
		if(USART2_RX_STA&0X8000)		//���յ�һ��������
		{
			rxlen=USART2_RX_STA&0X7FFF;	//�õ����ݳ���
			for(i=0;i<rxlen;i++)USART1_TX_BUF[i]=USART2_RX_BUF[i];	   
 			USART2_RX_STA=0;		   	//������һ�ν���
			USART1_TX_BUF[i]=0;			//�Զ���ӽ�����
			GPS_Analysis(&gpsx,(u8*)USART1_TX_BUF);//�����ַ���
			Gps_Msg_Show();				//��ʾ��Ϣ	
			if(upload)printf("\r\n%s\r\n",USART1_TX_BUF);//���ͽ��յ������ݵ�����1
 		}
    if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			printf("pitch:%f  roll:%f  yaw:%f\r\n",pitch ,roll ,yaw );
			printf("P:%f  R:%f  Y:%f\r\n",P ,R ,Y );
			printf("pitch1:%f  roll1:%f  yaw1:%f\r\n\r\n",(P-pitch) ,(R-roll) ,(Y-yaw));
//			delay_ms(100);
//			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
//			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			
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
					t++;//һ���ԼӴ��10us
					if(t==50000)//Ϊ��ʵ�ֵ͹���
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

