#include "sim900a.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"   	 
#include "key.h"	 	 	 	 	 	  	 
#include "string.h"    	
#include "usart3.h" 
#include "gps.h"

//////////////////////////////////////////////////////////////////////////////////	   
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������ 
//ATK-SIM900A GSM/GPRSģ������	  
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/4/12
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//******************************************************************************** 
//��
/////////////////////////////////////////////////////////////////////////////////// 
extern 	char lon_str_end[15];//γ��
extern 	char lat_str_end[15];//γ��

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

//sim900a���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//    ����,�ڴ�Ӧ������λ��(str��λ��)
u8* sim900a_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART3_RX_STA&0X8000)		//���յ�һ��������
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
//��sim900a��������
//cmd:���͵������ַ���(����Ҫ��ӻس���),��cmd<0XFF��ʱ��,��������(���緢��0X1A),���ڵ�ʱ�����ַ���.
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
u8 sim900a_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART3_RX_STA=0;
	if((u32)cmd<=0XFF)
	{
		while((USART3->SR&0X40)==0);//�ȴ���һ�����ݷ������  
		USART3->DR=(u32)cmd;
	}else u3_printf("%s\r\n",cmd);//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			delay_ms(10);
			if(USART3_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(sim900a_check_cmd(ack))break;//�õ���Ч���� 
				USART3_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
} 
//��1���ַ�ת��Ϊ16��������
//chr:�ַ�,0~9/A~F/a~F
//����ֵ:chr��Ӧ��16������ֵ
u8 sim900a_chr2hex(u8 chr)
{
	if(chr>='0'&&chr<='9')return chr-'0';
	if(chr>='A'&&chr<='F')return (chr-'A'+10);
	if(chr>='a'&&chr<='f')return (chr-'a'+10); 
	return 0;
}
//��1��16��������ת��Ϊ�ַ�
//hex:16��������,0~15;
//����ֵ:�ַ�
u8 sim900a_hex2chr(u8 hex)
{
	if(hex<=9)return hex+'0';
	if(hex>=10&&hex<=15)return (hex-10+'A'); 
	return '0';
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//tcp/udp����
//����������,��ά������
//mode:0:TCP����;1,UDP����)
//ipaddr:ip��ַ
//port:�˿� 
void sim900a_tcpudp_test()
{ 
	u8 key;
	u16 timex=0;
	u16 count=0;
	u8 connectsta=0;			//0,��������;1,���ӳɹ�;2,���ӹر�; 
	u8 hbeaterrcnt=0;			//�������������,����5�������ź���Ӧ��,����������
	
	char sensor_id_temp[]="0001";
	char OneNetServer[]="api.heclouds.com";
	char device_id_temp[]="your device id";//�豸ID
	char API_VALUE_temp[]="your apikey";//APIKEY
	
	char send_buf[400]= {0};
	char text[100] = {0};
	char tmp[25] = {0};


  memset(send_buf, 0, 400); 
   /*׼��JSON��*/
    sprintf(text,"{\"datastreams\":[{\"id\":\"%s\",\"datapoints\":[{\"value\":{\"lon\":%s,\"lat\":%s}}]}]}"
      ,sensor_id_temp,lon_str_end,lat_str_end);
  
    /*׼��HTTP��ͷ*/
    send_buf[0] = 0;
    strcat(send_buf,"POST /devices/");
    strcat(send_buf,device_id_temp);
    strcat(send_buf,"/datapoints HTTP/1.1\r\n");//ע���\r\n
    strcat(send_buf,"api-key:");
    strcat(send_buf,API_VALUE_temp);
    strcat(send_buf,"\r\n");
    strcat(send_buf,"Host:");
    strcat(send_buf,OneNetServer);
    strcat(send_buf,"\r\n");
    sprintf(tmp,"Content-Length:%d\r\n\r\n", strlen(text));//����JSON����
    strcat(send_buf,tmp);
    strcat(send_buf,text);	
		
	if(sim900a_send_cmd("AT+CIPSTART=\"TCP\",\"183.230.40.33\",\"80\"","OK",500))return;		//��������/////////IP
	while(1)
	{ 
		key=KEY_Scan(0);
		if(key==WKUP_PRES)//�˳�����		 
		{  
			sim900a_send_cmd("AT+CIPCLOSE=1","CLOSE OK",500);	//�ر�����
			sim900a_send_cmd("AT+CIPSHUT","SHUT OK",500);		//�ر��ƶ����� 
			break;												 
		}
		if((timex%20)==0)
		{
			LED0=!LED0;
			count++;
				if(count==3600)
				{
					count=0;
					printf("���ݷ���ʧ��!!!\r\n");
					return;
				}	
			if(connectsta==2||hbeaterrcnt>8)//�����ж���,��������8������û����ȷ���ͳɹ�,����������
			{
				sim900a_send_cmd("AT+CIPCLOSE=1","CLOSE OK",500);	//�ر�����
				sim900a_send_cmd("AT+CIPSHUT","SHUT OK",500);		//�ر��ƶ����� 
				sim900a_send_cmd("AT+CIPSTART=\"TCP\",\"183.230.40.33\",\"80\"","OK",500);//������������//////////////////IP
				connectsta=0;	
 				hbeaterrcnt=0;
			}
		}
		if(connectsta==0&&(timex%200)==0)//���ӻ�û������ʱ��,ÿ2���ѯһ��CIPSTATUS.
		{
			sim900a_send_cmd("AT+CIPSTATUS","OK",500);	//��ѯ����״̬
			if(strstr((const char*)USART3_RX_BUF,"CLOSED"))connectsta=2;
			if(strstr((const char*)USART3_RX_BUF,"CONNECT OK"))connectsta=1;
		}
		if(connectsta==1&&timex>=600)//����������ʱ��,ÿ6�뷢��һ������
		{
			timex=0;
			if(sim900a_send_cmd("AT+CIPSEND",">",200)==0)//��������
			{
				sim900a_send_cmd((u8*)send_buf,0,0);	//��������:0X00  ////////////////////////////////����
				delay_ms(200);						//�������ʱ
				sim900a_send_cmd((u8*)0X1A,0,0);	//CTRL+Z,�������ݷ���,����һ�δ���	
				delay_ms(1000);					
				break;
			}else sim900a_send_cmd((u8*)0X1B,0,0);	//ESC,ȡ������ 		
				
			hbeaterrcnt++; 
			printf("hbeaterrcnt:%d\r\n",hbeaterrcnt);//������Դ���
		} 
		delay_ms(10);
		timex++; 
	} 
}

//sim900a GPRS����
//���ڲ���TCP/UDP����
//����ֵ:0,����
//    ����,�������
u8 sim900a_gprs_test(void)
{
 	sim900a_send_cmd("AT+CIPCLOSE=1","CLOSE OK",100);	//�ر�����
	sim900a_send_cmd("AT+CIPSHUT","SHUT OK",100);		//�ر��ƶ����� 
	if(sim900a_send_cmd("AT+CGCLASS=\"B\"","OK",1000))return 1;				//����GPRS�ƶ�̨���ΪB,֧�ְ����������ݽ��� 
	if(sim900a_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",1000))return 2;//����PDP������,��������Э��,��������Ϣ
	if(sim900a_send_cmd("AT+CGATT=1","OK",500))return 3;					//����GPRSҵ��
	if(sim900a_send_cmd("AT+CIPCSGP=1,\"CMNET\"","OK",500))return 4;	 	//����ΪGPRS����ģʽ
	if(sim900a_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//���ý���������ʾIPͷ(�����ж�������Դ)
	printf("����GPRS��������������!!!\r\n");
	 sim900a_tcpudp_test();
	printf("���ݷ������!!!\r\n");
	printf("\r\n");
	return 0;
	
} 














