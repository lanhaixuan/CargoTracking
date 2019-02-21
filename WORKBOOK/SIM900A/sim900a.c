#include "sim900a.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"   	 
#include "key.h"	 	 	 	 	 	  	 
#include "string.h"    	
#include "usart3.h" 
#include "gps.h"

//////////////////////////////////////////////////////////////////////////////////	   
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板 
//ATK-SIM900A GSM/GPRS模块驱动	  
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/4/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//******************************************************************************** 
//无
/////////////////////////////////////////////////////////////////////////////////// 
extern 	char lon_str_end[15];//纬度
extern 	char lat_str_end[15];//纬度

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 

//sim900a发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//    其他,期待应答结果的位置(str的位置)
u8* sim900a_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART3_RX_STA&0X8000)		//接收到一次数据了
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//添加结束符
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
//向sim900a发送命令
//cmd:发送的命令字符串(不需要添加回车了),当cmd<0XFF的时候,发送数字(比如发送0X1A),大于的时候发送字符串.
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 sim900a_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART3_RX_STA=0;
	if((u32)cmd<=0XFF)
	{
		while((USART3->SR&0X40)==0);//等待上一次数据发送完成  
		USART3->DR=(u32)cmd;
	}else u3_printf("%s\r\n",cmd);//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			delay_ms(10);
			if(USART3_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(sim900a_check_cmd(ack))break;//得到有效数据 
				USART3_RX_STA=0;
			} 
		}
		if(waittime==0)res=1; 
	}
	return res;
} 
//将1个字符转换为16进制数字
//chr:字符,0~9/A~F/a~F
//返回值:chr对应的16进制数值
u8 sim900a_chr2hex(u8 chr)
{
	if(chr>='0'&&chr<='9')return chr-'0';
	if(chr>='A'&&chr<='F')return (chr-'A'+10);
	if(chr>='a'&&chr<='f')return (chr-'a'+10); 
	return 0;
}
//将1个16进制数字转换为字符
//hex:16进制数字,0~15;
//返回值:字符
u8 sim900a_hex2chr(u8 hex)
{
	if(hex<=9)return hex+'0';
	if(hex>=10&&hex<=15)return (hex-10+'A'); 
	return '0';
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//tcp/udp测试
//带心跳功能,以维持连接
//mode:0:TCP测试;1,UDP测试)
//ipaddr:ip地址
//port:端口 
void sim900a_tcpudp_test()
{ 
	u8 key;
	u16 timex=0;
	u16 count=0;
	u8 connectsta=0;			//0,正在连接;1,连接成功;2,连接关闭; 
	u8 hbeaterrcnt=0;			//心跳错误计数器,连续5次心跳信号无应答,则重新连接
	
	char sensor_id_temp[]="0001";
	char OneNetServer[]="api.heclouds.com";
	char device_id_temp[]="your device id";//设备ID
	char API_VALUE_temp[]="your apikey";//APIKEY
	
	char send_buf[400]= {0};
	char text[100] = {0};
	char tmp[25] = {0};


  memset(send_buf, 0, 400); 
   /*准备JSON串*/
    sprintf(text,"{\"datastreams\":[{\"id\":\"%s\",\"datapoints\":[{\"value\":{\"lon\":%s,\"lat\":%s}}]}]}"
      ,sensor_id_temp,lon_str_end,lat_str_end);
  
    /*准备HTTP报头*/
    send_buf[0] = 0;
    strcat(send_buf,"POST /devices/");
    strcat(send_buf,device_id_temp);
    strcat(send_buf,"/datapoints HTTP/1.1\r\n");//注意加\r\n
    strcat(send_buf,"api-key:");
    strcat(send_buf,API_VALUE_temp);
    strcat(send_buf,"\r\n");
    strcat(send_buf,"Host:");
    strcat(send_buf,OneNetServer);
    strcat(send_buf,"\r\n");
    sprintf(tmp,"Content-Length:%d\r\n\r\n", strlen(text));//计算JSON长度
    strcat(send_buf,tmp);
    strcat(send_buf,text);	
		
	if(sim900a_send_cmd("AT+CIPSTART=\"TCP\",\"183.230.40.33\",\"80\"","OK",500))return;		//发起连接/////////IP
	while(1)
	{ 
		key=KEY_Scan(0);
		if(key==WKUP_PRES)//退出测试		 
		{  
			sim900a_send_cmd("AT+CIPCLOSE=1","CLOSE OK",500);	//关闭连接
			sim900a_send_cmd("AT+CIPSHUT","SHUT OK",500);		//关闭移动场景 
			break;												 
		}
		if((timex%20)==0)
		{
			LED0=!LED0;
			count++;
				if(count==3600)
				{
					count=0;
					printf("数据发送失败!!!\r\n");
					return;
				}	
			if(connectsta==2||hbeaterrcnt>8)//连接中断了,或者连续8次心跳没有正确发送成功,则重新连接
			{
				sim900a_send_cmd("AT+CIPCLOSE=1","CLOSE OK",500);	//关闭连接
				sim900a_send_cmd("AT+CIPSHUT","SHUT OK",500);		//关闭移动场景 
				sim900a_send_cmd("AT+CIPSTART=\"TCP\",\"183.230.40.33\",\"80\"","OK",500);//尝试重新连接//////////////////IP
				connectsta=0;	
 				hbeaterrcnt=0;
			}
		}
		if(connectsta==0&&(timex%200)==0)//连接还没建立的时候,每2秒查询一次CIPSTATUS.
		{
			sim900a_send_cmd("AT+CIPSTATUS","OK",500);	//查询连接状态
			if(strstr((const char*)USART3_RX_BUF,"CLOSED"))connectsta=2;
			if(strstr((const char*)USART3_RX_BUF,"CONNECT OK"))connectsta=1;
		}
		if(connectsta==1&&timex>=600)//连接正常的时候,每6秒发送一次心跳
		{
			timex=0;
			if(sim900a_send_cmd("AT+CIPSEND",">",200)==0)//发送数据
			{
				sim900a_send_cmd((u8*)send_buf,0,0);	//发送数据:0X00  ////////////////////////////////数据
				delay_ms(200);						//必须加延时
				sim900a_send_cmd((u8*)0X1A,0,0);	//CTRL+Z,结束数据发送,启动一次传输	
				delay_ms(1000);					
				break;
			}else sim900a_send_cmd((u8*)0X1B,0,0);	//ESC,取消发送 		
				
			hbeaterrcnt++; 
			printf("hbeaterrcnt:%d\r\n",hbeaterrcnt);//方便调试代码
		} 
		delay_ms(10);
		timex++; 
	} 
}

//sim900a GPRS测试
//用于测试TCP/UDP连接
//返回值:0,正常
//    其他,错误代码
u8 sim900a_gprs_test(void)
{
 	sim900a_send_cmd("AT+CIPCLOSE=1","CLOSE OK",100);	//关闭连接
	sim900a_send_cmd("AT+CIPSHUT","SHUT OK",100);		//关闭移动场景 
	if(sim900a_send_cmd("AT+CGCLASS=\"B\"","OK",1000))return 1;				//设置GPRS移动台类别为B,支持包交换和数据交换 
	if(sim900a_send_cmd("AT+CGDCONT=1,\"IP\",\"CMNET\"","OK",1000))return 2;//设置PDP上下文,互联网接协议,接入点等信息
	if(sim900a_send_cmd("AT+CGATT=1","OK",500))return 3;					//附着GPRS业务
	if(sim900a_send_cmd("AT+CIPCSGP=1,\"CMNET\"","OK",500))return 4;	 	//设置为GPRS连接模式
	if(sim900a_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//设置接收数据显示IP头(方便判断数据来源)
	printf("正在GPRS联网，发送数据!!!\r\n");
	 sim900a_tcpudp_test();
	printf("数据发送完毕!!!\r\n");
	printf("\r\n");
	return 0;
	
} 














