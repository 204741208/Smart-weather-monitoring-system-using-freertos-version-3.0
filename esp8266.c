#include <stdio.h>
#include "main.h"
#include "esp8266.h"
#include "lcd.h"
extern UART_HandleTypeDef huart6;
char Fetch[50];
char Rx_buff[400];
extern int temp;
int x,y,z,k,t;
char str[2];


char * commands[] = {
/*0*/ "AT\r\n",
/*1*/ "AT+RST\r\n",
/*2*/ "AT+CWMODE=3\r\n",
/*3*/ "AT+CWJAP=\"Hari B firstfloor_5G\",\"hari@lig180\"\r\n",
/*4*/ "AT+CIFSR\r\n",
/*5*/ "AT+CIPMUX=0\r\n",
/*6*/ "AT+CIPMODE=0\r\n",
/*7*/ "AT+CIPSTATUS\r\n",
/*8*/ "AT+CIPSTART=\"TCP\",\"142.93.218.33\",80\r\n",
/*9*/ "AT+CIPSEND=40\r\n",
/*10*/ "AT+CIPCLOSE\r\n"
};


void WiFi_Init()
{
At_Cmd();
}
void At_Cmd(void)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT\r\n", 6, 100);
while(Check_WiFi_Response()!=0)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT\r\n", 6, 100);
}
At_Reset_Cmd();
}
void At_Reset_Cmd(void)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+RST\r\n", 10, 100);
HAL_Delay(100);
/* while(Check_WiFi_Response()!=0)
{
USART6_TX_Outstring("AT+RST\r\n");
}*/
At_SetWifi_Mode();
}
void At_SetWifi_Mode(void)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CWMODE=3\r\n", 15, 100);
while(Check_WiFi_Response()!=0)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CWMODE=3\r\n", 15, 100);
}
At_Connect_To_Wifi();
}
void At_Connect_To_Wifi(void)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CWJAP=\"Hari B firstfloor_5G\",\"hari@lig180\"\r\n", 45, 800);
HAL_Delay(500);
while(Check_WiFi_Response()!=0)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CWJAP=\"Hari B firstfloor_5G\",\"hari@lig180\"\r\n", 45, 800);
HAL_Delay(1000);
}
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIFSR\r\n", 12, 100);
while(Check_WiFi_Response()!=0)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIFSR\r\n", 12, 100);
}
At_Mux_Mode();
}
void At_Mux_Mode(void)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPMUX=0\r\n", 15, 100);
while(Check_WiFi_Response()!=0)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPMUX=0\r\n", 15, 100);
}
At_Mode_Selection();
}
void At_Mode_Selection(void)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPMODE=0\r\n", 16, 100);
while(Check_WiFi_Response()!=0)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPMODE=0\r\n", 16, 100);
}
}

void WiFi_Transmit(void)
{
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPSTATUS\r\n", 16, 1000);
x=Check_WiFi_Response();

HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPSTART=\"TCP\",\"142.93.218.33\",80\r\n", 45, 1000);

y=Check_WiFi_Response();

sprintf(Fetch,"GET /page.php?temp=%d&hum=81&dev=45\r\n\r\n",temp);
HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPSEND=40\r\n", 17, 1000);
z=Check_WiFi_Response();

if(x||y||z)
{
RM_LCD_Goto(10,1);
RM_LCD_Write_Str(11,1, "ERROR");
k=1;
}
else
{
RM_LCD_Goto(10,1);
RM_LCD_Write_Str(11,1, "OK   ");
k=0;
}

HAL_UART_Transmit(&huart6, (uint8_t *)Fetch, 43, 1000);
HAL_Delay(500);

HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CIPCLOSE\r\n", 15, 1000);
HAL_Delay(500);
}

int Check_WiFi_Response(void)
{
volatile int i;
memset((uint8_t *)Rx_buff, 0, sizeof(Rx_buff));
HAL_UART_Receive(&huart6, (uint8_t *)Rx_buff, 100, 1000);
for(i=0;i<200;i++)
{
if(Rx_buff[i]=='K' && Rx_buff[i-1]=='O')
{
return 0;
}
}
return 1;
}

void display(int a)
{
char m;
m=(a/10)+48;
RM_LCD_Put_Char(m);
m=(a%10)+48;
RM_LCD_Put_Char(m);

}
void mylcd(int a)
{
if(a==0)
RM_LCD_Write_CMD(0XC6);
else if(a==1)
RM_LCD_Write_CMD(0XC3);
else if(a==2)
RM_LCD_Write_CMD(0XC0);
else if(a==3)
RM_LCD_Write_CMD(0XC0);
else if(a==4)
RM_LCD_Write_CMD(0XC3);
else if(a==5)
RM_LCD_Write_CMD(0XC6);
else if(a==6)
RM_LCD_Write_CMD(0XC9);
else
RM_LCD_Write_CMD(0XCB);

}
void task(int a,uint8_t *A)
{
int SS=0;
static int c=0;
while(1)
{
if(SS>=0 &&( (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10))))
{

if(!( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)))
{
while(!( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)))
{
;
}
HAL_Delay(10);
SS+=1;
mylcd(c);
display(SS);
if(SS>a)
{
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET);

mylcd(c);
SS=0;
display(SS);
}
}
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET);
if(! (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)))
{
while(!( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)))
{
;
}
HAL_Delay(10);
SS-=1;
mylcd(c);
display(SS);
if(SS<0)
{
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);

mylcd(c);
SS=a;
display(SS);
}
}
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET);
}


if(!(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)))
{
c++;
*A=SS;
break;
}

}

}
