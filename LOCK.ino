#define BLINKER_WIFI 
#define BLINKER_MIOT_OUTLET
#include <Blinker.h> 
#include "stdio.h"                //这个头文件是用来使用sprint函数的

char auth[] = "4756863a1a05";//点灯秘钥
char ssid[] = "ABCS";//无线网名称
char pswd[] = "15091733023";//无线网密码

int freq = 50;                                    //设置频率
int channel = 8;                                  //设置通道
int resolution = 8;                               //计数位数
const int led = 13;

BlinkerButton Button1("btn-max");                 //注册按钮
BlinkerButton Button2("btn-min"); 
int servo_max=180,servo_min=0;

int calculatePWM(int degree)                      //PWM空占比计算
{ 
  const float deadZone =6.4;
  const float max = 32;
  if (degree < 0)
    degree = 0;
  if (degree > 180)
    degree = 180;
  return (int)(((max - deadZone) / 180) * degree + deadZone);
}

void OpenLock(){                                  //解锁操作
   ledcAttachPin(led, channel);                              //通道1与引脚led映射print(
          BLINKER_LOG("get button state: ", servo_max);             
          ledcWrite(channel, calculatePWM(180));                    //通过通道1输出PWM信号到角度180
         pinMode(2,OUTPUT); 
         digitalWrite(2,1);
         delay(5000);                                              //延时5秒
         ledcWrite(channel, calculatePWM(0));                     //通过通道1输出PWM信号到角度0
          digitalWrite(2,0);
         delay(5000);                                              //延时，防止复位前断开映射
         ledcDetachPin(13);                                        //取消映射
         Blinker.vibrate(500);                                   //返回震动
}

void Reposition(){                                //复位操作
     ledcAttachPin(led, channel);      
     BLINKER_LOG("get button state: ", servo_min);  
     ledcWrite(channel, calculatePWM(0));
     Blinker.vibrate(500);   
     delay(5000);
     ledcDetachPin(13);  
}

void miotPowerState(const String & state)           //小爱接入
{
    BLINKER_LOG("need set power state: ", state);

    if (state == BLINKER_CMD_ON) {
        BlinkerMIOT.powerState("on");               //返回状态
        BlinkerMIOT.print();
        OpenLock();
    }
    else if (state == BLINKER_CMD_OFF) {   
        BlinkerMIOT.powerState("off");              //返回状态
        BlinkerMIOT.print();
        Reposition();
    }
}


void button1_callback(const String & state)         //按钮1回调函数
{    
  OpenLock();
} 
void button2_callback(const String & state)         //按钮2回调函数
{    
 Reposition();
}

//以下为指纹相关



BlinkerButton Button_OneEnroll("OneEnroll");    //单次注册按钮
BlinkerButton Button_Delete("Delete");          //删除指纹按钮
BlinkerButton Button_Identify("Identify");      //搜索模式按钮
BlinkerButton Button_Empty("Empty");            //清空指纹按钮
BlinkerButton Button_MultEnroll("MultEnroll");  //连接注册按钮
BlinkerButton Button_Reset("Reset");            //复位模块按钮


char str[20];    //用于sprint函数的临时数组
int SearchID,EnrollID;    //搜索指纹的ID号和注册指纹的ID号
uint16_t ScanState = 0,PageID = 0;   //状态标志变量；输入ID号变量
byte PS_ReceiveBuffer[20];   //串口接收数据的临时缓冲数组

//休眠协议
byte PS_SleepBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x33,0x00,0x37};

//清空指纹协议
byte PS_EmptyBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x0D,0x00,0x11};

//获取图像协议
byte PS_GetImageBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x01,0x00,0x05};

//取消命令协议
byte PS_CancelBuffer[12] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x03,0x30,0x00,0x34};

//生成模块协议
byte PS_GetChar1Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x01,0x00,0x08};
byte PS_GetChar2Buffer[13] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x04,0x02,0x02,0x00,0x09};

//RGB颜色控制协议
byte PS_BlueLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x03,0x01,0x01,0x00,0x00,0x49};
byte PS_RedLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x02,0x04,0x04,0x02,0x00,0x50};
byte PS_GreenLEDBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x3C,0x02,0x02,0x02,0x02,0x00,0x4C};

//搜索指纹协议
byte PS_SearchMBBuffer[17] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x04,0x01,0x00,0x00,0xFF,0xFF,0x02,0x0C};

//自动注册指纹协议
byte PS_AutoEnrollBuffer[17] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x31,'\0','\0',0x04,0x00,0x16,'\0','\0'}; //PageID: bit 10:11，SUM: bit 15:16

//删除指纹协议
byte PS_DeleteBuffer[16] = {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x07,0x0C,'\0','\0',0x00,0x01,'\0','\0'}; //PageID: bit 10:11，SUM: bit 14:15

 
  //@brief   用在中断里面的延时函数
  // @param   ms：需要延时的毫秒数
 
void delay_ms(long int ms)
{
  for(int i=0;i<ms;i++)
  {
    delayMicroseconds(1000);
  }
}

 
  // @brief   串口发送函数
  // @param   len: 发送数组长度
  //@param   PS_Databuffer[]: 需要发送的功能协议数组，在上面已有定义
  
void FPM383C_SendData(int len,byte PS_Databuffer[])
{
  Serial.write(PS_Databuffer,len);
  Serial.flush();
}


 
  // @brief   串口接收函数
  // @param   Timeout：接收超时时间
 
void FPM383C_ReceiveData(uint16_t Timeout)
{
  byte i = 0;
  while(Serial.available() == 0 && (--Timeout))
  {
    delay(1);
  }
  while(Serial.available() > 0)
  {
    delay(2);
    PS_ReceiveBuffer[i++] = Serial.read();
    if(i > 15) break; 
  }
}

 
  // @brief   休眠函数，只有发送休眠后，模块的TOUCHOUT引脚才会变成低电平
  
void PS_Sleep()
{
  FPM383C_SendData(12,PS_SleepBuffer);
}


  // @brief   模块LED灯控制函数
  // @param   PS_ControlLEDBuffer[]：需要设置颜色的协议，一般定义在上面
 
void PS_ControlLED(byte PS_ControlLEDBuffer[])
{
  FPM383C_SendData(16,PS_ControlLEDBuffer);
}

 
   //@brief   模块任务取消操作函数，如发送了注册指纹命令，但是不想注册了，需要发送此函数
   //@param   None
   //@return  应答包第9位确认码或者无效值0xFF

byte PS_Cancel()
{
  FPM383C_SendData(12,PS_CancelBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


  // @brief   模块获取搜索指纹用的图像函数
  // @param   None
  // @return  应答包第9位确认码或者无效值0xFF

byte PS_GetImage()
{
  FPM383C_SendData(12,PS_GetImageBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

 
  // @brief   模块获取图像后生成特征，存储到缓冲区1
  //@param   None
  //@return  应答包第9位确认码或者无效值0xFF
  
byte PS_GetChar1()
{
  FPM383C_SendData(13,PS_GetChar1Buffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


  // @brief   生成特征，存储到缓冲区2
  //@param   None
  // @return  应答包第9位确认码或者无效值0xFF
  
byte PS_GetChar2()
{
  FPM383C_SendData(13,PS_GetChar2Buffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


  //@brief   搜索指纹模板函数
   //@param   None
  // @return  应答包第9位确认码或者无效值0xFF
  
byte PS_SearchMB()
{
  FPM383C_SendData(17,PS_SearchMBBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}



   //@brief   清空指纹模板函数
   //@param   None
   //@return  应答包第9位确认码或者无效值0xFF
 
byte PS_Empty()
{
  FPM383C_SendData(12,PS_EmptyBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}



  // @brief   自动注册指纹模板函数
  //@param   PageID：注册指纹的ID号，取值0 - 59
  //@return  应答包第9位确认码或者无效值0xFF
  
byte PS_AutoEnroll(uint16_t PageID)
{
  PS_AutoEnrollBuffer[10] = (PageID>>8);
  PS_AutoEnrollBuffer[11] = (PageID);
  PS_AutoEnrollBuffer[15] = (0x54+PS_AutoEnrollBuffer[10]+PS_AutoEnrollBuffer[11])>>8;
  PS_AutoEnrollBuffer[16] = (0x54+PS_AutoEnrollBuffer[10]+PS_AutoEnrollBuffer[11]);
  FPM383C_SendData(17,PS_AutoEnrollBuffer);
  FPM383C_ReceiveData(10000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}


  // @brief   删除指定指纹模板函数
   //@param   PageID：需要删除的指纹ID号，取值0 - 59
   //@return  应答包第9位确认码或者无效值0xFF
  
byte PS_Delete(uint16_t PageID)
{
  PS_DeleteBuffer[10] = (PageID>>8);
  PS_DeleteBuffer[11] = (PageID);
  PS_DeleteBuffer[14] = (0x15+PS_DeleteBuffer[10]+PS_DeleteBuffer[11])>>8;
  PS_DeleteBuffer[15] = (0x15+PS_DeleteBuffer[10]+PS_DeleteBuffer[11]);
  FPM383C_SendData(16,PS_DeleteBuffer);
  FPM383C_ReceiveData(2000);
  return PS_ReceiveBuffer[6] == 0x07 ? PS_ReceiveBuffer[9] : 0xFF;
}

  
   //@brief   二次封装自动注册指纹函数，实现注册成功闪烁两次绿灯，失败闪烁两次红灯
  //@param   PageID：注册指纹的ID号，取值0 - 59
  //@return  应答包第9位确认码或者无效值0xFF
  
//返回应答包的位9确认码。
byte PS_Enroll(uint16_t PageID)
{
  if(PS_AutoEnroll(PageID) == 0x00)
  {
    PS_ControlLED(PS_GreenLEDBuffer);
    return PS_ReceiveBuffer[9];
  }
  PS_ControlLED(PS_RedLEDBuffer);
  return 0xFF;
}


  // @brief   分步式命令搜索指纹函数
  //@param   None
  // @return  应答包第9位确认码或者无效值0xFF
  
byte PS_Identify()
{
  if(PS_GetImage() == 0x00)
  {
    if(PS_GetChar1() == 0x00)
    {
      if(PS_SearchMB() == 0x00)
      {
        if(PS_ReceiveBuffer[8] == 0x07 && PS_ReceiveBuffer[9] == 0x00)
        {
          PS_ControlLED(PS_GreenLEDBuffer);
          return PS_ReceiveBuffer[9];
        }
      }
    }
  }
  PS_ControlLED(PS_RedLEDBuffer);
  return 0xFF;
}


  // @brief   搜索指纹后的应答包校验，在此执行相应的功能，如开关继电器、开关灯等等功能
  // @param   ACK：各个功能函数返回的应答包
  
void SEARCH_ACK_CHECK(byte ACK)
{
	if(PS_ReceiveBuffer[6] == 0x07)
	{
		switch (ACK)
		{
			case 0x00:                          //指令正确
        SearchID = (int)((PS_ReceiveBuffer[10] << 8) + PS_ReceiveBuffer[11]);
        sprintf(str,"Now Search ID: %d",(int)SearchID);
        Blinker.notify(str);
        if(SearchID == 0)
        OpenLock();
				break;
		}
	}
  for(int i=0;i<20;i++) PS_ReceiveBuffer[i] = 0xFF;
}

 
  //@brief   注册指纹后返回的应答包校验
  // @param   ACK：注册指纹函数返回的应答包
void ENROLL_ACK_CHECK(byte ACK)
{
	if(PS_ReceiveBuffer[6] == 0x07)
	{
		switch (ACK)
		{
			case 0x00:                          //指令正确
        EnrollID = (int)((PS_AutoEnrollBuffer[10] << 8) + PS_AutoEnrollBuffer[11]);
        sprintf(str,"Now Enroll ID: %d",(int)EnrollID);
        Blinker.notify(str);
				break;
		}
	}
  for(int i=0;i<20;i++) PS_ReceiveBuffer[i] = 0xFF;
}


  
  // @brief   外部中断函数，触发中断后开启模块的LED蓝灯（代表正在扫描指纹），接着由搜索指纹函数修改成功（闪烁绿灯）或失败（闪烁红灯）

ICACHE_RAM_ATTR void InterruptFun()
{
  detachInterrupt(digitalPinToInterrupt(14));
  PS_ControlLED(PS_BlueLEDBuffer);
  delay_ms(10);
  ScanState |= 1<<4;
}

 
  //@brief   点灯科技APP里面的 “ 单次注册 “ 按键
  //@param   Unknown

void OneEnroll_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState |= 1<<2;
  Blinker.notify("OneEnroll Fingerprint");
}



  // @brief   点灯科技APP里面的 “ 删除指纹 “ 按键
  // @param   Unknown

  
void Delete_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState |= 1<<3;
  Blinker.notify("Delete Fingerprint");
}



  // @brief   点灯科技APP里面的 “ 搜索模式 “ 按键
   //@param   Unknown

  
void Identify_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState &= ~(1<<0);
  Blinker.notify("MultSearch Fingerprint");
}



  // @brief   点灯科技APP里面的 “ 连续注册 “ 按键
// @param   Unknown

void MultEnroll_callback(const String & state)
{
  Blinker.vibrate(500);
  ScanState |= 0x01;
  Blinker.notify("MultEnroll Fingerprint");
}



  // @brief   点灯科技APP里面的 “ 对话框 “ 
  // @param   Unknown

void DataRead(const String & data)
{
  PageID = data.toInt();
  ScanState |= 1<<1;
}



  //@brief   点灯科技APP里面的 “ 复位模块 “ 按键

void Reset_callback(const String & state)
{
  Blinker.vibrate(500);
  Blinker.notify("Reset Fingerprint");
  PS_Cancel();
  delay(500);
  PS_Sleep();
  attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
}


  // @brief   点灯科技APP里面的 “ 清空指纹 “ 按键
  // @param   Unknown

void Empty_callback(const String & state)
{
  PageID = 0;
  Blinker.vibrate(500);
  Blinker.notify("Empty Fingerprint");
  if(PS_Empty() == 0x00)
  {
    PS_ControlLED(PS_GreenLEDBuffer);
  }
  else
  {
    PS_ControlLED(PS_RedLEDBuffer);
  }
}





void setup()    
{   
  Serial.begin(9600);   
  ledcSetup(channel, freq, resolution); // 设置通道
  ledcAttachPin(led, channel);          // 将通道与对应的引脚连接
  BLINKER_DEBUG.stream(Serial);    
  Blinker.begin(auth, ssid, pswd); 
  Button1.attach(button1_callback);   
  Button2.attach(button2_callback);   
  BlinkerMIOT.attachPowerState(miotPowerState);  


  pinMode(2,OUTPUT);                                  //ESP8266，Builtin LED内置的灯引脚模式
  pinMode(14,INPUT);                                  //FPM383C的2脚TouchOUT引脚，用于外部中断
  Blinker.attachData(DataRead);
  Button_OneEnroll.attach(OneEnroll_callback);
  Button_Delete.attach(Delete_callback);
  Button_Identify.attach(Identify_callback);
  Button_Empty.attach(Empty_callback);
  Button_MultEnroll.attach(MultEnroll_callback);
  Button_Reset.attach(Reset_callback);

  delay_ms(200);                                      //用于FPM383C模块启动延时，不可去掉
  PS_Sleep();
  delay_ms(200);

  attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING); 

}

void loop() 
{   
Blinker.run();  
switch (ScanState)
  {
    //第一步
    case 0x10:    //搜索指纹模式
        SEARCH_ACK_CHECK(PS_Identify());
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第二步
    case 0x11:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        Blinker.notify("Please Enter ID First");
        PS_ControlLED(PS_RedLEDBuffer);
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;
    
    //第三步
    case 0x12:    //指纹中断提醒按下功能按键，执行完毕返回搜索指纹模式
        Blinker.notify("Please Press Enroll or Delete Key");
        PS_ControlLED(PS_RedLEDBuffer);
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第四步
    case 0x13:    //连续搜索指纹模式，每次搜索前都必须由APP发送指纹ID，由函数将ScanState bit1置位才进入下一次搜索，否则提醒输入指纹ID并返回搜索模式
        ENROLL_ACK_CHECK(PS_Enroll(PageID));
        delay(1000);
        PS_Sleep();
        ScanState = 0x01;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第五步
    case 0x14:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        ScanState = 0x11;   //返回第二步，提示输入指纹ID
    break;

    //第六步
    case 0x16:    //单次指纹注册模式，必须同时满足按下单次注册按键且已输入ID情况下才会执行
        ENROLL_ACK_CHECK(PS_Enroll(PageID));
        delay(1000);
        PS_Sleep();
        ScanState = 0x00;
        attachInterrupt(digitalPinToInterrupt(14),InterruptFun,RISING);
    break;

    //第七步
    case 0x08:    //指纹中断提醒输入指纹ID，执行完毕返回搜索指纹模式
        ScanState = 0x11;   //返回第二步，提示输入指纹ID
    break;

    //第八步
    case 0x0A:    //单独指纹删除模式
        if(PS_Delete(PageID) == 0x00)
        {
          Blinker.notify("Delete Success");
          PS_ControlLED(PS_GreenLEDBuffer);
        }
        ScanState = 0x00;
    break;
  }
  
}
