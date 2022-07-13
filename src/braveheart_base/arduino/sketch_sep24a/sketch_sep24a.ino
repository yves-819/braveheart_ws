#include <MotorProtocoltest.h>
#include <Encoder.h>

MotorProtocol protocol = MotorProtocol();

//小灯引脚
uint8_t ledPin = PC13;
bool ledState = false;


//左速度控制引脚定义
const uint8_t Left_encode_A = PB10;
const uint8_t Left_encode_B = PB11;
const uint8_t PWMPinL = PB1;   //tim3-4ch
const uint8_t OUTL1 = PB0;
const uint8_t OUTL2 = PA7;
uint16_t fadeValueL = 0;      //占空比

const uint8_t PWMPinL2 = PA1;  //tim2-2ch
uint16_t fadeValueL2 = 0; 
//Encoder是一个类 定义了一个knobLeft对象
Encoder knobLeft(Left_encode_A, Left_encode_B);     


//右速度控制引脚定义
const uint8_t Right_encode_A = PB13;
const uint8_t Right_encode_B = PB12;  //左右电机刚好相反
const uint8_t PWMPinR = PA8;   //tim1-1ch
const uint8_t OUTR1 = PB14;
const uint8_t OUTR2 = PB15;           //左右电机刚好相反
uint16_t fadeValueR = 0;      //占空比

const uint8_t PWMPinR2 = PB6; //tim4-1ch
uint16_t fadeValueR2 = 0; 
Encoder knobRight(Right_encode_A, Right_encode_B);


int left_set, right_set,left_set2, right_set2;      //串口读取到的四个pwm
int last_left_set, last_right_set;
long measurementLeft = 0, measurementRight = 0;               //读取到的编码器的计数值
uint8_t count;

/////////////////////////////////////读串口设置////////////////////////////////////////////////
//收数据缓存区
char *rxBuffer = new char[protocol.getSumrxSize() * 2];
//串口收缓存区长度
size_t res_len = 0;
//接收是否可以进行正常的读取
bool canRead = false;

/////////////////////////////////////读串口设置完毕////////////////////////////////////////////

//小灯闪烁
void blink()
{
  ledState = !ledState;
  if(ledState)
  {
    digitalWrite(ledPin, LOW);
  }
  else
  {
    digitalWrite(ledPin, HIGH);
  }
}
//




void setup() 
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  
  pinMode(OUTL1, OUTPUT);
  pinMode(OUTL2, OUTPUT);
  pinMode(PWMPinL, PWM);pinMode(PWMPinL2, PWM);
  
  pinMode(OUTR1, OUTPUT);
  pinMode(OUTR2, OUTPUT);
  pinMode(PWMPinR, PWM);pinMode(PWMPinR2, PWM);
  
  blink();
  count = 0;
}

void loop()
{
  //////////////////////////////////////写串口10hz/////////////////////////
  if (count >= 10)
  { 
    count = 0;
    //Encoder类里面的readAnReset函数：返回编码器数据 并且清0(从新计数)
    measurementLeft = knobLeft.readAndReset();
    measurementRight = knobRight.readAndReset();

    //将读取到的编码器数值写到协议里面去 
    protocol.setLeft(measurementLeft); 
    protocol.setRight(measurementRight);
    //对传入的数据，进行判断，设置，改写成符合协议的数据
    if (protocol.encode())
    {
      int res_tx = 0;//服务于串口，
	//获取可用于在串行缓冲区中写入而不阻塞写入操作的字节数 与协议规定数据进行比较
      if (Serial.availableForWrite() >= protocol.getSumSize() * 2)
      {
        res_tx = Serial.write(protocol.txBuffer, protocol.getSumSize()); //write函数不一定运行成功，进行判断。成功才返回返回的是byte
	//判断是否发送成功 如果没有就继续发送，直到成功        
	while (res_tx != protocol.getSumSize())  
        {
          delay(1000);
          res_tx = Serial.write(protocol.txBuffer, protocol.getSumSize());
        }
        //Serial.println();
      }
    }
  }
  //////////////////////////////////////写串口完毕//////////////////////
  /////////////////////////////////////速度控制100hz/////////////////////
  
  //电机不能立马转向，两个if缓冲下
  if ((last_left_set * left_set ) < 0)
  {
    left_set = 0;
  }
  if ((last_right_set * right_set ) < 0)
  {
    right_set = 0;
  }
  last_left_set = left_set;
  last_right_set = right_set;
  if(left_set<0) {fadeValueL = abs(left_set) * 2;}                                                                    //0.885
  else {fadeValueL = abs(left_set) * 2;}//pwm是16位 协议带符号位是15位                                                    //此处改比例0.893

  fadeValueR = abs(right_set) * 2;

  fadeValueL2 = abs(left_set2) * 2;
  fadeValueR2 = abs(right_set2) * 2;

  //方向   同时左右拐
  if (left_set >= 0)  //往左边拐弯
  {
    digitalWrite(OUTL1, HIGH);
    digitalWrite(OUTL2, LOW);  //10为正转
  }
  else
  {
    digitalWrite(OUTL1, LOW);
    digitalWrite(OUTL2, HIGH);
  }
  if (right_set >= 0)
  {
    digitalWrite(OUTR1, HIGH);
    digitalWrite(OUTR2, LOW);
  }
  else
  {
    digitalWrite(OUTR1, LOW);
    digitalWrite(OUTR2, HIGH);
  }
  pwmWrite(PWMPinL, fadeValueL);
  pwmWrite(PWMPinR, fadeValueR);
  pwmWrite(PWMPinL2, fadeValueL2);
  pwmWrite(PWMPinR2, fadeValueR2);
  /////////////////////////////////////速度控制结束100hz/////////////////////////
  //////////////////////////////////////读串口(100hz)//////////////////
   //获取可用于从串行端口读取的字节数,检测串口数据是否符合协议数据
  if (Serial.available() >= protocol.getSumrxSize())
  {
    //把数据读到缓存区
    res_len = Serial.readBytes(rxBuffer, protocol.getSumrxSize());
    if (res_len >= protocol.getSumrxSize())
    {

      for (size_t i = 0; i < protocol.getSumrxSize(); i ++)
      {
        canRead = protocol.decode((uint8_t)rxBuffer[i]);
        if (canRead)
        {
          //读取解码后的具体数据
          left_set = protocol.getLeft();
          right_set = protocol.getRight();
          left_set2 = protocol.getLeft2();
          right_set2 = protocol.getRight2();
          blink();
        }
      }
    }
  }
  //////////////////////////////////////读串口结束100hz//////////////////////

  delay(10);
  count++;
}
