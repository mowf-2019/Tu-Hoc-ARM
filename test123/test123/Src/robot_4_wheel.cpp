#include "robot_4_wheel.h"

extern TIM_HandleTypeDef htim4; //pwm
extern TIM_HandleTypeDef htim6;  //timer 1ms
extern TIM_HandleTypeDef htim2;  // encoder
extern UART_HandleTypeDef huart2;// mpu
extern UART_HandleTypeDef huart3;// laban 
extern DMA_HandleTypeDef husart2;
extern DMA_HandleTypeDef husart3;
extern PS2_stat ps2;
robot_4_wheel robot;
/////////////////////////////////////////////////////
const float PI=3.14;
int16_t maxSpeed=1000, minSpeed=10, stopSpeed=2;
float T=0;
float kp=2,ki=0.25,E,E1;
bool robotIsRun=false,robotIsRotate=false;
/////////////////////////////////////////////////////
int chay=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/////////////////////////////////////////////////////
  if(htim->Instance==htim6.Instance)
     {
       run();
       chay++;
       
       ///////////////////////////////////////////////////////
       if(chay>19)
          {
           chay=0;
           //xxxxxxxxxxxxxxxxxxxxxx//
           if(robot.rstMpu ==0)
              {
                robot.send_mpu ='a';
                HAL_UART_Transmit_IT(&huart2,(uint8_t*)robot.send_mpu,1);
                robot.angleMpu=robot.mpu[0]<<8|robot.mpu[1];          
              }
           else
              {
               robot.send_mpu ='z';
               HAL_UART_Transmit_IT(&huart2,(uint8_t*)robot.send_mpu,1);
               robot.angleMpu = robot.mpu[0]<<8|robot.mpu[1];
              }
           //xxxxxxxxxxxxxxxxxxxxx//
            ps2_read(robot.Data);
           if(robot.Data[0]!=1)
               {
                 HAL_UART_DMAStop(&huart3);
                 for(int i=0;i<1000;i++);
                 HAL_UART_Receive_DMA(&huart1,(uint8_t*)robot.Data,7);//ps2
                }
            else HAL_UART_Receive_DMA(&huart1,(uint8_t*)robot.Data,7);//ps2
          }
       }
}
void setUpDe()
{
  HAL_TIM_Base_Start_IT(&htim6);//runHAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);//encoder...HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1); //A
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1); //A
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2); //B
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);//C
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);//D
  HAL_UART_Receive_DMA(&huart2,(uint8_t*) robot.mpu,10);//la ban
  HAL_UART_Receive_DMA(&huart1,(uint8_t*)robot.Data,7); //ps2
  robot.rstMpu=0;
  robotLock();  
  resetEn();
}
//xxxxxxxxxx encoder xxxxxxxxxxxxx//
//int32_t sNow=0; // bien cuc bo
void resetEn()
{
  TIM2->CNT=0;
  robot.sNow=0;
}
int32_t readEn()
{
  int pulse=__HAL_TIM_GET_COUNTER(&htim2);
      robot.sNow+=fabs((float)pulse);
      TIM2->CNT=0;
   return robot.sNow;
}
void pwm(void){          //ham bam xung
	if (robot.va > 0) A_DUONG;
	else A_AM;
	if (robot.vb > 0) B_DUONG;
	else B_AM;
	if (robot.vc > 0) C_DUONG;
	else C_AM;
	if (robot.vd > 0) D_DUONG;
	else D_AM;

	 if(fabs(robot.va)>=maxSpeed) robot._va=maxSpeed;
        else robot._va = robot.va;
        
        if(fabs(robot.vb)>=maxSpeed) robot._vb=maxSpeed;
	else robot._vb = robot.vb;
        
        if(fabs(robot.vc)>=maxSpeed) robot._vc=maxSpeed;
	else robot._vc = robot.vc;
        
        if(fabs(robot.vd)>=maxSpeed) robot._vd=maxSpeed;
	else robot._vd = robot.vd;

	PWM_A((int)fabs(robot._va));
	PWM_B((int)fabs(robot._vb));
	PWM_C((int)fabs(robot._vc));
	PWM_D((int)fabs(robot._vd));
}
void run(){   //ham chay de dat trong timer
//gia toc//
	if (robot.speedSet != robot.speedNow)
        {
		if (robot.speedSet > robot.speedNow)
                    {
                      robot.speedNow=Acc(robot.speedNow,robot.acc);
                    }
		if (robot.speedSet < robot.speedNow)
                    {
                      robot.speedNow=Dcc(robot.speedNow,robot.dcc);
                    }
                if(robot.speedSet=robot.speedNow) T=0;
	}
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxx//
	//a = angleRun-angleMpu;  //goc chay
        angleFix(robot.angleSet,robot.limitSpeed);
        if(robotIsRun==false&&robotIsRotate==false)
        {
                robot.speedSet=0;
                robot.vOut=2;
        }
        else if(robotIsRun==false) robot.speedSet=0;
        else if(robotIsRotate==false) robot.vOut=0;
        else;
	robot.va = robot.speedNow*cos((robot.angleRobot + 1350 - robot.angleMpu)*PI/1800) + robot.vOut;
	robot.vb = robot.speedNow*cos((robot.angleRobot + 450 - robot.angleMpu)*PI/1800) + robot.vOut;
	robot.vc = robot.speedNow*cos((robot.angleRobot - 450 - robot.angleMpu)*PI/1800) + robot.vOut;
	robot.vd = robot.speedNow*cos((robot.angleRobot - 1350 - robot.angleMpu)*PI/1800) + robot.vOut;
	pwm();
}

float Acc(float _speedNow, float _acc)
      {
        float vo=_acc*T;
        T+=0.001; //timer 1ms
        _speedNow+=vo;
	return _speedNow;
      }
float Dcc(float _speedNow,float _dcc)
      {
        if(_dcc<0) return _speedNow+=_dcc;
        else return _speedNow-=1;
      }
void angleFix(int16_t _angleWant,float _limitSpeed)
{
        
        if(robot.deltaAngle!=_angleWant)
        {
                if(robot.deltaAngle<_angleWant) robot.deltaAngle+=2;
                else if(robot.deltaAngle>_angleWant) robot.deltaAngle-=2;
                else robot.deltaAngle=_angleWant;
        }
	E = robot.deltaAngle - robot.angleMpu; 
        E1 +=E*0.001; //timer 1ms
	if (E>-2 && E<2) E = 0;
	robot.vOut = kp*E+ki*E1;

	if (robot.vOut > _limitSpeed ) robot.vOut = _limitSpeed;
	if (robot.vOut < _limitSpeed) robot.vOut = _limitSpeed;

	robot.vOut =(int)robot.vOut;
}
void robotRun(float vt,int16_t ax,float _acc,float _dcc)
{  
        robotIsRun=true;
        robotIsRotate=false;
	robot.speedSet = vt;
	robot.angleRobot = ax;
        robot.limitSpeed=300;
        robot.acc=_acc;
        robot.dcc=_dcc;
}
void robotRotateAngle( float _limitSpeed,int16_t _aWant){   //ham xoay goc(goc quay, gia toc quay)
        robotIsRun=false;
        robotIsRotate=true;
        angleFix(_aWant,_limitSpeed);
}
void robotRunAndRotate(float _speedSet,float _limitSpeed, int16_t _aRobot,int16_t _aWant, float _acc,float _dcc){   //ham chay xoay(van toc chay, goc chay, goc quay,gia toc quay(nho nhat 1), gia toc chay)
        robotIsRun=true;
        robotIsRotate=true;
        robotRun(_speedSet,_aRobot,_acc,_dcc);
        robotRotateAngle(_limitSpeed,_aWant);
}
void robotLock(void){   //ham khoa
	robotIsRun=false;
        robotIsRotate=false;
        robot.speedNow=0;
}
void robotStop(void){
  robotIsRun=false;
  robotIsRotate=false;
}
 //ham chay(van toc chay, goc chay, s mong muon, s giam toc, encoder(s_now),gia toc chay)
//neu s_want=0 thi chay toi s_set thi giam toc nhung ko dung.de ket hop voi cam bien,...
/*void robotRunPosition(float v, int ax, int32_t s_want, int32_t s_set, int32_t s_now, float _value,float _Dcc){  
	if (s_now < s_set) robotRun(v, ax, _value,_Dcc);
	else
        {
          if(s_now<s_want)
            robotRun((200), ax, _value,_Dcc); 
        }
        if(s_want!=0){
	  if (s_now >= s_want) robotLock();
        }
}
//ham chay xoay vi tri(van toc chay, goc chay, s mong muon, s giam toc, encoder(luon la s_now), gia toc xoay, gia toc chay)
// s_want=0 cung tuong tu nhu ham tren
void robotRun_R_Position(float v,int _limitSpeed,int ax, int16_t a_set, int32_t s_want, int32_t s_set,int32_t s_now, int n, float _value, float _Dcc){    
  if (s_now < s_set) robotRunAndRotate(v,_limitSpeed,ax,a_set,n,_value,_Dcc);
	else robotRunAndRotate(200,_limitSpeed,ax,a_set,n,_value,_Dcc);
	if(s_want!=0){
	  if (s_now >= s_want) robotLock();
        }
}*/
