//#include "Dac.h"
//#include "Max6675.h"
//#include "Modbus.h"
#include "Pid.h"
//#include "Flash.h"

//#include "Process.h"

#define MIN_ERROR_VALUE 2
#define MAX_ERROR_VALUE 60

//volatile uint8_t  get_now_temp_flag   = 0;
//volatile uint8_t  pwm_con_time_flag = 0;
//volatile uint8_t  pid_tune_flag = 1;//初始为0 即pid阶段 采用默认的值    1 为自整定过程
//volatile uint8_t  enable_pid_sec_flag = 0;
//volatile uint8_t  pid_self_sec_flag   = 0;
volatile uint8_t  pid_self_first_status_flag = 0;
volatile uint8_t  once_add_1_flag     = 0;
volatile uint8_t  enable_calc_min_max_flag = 0;
volatile uint8_t  Pid_Setok = 0;

uint16_t time0_temp2=0;
int32_t zero_across_counter = 0;
int64_t pid_self_time_sec = 0;

int32_t cool_ack_counter    = 0;
int32_t hot_ack_counter     = 0;
int32_t k_pid_self_counter = 0;




float  Proportion  = 0.0;           //  比例常数 Proportional Const
float  Integral    = 0.0;           //  积分常数 Integral Const        
float  Derivative  = 0.0;           //  微分常数 Derivative Const

float  LastError   = 0.0;           //  Error[-1]
float  PrevError   = 0.0;           //  Error[-2]
float  SumError    = 0.0;           //  Sums of Errors
float  dError      = 0.0;
float  Error       = 0.0;
//pid
float           SV_value           = 50.0; //设定温度值
float           PV_value           = 0.0;  //用于参与计算的当前温度值
float           KV_value           = 0.0;  //温度差值
//float           PV_oldvalue        = 0.0;  //用于参与计算的当前温度值
//volatile float  P_value            = 0.0;  //比例带 比如56.3代表56.3%  0.0--200.0
//int             I_value            = 0;  //积分时间  秒  0-3600
//int             D_value            = 0;   //微分时间  秒  0-900
int32_t  pid_result = 0;                //DAC输出

float max_temp  = 0.0 ;  //初始温度等于0
float min_temp  = 100.0 ;//初始温度等于100
float sum_temp  = 0.0 ;  //初始温度等于0
float aver_temp	= 0.0 ;

float  KC = 1.0;  //临界比例系数  初始默认的值
int32_t    TC = 40;   //振荡周期      初始默认的值

float T_Hight = 0.0;
float T_LOW   = 100.0; //温度
int64_t TIME_Hight = 0;
int64_t TIME_LOW   = 0;	//具体的秒
float pid_self_calc_buffer[4]={0.0,0.0,0.0,0.0};


//0 算出 临界增益 KC 及 振荡周期 TC
  // KC = (4*d)/(3.14*A)  ---> d = 5(输出幅值) ; A = 记录的温度最高值与最低值的差值的0.5倍 即：(T_Hight - T_LOW)*0.5
  // KC = (4*5)/(3.14*((T_Hight - T_LOW)*0.5)) = 40/3.14/(T_Hight - T_LOW) =  12.7/(T_Hight - T_LOW)
  // KC = 12.7/(T_Hight - T_LOW)
  // TC = 2 * (TIME_Hight - TIME_LOW) ---> 2 * ( 高点温度对应时间 - 低点温度对应时间 )
  // TC = 2 * (TIME_Hight - TIME_LOW)
  //1 算出 具体的比例系数 积分秒数 微分秒数
  //Proportion = 0.6*KC
  //I_value    = 0.5*TC
  //D_value    = 0.125*TC
  //2 算出具体的 比例带 积分系数 微分系数 
  //P_value     = (1/Proportion)*100
  //Integral	  = Proportion/I_value = (0.6*KC)/(0.5*TC)
  //Derivative  = Proportion*D_value = (0.6*KC)*(0.125*TC)  
  //3显示用的3个变量的值
  //P_value     = (1/Proportion)*100  百分比
  //I_value     = 0.5*TC				秒 
  //D_value     = 0.125*TC			秒 
  //4pid计算用的3个变量的值
  //Proportion  = 0.6*KC
  //Integral	  = Proportion/I_value = (0.6*KC)/(0.5*TC)
  //Derivative  = Proportion*D_value = (0.6*KC)*(0.125*TC)  
  
  //KC = 21.4;//test 
  //TC = 471;//test 
void pid_pro(void)//pid 控制输出 ppppppppppppppppppppppppppppp
{
  int temp_pid;
  Error = SV_value - PV_value;                 // 偏差
  if(( Error < max_value_error  ) && ( Error > (min_value_error)  ))//只有在一定的温差范围内才pid计算
  {    
    SumError += Error;
    dError    = LastError - PrevError;   // 当前微分
    PrevError = LastError;
    LastError = Error;
    temp_pid  =  (int)((Proportion * Error) + (Integral * SumError) + (Derivative * dError));    
    //temp_pid  =  (int)(temp_pid * 0.5) ;//输出比例控制
  }
  else//只有开关作用
  {
    if( Error >= max_value_error )//远大于当前温度，加热
    {
      temp_pid = 100;
      //temp_pid = 80;
    }
    else if( Error <= (min_value_error) )//远小于当前温度，不加热
    {
      temp_pid = 0;
    }
  }
  if( temp_pid < 0 )
  {
    temp_pid = 0;
  }
  else if( temp_pid > 100 )
  {
    temp_pid = 100;
  } 
  Dac_Out(40 + (uint8_t)(temp_pid * 1.97) );//根据上一步的结果控制输出 
}


//PID自整定
void Pid_Sinceset(void)
{
    pid_self_time_sec++;
    if(pid_self_time_sec > (3600*3)) // 如果总的自整定时间大于了3/5=0.6个小时，则说明整定失败
    {
      pid_self_time_sec = 0;			
//      pid_tune_flag = 0;//那么将自动退出自整定过程 同时采用默认值  进入pid阶段
      KC = 1.0;//临界比例系数  初始默认的值
      TC = 40; //振荡周期    初始默认的值 
      memory[PIDSET_ADR]=0;     //关闭PID自整定
      Pid_Setok=0;   //PID自整定失败
    }
    if(( pid_self_first_status_flag == 1) || ( pid_self_first_status_flag == 0))//0 设定温度 低于 当前温度  //1设定温度 高于 或者 等于  当前温度  启动加热
    {
      //基本on/off控制
      if( SV_value >= PV_value )//启动加热
      {
        cool_ack_counter = 0;
        hot_ack_counter++;
        if(hot_ack_counter > 3)//连续3次都是一样的结果 说明确定 SV_value >= PV_value
        {
          Dac_Out(237);//全速加热
          if(once_add_1_flag == 0)
          {
            once_add_1_flag = 1;
            zero_across_counter++; 
            if(zero_across_counter == 3 )
            {
              TIME_LOW = pid_self_time_sec - 3;//此时的时间不是最低温度对应的时间
            }
          }
        }
      }
      else//当前温度 大于 设定温度 停止加热
      { 
        hot_ack_counter = 0;
        cool_ack_counter++;
        if(cool_ack_counter > 3)
        {
          Dac_Out(40);//不加热
          if(once_add_1_flag == 1)
          {
            once_add_1_flag = 0;
            zero_across_counter++;
            if(zero_across_counter == 3 )
            {
              TIME_LOW = pid_self_time_sec - 3;//此时的时间不是最低温度对应的时间
            }
          }
        }
      }
      
      //最低温度 出现在 zero_across_counter = 3 的阶段
      //最高温度 出现在 zero_across_counter = 4 的阶段
      if((zero_across_counter == 3 ) || (zero_across_counter == 4 ))
      {				
        pid_self_calc_buffer[k_pid_self_counter] = PV_value;
        k_pid_self_counter++;
        if(k_pid_self_counter > 3)//0--3 共4个元素
        {
          k_pid_self_counter = 0;
          enable_calc_min_max_flag = 1;
        }
        if(enable_calc_min_max_flag == 1)//只要有4个值，就可以计算了 后面来的值覆盖了前面的值 
        {
          //去掉最小值 最大值 取剩下2个值的平均值 
          sum_temp = 0.0;  //先清0
          min_temp = 1024.0;
          max_temp = 0.0;
          
          for(uint8_t k_max_min = 0; k_max_min < 4; k_max_min++ )
          {						
            if(pid_self_calc_buffer[k_max_min] <= min_temp)
            {
              min_temp = pid_self_calc_buffer[k_max_min];
            }
            if(pid_self_calc_buffer[k_max_min] >= max_temp)
            {
              max_temp = pid_self_calc_buffer[k_max_min];
            }						
            sum_temp = (sum_temp + pid_self_calc_buffer[k_max_min]);
          }
          sum_temp =  sum_temp - min_temp - max_temp ;
          
          
          //pid_self_first_status_flag = 1 时 最低温度出现在3阶段
          //pid_self_first_status_flag = 0 时 最低温度出现在4阶段
          if(pid_self_first_status_flag == 1)
          {
            if(zero_across_counter == 3 )//最低温度
            {
              aver_temp = (sum_temp/2.0);					
              if( aver_temp <= T_LOW )
              {
                T_LOW = aver_temp;
              }				
            }
            else if(zero_across_counter == 4 )//最高温度
            {
              aver_temp = (sum_temp/2.0);
              if( aver_temp >= T_Hight )
              {
                T_Hight = aver_temp;
              }
            }
          }
          else if(pid_self_first_status_flag == 0)
          {
            if(zero_across_counter == 4 )//最低温度
            {
              aver_temp = (sum_temp/2.0);					
              if( aver_temp <= T_LOW )
              {
                T_LOW = aver_temp;
              }				
            }
            else if(zero_across_counter == 3 )//最高温度
            {
              aver_temp = (sum_temp/2.0);
              if( aver_temp >= T_Hight )
              {
                T_Hight = aver_temp;
              }
            }
          }
        }
      }
      else if(zero_across_counter == 5 )//4次过0 则说明出现了振荡 整定成功
      {
        zero_across_counter = 0;				
        //        pid_tune_flag = 0;//进入pid阶段
        //pid_tune_flag = 1;//test
        TIME_Hight = pid_self_time_sec - 3;//此时的时间不是最高温度对应的时间
        //计算 T_Hight T_LOW TIME_Hight TIME_LOW 这4个值 
        //根据以上4个值  KC 与 TC 的值便会计算出来    
        KC = 12.7/(T_Hight - T_LOW);
        KC = 5.0 * KC;//因为是0.2s一次 所以扩大5倍
        TC = 1 * (TIME_Hight - TIME_LOW);//如果记录了 最低温度 与 最高温度对应的时间 那么沿用这个公式：TC = 2 * (TIME_Hight - TIME_LOW);
       
        memory[PIDSET_ADR]=0;     //关闭PID自整定
        Pid_Setok=1;   //PID自整定成功
      }
    }
}

void PidParameter_Sinceset(void)//PID自整定参数
{
  //记录此刻的状态 即设定温度是否 高于或等于 当前温度 
  PV_value = read_max6675_temper();
  if( SV_value >= PV_value )//设定温度 高于 或者 等于  当前温度  启动加热
  {
    pid_self_first_status_flag = 1;
    once_add_1_flag = 0;
  }
  else//设定温度 低于 当前温度
  {
    pid_self_first_status_flag = 0;
    once_add_1_flag = 1;
  } 
  zero_across_counter = 0;
  pid_self_time_sec = 0;      
  k_pid_self_counter = 0;
  enable_calc_min_max_flag = 0;
  max_temp = 0.0 ;  //初始温度等于0
  min_temp = 1024.0 ;//初始温度等于1024
  sum_temp = 0.0 ;  //初始温度等于0
  aver_temp = 0.0 ;
  T_Hight  = 0.0;
  T_LOW    = 1024.0; //温度
  TIME_Hight = 0;
  TIME_LOW   = 0;	//具体的0.2s 
}

void PidParameter_pro(void)//PID参数
{
  
 KC=*((float *)(&memory[PIDKC_ADR]));
 TC=*((int *)(&memory[PIDTC_ADR]));
  if(KC > 1666.0 )
  {
    KC = 1666.0;//对应 比例带为 0.1%
  }
  else if(KC < 0.5 )
  {
    KC = 0.5;//对应 比例带为 200.0%
  }
  if(TC > 7200 )
  {
    TC = 7200;
  }
  else if(TC < 8 )
  {
    TC = 8;
  }   
  Proportion  = 0.6*KC;
  Integral	= (0.6*KC)/(0.5*TC);
  Derivative  = (0.6*KC)*(0.125*TC); 
}


void Preheat(void)// 预热
{
  uint16_t preheat_cnt=0;
  uint8_t  preheat_out=40;
  while(preheat_cnt<240) //2分
  {
    preheat_cnt++;
    if(preheat_out<237)
      preheat_out++;
    else
      preheat_out=237;
     Dac_Out(preheat_out);
    if(PV_value>=(SV_value-50))
      break;
    HAL_Delay(500);
  }
}

uint16_t PID_Algorithm(float setValue, float feedbackValue)
{
	int16_t temp_pid;
	float errorValue;
	uint16_t duty_cycle = 0;	
	errorValue = setValue - feedbackValue;                 // 偏差
	
  if(errorValue > MIN_ERROR_VALUE && errorValue < MAX_ERROR_VALUE )//只有在一定的温差范围内才pid计算
  {    
    SumError += Error;
    dError    = LastError - PrevError;   // 当前微分
    PrevError = LastError;
    LastError = Error;
    temp_pid  =  (int)((Proportion * Error) + (Integral * SumError) + (Derivative * dError));    
    //temp_pid  =  (int)(temp_pid * 0.5) ;//输出比例控制
  }
  else if(errorValue >= MAX_ERROR_VALUE)//只有开关作用
  {
		
  }
	else
	{
		
	}
  if( temp_pid < 0 )
  {
    temp_pid = 0;
  }
  else if( temp_pid > 100 )
  {
    temp_pid = 100;
  } 
  Dac_Out(40 + (uint8_t)(temp_pid * 1.97) );//根据上一步的结果控制输�
	
	
	return duty_cycle;
}
