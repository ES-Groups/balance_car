#include "pid.h"

extern float Kp,Ki,Kd;   //直立环参数
float err;               //此次误差
float last_err;          //上次误差
float err_sum=0;         //误差累加
float err_difference;    //误差的差值

extern float VKp,VKi;    //zx
float filt_velocity;     //滤波后的速度
float last_filt_velocity;//上一次的滤波后的速度
float velocity_sum=0;    //速度的累加
float Target_velocity = 30 ;  // 设置期望速度


//直立环：
int vertical_PID_value(float measure,float calcu)
{

	err = measure - calcu;              //误差
	err_sum += err;                    	//误差的累加
	err_difference = err - last_err;   //误差的差值
	last_err = err;                    //此次误差记录为“上次误差”
	
	return Kp*err + Ki*err_sum + Kd*err_difference;
}


//速度环：
int velocity_PID_value(int velocity)   // 正反馈
{
	float a=0.3;                                           //滤波系数（反映滤波程度）
	filt_velocity = a*velocity + (1-a)*last_filt_velocity; //一阶速度滤波
	velocity_sum +=  filt_velocity;                        //速度的累加
	velocity_sum = velocity_sum - Target_velocity;
	I_xianfu(3000);                                        //累加限幅
	last_filt_velocity = filt_velocity;                    //此次速度记录为“上次速度”

	return VKp*filt_velocity + VKi*velocity_sum;
}


//I限幅：
void I_xianfu(int max)
{
	if(velocity_sum>max)  velocity_sum=max;
	if(velocity_sum<-max) velocity_sum=-max;
}
