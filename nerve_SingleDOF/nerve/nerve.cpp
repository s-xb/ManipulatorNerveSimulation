#include "nerve.h"
#include<iostream>
#include<math.h>
#include<fstream>
using std::ofstream;
/*
神经网络输出
double hi[] 隐含层神经元输出 
double wi[] 隐含层神经元权值
int num     隐含层神经元个数
*/
double nerveNetOutput(const double hi[], const double wi[], int num)
{
	int i = 0;
	double temp = 0;
	for (i = 0; i < num; i++)
	{
		temp += hi[i] * wi[i];
	}
	return temp;
}


/*
神经网络隐含层输出
double x[]   状态变量
double ci[]  中心点坐标向量
int num      状态变量维数
double bi    高斯基函数宽度
*/
double nerveNetHi(const double x[], const double ci[], int num, double bi)
{
	int i = 0;
	double temp = 0, tempHi = 0;

	for (i = 0; i < num; i++)
	{
		temp += ((x[i] - ci[i])*(x[i] - ci[i]));
	}
	tempHi = exp(-temp/(2*bi));
	return tempHi;
}



const double Gamma = 200.0;
const double k1=0.001;

/*
double hi[]  上一时刻隐含层神经元输出 
double wi[]  上一时刻隐含层神经元输出
int n_num    隐含层神经元数量
double x[]	 状态变量
int x_num	 状态变量个数
doouble time 时间步长
*/
void wiRefresh(double hi[], double wi[],int n_num, double x[], int x_num,double time)
{
	//微分方程形式
	//wi'(t)=a(t)+b(t)wi(t)
	//wi(t+1)=wi(t)+h(a(t)+b(t)wi(t))

	double a = 0, b = 0;
	int i = 0;
	for (i = 0; i < n_num; i++)
	{
		a = Gamma*hi[i]*(125/2*x[0]+625/8*x[1]);
		b = Gamma*k1*sqrt(x[0] * x[0] + x[1] * x[1]);
		wi[i] = wi[i] + time*(a + b*wi[i]);
	}
}




const double kp = 4;
const double kv = 4;
const double TIME = 10;
const double TIME_STEP = 0.01;				//仿真计算步长
#define STEP_LEN 2000						//仿真计算步数
#define NERVE_NUM 9							//隐含层神经元个数
const double bi[NERVE_NUM] = {5,5,5,5,5,5,5,5,5};
const double ci[NERVE_NUM][2] = { { -3, -3 }, { -2, -2 }, { -1.5, -1.5 }, { -1, -1 }, { 0, 0 }, { 1, 1 }, { 1.5, 1.5 }, { -2, -2 }, { -3, -3 } };


															
double qd[STEP_LEN] = { 0 }, dqd[STEP_LEN] = { 0 }, ddqd[STEP_LEN] = { 0 };		//理想轨迹,位置，速度，加速度
double q[STEP_LEN] = { 1,1,0 }, dq[STEP_LEN] = { 0 }, ddq[STEP_LEN] = { 0 };//实际轨迹，位置，速度，加速度
double e[STEP_LEN] = { 0 }, de[STEP_LEN] = { 0 };								//误差，误差微分
double torque[STEP_LEN] = { 0 };												//没有时刻控制器输出的力矩
double hi[NERVE_NUM];															//每个隐含层输出
double wi[NERVE_NUM];															//每个隐含层神经元权值


void RBFController()
{
	ofstream fOutRefq("refq.txt");
	ofstream fOutRefDq("refDq.txt");
	ofstream fOutRefDDq("refDDq.txt");

	ofstream fOutActuq("actuq.txt");
	ofstream fOutActuDq("actuDq.txt");
	ofstream fOutActuDDq("actuDDq.txt");

	ofstream fOutErr("err.txt");
	ofstream fOutDerr("Derr.txt");
	ofstream fOutfi("fi.txt");
	ofstream fOutTorque("torque.txt");
	int t = 0;
	int i = 0;
	double fi = 0;
	double x[2] = {0};
	for (t = 0; t < STEP_LEN; t++)	//使用查表的方式，首先计算好参考值
	{
		qd[t] = sin(TIME_STEP*t);
		dqd[t] = cos(TIME_STEP*t);
		ddqd[t] = -sin(TIME_STEP*t);
		fOutRefq << qd[t] << " ";
		fOutRefDq << dqd[t] << " ";
		fOutRefDDq << ddqd[t] << " ";
	}

	fOutActuq << q[0] << " ";
	fOutActuDq << dq[0] << " ";
	fOutActuDDq << ddq[0] << " ";
	fOutErr << q[0] - qd[0] << " ";
	fOutDerr << dq[0] - dqd[0] << " ";

	//进行控制的主循环
	for (t = 1; t < STEP_LEN-1; t++)
	{
		//实际系统运行时在此处获取实际位置q[t]
		//dq[t] = (q[t] - q[t - 1]) / TIME_STEP;
		//ddq[t] = (dq[t] - dq[t - 1]) / TIME_STEP;//计算实际的速度，加速度

		e[t] = q[t] - qd[t];
		de[t] = dq[t] - dqd[t];

		fOutErr << e[t] << " ";
		fOutDerr << de[t] << " ";


		/*
		神经网络计算
		*/
		x[0] = e[t];
		x[1] = de[t];

		wiRefresh(hi, wi, NERVE_NUM, x, 2, TIME_STEP);//更新wi
		for (i = 0; i < NERVE_NUM; i++)//计算各个隐含层神经元输出
		{
			hi[i] = nerveNetHi(x, ci[i], 2, bi[i]);
		}
		fi=nerveNetOutput(hi, wi, NERVE_NUM);//神经网络输出
		fOutfi << fi << " ";

		torque[t] = 10 * (ddqd[t] - kv*de[t] - kp*e[t]) - fi;//力矩控制
		
		//计算实际速度
		if (dq[t]>0)
		{
			dq[t + 1] = dq[t] + TIME_STEP*(0.1*torque[t]-1.5*dq[t]-1);
		}
		else if (dq[t] < 0)
		{
			dq[t + 1] = dq[t] + TIME_STEP*(0.1*torque[t] - 1.5*dq[t] + 1);
		}
		else
		{
			dq[t + 1] = dq[t] + TIME_STEP*(0.1*torque[t] - 1.5*dq[t]);
		}
		q[t + 1] = dq[t + 1] * TIME_STEP + q[t];

		fOutActuq << q[t]<<" ";
		fOutActuDq << dq[t] << " ";
		fOutActuDDq << ddq[t] << " ";

		fOutTorque << torque[t]<<" ";
	}

	fOutActuq << q[t-1] << " ";
	fOutActuDq << dq[t-1] << " ";
	fOutActuDDq << ddq[t-1] << " ";

	fOutErr << q[t-1] - qd[t-1] << " ";
	fOutDerr << dq[t-1] - dqd[t-1] << " ";

	fOutRefq.close();
	fOutRefDq.close();
	fOutRefDDq.close();
	fOutActuq.close();
	fOutActuDq.close();
	fOutActuDDq.close();
	fOutTorque.close();
	fOutErr.close();
	fOutDerr.close();;
	fOutfi.close();
}


