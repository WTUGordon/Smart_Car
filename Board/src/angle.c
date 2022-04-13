/******************************************************
* 龙邱库移植到野火库。
* by          FQF
* 时间        2019/3/21
* 文件名      angle.c
* 原文件名    angle.c
* 内容        九轴数据处理
* 软件        IAR7.3
* 单片机      MK60DN512ZVLQ10

******************************************************/

#include "include.h"
#include "angle.h"

Acctypedef  ACC_Real;
Gyrotypedef GYRO_Real;

#define AcceRatio 	4096.0f   //灵敏度 MPU6050 :16384.0  LSB/g           九轴    4096      LSB/g
#define GyroRatio 	0.0625f   //灵敏度                  16.4        LSB/°/s                 0.0625  °/s/LSB
#define Gyro_Gr		0.0010653	// 角速度变成弧度	此参数对应陀螺2000度每秒
#define ACC_FILTER_NUM 7		// 加速度计滤波深度
#define GYRO_FILTER_NUM 3		// 陀螺仪滤波深度

int32 ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];	// 滤波缓存数组
int32 GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];
int16 Gyro_z=0;     //滤波后角速度
float Angle = 0;     //滤波后角度

float  gyro_x = 0;   //Kerman输出角速度
float Angle2 = 0;   //使用正切得出角度
float Yaw = 0;       //俯仰角(未用)

void Angle_Read(void)
{
	int16 ACC_X, ACC_Y, ACC_Z, GYRO_X, GYRO_Y, GYRO_Z;
	Get_Acc(&ACC_X, &ACC_Y, &ACC_Z);
	Get_Gyro(&GYRO_X, &GYRO_Y, &GYRO_Z);
	uint8 i;

	int64 temp1 = 0,  temp3 = 0,  temp5 = 0;   //temp2 = 0,  temp4 = 0   temp6 = 0

	ACC_X_BUF[0] = ACC_X;	// 更新滑动窗口数组
//	ACC_Y_BUF[0] = ACC_Y;
	ACC_Z_BUF[0] = ACC_Z;
//	GYRO_X_BUF[0] = GYRO_X;
	GYRO_Y_BUF[0] = GYRO_Y;
//	GYRO_Z_BUF[0] = GYRO_Z;

	for (i = 0; i<ACC_FILTER_NUM; i++)
	{
		temp1 += ACC_X_BUF[i];
//		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];

	}
	for (i = 0; i<GYRO_FILTER_NUM; i++)
	{
//		temp4 += GYRO_X_BUF[i];
		temp5 += GYRO_Y_BUF[i];
//		temp6 += GYRO_Z_BUF[i];
	}

	ACC_Real.X = temp1 / ACC_FILTER_NUM ;
//	ACC_Real.Y = temp2 / ACC_FILTER_NUM;
	ACC_Real.Z = temp3 / ACC_FILTER_NUM;

//	GYRO_Real.X = (temp4 / GYRO_FILTER_NUM - GYRO_Real.Xoffset) * GyroRatio;
//	GYRO_Real.Y = (temp5 / GYRO_FILTER_NUM - GYRO_Real.Xoffset) * GyroRatio;
//	GYRO_Real.Z = (temp6 / GYRO_FILTER_NUM - GYRO_Real.Xoffset) * GyroRatio;

//    GYRO_Real.X = temp4 / GYRO_FILTER_NUM  * GyroRatio;
	GYRO_Real.Y = temp5 / GYRO_FILTER_NUM * GyroRatio;
//	GYRO_Real.Z = temp6 / GYRO_FILTER_NUM  * GyroRatio;


	for (i = 0; i < ACC_FILTER_NUM - 1; i++)
	{
		ACC_X_BUF[ACC_FILTER_NUM - 1 - i] = ACC_X_BUF[ACC_FILTER_NUM - 2 - i];
//		ACC_Y_BUF[ACC_FILTER_NUM - 1 - i] = ACC_Y_BUF[ACC_FILTER_NUM - 2 - i];
		ACC_Z_BUF[ACC_FILTER_NUM - 1 - i] = ACC_Z_BUF[ACC_FILTER_NUM - 2 - i];

	}
	for (i = 0; i < GYRO_FILTER_NUM - 1; i++)
	{
//		GYRO_X_BUF[GYRO_FILTER_NUM - 1 - i] = GYRO_X_BUF[GYRO_FILTER_NUM - 2 - i];
		GYRO_Y_BUF[GYRO_FILTER_NUM - 1 - i] = GYRO_Y_BUF[GYRO_FILTER_NUM - 2 - i];
//		GYRO_Z_BUF[GYRO_FILTER_NUM - 1 - i] = GYRO_Z_BUF[GYRO_FILTER_NUM - 2 - i];
	}

	/*角度得到*/
	//   Angle=(int16)((ACC_RealZ-13950)/AcceRatio);
	//    Angle=(int16)(ACC_RealZ*1000);

	//    	IMUupdate(GYRO_Real.X*Gyro_Gr*GyroRatio,
	//			  GYRO_Real.Y*Gyro_Gr*GyroRatio,
	//			  GYRO_Real.Z*Gyro_Gr*GyroRatio,
	//			  ACC_Real.X * AcceRatio,
	//			  ACC_Real.Y * AcceRatio,
	//			  ACC_Real.Z * AcceRatio);	// 姿态解算出欧拉角

	Angle2 = (atan2(ACC_Real.X, -ACC_Real.Z)) * 573;
    Gyro_z = (int16)(-GYRO_Real.Y * 10);
	Matrix_KalmanFilter(Gyro_z, Angle2);
	Gyro_z = RANGE_INT16(Gyro_z, -2000, 2000);

	/*角速度得到  由于九轴安装问题，采用的是Y轴数据*/

}

//卓晴方案
void  Filter(float ACC_Angle)
{
	float   fDeltaValue = 0;
	static float GravityAngle = 0;
	static float fGyroscopeAngleIntegral = 0;
	GravityAngle = (ACC_Angle - 15085)*1.75;
	float Gyro = GYRO_Real.Y*0.58;
	Angle = fGyroscopeAngleIntegral;
	fDeltaValue = (GravityAngle - Angle) / 2;      //补偿量
	fGyroscopeAngleIntegral += (Gyro + fDeltaValue) / 100;
}

#define AANGPERIODFAV  (5)
int16 AAangPWMOut(int16 NewAangPWM, int16 LastAangPWM, uint8_t PeriodCount)
{
	int16  AangPWMfav;
	int16  AangOUT;
	AangPWMfav = NewAangPWM - LastAangPWM;
	AangOUT = AangPWMfav *(PeriodCount) / AANGPERIODFAV + LastAangPWM;

	return AangOUT;

}


//矩阵卡尔曼滤波
//Q 的取值越小越好  当Q 取值逐渐增大时，滤波收敛变慢，且状态变量的扰动变大
float  Q_angle = 0.001;//0.0001;    0.1     陀螺仪噪声的协方差(估计过程的误差协方差)
float  Q_gyro = 0.0005;//0.00003;  1       陀螺仪漂移噪声的协方差      Cov(gyro,gyro)
float  R_angle = 0.05;    //0.01        0.001  加速度计测量噪声的协方差   R取值越小收敛越快
float  dt = 0.003;  //积分时间 dt为kalman滤波器采样时间
float  C_0 = 1.0;              //H矩阵的一个数
float  Q_bias = 0, Angle_err = 0;      //Q_bias为陀螺仪漂移
float  PCt_0 = 0.0, PCt_1 = 0, E = 0.0;    //中间变量
float  K_0 = 0.0, K_1 = 0.0, t_0 = 0.0, t_1 = 0.0;   //K是卡尔曼增益，t是中间变量
float  Pdot[4] = { 0,0,0,0 };                 //计算P矩阵的中间变量
float  PP[2][2] = { { 1.0, 0 },            //公式中P矩阵，X的协方差
{ 0, 1.0 } };
void Matrix_KalmanFilter(float Gyro, float Accel)
{
	/*X(k|k-1)=AX(k-1|k-1)+BU(k)*/
	Angle += (Gyro - Q_bias) * dt; //先验估计
    //角度测量方程模型方程，角度估计值=上一次的最优角度+（角速度-上一次的最优零漂）*dt
    //就漂移来说认为每次都是相同的 Q_bias

    /* P(k|k-1)=AP(K-1|k-1)A' +Q*/
	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-' 先验估计误差协方差的微分
	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt; // Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
	PP[0][1] += Pdot[1] * dt;
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	/* Kg(k)=P(k|k-1)H'/(HP(k|k-1)H'+R)*/
	PCt_0 = C_0 * PP[0][0];    //矩阵乘法的中间变量
	PCt_1 = C_0 * PP[1][0];    //C_0=1
	E = R_angle + C_0 * PCt_0;//分母
							  //卡尔曼增益  一个是Angle的  一个是Q_bias的
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	/* X(k|k)=X(k|k-1)+Kg(K)(Z(k)-HX(k|k-1))*/
	Angle_err = Accel - Angle; //zk-先验估计
	Angle += K_0 * Angle_err;  //后验估计  计算最优角度
	Q_bias += K_1 * Angle_err;  //后验估计   计算最优零漂
	gyro_x = Gyro - Q_bias;     //输出值（后验估计）的微分 = 角速度

    /* P(k|k)=(I-Kg(K)H)P(k|k-1)*/
	t_0 = PCt_0;               //矩阵计算中间变量 相当于a
	t_1 = C_0 * PP[0][1];    //矩阵计算中间变量 相当于b

	PP[0][0] -= K_0 * t_0;  //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
}

#define ACC_Z_Q (10)
#define ACC_Z_R (80)     // Q/(Q+R)=0.322
float Finaldata_ACC=0;
//    Q/(Q+R)的值就是卡尔曼增益的收敛值
//卡尔曼增益越大，说明测量值越可靠，最优化角度越接近测量值
//卡尔曼增益越小，说明预测值越可靠，最优化角度越接近预测值
float Kaman_ACC(float signal)
{
    static float nowdata_p=20;
    float nowdata;
    float kg =0.1;
    nowdata = Finaldata_ACC;
    nowdata_p = nowdata_p+ACC_Z_Q;
    kg =nowdata_p/(nowdata_p+ACC_Z_R);
    Finaldata_ACC = nowdata+kg*(signal - nowdata);
    nowdata_p=(1-kg)*nowdata_p;
    return Finaldata_ACC;
}

/*   姿态解算 https://blog.csdn.net/u010097644/article/details/70881395/  */
#define Kp 10.0f //1.6f                       //加速度权重，越大则向加速度测量值收敛越快
#define Ki 0.008f //0.001f //1.2f         //误差积分增益
#define halfT 0.001f             // 采样周期的一半，用于求解四元数微分方程时计算角增量
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始姿态四元数，由上篇博文提到的变换四元数公式得来
float exInt = 0, eyInt = 0, ezInt = 0;    //当前加计测得的重力加速度在三轴上的分量
										  //与用当前姿态计算得来的重力在三轴上的分量的误差的积分
										  //传入的陀螺仪数据需乘一个系数将其转化为弧度制角速度
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)//g表陀螺仪，a表加计
{
	float q0temp, q1temp, q2temp, q3temp;//四元数暂存变量，求解微分方程时要用
	float norm; //矢量的模或四元数的范数
	float vx, vy, vz;//当前姿态计算得来的重力在三轴上的分量
	float ex, ey, ez;//当前加计测得的重力加速度在三轴上的分量
					 //与用当前姿态计算得来的重力在三轴上的分量的误差

					 // 先把这些用得到的值算好
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q1q1 = q1*q1;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	if (ax*ay*az == 0)//加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
		return;
	norm = sqrt(ax*ax + ay*ay + az*az);//单位化加速度计，
	ax = ax / norm;// 这样变更了量程也不需要修改KP参数，因为这里归一化了
	ay = ay / norm;
	az = az / norm;
	//用当前姿态计算出重力在三个轴上的分量，
	//参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是（博文一中有提到）
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	//计算测得的重力与计算得重力间的误差，向量外积可以表示这一误差
	//原因我理解是因为两个向量是单位向量且sin0等于0
	//不过要是夹角是180度呢~这个还没理解
	ex = (ay*vz - az*vy);    // 向量外积在相减得到差分就是误差
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	exInt = exInt + ex * Ki;  //对误差进行积分
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;  //将误差PI后补偿到陀螺仪，即补偿零点漂移
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;    //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
								//下面进行姿态的更新，也就是四元数微分方程的求解
	q0temp = q0;//暂存当前值用于计算
	q1temp = q1;//网上传的这份算法大多没有注意这个问题，在此更正
	q2temp = q2;
	q3temp = q3;
	//采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
	q0 = q0temp + (-q1temp*gx - q2temp*gy - q3temp*gz)*halfT;
	q1 = q1temp + (q0temp*gx + q2temp*gz - q3temp*gy)*halfT;
	q2 = q2temp + (q0temp*gy - q1temp*gz + q3temp*gx)*halfT;
	q3 = q3temp + (q0temp*gz + q1temp*gy - q2temp*gx)*halfT;
	//单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;
	//四元数到欧拉角的转换，公式推导见博文一
	//其中YAW航向角由于加速度计对其没有修正作用，因此此处直接用陀螺仪积分代替
	//Q_ANGLE.Z = GYRO_I.Z; // yaw   航向角
	//Q_ANGLE.Z = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)*57.3; // yaw
	//Q_ANGLE.Y = asin(-2 * q1 * q3 + 2 * q0* q2)*57.3; // pitch     俯仰角
	//Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1,-2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll  翻滚角
	Angle = asin(-2 * q1 * q3 + 2 * q0* q2) * 5730; // pitch     俯仰角   *100
	Yaw = atan2(2 * q1*q2 + 2 * q0*q3, -2 * q2*q2 - 2 * q3*q3 + 1)*57.3; // yaw
}

////非矩阵卡尔曼滤波
//#define Peried 0.003f		//卡尔曼积分周期  500HZ
//#define Q 1.5f				//过程噪声2.0		越小积分越慢，跟踪加速度计越慢越平滑
//#define R 250.0f			//测量噪声5000.0	越小跟踪加速度计越快

//float KalmanGain = 1.0f;	//卡尔曼增益
//#define Gyroscope_Q (10)
//#define Gyroscope_R (100)
//#define ACC_Z_Q (10)
//#define ACC_Z_R (200)
//int Kaman_Gyroscope(int signal)
//{
//    static float nowdata_p=20;
//    int16_t nowdata;
//    float kg =0.1;
//    nowdata = Finaldata_Gyroscope;
//    nowdata_p = nowdata_p+Gyroscope_Q;
//    kg =nowdata_p/(nowdata_p+Gyroscope_R);
//    Finaldata_Gyroscope = nowdata+kg*(signal - nowdata);
//    nowdata_p=(1-kg)*nowdata_p;
//    return Finaldata_Gyroscope;
//}


