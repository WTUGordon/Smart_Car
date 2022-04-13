/******************************************************
* 龙邱库移植到野火库。
* by          FQF
* 时间        2019/6/1
* 文件名      8700_2100.c
* 原文件名    8700_2100.c
* 内容        陀螺仪加速度计数据获取
* 软件        IAR7.3
* 单片机      MK60DN512ZVLQ10

******************************************************/

#include "include.h"
#include "camera.h"


uint8  seek_stop_line = 0;				//寻边停止行
float  midcourt_line_slope = 0;           //中线斜率
uint8  Middle_line[CAMERA_H] = { 0 };  //中线坐标数组
float  Dolly_now_angle = 0;               //角度
uint8  track_judge_finish = 0;            //赛道判断完成标志  完成置1

uint8 Start_Line = 0; //开始检测行数
uint8 End_Line = 0; //结束检测行数
uint8 Emit_Flag = 0;//超声波发出标志
uint8 Track_Flag=CURVE;//赛道类型

extern int32 Speed_Horizon;


/*
*	图像处理
*
*	说明：判断斑马线或路障或断路
*/
void Img_Process(uint8 *data)
{
	//uint8 img[CAMERA_H][CAMERA_W];   //由于小钻风摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
	//1 表示 白色，0表示 黑色

	uint8 Zebra_Line = 0;//斑马线行数
	uint8 Suspect = 0;//疑点标志
    uint8 street_len=0;//中线长度
	uint8 i;//控制行
	uint8 Track_Stop = 0;


	uint8 Jump[60] = { 0 };//腐蚀滤波跳变
	uint8 Left_Line[62] = { 0 };
    uint8 Right_Line[62] = { 0 };
    uint8 Mid_Line[62] = { 0 };	// 原始左右边界数据

	if (Trans_Flag & 0x01)//水平状态
	{
        Start_Line = 56;
		End_Line = 10;
		Zebra_Line = 17;

        uint8 street_len = 59 - Limit_Scan(Start_Line, data, 40); //中线长度
        if (street_len>5)
        {
            Mid_Line[Start_Line + 1] = 40;
            for (i = Start_Line; i >= End_Line; i--) //隔行检查   倒着找
            {
                data[i * 80 + 1] = 0;//强行补线
                data[i * 80 + 79] = 0;

                Jump[i] = Corrode_Filter(i, data, 1, 79);	// 使用腐蚀滤波算法先对本行赛道进行预处理，返回跳变点数量
                if (Jump[i] ==0)
                {
                    if (data[((i - 2) * 80 + Mid_Line[i - 2])] == 0 && i>Zebra_Line)  //<或>?越大说明离得越近
                        Suspect++;
                    if (Track_Stop == 0)
                        Track_Stop = i;
                    Mid_Line[i] = Mid_Line[i + 1];
                }
                else if(Jump[i] < 5)// 使用前行中点向两边扫描边界
                {
                    Traversal_Mid_Line(i, data, Mid_Line[i + 1], 1, 79, Left_Line, Right_Line, Mid_Line);
                }
            }
            Track_Flag=Track_Judge(data, Mid_Line, Track_Stop);

             if(Track_Flag == STRAIGHT)
                Speed_Set=Speed_Horizon;
            else
                Speed_Set=Speed_Horizon-100;
            Speed_Set=(int32)(0.7*Speed_Set+0.3*Last_Speed_Set);

            Last_Speed_Set=Speed_Set;
            if (Suspect>3 && Emit_Flag == 0&&Round_State == 0&&(Star_Flag & 0x40)&&Track_Flag == STRAIGHT&&Break_Flag==0)
            {
                Emit_Flag=1;
                beep_on();
                DELAY_MS(100);
                beep_off();
            }
        }
    }
	else //直立
	{
        Start_Line = 59;
		End_Line = 37;
        if(Angle<=200)
            Zebra_Line = 35;
        else if(Angle>200&&Angle<=225)
            Zebra_Line = 30+(uint8)(Angle-200)/5;
        else if(Angle>225&&Angle<=250)
            Zebra_Line = 35+(uint8)(Angle-225)/5;
        else if(Angle>250&&Angle<=275)
            Zebra_Line = 40+(uint8)(Angle-250)/5;
        else
            Zebra_Line=45;

        Mid_Line[Start_Line + 1] = 40;
        for (i = Start_Line; i >= End_Line; i--) //隔行检查   倒着找
        {
            data[i * 80 + 1] = 0;//强行补线
            data[i * 80 + 79] = 0;

            Jump[i] = Corrode_Filter(i, data, 1, 79);	// 使用腐蚀滤波算法先对本行赛道进行预处理，返回跳变点数量
            if (Jump[i] == 0)
            {
                if (data[((i - 2) * 80 + Mid_Line[i - 2])] == 0 &&data[((i - 1) * 80 + Mid_Line[i - 1])] == 0&& i>Zebra_Line)  //<或>?越大说明离得越近
                    Suspect++;
                if (Track_Stop == 0)
                    Track_Stop = i;
                Mid_Line[i] = Mid_Line[i + 1];
            }
            else if(Jump[i] < 5)// 使用前行中点向两边扫描边界
            {
                Traversal_Mid_Line(i, data, Mid_Line[i + 1], 1, 79, Left_Line, Right_Line, Mid_Line);
            }
        }

        if (Suspect>2 && Emit_Flag == 0&&Round_State == 0&&(Star_Flag & 0x40)&&Break_Flag==0)
            if (Track_Judge(data, Mid_Line, Track_Stop) == STRAIGHT)
            {
                street_len = 59 - Limit_Scan(Start_Line, data, 40); //中线长度
                if(street_len>1)
                {
                    Emit_Flag=1;
                    beep_on();
                    DELAY_MS(200);
                    beep_off();
                }
            }

	}
}


/*
*	从中间向两边搜索边界
*
*	说明：本函数使用后将保存边界数据和中线数据
*/
void Traversal_Mid_Line(uint8 i, uint8 *data, uint8 Mid, uint8 Left_Min, uint8 Right_Max, uint8 *Left_Line, uint8 *Right_Line, uint8 *Mid_Line)
{
	uint8 j;
	uint8 Left_Add_Flag, Right_Add_Flag;		// 左右边界补线标志位

	Left_Add_Flag = 1;	// 初始化补线标志位
	Right_Add_Flag = 1;

	Left_Min = RANGE_UINT8(Left_Min, 1, 79);	// 限幅，防止出错
	Right_Max = RANGE_UINT8(Right_Max, 1, 79);

	Right_Line[i] = Right_Max;
	Left_Line[i] = Left_Min;	// 给定边界初始值

	for (j = Mid; j >= Left_Min; j--)	// 以前一行中点为起点向左查找边界
	{
		if (!data[i * 80 + j])	// 检测到黑点
		{
			Left_Add_Flag = 0;	//左边界不需要补线，清除标志位
			Left_Line[i] = j + 1;	//记录当前j值为本行实际左边界

			break;
		}
	}

	for (j = Mid; j <= Right_Max; j++)	// 以前一行中点为起点向右查找右边界
	{
		if (!data[i * 80 + j])	//检测到黑点
		{
			Right_Add_Flag = 0;		//右边界不需要补线，清除标志位
			Right_Line[i] = j - 1;	//记录当前j值为本行右边界

			break;
		}
	}
	if (Left_Add_Flag == 1 && Right_Add_Flag == 1)
		Mid_Line[i] = Mid_Line[i + 1];
	else
		Mid_Line[i] = (Right_Line[i] + Left_Line[i]) / 2;			// 计算中线

	data[i * 80 + Mid_Line[i]] = 0;    //显示中线

}


/*
*	判断直道或弯道
*
*	通过斜率返回判断情况
*/
uint8 Track_Judge(uint8*data, uint8*Middle_line, uint8 Track_Stop)
{
	uint8 track_type = 0;
	uint8 Track_Start = 0;
    midcourt_line_slope=100;
	for (uint8 i = Start_Line; i>End_Line; i--)
	{
		if (Middle_line[i] != 40)
		{
			Track_Start = i;
			break;
		}
	}
	if (Track_Start == 0)
	{
		track_type = STRAIGHT;
	}
	else
	{
		midcourt_line_slope = regression(Middle_line, Track_Stop, Track_Start);
		Dolly_now_angle = atanf(midcourt_line_slope) * 180 / PI; //求角度
		if ((ABS(Dolly_now_angle) < 20))
		{
			track_type = STRAIGHT;
		}
		else
		{
			track_type = CURVE;
		}

	}
	return track_type;
}

/*
*	最小二乘法
*
*	计算斜率值
*/
float regression(uint8 Pick_table[], uint8 startline, uint8 endline)
{
	int num = 0, i;
	float sumX = 0, sumY = 0, avrX = 0, avrY = 0;
	float B_up1 = 0, B_up2 = 0;
	float B_up = 0, B_down = 0;
	float slope = 0;
	for (i = startline; i <= endline; i++)
	{
		if (Pick_table[i])
		{
			num++;
			sumX += i;
			sumY += Pick_table[i];
		}
	}
	avrX = sumX / num;
	avrY = sumY / num;
	for (i = startline; i <= endline; i++)
	{
		if (Pick_table[i])
		{
			B_up1 = (int)Pick_table[i] - (int)avrY;
			B_up2 = i - avrX;
			B_up += (int)B_up1*(int)B_up2;
			B_down += (int)(i - avrX)*(int)(i - avrX);
		}
	}
	if (B_down == 0) slope = 0;
	else slope = B_up / B_down;
	return slope;
}

/*
*	尽头搜索
*
*	说明：从某一点开始竖直向上搜索，返回最远行坐标
*/
uint8 Limit_Scan(uint8 i, uint8 *data, uint8 Point)
{
	for (; i >= 5; i--)
	{
		if (!data[80 * i + Point])	// 搜索到黑点
		{
			break;
		}
	}

	return i;	// 返回最远行坐标
}

uint8 Test_Jump;
/*
*	腐蚀滤波
*   Butter-fly
*	说明：将干扰滤除，并统计黑白跳变点数量，用于起跑线检测   白255  黑0
*/
uint8 Corrode_Filter(uint8 i, uint8 *data, uint8 Left_Min, uint8 Right_Max)
{
	uint8 j;
	uint8 White_Flag = 0;
	uint8 Jump_Count = 0;	// 跳变点计数

	Test_Jump = 0;

	Right_Max = RANGE_UINT8(Right_Max, 1, 79);	// 保留右侧部分区域，防止溢出

	for (j = Left_Min; j <= Right_Max; j++)	// 从左向右扫描，方向不影响结果
	{
		if (!White_Flag)	// 先查找白点，只滤黑点，不滤白点
		{
			if (data[i * 80 + j])	// 检测到白点
			{
				White_Flag = 1;	// 开始找黑点
			}
		}
		else
		{
			if (!data[i * 80 + j])	// 检测到黑点
			{
				Jump_Count++;	// 视为一次跳变

				Test_Jump = Jump_Count;

				if (!data[i * 80 + j + 1] && j + 1 <= Right_Max)	// 连续两个黑点
				{
					if (!data[i * 80 + j + 2] && j + 2 <= Right_Max)	// 连续三个黑点
					{
						if (!data[i * 80 + j + 3] && j + 3 <= Right_Max)	// 连续四个黑点
						{
							if (!data[i * 80 + j + 4] && j + 4 <= Right_Max)	// 连续五个黑点
							{
								if (!data[i * 80 + j + 5] && j + 5 <= Right_Max)	// 连续六个黑点
								{
									if (!data[i * 80 + j + 6] && j + 6 <= Right_Max)	// 连续七个黑点
									{
										if (!data[i * 80 + j + 7] && j + 7 <= Right_Max)	// 连续八个黑点
										{
											if (!data[i * 80 + j + 8] && j + 8 <= Right_Max)	// 连续九个黑点
											{
												if (!data[i * 80 + j + 9] && j + 9 <= Right_Max)	// 连续十个黑点
												{
													if (!data[i * 80 + j + 10] && j + 10 <= Right_Max)	// 连续11个黑点
													{
														White_Flag = 0;	// 认为不是干扰，不做任何处理，下次搜索白点
														j += 10;
													}
													else if (j + 10 <= Right_Max)
													{
														data[i * 80 + j] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 1] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 2] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 3] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 4] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 5] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 6] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 7] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 8] = 255;	// 仅有连续10个黑点，滤除掉
														data[i * 80 + j + 9] = 255;	// 仅有连续10个黑点，滤除掉

														j += 10;
													}
													else
													{
														j += 10;
													}
												}
												else if (j + 9 <= Right_Max)
												{
													data[i * 80 + j] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 1] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 2] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 3] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 4] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 5] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 6] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 7] = 255;	// 仅有连续九个黑点，滤除掉
													data[i * 80 + j + 8] = 255;	// 仅有连续九个黑点，滤除掉

													j += 9;
												}
												else
												{
													j += 9;
												}
											}
											else if (j + 8 <= Right_Max)
											{
												data[i * 80 + j] = 255;	// 仅有连续八个黑点，滤除掉
												data[i * 80 + j + 1] = 255;	// 仅有连续八个黑点，滤除掉
												data[i * 80 + j + 2] = 255;	// 仅有连续八个黑点，滤除掉
												data[i * 80 + j + 3] = 255;	// 仅有连续八个黑点，滤除掉
												data[i * 80 + j + 4] = 255;	// 仅有连续八个黑点，滤除掉
												data[i * 80 + j + 5] = 255;	// 仅有连续八个黑点，滤除掉
												data[i * 80 + j + 6] = 255;	// 仅有连续八个黑点，滤除掉
												data[i * 80 + j + 7] = 255;	// 仅有连续八个黑点，滤除掉

												j += 8;
											}
											else
											{
												j += 8;
											}
										}
										else if (j + 7 <= Right_Max)
										{
											data[i * 80 + j] = 255;	// 仅有连续七个黑点，滤除掉
											data[i * 80 + j + 1] = 255;	// 仅有连续七个黑点，滤除掉
											data[i * 80 + j + 2] = 255;	// 仅有连续七个黑点，滤除掉
											data[i * 80 + j + 3] = 255;	// 仅有连续七个黑点，滤除掉
											data[i * 80 + j + 4] = 255;	// 仅有连续七个黑点，滤除掉
											data[i * 80 + j + 5] = 255;	// 仅有连续七个黑点，滤除掉
											data[i * 80 + j + 6] = 255;	// 仅有连续七个黑点，滤除掉

											j += 7;
										}
										else
										{
											j += 7;
										}
									}
									else if (j + 6 <= Right_Max)
									{
										data[i * 80 + j] = 255;	// 仅有连续六个黑点，滤除掉
										data[i * 80 + j + 1] = 255;	// 仅有连续六个黑点，滤除掉
										data[i * 80 + j + 2] = 255;	// 仅有连续六个黑点，滤除掉
										data[i * 80 + j + 3] = 255;	// 仅有连续六个黑点，滤除掉
										data[i * 80 + j + 4] = 255;	// 仅有连续六个黑点，滤除掉
										data[i * 80 + j + 5] = 255;	// 仅有连续六个黑点，滤除掉

										j += 6;
									}
									else
									{
										j += 6;
									}
								}
								else if (j + 5 <= Right_Max)
								{
									data[i * 80 + j] = 255;	// 仅有连续五个黑点，滤除掉
									data[i * 80 + j + 1] = 255;	// 仅有连续五个黑点，滤除掉
									data[i * 80 + j + 2] = 255;	// 仅有连续五个黑点，滤除掉
									data[i * 80 + j + 3] = 255;	// 仅有连续五个黑点，滤除掉
									data[i * 80 + j + 4] = 255;	// 仅有连续五个黑点，滤除掉

									j += 5;
								}
								else
								{
									j += 5;
								}
							}
							else if (j + 4 <= Right_Max)
							{
								data[i * 80 + j] = 255;	// 仅有连续四个黑点，滤除掉
								data[i * 80 + j + 1] = 255;	// 仅有连续四个黑点，滤除掉
								data[i * 80 + j + 2] = 255;	// 仅有连续四个黑点，滤除掉
								data[i * 80 + j + 3] = 255;	// 仅有连续四个黑点，滤除掉

								j += 4;
							}
							else
							{
								j += 4;
							}
						}
						else if (j + 3 <= Right_Max)
						{
							data[i * 80 + j] = 255;	// 仅有连续三个黑点，滤除掉
							data[i * 80 + j + 1] = 255;	// 仅有连续三个黑点，滤除掉
							data[i * 80 + j + 2] = 255;	// 仅有连续三个黑点，滤除掉

							j += 3;
						}
						else
						{
							j += 3;
						}
					}
					else if (j + 2 <= Right_Max)
					{
						data[i * 80 + j] = 255;	// 仅有连续两个黑点，滤除掉
						data[i * 80 + j + 1] = 255;	// 仅有连续两个黑点，滤除掉

						j += 2;
					}
					else
					{
						j += 2;
					}
				}
				else if (j + 1 <= Right_Max)
				{
					data[i * 80 + j] = 255;	// 有一个黑点，滤除掉

					j += 1;
				}
				else
				{
					j += 1;
				}
			}
		}
	}

	//	if (White_Flag)
	//	{
	//		Jump_Count++;	// 视为一次跳变
	//	}

	return Jump_Count;	// 返回跳变点计数
}
