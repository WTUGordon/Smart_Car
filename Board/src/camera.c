/******************************************************
* �������ֲ��Ұ��⡣
* by          FQF
* ʱ��        2019/6/1
* �ļ���      8700_2100.c
* ԭ�ļ���    8700_2100.c
* ����        �����Ǽ��ٶȼ����ݻ�ȡ
* ���        IAR7.3
* ��Ƭ��      MK60DN512ZVLQ10

******************************************************/

#include "include.h"
#include "camera.h"


uint8  seek_stop_line = 0;				//Ѱ��ֹͣ��
float  midcourt_line_slope = 0;           //����б��
uint8  Middle_line[CAMERA_H] = { 0 };  //������������
float  Dolly_now_angle = 0;               //�Ƕ�
uint8  track_judge_finish = 0;            //�����ж���ɱ�־  �����1

uint8 Start_Line = 0; //��ʼ�������
uint8 End_Line = 0; //�����������
uint8 Emit_Flag = 0;//������������־
uint8 Track_Flag=CURVE;//��������

extern int32 Speed_Horizon;


/*
*	ͼ����
*
*	˵�����жϰ����߻�·�ϻ��·
*/
void Img_Process(uint8 *data)
{
	//uint8 img[CAMERA_H][CAMERA_W];   //����С�������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
	//1 ��ʾ ��ɫ��0��ʾ ��ɫ

	uint8 Zebra_Line = 0;//����������
	uint8 Suspect = 0;//�ɵ��־
    uint8 street_len=0;//���߳���
	uint8 i;//������
	uint8 Track_Stop = 0;


	uint8 Jump[60] = { 0 };//��ʴ�˲�����
	uint8 Left_Line[62] = { 0 };
    uint8 Right_Line[62] = { 0 };
    uint8 Mid_Line[62] = { 0 };	// ԭʼ���ұ߽�����

	if (Trans_Flag & 0x01)//ˮƽ״̬
	{
        Start_Line = 56;
		End_Line = 10;
		Zebra_Line = 17;

        uint8 street_len = 59 - Limit_Scan(Start_Line, data, 40); //���߳���
        if (street_len>5)
        {
            Mid_Line[Start_Line + 1] = 40;
            for (i = Start_Line; i >= End_Line; i--) //���м��   ������
            {
                data[i * 80 + 1] = 0;//ǿ�в���
                data[i * 80 + 79] = 0;

                Jump[i] = Corrode_Filter(i, data, 1, 79);	// ʹ�ø�ʴ�˲��㷨�ȶԱ�����������Ԥ�����������������
                if (Jump[i] ==0)
                {
                    if (data[((i - 2) * 80 + Mid_Line[i - 2])] == 0 && i>Zebra_Line)  //<��>?Խ��˵�����Խ��
                        Suspect++;
                    if (Track_Stop == 0)
                        Track_Stop = i;
                    Mid_Line[i] = Mid_Line[i + 1];
                }
                else if(Jump[i] < 5)// ʹ��ǰ���е�������ɨ��߽�
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
	else //ֱ��
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
        for (i = Start_Line; i >= End_Line; i--) //���м��   ������
        {
            data[i * 80 + 1] = 0;//ǿ�в���
            data[i * 80 + 79] = 0;

            Jump[i] = Corrode_Filter(i, data, 1, 79);	// ʹ�ø�ʴ�˲��㷨�ȶԱ�����������Ԥ�����������������
            if (Jump[i] == 0)
            {
                if (data[((i - 2) * 80 + Mid_Line[i - 2])] == 0 &&data[((i - 1) * 80 + Mid_Line[i - 1])] == 0&& i>Zebra_Line)  //<��>?Խ��˵�����Խ��
                    Suspect++;
                if (Track_Stop == 0)
                    Track_Stop = i;
                Mid_Line[i] = Mid_Line[i + 1];
            }
            else if(Jump[i] < 5)// ʹ��ǰ���е�������ɨ��߽�
            {
                Traversal_Mid_Line(i, data, Mid_Line[i + 1], 1, 79, Left_Line, Right_Line, Mid_Line);
            }
        }

        if (Suspect>2 && Emit_Flag == 0&&Round_State == 0&&(Star_Flag & 0x40)&&Break_Flag==0)
            if (Track_Judge(data, Mid_Line, Track_Stop) == STRAIGHT)
            {
                street_len = 59 - Limit_Scan(Start_Line, data, 40); //���߳���
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
*	���м������������߽�
*
*	˵����������ʹ�ú󽫱���߽����ݺ���������
*/
void Traversal_Mid_Line(uint8 i, uint8 *data, uint8 Mid, uint8 Left_Min, uint8 Right_Max, uint8 *Left_Line, uint8 *Right_Line, uint8 *Mid_Line)
{
	uint8 j;
	uint8 Left_Add_Flag, Right_Add_Flag;		// ���ұ߽粹�߱�־λ

	Left_Add_Flag = 1;	// ��ʼ�����߱�־λ
	Right_Add_Flag = 1;

	Left_Min = RANGE_UINT8(Left_Min, 1, 79);	// �޷�����ֹ����
	Right_Max = RANGE_UINT8(Right_Max, 1, 79);

	Right_Line[i] = Right_Max;
	Left_Line[i] = Left_Min;	// �����߽��ʼֵ

	for (j = Mid; j >= Left_Min; j--)	// ��ǰһ���е�Ϊ���������ұ߽�
	{
		if (!data[i * 80 + j])	// ��⵽�ڵ�
		{
			Left_Add_Flag = 0;	//��߽粻��Ҫ���ߣ������־λ
			Left_Line[i] = j + 1;	//��¼��ǰjֵΪ����ʵ����߽�

			break;
		}
	}

	for (j = Mid; j <= Right_Max; j++)	// ��ǰһ���е�Ϊ������Ҳ����ұ߽�
	{
		if (!data[i * 80 + j])	//��⵽�ڵ�
		{
			Right_Add_Flag = 0;		//�ұ߽粻��Ҫ���ߣ������־λ
			Right_Line[i] = j - 1;	//��¼��ǰjֵΪ�����ұ߽�

			break;
		}
	}
	if (Left_Add_Flag == 1 && Right_Add_Flag == 1)
		Mid_Line[i] = Mid_Line[i + 1];
	else
		Mid_Line[i] = (Right_Line[i] + Left_Line[i]) / 2;			// ��������

	data[i * 80 + Mid_Line[i]] = 0;    //��ʾ����

}


/*
*	�ж�ֱ�������
*
*	ͨ��б�ʷ����ж����
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
		Dolly_now_angle = atanf(midcourt_line_slope) * 180 / PI; //��Ƕ�
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
*	��С���˷�
*
*	����б��ֵ
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
*	��ͷ����
*
*	˵������ĳһ�㿪ʼ��ֱ����������������Զ������
*/
uint8 Limit_Scan(uint8 i, uint8 *data, uint8 Point)
{
	for (; i >= 5; i--)
	{
		if (!data[80 * i + Point])	// �������ڵ�
		{
			break;
		}
	}

	return i;	// ������Զ������
}

uint8 Test_Jump;
/*
*	��ʴ�˲�
*   Butter-fly
*	˵�����������˳�����ͳ�ƺڰ���������������������߼��   ��255  ��0
*/
uint8 Corrode_Filter(uint8 i, uint8 *data, uint8 Left_Min, uint8 Right_Max)
{
	uint8 j;
	uint8 White_Flag = 0;
	uint8 Jump_Count = 0;	// ��������

	Test_Jump = 0;

	Right_Max = RANGE_UINT8(Right_Max, 1, 79);	// �����Ҳಿ�����򣬷�ֹ���

	for (j = Left_Min; j <= Right_Max; j++)	// ��������ɨ�裬����Ӱ����
	{
		if (!White_Flag)	// �Ȳ��Ұ׵㣬ֻ�˺ڵ㣬���˰׵�
		{
			if (data[i * 80 + j])	// ��⵽�׵�
			{
				White_Flag = 1;	// ��ʼ�Һڵ�
			}
		}
		else
		{
			if (!data[i * 80 + j])	// ��⵽�ڵ�
			{
				Jump_Count++;	// ��Ϊһ������

				Test_Jump = Jump_Count;

				if (!data[i * 80 + j + 1] && j + 1 <= Right_Max)	// ���������ڵ�
				{
					if (!data[i * 80 + j + 2] && j + 2 <= Right_Max)	// ���������ڵ�
					{
						if (!data[i * 80 + j + 3] && j + 3 <= Right_Max)	// �����ĸ��ڵ�
						{
							if (!data[i * 80 + j + 4] && j + 4 <= Right_Max)	// ��������ڵ�
							{
								if (!data[i * 80 + j + 5] && j + 5 <= Right_Max)	// ���������ڵ�
								{
									if (!data[i * 80 + j + 6] && j + 6 <= Right_Max)	// �����߸��ڵ�
									{
										if (!data[i * 80 + j + 7] && j + 7 <= Right_Max)	// �����˸��ڵ�
										{
											if (!data[i * 80 + j + 8] && j + 8 <= Right_Max)	// �����Ÿ��ڵ�
											{
												if (!data[i * 80 + j + 9] && j + 9 <= Right_Max)	// ����ʮ���ڵ�
												{
													if (!data[i * 80 + j + 10] && j + 10 <= Right_Max)	// ����11���ڵ�
													{
														White_Flag = 0;	// ��Ϊ���Ǹ��ţ������κδ����´������׵�
														j += 10;
													}
													else if (j + 10 <= Right_Max)
													{
														data[i * 80 + j] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 1] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 2] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 3] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 4] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 5] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 6] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 7] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 8] = 255;	// ��������10���ڵ㣬�˳���
														data[i * 80 + j + 9] = 255;	// ��������10���ڵ㣬�˳���

														j += 10;
													}
													else
													{
														j += 10;
													}
												}
												else if (j + 9 <= Right_Max)
												{
													data[i * 80 + j] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 1] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 2] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 3] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 4] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 5] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 6] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 7] = 255;	// ���������Ÿ��ڵ㣬�˳���
													data[i * 80 + j + 8] = 255;	// ���������Ÿ��ڵ㣬�˳���

													j += 9;
												}
												else
												{
													j += 9;
												}
											}
											else if (j + 8 <= Right_Max)
											{
												data[i * 80 + j] = 255;	// ���������˸��ڵ㣬�˳���
												data[i * 80 + j + 1] = 255;	// ���������˸��ڵ㣬�˳���
												data[i * 80 + j + 2] = 255;	// ���������˸��ڵ㣬�˳���
												data[i * 80 + j + 3] = 255;	// ���������˸��ڵ㣬�˳���
												data[i * 80 + j + 4] = 255;	// ���������˸��ڵ㣬�˳���
												data[i * 80 + j + 5] = 255;	// ���������˸��ڵ㣬�˳���
												data[i * 80 + j + 6] = 255;	// ���������˸��ڵ㣬�˳���
												data[i * 80 + j + 7] = 255;	// ���������˸��ڵ㣬�˳���

												j += 8;
											}
											else
											{
												j += 8;
											}
										}
										else if (j + 7 <= Right_Max)
										{
											data[i * 80 + j] = 255;	// ���������߸��ڵ㣬�˳���
											data[i * 80 + j + 1] = 255;	// ���������߸��ڵ㣬�˳���
											data[i * 80 + j + 2] = 255;	// ���������߸��ڵ㣬�˳���
											data[i * 80 + j + 3] = 255;	// ���������߸��ڵ㣬�˳���
											data[i * 80 + j + 4] = 255;	// ���������߸��ڵ㣬�˳���
											data[i * 80 + j + 5] = 255;	// ���������߸��ڵ㣬�˳���
											data[i * 80 + j + 6] = 255;	// ���������߸��ڵ㣬�˳���

											j += 7;
										}
										else
										{
											j += 7;
										}
									}
									else if (j + 6 <= Right_Max)
									{
										data[i * 80 + j] = 255;	// �������������ڵ㣬�˳���
										data[i * 80 + j + 1] = 255;	// �������������ڵ㣬�˳���
										data[i * 80 + j + 2] = 255;	// �������������ڵ㣬�˳���
										data[i * 80 + j + 3] = 255;	// �������������ڵ㣬�˳���
										data[i * 80 + j + 4] = 255;	// �������������ڵ㣬�˳���
										data[i * 80 + j + 5] = 255;	// �������������ڵ㣬�˳���

										j += 6;
									}
									else
									{
										j += 6;
									}
								}
								else if (j + 5 <= Right_Max)
								{
									data[i * 80 + j] = 255;	// ������������ڵ㣬�˳���
									data[i * 80 + j + 1] = 255;	// ������������ڵ㣬�˳���
									data[i * 80 + j + 2] = 255;	// ������������ڵ㣬�˳���
									data[i * 80 + j + 3] = 255;	// ������������ڵ㣬�˳���
									data[i * 80 + j + 4] = 255;	// ������������ڵ㣬�˳���

									j += 5;
								}
								else
								{
									j += 5;
								}
							}
							else if (j + 4 <= Right_Max)
							{
								data[i * 80 + j] = 255;	// ���������ĸ��ڵ㣬�˳���
								data[i * 80 + j + 1] = 255;	// ���������ĸ��ڵ㣬�˳���
								data[i * 80 + j + 2] = 255;	// ���������ĸ��ڵ㣬�˳���
								data[i * 80 + j + 3] = 255;	// ���������ĸ��ڵ㣬�˳���

								j += 4;
							}
							else
							{
								j += 4;
							}
						}
						else if (j + 3 <= Right_Max)
						{
							data[i * 80 + j] = 255;	// �������������ڵ㣬�˳���
							data[i * 80 + j + 1] = 255;	// �������������ڵ㣬�˳���
							data[i * 80 + j + 2] = 255;	// �������������ڵ㣬�˳���

							j += 3;
						}
						else
						{
							j += 3;
						}
					}
					else if (j + 2 <= Right_Max)
					{
						data[i * 80 + j] = 255;	// �������������ڵ㣬�˳���
						data[i * 80 + j + 1] = 255;	// �������������ڵ㣬�˳���

						j += 2;
					}
					else
					{
						j += 2;
					}
				}
				else if (j + 1 <= Right_Max)
				{
					data[i * 80 + j] = 255;	// ��һ���ڵ㣬�˳���

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
	//		Jump_Count++;	// ��Ϊһ������
	//	}

	return Jump_Count;	// ������������
}
