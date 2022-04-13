/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_camera.h
 * @brief      ����ͷ�����ӿ��ض���
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-09-01
 */


#ifndef _CAMERA_H_
#define _CAMERA_H_

#define CAMERA_OV7725_EAGLE         2       //ɽ��ӥ��
#define USE_CAMERA      CAMERA_OV7725_EAGLE       //ѡ��ʹ�õ� ����ͷ

typedef struct
{
    uint8 addr;                 /*�Ĵ�����ַ*/
    uint8 val;                   /*�Ĵ���ֵ*/
} reg_s;

//����ͼ��ɼ�״̬
typedef enum
{
    IMG_NOTINIT = 0,
    IMG_FINISH,             //ͼ��ɼ����
    IMG_FAIL,               //ͼ��ɼ�ʧ��(�ɼ���������)
    IMG_GATHER,             //ͼ��ɼ���
    IMG_START,              //��ʼ�ɼ�ͼ��
    IMG_STOP,               //��ֹͼ��ɼ�
} IMG_STATUS_e;



#include  "SCCB.h"
#include  "OV7725.h"

// ����ͷ �ӿ�ͳһ�ĳ� ����ģʽ

//  camera_init(imgaddr);
//  camera_get_img();
//  camera_cfg(rag,val)


//  camera_vsync()  //���ж�
//  camera_href()   //���ж�
//  camera_dma()    //DMA�ж�

// ��Ҫ �ṩ ���� �궨��
// #define  CAMERA_USE_HREF    1     //�Ƿ�ʹ�� ���ж� (0 Ϊ ��ʹ�ã�1Ϊʹ��)
// #define  CAMERA_COLOR       1     //����ͷ�����ɫ �� 0 Ϊ �ڰ׶�ֵ��ͼ�� ��1 Ϊ �Ҷ� ͼ�� ��2 Ϊ RGB565 ͼ��
// #define  CAMERA_POWER       0     //����ͷ ��Դѡ�� 0 Ϊ 3.3V ,1 Ϊ 5V


#define STRAIGHT    0
#define CURVE         1
extern uint8 Emit_Flag;
extern float  Dolly_now_angle;               //�Ƕ�

void Img_Process(uint8 *data);
uint8 track_judge(void);
float regression(uint8 Pick_table[], uint8 startline, uint8 endline);
uint8 Corrode_Filter(uint8 i, uint8 *data, uint8 Left_Min, uint8 Right_Max); //��ʴ�˲�
uint8 Limit_Scan(uint8 i, uint8 *data, uint8 Point);//��ͷ����
uint8 Left_Scan(uint8 i, uint8 *data);//��߽�����
void Traversal_Mid_Line(uint8 i, uint8 *data, uint8 Mid, uint8 Left_Min, uint8 Right_Max, uint8 *Left_Line, uint8 *Right_Line, uint8 *Mid_Line);//�߽�����
uint8 Track_Judge(uint8*data, uint8*Middle_line,uint8 Track_Stop);//�ж�ֱ�������


#endif


