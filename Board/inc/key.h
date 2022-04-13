/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ���ѧ��̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_KEY.h
 * @brief      KEY����ͷ�ļ�
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-07-10
 */

#ifndef __KEY_H__
#define __KEY_H__


//�����Ƕ��尴����ʱ�䣬��λΪ �� 10ms���ж�ʱ�䣩
#define KEY_DOWN_TIME           1       //����ȷ�ϰ���ʱ��
#define KEY_HOLD_TIME           50      //����holdȷ��ʱ�䣬���253��������Ҫ�޸� keytime ������
                                        //�������һֱ����ȥ����ÿ�� KEY_HOLD_TIME - KEY_DOWN_TIME ʱ��ᷢ��һ�� KEY_HOLD ��Ϣ

//���尴����ϢFIFO��С
#define KEY_MSG_FIFO_SIZE       20      //��� 255��������Ҫ�޸�key_msg_front/key_msg_rear����

//�����˿ڵ�ö��
typedef enum
{
    SW_1,
    SW_2,

    SW_3,
    SW_4,



    KEY_1,  //��
    KEY_2,  //��

    KEY_3,  //��
    KEY_4,  //��

    KEY_MAX,


//
//    KEY_START,  //��ʼ
//    KEY_STOP,   //ֹͣ

//    KEY_MAX,

    KEY_A,  //ȡ��
    KEY_B,  //ѡ��
} KEY_e;


//key״̬�궨��
typedef enum
{
    KEY_DOWN  =   0,         //��������ʱ��Ӧ��ƽ
    KEY_UP    =   1,         //��������ʱ��Ӧ��ƽ

    KEY_HOLD,

}  KEY_STATUS_e;

//������Ϣ�ṹ��
typedef struct
{
    KEY_e           key;
    KEY_STATUS_e    status;
} KEY_MSG_t;


void            key_init(KEY_e key);            // KEY��ʼ������(key С�� KEY_MAX ʱ��ʼ�� ��Ӧ�˿ڣ������ʼ��ȫ���˿�)
KEY_STATUS_e    key_check(KEY_e key);           //���key״̬������ʱ������
KEY_STATUS_e    key_get(KEY_e key);            //���key״̬��������ʱ������

//��ʱɨ�谴��
uint8   get_key_msg(KEY_MSG_t *keymsg);         //��ȡ������Ϣ������1��ʾ�а�����Ϣ��0Ϊ�ް�����Ϣ
void    key_IRQHandler(void);                   //��Ҫ��ʱɨ����жϸ�λ��������ʱʱ��Ϊ10ms��


#endif  //__VCAN_KEY_H__