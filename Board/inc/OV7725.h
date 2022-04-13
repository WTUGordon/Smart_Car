#ifndef _OV7725_H_
#define _OV7725_H_

#include "OV7725_REG.h"

#if ( USE_CAMERA == CAMERA_OV7725_EAGLE )
//��������ͷ������
#define CAMERA_USE_HREF     0               //�Ƿ�ʹ�� ���ж� (0 Ϊ ��ʹ�ã�1Ϊʹ��)
#define CAMERA_COLOR        0               //����ͷ�����ɫ �� 0 Ϊ �ڰ׶�ֵ��ͼ�� ��1 Ϊ �Ҷ� ͼ�� ��2 Ϊ RGB565 ͼ��
#define CAMERA_POWER        0               //����ͷ ��Դѡ�� 0 Ϊ 3.3V ,1 Ϊ 5V

//��������ͷ����ӿ�
#define camera_init(imgaddr)    ov7725_eagle_init(imgaddr)
#define camera_get_img()        ov7725_eagle_get_img()

#define camera_vsync()          ov7725_eagle_vsync()
#define camera_href()           //ov7725_eagle_href()
#define camera_dma()            ov7725_eagle_dma()

//���� ����ͷ ����
#define CAMERA_DMA_CH       OV7725_EAGLE_DMA_CH         //��������ͷ��DMA�ɼ�ͨ��
#define CAMERA_W            OV7725_EAGLE_W              //��������ͷͼ����
#define CAMERA_H            OV7725_EAGLE_H              //��������ͷͼ��߶�

#define CAMERA_SIZE         OV7725_EAGLE_SIZE           //ͼ��ռ�ÿռ��С
#define CAMERA_DMA_NUM      OV7725_EAGLE_DMA_NUM        //DMA�ɼ�����

#endif  //#if ( USE_CAMERA == CAMERA_OV7725_EAGLE )


//��������ͷ ����
#define OV7725_EAGLE_DMA_CH       DMA_CH0                               //��������ͷ��DMA�ɼ�ͨ��
#define OV7725_EAGLE_W            80                                    //��������ͷͼ����
#define OV7725_EAGLE_H            60                                    //��������ͷͼ��߶�
#define OV7725_EAGLE_SIZE         (OV7725_EAGLE_W * OV7725_EAGLE_H/8 )  //ͼ��ռ�ÿռ��С
#define OV7725_EAGLE_DMA_NUM      (OV7725_EAGLE_SIZE )                  //DMA�ɼ�����

extern  uint8   imgbuff[CAMERA_SIZE];                              //����洢����ͼ�������
extern  uint8   img[CAMERA_H *CAMERA_W];                           //����С�������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
extern  uint8   ov7725_eagle_init(uint8 *imgaddr);
extern  void    ov7725_eagle_get_img(void);

extern  void    ov7725_eagle_vsync(void);
extern  void    ov7725_eagle_dma(void);
extern void img_extract(void *dst, void *src, uint32_t srclen);
extern  void    Image_Decompression(uint8 *data1,uint8 *data2);
extern void img_recontract(uint8 *dst, uint8 *src, uint32 srclen);

#endif  //_OV7725_EAGLE_H_


