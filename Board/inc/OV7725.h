#ifndef _OV7725_H_
#define _OV7725_H_

#include "OV7725_REG.h"

#if ( USE_CAMERA == CAMERA_OV7725_EAGLE )
//配置摄像头的特性
#define CAMERA_USE_HREF     0               //是否使用 行中断 (0 为 不使用，1为使用)
#define CAMERA_COLOR        0               //摄像头输出颜色 ， 0 为 黑白二值化图像 ，1 为 灰度 图像 ，2 为 RGB565 图像
#define CAMERA_POWER        0               //摄像头 电源选择， 0 为 3.3V ,1 为 5V

//配置摄像头顶层接口
#define camera_init(imgaddr)    ov7725_eagle_init(imgaddr)
#define camera_get_img()        ov7725_eagle_get_img()

#define camera_vsync()          ov7725_eagle_vsync()
#define camera_href()           //ov7725_eagle_href()
#define camera_dma()            ov7725_eagle_dma()

//配置 摄像头 参数
#define CAMERA_DMA_CH       OV7725_EAGLE_DMA_CH         //定义摄像头的DMA采集通道
#define CAMERA_W            OV7725_EAGLE_W              //定义摄像头图像宽度
#define CAMERA_H            OV7725_EAGLE_H              //定义摄像头图像高度

#define CAMERA_SIZE         OV7725_EAGLE_SIZE           //图像占用空间大小
#define CAMERA_DMA_NUM      OV7725_EAGLE_DMA_NUM        //DMA采集次数

#endif  //#if ( USE_CAMERA == CAMERA_OV7725_EAGLE )


//配置摄像头 属性
#define OV7725_EAGLE_DMA_CH       DMA_CH0                               //定义摄像头的DMA采集通道
#define OV7725_EAGLE_W            80                                    //定义摄像头图像宽度
#define OV7725_EAGLE_H            60                                    //定义摄像头图像高度
#define OV7725_EAGLE_SIZE         (OV7725_EAGLE_W * OV7725_EAGLE_H/8 )  //图像占用空间大小
#define OV7725_EAGLE_DMA_NUM      (OV7725_EAGLE_SIZE )                  //DMA采集次数

extern  uint8   imgbuff[CAMERA_SIZE];                              //定义存储接收图像的数组
extern  uint8   img[CAMERA_H *CAMERA_W];                           //由于小钻风摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
extern  uint8   ov7725_eagle_init(uint8 *imgaddr);
extern  void    ov7725_eagle_get_img(void);

extern  void    ov7725_eagle_vsync(void);
extern  void    ov7725_eagle_dma(void);
extern void img_extract(void *dst, void *src, uint32_t srclen);
extern  void    Image_Decompression(uint8 *data1,uint8 *data2);
extern void img_recontract(uint8 *dst, uint8 *src, uint32 srclen);

#endif  //_OV7725_EAGLE_H_


