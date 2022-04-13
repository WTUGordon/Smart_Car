/******************************************************
 * by          Gordon
 * 时间        2019/3/31
 * 文件名      Fuzzy.c
 * 内容        模糊控制函数
 * 软件        IAR7.3
 * 单片机      MK60DN512ZVLQ10

******************************************************/

#include "include.h"
#include "Fuzzy.h"




int rule_P [7][7] = {
    //-6 -4 -2  0 2 4 6 EC  E
      {5, 5, 4,4,3,3,2}, //-6
      {5, 4, 4,3,3,2,1}, //-4
      {4, 4, 3,2,1,0,0}, //-2
      {2, 1, 0,0,0,1,2}, //0
      {0, 0, 1,2,3,4,4}, //2
      {1, 2, 3,3,4,4,5}, //4
      {2, 3, 3,4,4,5,5}  //6
};

int rule_D [7][7] = {
    //-6 -4 -2  0 2 4 6 EC   E
      {0, 0, 1,1,2,2,3},  //-6
      {0, 1, 1,2,2,3,4},  //-4
      {1, 1, 2,3,3,4,4},  //-2
      {1, 2, 3,4,3,2,1},  //0
      {4, 4, 3,3,2,1,1},  //2
      {4, 3, 2,2,1,1,0},  //4
      {3, 2, 2,1,1,0,0}   //6
};

float PFF[7] = {-6,-4,-2,0,2,4,6};      //偏差模糊论域
float DFF[7] = {-7,-4,-2,0,2,4,7};      //偏差变化率模糊论域
float PUFF[6] = {1500,2000,2500,3000,3500,4000};
float IUFF[5] = {0,0,0,0,0};
float DUFF[5] = {0,0.03,0.06,0.09,0.12};
//float DUFF[6] = {0.12,0.1,0.08,0.06,0.04,0.02};
/*模糊控制P计算*/
float Fuzzy_P (float P,float D)  // 输入偏差和偏差变化率
{
    float U=0; //最终模糊输出
    /*偏差,偏差微分以及输出值的精确量*/ 
    float PF[2]={0},DF[2]={0},UF[4]={0};
    /*偏差,偏差微分以及输出值的隶属度*/ 
    int Pn=0,Dn=0,Un[4]={0};
    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
    /*隶属度的确定*/ 
    /*根据PD的指定语言值获得有效隶属度*/ 
   if(P>PFF[0] && P<PFF[6])
  {
   if(P<=PFF[1])
   {
    Pn=-2;
    PF[0]=(PFF[1]-P)/(PFF[1]-PFF[0]);
   }
   else if(P<=PFF[2])
   {
    Pn=-1;
    PF[0]=(PFF[2]-P)/(PFF[2]-PFF[1]);
   }
   else if(P<=PFF[3])
   {
    Pn=0;
    PF[0]=(PFF[3]-P)/(PFF[3]-PFF[2]);
   }
   else if(P<=PFF[4])
   {
    Pn=1;
    PF[0]=(PFF[4]-P)/(PFF[4]-PFF[3]);
   }
   else if(P<=PFF[5])
   {
    Pn=2;
    PF[0]=(PFF[5]-P)/(PFF[5]-PFF[4]);
   }
   else if(P<=PFF[6])
   {
    Pn=3;
    PF[0]=(PFF[6]-P)/(PFF[6]-PFF[5]);
   }
  }
   
   else if(P<=PFF[0])
   {
    Pn=-2;
    PF[0]=1;
   }
   else if(P>=PFF[6])
   {
    Pn=3;
    PF[0]=0;
   }

  PF[1]=1-PF[0];


  //判断D的隶属度
   if(D>DFF[0] && D<DFF[6])
  {
   if(D<=DFF[1])
   {
    Dn=-2;
    DF[0]=(DFF[1]-D)/(DFF[1]-DFF[0]);
   }
   else if(D<=DFF[2])
   {
    Dn=-1;
    DF[0]=(DFF[2]-D)/(DFF[2]-DFF[1]);
   }
   else if(D<=DFF[3])
   {
    Dn=0;
    DF[0]=(DFF[3]-D)/(DFF[3]-DFF[2]);
   }
   else if(D<=DFF[4])
   {
    Dn=1;
    DF[0]=(DFF[4]-D)/(DFF[4]-DFF[3]);
   }
   else if(D<=DFF[5])
   {
    Dn=2;
    DF[0]=(DFF[5]-D)/(DFF[5]-DFF[4]);
   }
   else if(D<=DFF[6])
   {
    Dn=3;
    DF[0]=(DFF[6]-D)/(DFF[6]-DFF[5]);
   }
  }
   
   else if(D<=DFF[0])
   {
    Dn=-2;
    DF[0]=1;
   }
   else if(D>=DFF[6])
   {
    Dn=3;
    DF[0]=0;
   }

  DF[1]=1-DF[0];

   /*使用误差范围优化后的规则表rule_P[7][7]*/ 
   /*输出值使用13个隶属函数,中心值由UFF[7]指定*/ 
   /*一般都是四个规则有效*/ 
  Un[0]=rule_P[Pn-1+3][Dn-1+3]; 
  Un[1]=rule_P[Pn+3][Dn-1+3]; 
  Un[2]=rule_P[Pn-1+3][Dn+3]; 
  Un[3]=rule_P[Pn+3][Dn+3]; 
   
   if(PF[0]<=DF[0])    //求小   
    UF[0]=PF[0];
   else
    UF[0]=DF[0];
   if(PF[1]<=DF[0])
    UF[1]=PF[1];
   else
    UF[1]=DF[0];
   if(PF[0]<=DF[1])
    UF[2]=PF[0];
   else
    UF[2]=DF[1];
   if(PF[1]<=DF[1])
    UF[3]=PF[1];
   else
    UF[3]=DF[1];
  /*同隶属函数输出语言值求大*/ 
   if(Un[0]==Un[1])
   {
    if(UF[0]>UF[1])
    UF[1]=0;
    else
    UF[0]=0;
   }
   if(Un[0]==Un[2])
   {
    if(UF[0]>UF[2])
    UF[2]=0;
    else
    UF[0]=0;
   }
   if(Un[0]==Un[3])
   {
    if(UF[0]>UF[3])
    UF[3]=0;
    else
    UF[0]=0;
   }
   if(Un[1]==Un[2])
   {
    if(UF[1]>UF[2])
    UF[2]=0;
    else
    UF[1]=0;
   }
   if(Un[1]==Un[3])
   {
    if(UF[1]>UF[3])
    UF[3]=0;
    else
    UF[1]=0;
   } 
   if(Un[2]==Un[3])
   {
    if(UF[2]>UF[3])
    UF[3]=0;
    else
    UF[2]=0;
   }
   t1=UF[0]*PUFF[Un[0]];
   t2=UF[1]*PUFF[Un[1]];
   t3=UF[2]*PUFF[Un[2]];
   t4=UF[3]*PUFF[Un[3]];
   temp1=t1+t2+t3+t4;
   temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
   U=temp1/temp2;
   return U;
}


/*模糊控制D计算*/
float Fuzzy_D (float P,float D)  // 输入偏差和偏差变化率
{
    float U=0; //最终模糊输出
    /*偏差,偏差微分以及输出值的精确量*/ 
    float PF[2]={0},DF[2]={0},UF[4]={0};
    /*偏差,偏差微分以及输出值的隶属度*/ 
    int Pn=0,Dn=0,Un[4]={0};
    float t1=0,t2=0,t3=0,t4=0,temp1=0,temp2=0;
    /*隶属度的确定*/ 
    /*根据PD的指定语言值获得有效隶属度*/ 
   if(P>PFF[0] && P<PFF[6])
  {
   if(P<=PFF[1])
   {
    Pn=-2;
    PF[0]=(PFF[1]-P)/(PFF[1]-PFF[0]);
   }
   else if(P<=PFF[2])
   {
    Pn=-1;
    PF[0]=(PFF[2]-P)/(PFF[2]-PFF[1]);
   }
   else if(P<=PFF[3])
   {
    Pn=0;
    PF[0]=(PFF[3]-P)/(PFF[3]-PFF[2]);
   }
   else if(P<=PFF[4])
   {
    Pn=1;
    PF[0]=(PFF[4]-P)/(PFF[4]-PFF[3]);
   }
   else if(P<=PFF[5])
   {
    Pn=2;
    PF[0]=(PFF[5]-P)/(PFF[5]-PFF[4]);
   }
   else if(P<=PFF[6])
   {
    Pn=3;
    PF[0]=(PFF[6]-P)/(PFF[6]-PFF[5]);
   }
  }
   
   else if(P<=PFF[0])
   {
    Pn=-2;
    PF[0]=1;
   }
   else if(P>=PFF[6])
   {
    Pn=3;
    PF[0]=0;
   }

  PF[1]=1-PF[0];


  //判断D的隶属度
   if(D>DFF[0] && D<DFF[6])
  {
   if(D<=DFF[1])
   {
    Dn=-2;
    DF[0]=(DFF[1]-D)/(DFF[1]-DFF[0]);
   }
   else if(D<=DFF[2])
   {
    Dn=-1;
    DF[0]=(DFF[2]-D)/(DFF[2]-DFF[1]);
   }
   else if(D<=DFF[3])
   {
    Dn=0;
    DF[0]=(DFF[3]-D)/(DFF[3]-DFF[2]);
   }
   else if(D<=DFF[4])
   {
    Dn=1;
    DF[0]=(DFF[4]-D)/(DFF[4]-DFF[3]);
   }
   else if(D<=DFF[5])
   {
    Dn=2;
    DF[0]=(DFF[5]-D)/(DFF[5]-DFF[4]);
   }
   else if(D<=DFF[6])
   {
    Dn=3;
    DF[0]=(DFF[6]-D)/(DFF[6]-DFF[5]);
   }
  }
   
   else if(D<=DFF[0])
   {
    Dn=-2;
    DF[0]=1;
   }
   else if(D>=DFF[6])
   {
    Dn=3;
    DF[0]=0;
   }

  DF[1]=1-DF[0];

   /*使用误差范围优化后的规则表rule_D[7][7]*/ 
   /*输出值使用13个隶属函数,中心值由UFF[7]指定*/ 
   /*一般都是四个规则有效*/ 
  Un[0]=rule_D[Pn-1+3][Dn-1+3]; 
  Un[1]=rule_D[Pn+3][Dn-1+3]; 
  Un[2]=rule_D[Pn-1+3][Dn+3]; 
  Un[3]=rule_D[Pn+3][Dn+3]; 
   
   if(PF[0]<=DF[0])    //求小   
    UF[0]=PF[0];
   else
    UF[0]=DF[0];
   if(PF[1]<=DF[0])
    UF[1]=PF[1];
   else
    UF[1]=DF[0];
   if(PF[0]<=DF[1])
    UF[2]=PF[0];
   else
    UF[2]=DF[1];
   if(PF[1]<=DF[1])
    UF[3]=PF[1];
   else
    UF[3]=DF[1];
  /*同隶属函数输出语言值求大*/ 
   if(Un[0]==Un[1])
   {
    if(UF[0]>UF[1])
    UF[1]=0;
    else
    UF[0]=0;
   }
   if(Un[0]==Un[2])
   {
    if(UF[0]>UF[2])
    UF[2]=0;
    else
    UF[0]=0;
   }
   if(Un[0]==Un[3])
   {
    if(UF[0]>UF[3])
    UF[3]=0;
    else
    UF[0]=0;
   }
   if(Un[1]==Un[2])
   {
    if(UF[1]>UF[2])
    UF[2]=0;
    else
    UF[1]=0;
   }
   if(Un[1]==Un[3])
   {
    if(UF[1]>UF[3])
    UF[3]=0;
    else
    UF[1]=0;
   } 
   if(Un[2]==Un[3])
   {
    if(UF[2]>UF[3])
    UF[3]=0;
    else
    UF[2]=0;
   }
   t1=UF[0]*DUFF[Un[0]];
   t2=UF[1]*DUFF[Un[1]];
   t3=UF[2]*DUFF[Un[2]];
   t4=UF[3]*DUFF[Un[3]];
   temp1=t1+t2+t3+t4;
   temp2=UF[0]+UF[1]+UF[2]+UF[3];//模糊量输出
   U=temp1/temp2;
   return U;
}