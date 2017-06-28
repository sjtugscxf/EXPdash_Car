/*
Arthor : Qian Qiyang (KisaragiAyanoo@twitter)
Date : 2015/12/01
License : MIT
*/

#include "includes.h"


// ====== Variables ======
// ---- Global ----
u8 cam_buffer_safe[BLACK_WIDTH*2];
u8 cam_buffer[IMG_ROWS][IMG_COLS+BLACK_WIDTH];   //64*155，把黑的部分舍去是59*128
Road road_B[ROAD_SIZE];//由近及远存放
float mid_ave;//road中点加权后的值
//float  weight[10] = {1,1,1.118, 1.454, 2.296, 3.744, 5.304, 6.000, 5.304, 3.744}; //2.296};//, 1.454};//上一次的权值
//float weight[10] = {1.04,1.14,1.41,2.01,3.03,4.35,5.52,6,5.52,4.35};//待测试
float weight[4][10] ={ {0,0,0,0,0,0,0,0,0,0},
                        {1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29},
                        {1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29},
                        {1.00,1.03,1.14,1.54,2.56,               4.29,6.16,7.00,6.16,4.29}
                        //{1.118, 1.454, 2.296, 3.744, 5.304,      6.000, 5.304, 3.744, 2.296, 1.454}
                      };

int valid_row=0;//与有效行相关，未有效识别
int valid_row_thr=10;//有效行阈值
u8 car_state=0;//智能车状态标志 0：停止  1：测试舵机  2：正常巡线
u8 remote_state = 0;//远程控制
u8 road_state = 0;//前方道路状态 1、直道   2、弯道  3、环岛  4、障碍
                  //2 状态下减速
int road_flag=0;
//环岛处理========================================
//以下出岛时需要全部置零
bool roundabout_flag=0;//0-未判断 1-已判断
int roundabout_choice=0;//0-未选择 1-左 2-右 3-左右皆可
int huandaochukou=0;
//========================================================
float motor_L=MIN_SPEED;
float motor_R=MIN_SPEED;

//OLED调参
int debug_speed=0;
PIDInfo debug_dir;
int margin=30;
circle C;
int c1=15, c2=10, c3=5;

//=====================
int CAM_HOLE_ROW=27; //用来向两边扫描检测黑洞·环岛的cam_buffer行位置
int ROAD_OBST_ROW=10; //用来检测障碍物的road_B行位置//不能太远，也不能太近
int OBSTACLE_THR=40;  //有障碍物时赛道宽度阈值


// ---- Local ----
u8 cam_row = 0, img_row = 0;

// ====== 

void Cam_Algorithm(){
  static u8 img_row_used = 0;
  while(img_row_used ==  img_row%IMG_ROWS); // wait for a new row received
  
  // -- Handle the row --
  
  
  
  
  //  -- The row is used --
  img_row_used++;
  img_row_used%=IMG_ROWS;
}




float constrain(float lowerBoundary, float upperBoundary, float input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

int constrainInt(int lowerBoundary, int upperBoundary, int input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

void Cam_B_Init()//初始化Cam_B
{
  int i=0;
  for(i=0;i<ROAD_SIZE;i++)
  {
    road_B[i].left=WID/2;
    road_B[i].right=WID/2+2;
    road_B[i].mid=WID/2+1;
  }
  mid_ave=WID/2+1;
  //以下为road->mid加权值weight的初始化，由近到远
  //方案一：分段函数
  /*for(i=0;i<3;i++)
  {  
    weight[i]=1;
  }
  for(i=3;i<7;i++)
  {  
    weight[i]=2;
  }
  for(i=7;i<10;i++)
  {
    weight[i]=1;
  }*/
  
  //方案二：遵从正态分布，最高值在weight[MaxWeight_index]，在头文件定义相关参数

  
  // design 3 ——>声明与定义放一块，global
//  weight = {1.118, 1.454, 2.296, 3.744, 5.304, 6.000, 5.304, 3.744, 2.296, 1.454};
  
}

    //出界保护  //无效……
  /*  static bool flag_protect=0;
    if((cam_buffer[56][60]+cam_buffer[57][60]+cam_buffer[58][60])<3*thr||flag_protect==1){                       //保护机制，貌似不太好用cam_buffer[45][64]<70 && cam_buffer[50][64]<70 &&
      Servo_Output(0);
      PWM(0, 0, &L, &R); 
      flag_protect=1;
    }*/

  //第一次进化版巡线程序
void Cam_B(){
      static int island=0;
    //#define WID 120//  //以后摄像头需要调整，与车轴平行，这样的话是128//改成全局
    //#define thr 70  //改成全局
    //获取road->mid

      for(int j=0;j<ROAD_SIZE;j++){
        if(cam_buffer[0][j]>thr){
          road_B[0].left=j;
          break;
        }
      } 
      for(int j=128;j>=0;j--){
        if(cam_buffer[0][j]>thr){
          road_B[0].right=j;
          break;
        }
      }
//检测道路最远端第0行
      
      int huandaochukou=0;
     
      if((road_B[0].left+road_B[0].right)/2>=64){
        huandaochukou=1;
      }
      else{
        huandaochukou=0;
      }
      
    for(int j=0;j<ROAD_SIZE;j++)//从下向上扫描
    {
      int i;
      //left
      for (i = road_B[j].mid; i > 0; i--){
        if (cam_buffer[60-2*j][i] < thr)
          break;
        }
      road_B[j].left = i;
      //right
      for (i = road_B[j].mid; i < WID; i++){
        if (cam_buffer[60-2*j][i] < thr)
          break;
        }
      road_B[j].right = i;
      //mid
      road_B[j].mid = (road_B[j].left + road_B[j].right)/2;//分别计算并存储25行的mid
      //store
      if(j<(ROAD_SIZE-1))
        road_B[j+1].mid=road_B[j].mid;//后一行从前一行中点开始扫描
    }
    
    
    
     //区分环岛与十字的延长线法如下：
   if(roundabout_flag==0){     //在出弯的时候清零
      
      int cntl=0,cntr=0,tmpl1=0,tmpl2=0,tmpr1=0,tmpr2=0,cntrow=0,tmprow=0;
      double suml=0,sumr=0;
      //int thr_tmp=0;//未用
      bool flag_left_jump=0,flag_right_jump=0,flag_row_jump=0;
      int jump[3][2];//存拐点坐标 0左 1右 2上 0-x 1-y
      
      
      
      for(cntl=0;cntl<ROAD_SIZE-1;cntl++){
        if(flag_left_jump==0){
          tmpl2=tmpl1;
          tmpl1=road_B[cntl+1].left-road_B[cntl].left;
          if(tmpl1<0&&tmpl2>0) {
            flag_left_jump=1;
            jump[0][0]=road_B[cntl].left;
            jump[0][1]=60-CAM_STEP*cntl;
          }
          suml+=tmpl1;
        }
      }
      
      for (cntr=0;cntr<ROAD_SIZE-1;cntr++){
        if(flag_right_jump==0){
          tmpr2=tmpr1;
          tmpr1=road_B[cntr+1].right-road_B[cntr].right;
          if(tmpr1>0&&tmpr2<0) {
            flag_right_jump=1;
            jump[1][0]=road_B[cntr].right;
            jump[1][1]=60-CAM_STEP*cntr;
          }
          sumr+=tmpr1;
        } 
      }

      if(flag_left_jump==1&&flag_right_jump==1){//1.若检测到两个拐点，则判断是环岛还是十字，如下：
        //suml  cnt*CAM_STEP
        int cnt_black_row=0;//标志
        int left_now,right_now;//存当前行扫描边界
        int cnt=(cntr>cntl)?cntr:cntl;
        for(int j=cnt;(60-CAM_STEP*j)>BLACK_HEIGHT;j++){//BLACK_HEIGHT要改，根据赛道宽度与OLED行数的对应关系
          left_now=jump[0][0]+suml*j/(cnt*CAM_STEP);
          right_now=jump[1][0]+sumr*j/(cnt*CAM_STEP);
          int cnt_black=0;
          for (int i = left_now; i < right_now; i++){
            if (cam_buffer[60-CAM_STEP*j][i] < thr)
              cnt_black++;
            if(cnt_black>(right_now-left_now)*0.8) cnt_black_row++;
          }
          if(cnt_black_row>=3){
            road_state=3;                       //完成环岛判断
            break;
          }
        }
      }
      else if (flag_left_jump==1||flag_right_jump==1){//2.如果只有左边或右边有拐点，则应当判断何时要出弯
        
        // 先找水平方向上的拐点
        int max_y=0,i=0,j=0;
        bool whiteline=1;
        max_y=(jump[1][1]>jump[0][1])?jump[1][1]:jump[0][1];
        for (i=0;CAM_STEP*i<max_y;i++){
          for (j=0;j<WID;j++){
            
            if (cam_buffer[CAM_STEP*i][j]<thr){ 
               whiteline=0;break;
            }
        
          }
          if (whiteline) 
          {jump[2][0]=j;jump[2][1]=CAM_STEP*i;break;}
      
        }
      }
        
        float slope;
        if (flag_left_jump){
          slope=(float)(jump[2][1]-jump[0][1])/(jump[2][0]-jump[0][0]);
          
          slope=(-1.0)/slope;
          
          for (int k=(slope*(WID-jump[2][0])+jump[2][1])/CAM_STEP;k<(jump[2][1])/3;k++)
          {
            road_B[k].right = (-1.0)/slope*(jump[2][1]-k*CAM_STEP) + jump[2][0];
            road_B[k].mid = (road_B[k].left + road_B[k].right)/2;
          }
        }
        else if (flag_right_jump){
          slope=(float)(jump[2][1]-jump[1][1])/(jump[2][0]-jump[1][0]);
          slope=(-1.0)/slope;
          
          for (int k=(slope*(WID-jump[2][0])+jump[2][1])/CAM_STEP;k<(jump[2][1])/3;k++)
          {
            road_B[k].left = (-1.0)/slope*(jump[2][1]-k*CAM_STEP) + jump[2][0];
            road_B[k].mid = (road_B[k].left + road_B[k].right)/2;
          }
        }  
    }
    else 
    {
      roundabout_flag=0;
    }
    
    
       //===========================区分前方道路类型//需要设置一个优先级！！！
    
    

    static int mid_ave3;
    bool flag_valid_row=0;
    for(int i_valid=0;i_valid<(ROAD_SIZE-3) && flag_valid_row==0;i_valid++)
    {
      mid_ave3 = (road_B[i_valid].mid + road_B[i_valid+1].mid + road_B[i_valid+2].mid)/3;
      if(mid_ave3<margin||mid_ave3>(CAM_WID-margin))
      {
        flag_valid_row=1;
        valid_row=i_valid;
      }
      else valid_row=ROAD_SIZE-3;
    }
    if(valid_row<valid_row_thr)
      road_state=2;//弯道
    else road_state=1;//直道
    
    
    
    
    
/*    if (island==0)
    {
        int whitep=0;
        
        for (int i=0;i<WID;i++)
        {
          if (cam_buffer[3][i]>thr) whitep++;
        }
        if (whitep>WID*0.8&&cam_buffer[3][WID/2]<thr) 
        {
         island=1;
         road_state=3;
        }
    }
*/    
      
     
    int road_check=1;
    for (int i=0;i<8;i++)
    {
      int temp=0,last_temp=0,cnt=0;
      for ( int j=0;j<WID;j++)
      {
        temp=cam_buffer[62-i*CAM_STEP][j]-thr;
        if (temp*last_temp<0) cnt++;
        last_temp=temp;
      }
      if (cnt!=2||cam_buffer[62-i*CAM_STEP][WID/2]>thr) 
      {
        road_check=0;break;
      }
    }
    if (road_check) road_state=3;
       
       

    
    //================================对十行mid加权：
    float weight_sum=0;
    for(int j=0;j<10;j++)
    {
      mid_ave += road_B[j].mid * weight[road_state][j];
      weight_sum += weight[road_state][j];
    }
    mid_ave/=weight_sum;
    
    
    
    //舵机的PD控制
    static float err;
    static float last_err;
    
    err = mid_ave  - WID / 2;
    int dir;
    dir = (Dir_Kp+debug_dir.kp) * err + (Dir_Kd+debug_dir.kd) * (err-last_err);     //舵机转向  //参数: (7,3)->(8,3.5)
    if(dir>0)
      dir*=1.2;//修正舵机左右不对称的问题//不可删
    last_err = err;
    
    dir=constrainInt(-230,230,dir);
    if(car_state!=0)
    {
//    {
        if (island)
       {
          
           if( cam_buffer[45][WID/2]<thr) Servo_Output(-200);
           else  
           {
             Servo_Output(dir);
             island=0;
           }
      }
        else Servo_Output(dir);
    }
      
        
//    else 
//      Servo_Output(0);
    
    
    
    
    //PWM以dir为参考，前期分级控制弯道速度；中期分段线性控速；后期找到合适参数的时候，再进行拟合——PWM关于dir的函数
    static float motor_L=MIN_SPEED;
    static float motor_R=MIN_SPEED;
    float max_speed=MAX_SPEED+debug_speed;
    int range=max_speed-MIN_SPEED;//速度范围大小
    //正常前进
    if(car_state==2 ){
      //PWM闭环控制尝试：分段线性控速
      
      
      if (valid_row>valid_row_thr) motor_L=motor_R=max_speed+5; //此处设为maxspeed有可能加速不够激进
      
      else if (valid_row<valid_row_thr&&valid_row>=valid_row_thr-5&&(tacho1-tacho0)/2>MIN_SPEED) // tacho1_thr 为目标进弯速度 此处设置下界是为了防止过减速，如果去掉则会让
                                                                                        //   tacho1_thr成为除了高速段的速度最大值 具体下界为多少还是需要调试
        motor_L=motor_R=0;
      
      else //进入下一层的dir-speed的控制
      {
        if(abs(dir)<50 ){//&& valid_row>valid_row_thr
          motor_L=motor_R=max_speed;
        }
        else if(abs(dir)<95){
          motor_L=motor_R=max_speed-0.33*range*(abs(dir)-50)/45;
          if(dir>0) motor_R-=2;//右转
          else motor_L-=2;
        }
        else if(abs(dir)<185){
          motor_L=motor_R=max_speed-0.33*range-0.33*range*(abs(dir)-95)/90;
          if(dir>0) motor_R-=3;//右转
          else motor_L-=3;
        }
        else if(abs(dir)<230){
          motor_L=motor_R=max_speed-0.66*range-0.33*range*(abs(dir)-185)/45;
          if(dir>0) motor_R-=4;//右转
          else motor_L-=4;
        }
        else{
          motor_L=motor_R=MIN_SPEED;
        }
      }
    PWM(motor_L, motor_R, &L, &R);
      //后轮速度
 //    PWM(20,20, &L, &R);
    }
   else
     PWM(0, 0, &L, &R);
}
    
    //方案二//不适合
//    C=getR(road_B[c1].mid,20-c1,road_B[c2].mid,20-c2,road_B[c3].mid,20-c3);
    


// ====== Basic Drivers ======

void PORTC_IRQHandler(){
  if((PORTC->ISFR)&PORT_ISFR_ISF(1 << 8)){  //CS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 8);
    
    if(img_row < IMG_ROWS && cam_row % IMG_STEP == 0 ){
      DMA0->TCD[0].DADDR = (u32)&cam_buffer[img_row][0];
      DMA0->ERQ |= DMA_ERQ_ERQ0_MASK; //Enable DMA0
      ADC0->SC1[0] |= ADC_SC1_ADCH(4); //Restart ADC
      DMA0->TCD[0].CSR |= DMA_CSR_START_MASK; //Start
	}
	cam_row++;
  }
  else if(PORTC->ISFR&PORT_ISFR_ISF(1 << 9)){   //VS
    PORTC->ISFR |= PORT_ISFR_ISF(1 << 9);
    
    cam_row = img_row = 0;
    
  }
}

void DMA0_IRQHandler(){
  DMA0->CINT &= ~DMA_CINT_CINT(7); //Clear DMA0 Interrupt Flag
  
  img_row++; 
}

void Cam_Init(){
  
  // --- IO ---
  
  PORTC->PCR[8] |= PORT_PCR_MUX(1); //cs
  PORTC->PCR[9] |= PORT_PCR_MUX(1); //vs
  PORTC->PCR[11] |= PORT_PCR_MUX(1);    //oe
  PTC->PDDR &=~(3<<8);
  PTC->PDDR &=~(1<<11);
  PORTC->PCR[8] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(10);	//PULLUP | falling edge
  PORTC->PCR[9] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK | PORT_PCR_IRQC(9);  // rising edge
  PORTC->PCR[11] |= PORT_PCR_PE_MASK | PORT_PCR_PS_MASK ;
  
  NVIC_EnableIRQ(PORTC_IRQn);
  NVIC_SetPriority(PORTC_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
  
  // --- AD ---
  
  /*
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;  //ADC1 Clock Enable
  ADC0->CFG1 |= 0
             //|ADC_CFG1_ADLPC_MASK
             | ADC_CFG1_ADICLK(1)
             | ADC_CFG1_MODE(0);     // 8 bits
             //| ADC_CFG1_ADIV(0);
  ADC0->CFG2 |= //ADC_CFG2_ADHSC_MASK |
                ADC_CFG2_MUXSEL_MASK |  // b
                ADC_CFG2_ADACKEN_MASK; 
  
  ADC0->SC1[0]&=~ADC_SC1_AIEN_MASK;//disenble interrupt
  
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA
  
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; // continuous
  
  //PORTC->PCR[2]|=PORT_PCR_MUX(0);//adc1-4a
  
  ADC0->SC1[0] |= ADC_SC1_ADCH(4);
  */
  
  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; //ADC1 Clock Enable
  ADC0->SC1[0] &= ~ADC_SC1_AIEN_MASK; //ADC1A
  ADC0->SC1[0] = 0x00000000; //Clear
  ADC0->SC1[0] |= ADC_SC1_ADCH(4); //ADC1_5->Input, Single Pin, No interrupt
  ADC0->SC1[1] &= ~ADC_SC1_AIEN_MASK; //ADC1B
  ADC0->SC1[1] |= ADC_SC1_ADCH(4); //ADC1_5b
  ADC0->SC2 &= 0x00000000; //Clear all.
  ADC0->SC2 |= ADC_SC2_DMAEN_MASK; //DMA, SoftWare
  ADC0->SC3 &= (~ADC_SC3_AVGE_MASK&~ADC_SC3_AVGS_MASK); //hardware average disabled
  ADC0->SC3 |= ADC_SC3_ADCO_MASK; //Continuous conversion enable
  ADC0->CFG1|=ADC_CFG1_ADICLK(1)|ADC_CFG1_MODE(0)|ADC_CFG1_ADIV(0);//InputClk, ShortTime, 8bits, Bus
  ADC0->CFG2 |= ADC_CFG2_MUXSEL_MASK; //ADC1  b
  ADC0->CFG2 |= ADC_CFG2_ADACKEN_MASK; //OutputClock
    
  // --- DMA ---
  
  SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK; //DMAMUX Clock Enable
  SIM->SCGC7 |= SIM_SCGC7_DMA_MASK; //DMA Clock Enable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_SOURCE(40); //DMA0->No.40 request, ADC0
  DMA0->TCD[0].SADDR = (uint32_t) & (ADC0->R[0]); //Source Address 0x400B_B010h
  DMA0->TCD[0].SOFF = 0; //Source Fixed
  DMA0->TCD[0].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0); //Source 8 bits, Aim 8 bits
  DMA0->TCD[0].NBYTES_MLNO = DMA_NBYTES_MLNO_NBYTES(1); //one byte each
  DMA0->TCD[0].SLAST = 0; //Last Source fixed
  DMA0->TCD[0].DADDR = (u32)cam_buffer;
  DMA0->TCD[0].DOFF = 1;
  DMA0->TCD[0].CITER_ELINKNO = DMA_CITER_ELINKNO_CITER(IMG_COLS);
  DMA0->TCD[0].DLAST_SGA = 0;
  DMA0->TCD[0].BITER_ELINKNO = DMA_BITER_ELINKNO_BITER(IMG_COLS);
  DMA0->TCD[0].CSR = 0x00000000; //Clear
  DMA0->TCD[0].CSR |= DMA_CSR_DREQ_MASK; //Auto Clear
  DMA0->TCD[0].CSR |= DMA_CSR_INTMAJOR_MASK; //Enable Major Loop Int
  DMA0->INT |= DMA_INT_INT0_MASK; //Open Interrupt
  //DMA->ERQ&=~DMA_ERQ_ERQ0_MASK;//Clear Disable
  DMAMUX->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK; //Enable
  
  NVIC_EnableIRQ(DMA0_IRQn);
  NVIC_SetPriority(DMA0_IRQn, NVIC_EncodePriority(NVIC_GROUP, 1, 2));
  
}