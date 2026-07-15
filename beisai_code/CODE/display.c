#include "basefile.h"


//发送的数据 两个通道
float channel[6];
float Data[5] = {0};
//参数控制
uint8 row = 0,page00 = 0,page01 = 0,page02 = 0;
uint8 add_flag = 0,sub_flag = 0;
int8 precision = 0;//调节控制精度
int8 back = 0;
//float转化为4个字节数据
uint8 byte1[4]={0},byte2[4]={0},byte3[4]={0},byte4[4]={0},byte5[4]={0},byte6[4]={0};
uint8 error[3]={6,6,6};
//帧尾
uint8 tail[4]={0x00, 0x00, 0x80, 0x7f};
//串口开关
int8 chuankou_button = 0;
int8 bizhang_button = 0;
//拨码开关
int8 page_flag = 0,page_olde = 0;
//
int8 old_state = 0,now_state = 0;
//
int8 change_ring = 0;


typedef union
{
    float fdata;
    unsigned long ldata;
} FloatLongType;


//--------------------------------------------------------chuan_kou-------------------------------------------------------------/
//通道数据设置
void Channel_Set(void)
{
	channel[0] = channel1;
	channel[1] = channel2;
	channel[2] = channel3;
	channel[3] = channel4;
	channel[4] = channel5;
	channel[5] = channel6;
}

//-----------------------------------------------------------VOFA+--------------------------------------------------------------/
//将浮点数f转化为4个字节数据存放在byte[4]中
void Float_to_Byte(float f,unsigned char byte[])
{
    FloatLongType fl;
    fl.fdata=f;
    byte[0]=(unsigned char)fl.ldata;
    byte[1]=(unsigned char)(fl.ldata>>8);
    byte[2]=(unsigned char)(fl.ldata>>16);
    byte[3]=(unsigned char)(fl.ldata>>24);
}

//数据发送
void Picture_Datashow(uint8 symbol)
{
	switch(symbol)
	{
		case 6:Float_to_Byte(channel[5],byte6);uart_putbuff(UART_4,&byte6[0],4);
		case 5:Float_to_Byte(channel[4],byte5);uart_putbuff(UART_4,&byte5[0],4);
		case 4:Float_to_Byte(channel[3],byte4);uart_putbuff(UART_4,&byte4[0],4);
		case 3:Float_to_Byte(channel[2],byte3);uart_putbuff(UART_4,&byte3[0],4);
		case 2:Float_to_Byte(channel[1],byte2);uart_putbuff(UART_4,&byte2[0],4);
		case 1:Float_to_Byte(channel[0],byte1);uart_putbuff(UART_4,&byte1[0],4),uart_putbuff(UART_4,&tail[0],4);break;
		case 0:break;
		default:uart_putbuff(UART_4,&error[0],3);break;
	}
}

//-------------------------------------------------开启串口---------------------------------------------------------/
void ChuanKou_Start(void)
{
	if(chuankou_button == ON)
	{
		Picture_Datashow(channelsum);
	}
}
//----------------------------------------------------屏幕---------------------------------------------------------/
void screen_update(void)
{
	if(old_state != now_state)
	{
		ips114_clear(WHITE);
		now_state = 0;
	}
	
}

//--------------------------------------------------------------------参数页面---------------------------------------------------------------------------------/
void PageO1(void)
{
	ips114_showstr(28*8,row,"<-");
	
	ips114_showstr(14*8,0,"|");
	ips114_showstr(14*8,1,"|");
	ips114_showstr(14*8,2,"|");
	ips114_showstr(14*8,3,"|");
	ips114_showstr(14*8,4,"|");
	ips114_showstr(14*8,5,"|");
	ips114_showstr(14*8,6,"|");
	ips114_showstr(14*8,7,"|");
	ips114_showstr(14*8,8,"|");	

	ips114_showstr(0*8,2,"--------------");ips114_showstr(15*8,2,"------------");
	ips114_showstr(0*8,5,"--------------");ips114_showstr(15*8,5,"------------");
	
	ips114_showstr(0*8,0,"AD_sL");ips114_showuint16(5*8,0,MYADC[0][0]);			ips114_showstr(15*8,0,"AD_sR");ips114_showuint16(20*8,0,MYADC[1][0]);
	ips114_showstr(0*8,1,"AD_mL");ips114_showuint16(5*8,1,MYADC[2][0]);			ips114_showstr(15*8,1,"AD_mR");ips114_showuint16(20*8,1,MYADC[3][0]);

	ips114_showstr(0*8,3,"AD_MsL");ips114_showuint16(6*8,3,ADC_Max[0]);			ips114_showstr(15*8,3,"AD_MsR");ips114_showuint16(21*8,3,ADC_Max[1]);
	ips114_showstr(0*8,4,"AD_MmL");ips114_showuint16(6*8,4,ADC_Max[2]);			ips114_showstr(15*8,4,"AD_MmR");ips114_showuint16(21*8,4,ADC_Max[3]);

	ips114_showstr(0*8,6,"AD_M");ips114_showint16(6*8,6,MYADC[4][0]);			ips114_showstr(15*8,6,"dis");ips114_showint16(21*8,6,wave_dis);
	ips114_showstr(0*8,7,"dir_err");ips114_showint16(7*8,7,inductance);			ips114_showstr(15*8,7,"agl_spe");ips114_showint16(22*8,7,angle_x[0]);
}

void PageO2(void)
{
	ips114_showstr(28*8,row,"<-");

	ips114_showstr(14*8,0,"|");
	ips114_showstr(14*8,1,"|");
	ips114_showstr(14*8,2,"|");
	ips114_showstr(14*8,3,"|");
	ips114_showstr(14*8,4,"|");
	ips114_showstr(14*8,5,"|");
	ips114_showstr(14*8,6,"|");
	ips114_showstr(14*8,7,"|");
	ips114_showstr(14*8,8,"|");
	
	ips114_showstr(0*8,2,"--------------");ips114_showstr(15*8,2,"-------------");
	ips114_showstr(0*8,5,"--------------");ips114_showstr(15*8,5,"-------------");

	ips114_showstr(0*8,0,"flag1");ips114_showint8(6*8,0,prering_flag);			ips114_showstr(15*8,0,"flag2");ips114_showint8(21*8,0,ringdir_flag);
	ips114_showstr(0*8,1,"flag3");ips114_showint8(7*8,1,staring_flag);			ips114_showstr(15*8,1,"flag4");ips114_showint8(22*8,1,inring_flag);		
}

void PageO3(void)
{
	ips114_showstr(28*8,row,"<-");
	
	ips114_showstr(14*8,0,"|");
	ips114_showstr(14*8,1,"|");
	ips114_showstr(14*8,2,"|");
	ips114_showstr(14*8,3,"|");
	ips114_showstr(14*8,4,"|");
	ips114_showstr(14*8,5,"|");
	ips114_showstr(14*8,6,"|");
	ips114_showstr(14*8,7,"|");
	ips114_showstr(14*8,8,"|");
	
	ips114_showstr(0*8,2,"--------------");ips114_showstr(15*8,2,"-------------");
	ips114_showstr(0*8,5,"--------------");ips114_showstr(15*8,5,"-------------");

	ips114_showstr(0*8,0,"S");ips114_showfloat(1*8,0,S,2,2);					ips114_showstr(15*8,0,"V");ips114_showuint16(16*8,0,voltage);
	ips114_showstr(0*8,1,"Lspeed");ips114_showfloat(7*8,1,speed_left,2,1);		ips114_showstr(15*8,1,"Rspeed");ips114_showfloat(22*8,1,speed_right,2,1);	
}

//-----------------------------------------------------------------调参页面-------------------------------------------------------------------------------------/

void Page1(void)
{
	ips114_showstr(28*8,row,"<-");
	ips114_showstr(0*8,0,"precision");											ips114_showint8(15*8,0,precision);
	ips114_showstr(0,1,"dir_kp");												ips114_showint16(15*8,1,direction_Kp);
	ips114_showstr(0,2,"dir_ki");												ips114_showint16(15*8,2,direction_Ki);
	ips114_showstr(0,3,"dir_kd");												ips114_showint16(15*8,3,direction_Kd);
	ips114_showstr(0,4,"agl_Kd");												ips114_showint16(15*8,4,dirangle_Kd);	
	ips114_showstr(0,5,"err_pa");												ips114_showfloat(15*8,5,Pa,2,1);
	ips114_showstr(0,6,"err_pb");												ips114_showfloat(15*8,6,Pb,2,1);
	ips114_showstr(0,7,"err_pc");												ips114_showfloat(15*8,7,Pc,2,1);

}

void Page2(void)
{
	ips114_showstr(28*8,row,"<-");
	ips114_showstr(0*8,0,"precision");											ips114_showint8(15*8,0,precision);
	ips114_showstr(0,1,"spe_lp");												ips114_showint16(15*8,1,left_Kp);
	ips114_showstr(0,2,"spe_li");												ips114_showint16(15*8,2,left_Ki);
	ips114_showstr(0,3,"spe_rp");												ips114_showint16(15*8,3,right_Kp);
	ips114_showstr(0,4,"spe_ri");												ips114_showint16(15*8,4,right_Ki);
//	ips114_showstr(0,5,"---------------------------");
	ips114_showstr(0,5,"Ka------------");										ips114_showint16(15*8,5,Ka);
	ips114_showstr(0,6,"spe_limit");											ips114_showint16(15*8,6,speed_out_limit);
	ips114_showstr(0,7,"dir_limit");											ips114_showint16(15*8,7,direction_out_limit);	
}
void Page3(void)
{
	ips114_showstr(28*8,row,"<-");
	ips114_showstr(0*8,0,"precision");											ips114_showint8(15*8,0,precision);
	ips114_showstr(0,1,"agl1_limit");											ips114_showint16(15*8,1,agl1_limit);
	ips114_showstr(0,2,"agl2_limit");											ips114_showint16(15*8,2,agl2_limit);
	ips114_showstr(0,3,"agl1");													ips114_showint16(15*8,3,agl1);
	ips114_showstr(0,4,"agl2");													ips114_showint16(15*8,4,agl2);
	ips114_showstr(0,5,"agl2_Kp");												ips114_showint16(15*8,5,agl2_Kp);
	ips114_showstr(0,6,"agl2_Kd");												ips114_showint16(15*8,6,agl2_Kd);
	ips114_showstr(0,7,"error_limit");											ips114_showint16(15*8,7,error_limit);
}
void Page4(void)
{
	ips114_showstr(28*8,row,"<-");
	ips114_showstr(0,0,"Kp_start");												ips114_showint16(15*8,0,Kp_start);
	ips114_showstr(0,1,"Kp_out");												ips114_showint16(15*8,1,Kp_out);	
	ips114_showstr(0,2,"Kd0_start");											ips114_showint16(15*8,2,Kd0_start);
	ips114_showstr(0,3,"Kd0_out");												ips114_showint16(15*8,3,Kd0_out);
	ips114_showstr(0,4,"Kd1_start");											ips114_showint16(15*8,4,Kd1_start);
	ips114_showstr(0,5,"Kd1_out");												ips114_showint16(15*8,5,Kd1_out);
	ips114_showstr(0,6,"ADC_Mlimt");											ips114_showint16(15*8,6,ADC_Mlimt);
//	ips114_showstr(0,7,"error_limit");											ips114_showint16(15*8,7,error_limit);
}
void Page5(void)
{
	ips114_showstr(28*8,row,"<-");
	ips114_showstr(0,0,"bingji_button");										ips114_showint16(15*8,0,bingji_button);
	ips114_showstr(0,1,"Kb");													ips114_showint16(15*8,1,Kb);	
	ips114_showstr(0,2,"bingji_limit");											ips114_showint16(15*8,2,bingji_limit);
	ips114_showstr(0,3,"sub_limit");											ips114_showint16(15*8,3,sub_limit);	
}
//-------------------------------------------------------------模式页面-----------------------------------------------------------------------------/
void Page_mode1(void)
{	
	if((bizhang_button == ON)&&(back == 1))
	{
		ips114_showstr(28*8,row,"<-");
		ips114_showstr(0,0,"bz_mode");												ips114_showint8(15*8,0,bz_mode);
		ips114_showstr(0,1,"diff_dir");												ips114_showint8(15*8,1,diff_dir);
		ips114_showstr(0,2,"bz_dis");												ips114_showfloat(15*8,2,bz_dis,2,1);
		ips114_showstr(0,3,"bz_cisu");												ips114_showfloat(15*8,3,bz_ci,2,1);
		ips114_showstr(0,4,"wave_dis");												ips114_showfloat(15*8,4,wave_limit,2,1);
		ips114_showstr(0,5,"dis_limit");											ips114_showint8(15*8,5,dis_limit);
		ips114_showstr(0,6,"run_dis");												ips114_showfloat(15*8,6,run_dis,1,1);

	}
	else if((change_ring == 1)&&(back == 2))
	{
		ips114_showstr(28*8,row,"<-");
		ips114_showstr(0,0,"inring_count");											ips114_showint8(15*8,0,inring_count);
	
	}
	else if((zuizhi_button == 1)&&(back == 3))
	{
		ips114_showstr(28*8,row,"<-");
		ips114_showstr(0,0,"AD_MsL");												ips114_showuint16(15*8,0,ADC_Max[0]);
		ips114_showstr(0,1,"AD_MsR");												ips114_showuint16(15*8,1,ADC_Max[1]);
		ips114_showstr(0,2,"AD_MmL");												ips114_showuint16(15*8,2,ADC_Max[2]);	
		ips114_showstr(0,3,"AD_MmR");												ips114_showuint16(15*8,3,ADC_Max[3]);		
	}
	else if((fold_button == 1)&&(back == 4))
	{
		ips114_showstr(28*8,row,"<-");
		ips114_showstr(0,0,"speed_downtime");										ips114_showint8(15*8,0,speed_downtime);		
	}
	else if(back == 0)
	{
		ips114_showstr(28*8,row,"<-");
		ips114_showstr(0,0,"car_mode");												ips114_showint8(15*8,0,car_mode);
		ips114_showstr(0,1,"guiyihua_button");										ips114_showint8(15*8,1,guiyihua_button);		
		ips114_showstr(0,2,"change_ring");											ips114_showint8(15*8,2,change_ring);
		ips114_showstr(0,3,"bizhang_button");										ips114_showint8(15*8,3,bizhang_button);
		ips114_showstr(0,4,"fold_button");									    	ips114_showuint8(15*8,4,fold_button);
		ips114_showstr(0,5,"zuizhi_button");										ips114_showint8(15*8,5,zuizhi_button);
		ips114_showstr(0,6,"chuankou_button");									    ips114_showuint8(15*8,6,chuankou_button);
	}
}

//void Page_mode2(void)
//{	
//	
//}
//-------------------------------------------------------------发车页面------------------------------------------------------------------------------/
void Page_start(void)
{
	if(Go > 0)
	{
		if(speed_count < 2)
		{
			ips114_clear(WHITE);			
		}
		else if((speed_count >= 2)&&(speed_count < 100))
		{
			ips114_showint16(13*8,4,speed_count);			
		}
		else if((speed_count >= 100)&&(speed_count < 102))
		{
			ips114_clear(WHITE);			
		}
		else if((speed_count >= 102)&&(speed_count < 150))
		{
			ips114_showstr(13*8,4,"OK");
		}
		else if((speed_count >= 150)&&(speed_count < 152))
		{
			ips114_clear(WHITE);
		}
		else if(speed_count >= 152)
		{
			ips114_showstr(12*8,4,"GO!!!");
		}
	}
	else
	{
		ips114_showstr(28*8,row,"<-");
		ips114_showstr(0*8,0,"Go");													ips114_showuint8(15*8,0,Go);
		ips114_showstr(0*8,1,"speed_target");										ips114_showfloat(15*8,1,speed_target,2,1);
		ips114_showstr(0*8,2,"STOP");												ips114_showint8(15*8,2,distance);

	}
	
}


//void PageO6(void)
//{
//	ips114_showstr(28*8,row,"<-");
//	ips114_showstr(0*8,0,"flag1");ips114_showint8(6*8,0,difficult_flag);			ips114_showstr(15*8,0,"flag2");ips114_showint8(21*8,0,diuxian_flag);
//	ips114_showstr(0*8,1,"flag3");ips114_showint8(7*8,1,outring_flag);			ips114_showstr(15*8,1,"flag4");ips114_showint8(22*8,1,inring_flag);	
//}



void IPS_Show(void)
{
	float num = 0;

	//精度调节
	if(precision == 0)
	{
		num = 1;
	}
	else if(precision <= -1)
	{
		num = 0.1;
	}
	else if(precision >= 1)
	{
		num = 10;
	}

	if(page_flag == 0)
	{
		switch(page00)
		{
			case 0:PageO1();break;
			case 1:PageO2();break;
			case 2:PageO3();break;
//			case 3:PageO4();break;
//			case 4:PageO5();break;
//			case 5:PageO6();break;
			default:break;
		}
	}
	else if(page_flag == 1)
	{
		switch(page01)
		{
			case 0:Page1();break;
			case 1:Page2();break;
			case 2:Page3();break;
			case 3:Page4();break;
			case 4:Page5();break;
//			case 5:break;
			default:break;
		}

		if(page01 == 0)
		{
			if(add_flag == 1)
			{
				switch(row)
				{
					case 0:precision++;break;
					case 1:direction_Kp=direction_Kp+num;break;
					case 2:direction_Ki=direction_Ki+num;break;
					case 3:direction_Kd=direction_Kd+num;break;
					case 4:dirangle_Kd = dirangle_Kd+num;break;
					case 5:Pa=Pa+num;break;
					case 6:Pb=Pb+num;break;
					case 7:Pc=Pc+num;break;
				}
			}
			add_flag = 0;
			if(sub_flag == 1)
			{
				switch(row)
				{
					case 0:precision--;break;
					case 1:direction_Kp=direction_Kp-num;break;
					case 2:direction_Ki=direction_Ki-num;break;
					case 3:direction_Kd=direction_Kd-num;break;
					case 4:dirangle_Kd = dirangle_Kd-num;break;
					case 5:Pa=Pa-num;break;
					case 6:Pb=Pb-num;break;
					case 7:Pc=Pc-num;break;
				}
			}
			sub_flag = 0;
		}
		else if(page01 == 1)
		{
			if(add_flag == 1)
			{
				switch(row)
				{
					case 0:precision++;break;
					case 1:left_Kp=left_Kp+num;break;
					case 2:left_Ki=left_Ki+num;break;
					case 3:right_Kp=right_Kp+num;break;
					case 4:right_Ki=right_Ki+num;break;
					case 5:Ka++;break;
					case 6:speed_out_limit=speed_out_limit+500;break;
					case 7:direction_out_limit=direction_out_limit+500;break;
				}
			}
			add_flag = 0;
			if(sub_flag == 1)
			{
				switch(row)
				{
					case 0:precision--;break;
					case 1:left_Kp=left_Kp-num;break;
					case 2:left_Ki=left_Ki-num;break;
					case 3:right_Kp=right_Kp-num;break;
					case 4:right_Ki=right_Ki-num;break;
					case 5:Ka--;break;
					case 6:speed_out_limit=speed_out_limit-500;break;
					case 7:direction_out_limit=direction_out_limit-500;break;
				}
			}
			sub_flag = 0;
		}
		else if(page01 == 2)
		{
			if(add_flag == 1)
			{
				switch(row)
				{
					case 0:precision++;break;
					case 1:agl1_limit++;break;
					case 2:agl2_limit++;break;
					case 3:agl1++;break;
					case 4:agl2++;break;
					case 5:agl2_Kp=agl2_Kp+10;break;
					case 6:agl2_Kd=agl2_Kd+10;break;
					case 7:error_limit=error_limit+10;break;
				}
			}
			add_flag = 0;
			if(sub_flag == 1)
			{
				switch(row)
				{
					case 0:precision--;break;
					case 1:agl1_limit--;break;
					case 2:agl2_limit--;break;
					case 3:agl1--;break;
					case 4:agl2--;break;
					case 5:agl2_Kp=agl2_Kp-10;break;
					case 6:agl2_Kd=agl2_Kd-10;break;
					case 7:error_limit=error_limit-10;break;
				}
			}
			sub_flag = 0;
		}
		else if(page01 == 3)
		{
			if(add_flag == 1)
			{
				switch(row)
				{
					case 0:Kp_start=Kp_start+10;break;
					case 1:Kp_out=Kp_out+10;break;
					case 2:Kd0_start=Kd0_start+10;break;
					case 3:Kd0_out=Kd0_out+10;break;
					case 4:Kd1_start=Kd1_start+10;break;
					case 5:Kd1_out=Kd1_out+10;break;
					case 6:ADC_Mlimt=ADC_Mlimt+100;break;
//					case 7:error_limit=error_limit+10;break;
				}
			}
			add_flag = 0;
			if(sub_flag == 1)
			{
				switch(row)
				{
					case 0:Kp_start=Kp_start-10;break;
					case 1:Kp_out=Kp_out-10;break;
					case 2:Kd0_start=Kd0_start-10;break;
					case 3:Kd0_out=Kd0_out-10;break;
					case 4:Kd1_start=Kd1_start-10;break;
					case 5:Kd1_out=Kd1_out-10;break;
					case 6:ADC_Mlimt=ADC_Mlimt-100;break;
//					case 7:error_limit=error_limit-10;break;
				}
			}
			sub_flag = 0;
		}
		else if(page01 == 4)
		{
			if(add_flag == 1)
			{
				switch(row)
				{
					case 0:bingji_button=1;break;
					case 1:Kb++;break;
					case 2:bingji_limit=bingji_limit+100;break;
					case 3:sub_limit=sub_limit+100;break;
					case 4:break;
					case 5:break;
					case 6:break;
//					case 7:error_limit=error_limit+10;break;
				}
			}
			add_flag = 0;
			if(sub_flag == 1)
			{
				switch(row)
				{
					case 0:bingji_button=0;break;
					case 1:Kb--;break;
					case 2:bingji_limit=bingji_limit-100;break;
					case 3:sub_limit=sub_limit-100;break;
					case 4:break;
					case 5:break;
					case 6:break;
//					case 7:error_limit=error_limit-10;break;
				}
			}
			sub_flag = 0;
		}
	}
	else if(page_flag == 2)
	{
		switch(page02)
		{
			case 0:Page_mode1();break;
			case 1:break;
			case 2:break;
			case 3:break;
			case 4:break;
			case 5:break;
			case 6:break;
			case 7:break;
			default:break;
		}
		
		if(page02 == 0)
		{
			if(add_flag == 1)
			{					
				if((bizhang_button == ON)&&(back == 1))
				{
					switch(row)
					{
						case 0:bz_mode=1;break;
						case 1:diff_dir=1;break;
						case 2:bz_dis=bz_dis+num;break;
						case 3:bz_ci=2;break;
						case 4:wave_limit++;break;
						case 5:dis_limit++;break;
						case 6:run_dis=run_dis+0.1;break;
					}
				}
				else if((change_ring == 1)&&(back == 2))
				{
					switch(row)
					{
						case 0:inring_count=inring_count+num;break;				
					}
				}
				else if((zuizhi_button == 1)&&(back == 3))
				{
					switch(row)
					{
						case 0:ADC_Max[0] = ADC_Max[0] + 100;break;
						case 1:ADC_Max[1] = ADC_Max[1] + 100;break;
						case 2:ADC_Max[2] = ADC_Max[2] + 100;break;
						case 3:ADC_Max[3] = ADC_Max[3] + 100;break;
					}
				}
				else if((fold_button == 1)&&(back == 4))
				{
					switch(row)
					{
						case 0:speed_downtime++;break;
					}					
				}
				else if(back == 0)
				{
					switch(row)
					{
						case 0:car_mode++;break;
						case 1:guiyihua_button=ON;break;
						case 2:change_ring=1;back=2;break;
						case 3:bizhang_button=ON;back=1;break;
						case 4:fold_button=1;back=4;break;
						case 5:zuizhi_button=ON;back=3;break;
						case 6:chuankou_button=ON;break;
						case 7:break;
					}
				}
			}
			add_flag = 0;
			if(sub_flag == 1)
			{
				if((bizhang_button == ON)&&(back == 1))
				{
					switch(row)
					{
						case 0:bz_mode=-1;break;
						case 1:diff_dir=-1;break;
						case 2:bz_dis=bz_dis-num;break;
						case 3:bz_ci=1;break;
						case 4:wave_limit--;break;
						case 5:dis_limit--;break;
						case 6:run_dis=run_dis-0.1;break;
					}
				}
				else if((change_ring == 1)&&(back == 2))
				{
					switch(row)
					{
						case 0:ADC_Max[0] = ADC_Max[0] - 100;break;
						case 1:ADC_Max[1] = ADC_Max[1] - 100;break;
						case 2:ADC_Max[2] = ADC_Max[2] - 100;break;
						case 3:ADC_Max[3] = ADC_Max[3] - 100;break;
					}
				}
				else if((zuizhi_button == 1)&&(back == 3))
				{
					switch(row)
					{
						case 0:inring_count=inring_count-num;break;				
					}					
				}
				else if((fold_button == 1)&&(back == 4))
				{
					switch(row)
					{
						case 0:speed_downtime--;break;
					}					
				}
				else if(back == 0)
				{
					switch(row)
					{
						case 0:car_mode--;break;
						case 1:guiyihua_button=OFF;break;
						case 2:change_ring=0;back=0;break;
						case 3:bizhang_button=OFF;back=0;break;
						case 4:fold_button=0;back=0;break;
						case 5:zuizhi_button=OFF;back=0;break;
						case 6:chuankou_button=OFF;break;
						case 7:break;
					}
				}
			}
			sub_flag = 0;
		}
	}
	else if(page_flag == 3)
	{

		Page_start();
		
		if(add_flag == 1)
		{
			switch(row)
			{
				case 0:Go = 1;break;
				case 1:speed_target = speed_target+0.1;break;
				case 2:distance++;break;
				default:row=0;break;
			}
		}
		add_flag = 0;
		if(sub_flag == 1)
		{
			switch(row)
			{
				case 0:Go = 0;break;
				case 1:speed_target = speed_target-0.1;break;
				case 2:distance++;break;
				default:row=0;break;
			}
		}
		sub_flag = 0;
	}

	screen_update();	

}

void boma(void)
{
	//页面刷新
	if(page_olde != page_flag)
	{
		page_olde = page_flag;//开始相等，变化更新
		now_state = 1;
		row = 0;
	}
	
	if((Boma1 == 1)&&(Boma2 == 0))
	{
		page_flag = 1;
	}
	else if((Boma1 == 0)&&(Boma2 == 1))
	{
		page_flag = 2;
	}
	else if((Boma1 == 0)&&(Boma2 == 0))
	{
		page_flag = 3;	
	}
	else if((Boma1 == 1)&&(Boma2 == 1))
	{
		page_flag = 0;
	}

}

void Key_Control(void)
{
	if(Key5 == 0)
	{
		Bee(1);
		delay_ms(Time_ms);
		if(Key5 == 0)
		{
			//页面刷新
//			page_update();
			now_state = 1;
			
			if(page_flag == 0)
			{
				page00++;
				page00 = (page00>(cansu_page-1)?0:page00);
			}
			else if(page_flag == 1)
			{
				page01++;
				page01 = (page01>(tiaocan_page2-1)?0:page01);				
			}
			else if(page_flag == 2)
			{
				if(mode_page>1)
				{
					page02++;
					page02 = (page02>(mode_page-1)?0:page02);
				}
				else
				{
					row--;
					row = (row>7?0:row);				
				}
			}
			else if(page_flag == 3)
			{
				row++;
				row = (row>7?0:row);
			}			
		}
		Bee(0);
	}

	if(Key4 == 0)
	{
		Bee(1);
		delay_ms(Time_ms);
		if(Key4 == 0)
		{
			//页面刷新
			now_state = 1;

			if(page_flag == 0)
			{
				page00--;
				page00 = (page00>(cansu_page-1)?0:page00);
			}
			else if((page_flag == 1)||(page_flag == 2))
			{
				row++;
				row = (row>7?0:row);
			}
			else if(page_flag == 3)
			{
				row--;
				row = (row>7?0:row);
			}
		}
		Bee(0);
	}
	if(Key3 == 0)
	{
		Bee(1);
		delay_ms(Time_ms);
		if(Key3 == 0)
		{
			//页面刷新
			now_state = 1;

			if(page_flag == 0)
			{
				row++;
				row = (row>7?0:row);
			}
			else if((page_flag == 1)||(page_flag == 2))
			{	
				add_flag = 1;
			}
			else if(page_flag == 3)
			{
				add_flag = 1;
			}
		}
		Bee(0);
	}
	if(Key2 == 0)
	{
		Bee(1);
		delay_ms(Time_ms);
		if(Key2 == 0)
		{
			//页面刷新
			now_state = 1;

			if(page_flag == 0)
			{
				row--;
				row = (row>7?0:row);
			}
			else if((page_flag == 1)||(page_flag == 2))
			{
				sub_flag = 1;
			}
			else if(page_flag == 3)
			{
				sub_flag = 1;
			}

		}
		Bee(0);
	}
	if(Key1 == 0)
	{
		Bee(1);
		delay_ms(Time_ms);
		if(Key1 == 0)
		{
			//页面刷新
			now_state = 1;
			
			if(stop_flag == 1)
			{
				rest();
			}
			else if(page_flag == 1)
			{
				row--;
				row = (row>7?0:row);	
			}
			else if(page_flag == 2)
			{
				back++;
				back = (back>4?0:back);				
			}
		}
		Bee(0);
	}
}

