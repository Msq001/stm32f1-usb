#include "sd.h"

u8  CardType=0;  //SD�������� 

/***************************************** SD SPI ģʽ�ײ���ֲ�Ľӿ�***********************************************/
#define SD_SPI _SPI1
#define SD_LOW_SPEED  7
#define SD_HIGH_SPEED 1

//��д����	
u8 SD_SPI_Read_Write_Byte( u8 data)
{
  return SPI_Read_Write(SD_SPI,data);		
}

//Ƭѡ
void SD_SPI_CS_Set(u8 level)
{
  SPI_CS_Set(SD_SPI,level);
}

//��ʼ��
void SD_SPI_Init(void)
{
  SPI_Config(SD_SPI);
  SD_SPI_CS_Set(1);
}

//��ʼ��ʱ��Ҫ����
void SD_SetLowSpeed(void)
{
  SPI_Protocol_Init(SD_SPI, SD_LOW_SPEED);
}

//��������ʱʹ�ø���
void SD_SetHighSpeed(void)
{
  SPI_Protocol_Init(SD_SPI, SD_HIGH_SPEED);
}
/******************************************************************************************************************/


/************************************************************************************
**ȡ��ѡ��,�ͷ�SPI����
*************************************************************************************/
void SD_Cancel_CS(void)
{
  SD_SPI_CS_Set(1);
  SD_SPI_Read_Write_Byte(0xff);//�ṩ�����8��ʱ��
}

/************************************************************************************
**ѡ��sd��,���ҵȴ���׼��OK
**����ֵ:0,�ɹ�;1,ʧ��;
*************************************************************************************/
u8 SD_Select(void)
{
  SD_SPI_CS_Set(0);
  if(SD_Wait_Ready()==0)return 0;//�ȴ��ɹ�
  SD_Cancel_CS();
  return 1;//�ȴ�ʧ��
}


/************************************************************************************
**�ȴ���׼����
**����ֵ:0,׼������;����,�������
**************************************************************************************/
u8 SD_Wait_Ready(void)
{
  u32 t=0;
  do
  {
    if(SD_SPI_Read_Write_Byte(0XFF)==0XFF)return 0;//OK
    t++;		  	
  }while(t<0XFFFFFF);//�ȴ� 
  return 1;
}


/************************************************************************************
**�ȴ�SD����Ӧ
**Response:Ҫ�õ��Ļ�Ӧֵ
**����ֵ:0,�ɹ��õ��˸û�Ӧֵ
**    ����,�õ���Ӧֵʧ��
*************************************************************************************/
u8 SD_Get_Ack(u8 Response)
{
  u16 Count=0xFFFF;//�ȴ�����	   						  
  while ((SD_SPI_Read_Write_Byte(0XFF)!=Response) && Count)  Count--; //�ȴ��õ�׼ȷ�Ļ�Ӧ 
  
  if (Count==0)
    return SD_RESPONSE_FAILURE;//�õ���Ӧʧ��   
  else 
    return SD_RESPONSE_NO_ERROR;//��ȷ��Ӧ
}


/*************************************************************************************
**��sd����ȡһ�����ݰ�������
**buf:���ݻ�����
**len:Ҫ��ȡ�����ݳ���.
**����ֵ:0,�ɹ�;����,ʧ��;	
****************************************************************************************/
u8 SD_RecvData(u8*buf,u16 len)
{			  	  
  if(SD_Get_Ack(0xFE))return 1;//�ȴ�SD������������ʼ����0xFE
  while(len--)//��ʼ��������
  {
    *buf=SD_SPI_Read_Write_Byte(0xFF);
    buf++;
  }
  //������2��αCRC��dummy CRC��
  SD_SPI_Read_Write_Byte(0xFF);
  SD_SPI_Read_Write_Byte(0xFF);									  					    
  return 0;//��ȡ�ɹ�
}



/************************************************************************************
**��sd��д��һ�����ݰ������� 512�ֽ�
**buf:���ݻ�����
**cmd:ָ��
**����ֵ:0,�ɹ�;����,ʧ��;	
*************************************************************************************/
u8 SD_Send_Data(u8*buf,u8 cmd)
{	
  u16 t;		  	  
  if(SD_Wait_Ready())  return 1;  //�ȴ�׼��ʧЧ
  SD_SPI_Read_Write_Byte(cmd);
  if(cmd!=0XFD)//���ǽ���ָ��
  {
    for(t=0;t<512;t++)SD_SPI_Read_Write_Byte(buf[t]);

    SD_SPI_Read_Write_Byte(0xFF); //����crc
    SD_SPI_Read_Write_Byte(0xFF);
    t = SD_SPI_Read_Write_Byte(0xFF); //������Ӧ
    if((t&0x1F) != 0x05)return 2;   //��Ӧ����									  					    
  }						 									  					    
  return 0;//д��ɹ�
}



/*************************************************************************************
**��SD������һ������
**����: u8 cmd   ���� 
**      u32 arg  �������
**      u8 crc   crcУ��ֵ	
**����ֵ:SD�����ص���Ӧ
***************************************************************************************/												  
u8 SD_SendCmd(u8 cmd, u32 arg, u8 crc)
{
  u8 r1;	
  u8 Retry=0; 
  SD_Cancel_CS();              //ȡ���ϴ�Ƭѡ
  if(SD_Select())  return 0XFF;//ƬѡʧЧ 
  //����
  SD_SPI_Read_Write_Byte(cmd | 0x40);//�ֱ�д������
  SD_SPI_Read_Write_Byte(arg >> 24);
  SD_SPI_Read_Write_Byte(arg >> 16);
  SD_SPI_Read_Write_Byte(arg >> 8);
  SD_SPI_Read_Write_Byte(arg);	  
  SD_SPI_Read_Write_Byte(crc); 
  if(cmd==CMD12)  SD_SPI_Read_Write_Byte(0xff);//Skip a stuff byte when stop reading
  //�ȴ���Ӧ����ʱ�˳�
  Retry=0X1F;
  do
  {
    r1=SD_SPI_Read_Write_Byte(0xFF);
  }while((r1&0X80) && Retry--);	 
  //����״ֵ̬
  return r1;
}	


/*************************************************************************************
**��ȡSD����CID��Ϣ��������������Ϣ
**����: u8 *cid_data(���CID���ڴ棬����16Byte��	  
**����ֵ:0��NO_ERR
**		 1������				
*************************************************************************************/
u8 SD_GetCID(u8 *cid_data)
{
  u8 r1;	   
  //��CMD10�����CID
  r1=SD_SendCmd(CMD10,0,0x01);
  if(r1==0x00)
  {
    r1=SD_RecvData(cid_data,16);//����16���ֽڵ�����	 
  }
  SD_Cancel_CS();//ȡ��Ƭѡ
  if(r1)  return 1;
  else    return 0;
}	


/*************************************************************************************
��ȡSD����CSD��Ϣ�������������ٶ���Ϣ
����:u8 *cid_data(���CID���ڴ棬����16Byte��	    
����ֵ:0��NO_ERR
		 1������	
*************************************************************************************/
u8 SD_GetCSD(u8 *csd_data)
{
  u8 r1;	 
  r1=SD_SendCmd(CMD9,0,0x01);    //��CMD9�����CSD
  if(r1==0)
  {
    r1=SD_RecvData(csd_data, 16);//����16���ֽڵ����� 
  }
  SD_Cancel_CS();//ȡ��Ƭѡ
  if(r1)  return 1;
  else    return 0;
}  

u8 SD_GetCardInfo(SD_CardInfo *cardinfo)
{
  u32 CSD_Tab[4],CID_Tab[4],RCA;
  SD_GetCID((u8 *)CID_Tab);                         //��SD��CID
  SD_GetCSD((u8 *)CSD_Tab);                         //��SD��CSD
  
 	u8 errorstatus = 0;
	u8 tmp=0;	   
	cardinfo->CardType=(u8)CardType; 				//������
	cardinfo->RCA=(u16)RCA;							//��RCAֵ
	tmp=(u8)((CSD_Tab[0]&0xFF000000)>>24);
	cardinfo->SD_csd.CSDStruct=(tmp&0xC0)>>6;		//CSD�ṹ
	cardinfo->SD_csd.SysSpecVersion=(tmp&0x3C)>>2;	//2.0Э�黹û�����ⲿ��(Ϊ����),Ӧ���Ǻ���Э�鶨���
	cardinfo->SD_csd.Reserved1=tmp&0x03;			//2������λ  
	tmp=(u8)((CSD_Tab[0]&0x00FF0000)>>16);			//��1���ֽ�
	cardinfo->SD_csd.TAAC=tmp;				   		//���ݶ�ʱ��1
	tmp=(u8)((CSD_Tab[0]&0x0000FF00)>>8);	  		//��2���ֽ�
	cardinfo->SD_csd.NSAC=tmp;		  				//���ݶ�ʱ��2
	tmp=(u8)(CSD_Tab[0]&0x000000FF);				//��3���ֽ�
	cardinfo->SD_csd.MaxBusClkFrec=tmp;		  		//�����ٶ�	   
	tmp=(u8)((CSD_Tab[1]&0xFF000000)>>24);			//��4���ֽ�
	cardinfo->SD_csd.CardComdClasses=tmp<<4;    	//��ָ�������λ
	tmp=(u8)((CSD_Tab[1]&0x00FF0000)>>16);	 		//��5���ֽ�
	cardinfo->SD_csd.CardComdClasses|=(tmp&0xF0)>>4;//��ָ�������λ
	cardinfo->SD_csd.RdBlockLen=tmp&0x0F;	    	//����ȡ���ݳ���
	tmp=(u8)((CSD_Tab[1]&0x0000FF00)>>8);			//��6���ֽ�
	cardinfo->SD_csd.PartBlockRead=(tmp&0x80)>>7;	//����ֿ��
	cardinfo->SD_csd.WrBlockMisalign=(tmp&0x40)>>6;	//д���λ
	cardinfo->SD_csd.RdBlockMisalign=(tmp&0x20)>>5;	//�����λ
	cardinfo->SD_csd.DSRImpl=(tmp&0x10)>>4;
	cardinfo->SD_csd.Reserved2=0; 					//����
  
 	if((CardType==SD_TYPE_V1)||(CardType==SD_TYPE_V2)||(SD_TYPE_MMC==CardType))//��׼1.1/2.0��/MMC��
	{
		cardinfo->SD_csd.DeviceSize=(tmp&0x03)<<10;	//C_SIZE(12λ)
	 	tmp=(u8)(CSD_Tab[1]&0x000000FF); 			//��7���ֽ�	
		cardinfo->SD_csd.DeviceSize|=(tmp)<<2;
 		tmp=(u8)((CSD_Tab[2]&0xFF000000)>>24);		//��8���ֽ�	
		cardinfo->SD_csd.DeviceSize|=(tmp&0xC0)>>6;
 		cardinfo->SD_csd.MaxRdCurrentVDDMin=(tmp&0x38)>>3;
		cardinfo->SD_csd.MaxRdCurrentVDDMax=(tmp&0x07);
 		tmp=(u8)((CSD_Tab[2]&0x00FF0000)>>16);		//��9���ֽ�	
		cardinfo->SD_csd.MaxWrCurrentVDDMin=(tmp&0xE0)>>5;
		cardinfo->SD_csd.MaxWrCurrentVDDMax=(tmp&0x1C)>>2;
		cardinfo->SD_csd.DeviceSizeMul=(tmp&0x03)<<1;//C_SIZE_MULT
 		tmp=(u8)((CSD_Tab[2]&0x0000FF00)>>8);	  	//��10���ֽ�	
		cardinfo->SD_csd.DeviceSizeMul|=(tmp&0x80)>>7;
 		cardinfo->CardCapacity=(cardinfo->SD_csd.DeviceSize+1);//���㿨����
		cardinfo->CardCapacity*=(1<<(cardinfo->SD_csd.DeviceSizeMul+2));
		cardinfo->CardBlockSize=1<<(cardinfo->SD_csd.RdBlockLen);//���С
		cardinfo->CardCapacity*=cardinfo->CardBlockSize;
	}else if(CardType==SD_TYPE_V2HC)	//��������
	{
 		tmp=(u8)(CSD_Tab[1]&0x000000FF); 		//��7���ֽ�	
		cardinfo->SD_csd.DeviceSize=(tmp&0x3F)<<16;//C_SIZE
 		tmp=(u8)((CSD_Tab[2]&0xFF000000)>>24); 	//��8���ֽ�	
 		cardinfo->SD_csd.DeviceSize|=(tmp<<8);
 		tmp=(u8)((CSD_Tab[2]&0x00FF0000)>>16);	//��9���ֽ�	
 		cardinfo->SD_csd.DeviceSize|=(tmp);
 		tmp=(u8)((CSD_Tab[2]&0x0000FF00)>>8); 	//��10���ֽ�	
 		cardinfo->CardCapacity=(long long)(cardinfo->SD_csd.DeviceSize+1)*512*1024;//���㿨����
		cardinfo->CardBlockSize=512; 			//���С�̶�Ϊ512�ֽ�
	}	  
	cardinfo->SD_csd.EraseGrSize=(tmp&0x40)>>6;
	cardinfo->SD_csd.EraseGrMul=(tmp&0x3F)<<1;	   
	tmp=(u8)(CSD_Tab[2]&0x000000FF);			//��11���ֽ�	
	cardinfo->SD_csd.EraseGrMul|=(tmp&0x80)>>7;
	cardinfo->SD_csd.WrProtectGrSize=(tmp&0x7F);
 	tmp=(u8)((CSD_Tab[3]&0xFF000000)>>24);		//��12���ֽ�	
	cardinfo->SD_csd.WrProtectGrEnable=(tmp&0x80)>>7;
	cardinfo->SD_csd.ManDeflECC=(tmp&0x60)>>5;
	cardinfo->SD_csd.WrSpeedFact=(tmp&0x1C)>>2;
	cardinfo->SD_csd.MaxWrBlockLen=(tmp&0x03)<<2;	 
	tmp=(u8)((CSD_Tab[3]&0x00FF0000)>>16);		//��13���ֽ�
	cardinfo->SD_csd.MaxWrBlockLen|=(tmp&0xC0)>>6;
	cardinfo->SD_csd.WriteBlockPaPartial=(tmp&0x20)>>5;
	cardinfo->SD_csd.Reserved3=0;
	cardinfo->SD_csd.ContentProtectAppli=(tmp&0x01);  
	tmp=(u8)((CSD_Tab[3]&0x0000FF00)>>8);		//��14���ֽ�
	cardinfo->SD_csd.FileFormatGrouop=(tmp&0x80)>>7;
	cardinfo->SD_csd.CopyFlag=(tmp&0x40)>>6;
	cardinfo->SD_csd.PermWrProtect=(tmp&0x20)>>5;
	cardinfo->SD_csd.TempWrProtect=(tmp&0x10)>>4;
	cardinfo->SD_csd.FileFormat=(tmp&0x0C)>>2;
	cardinfo->SD_csd.ECC=(tmp&0x03);  
	tmp=(u8)(CSD_Tab[3]&0x000000FF);			//��15���ֽ�
	cardinfo->SD_csd.CSD_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_csd.Reserved4=1;		 
	tmp=(u8)((CID_Tab[0]&0xFF000000)>>24);		//��0���ֽ�
	cardinfo->SD_cid.ManufacturerID=tmp;		    
	tmp=(u8)((CID_Tab[0]&0x00FF0000)>>16);		//��1���ֽ�
	cardinfo->SD_cid.OEM_AppliID=tmp<<8;	  
	tmp=(u8)((CID_Tab[0]&0x000000FF00)>>8);		//��2���ֽ�
	cardinfo->SD_cid.OEM_AppliID|=tmp;	    
	tmp=(u8)(CID_Tab[0]&0x000000FF);			//��3���ֽ�	
	cardinfo->SD_cid.ProdName1=tmp<<24;				  
	tmp=(u8)((CID_Tab[1]&0xFF000000)>>24); 		//��4���ֽ�
	cardinfo->SD_cid.ProdName1|=tmp<<16;	  
	tmp=(u8)((CID_Tab[1]&0x00FF0000)>>16);	   	//��5���ֽ�
	cardinfo->SD_cid.ProdName1|=tmp<<8;		 
	tmp=(u8)((CID_Tab[1]&0x0000FF00)>>8);		//��6���ֽ�
	cardinfo->SD_cid.ProdName1|=tmp;		   
	tmp=(u8)(CID_Tab[1]&0x000000FF);	  		//��7���ֽ�
	cardinfo->SD_cid.ProdName2=tmp;			  
	tmp=(u8)((CID_Tab[2]&0xFF000000)>>24); 		//��8���ֽ�
	cardinfo->SD_cid.ProdRev=tmp;		 
	tmp=(u8)((CID_Tab[2]&0x00FF0000)>>16);		//��9���ֽ�
	cardinfo->SD_cid.ProdSN=tmp<<24;	   
	tmp=(u8)((CID_Tab[2]&0x0000FF00)>>8); 		//��10���ֽ�
	cardinfo->SD_cid.ProdSN|=tmp<<16;	   
	tmp=(u8)(CID_Tab[2]&0x000000FF);   			//��11���ֽ�
	cardinfo->SD_cid.ProdSN|=tmp<<8;		   
	tmp=(u8)((CID_Tab[3]&0xFF000000)>>24); 		//��12���ֽ�
	cardinfo->SD_cid.ProdSN|=tmp;			     
	tmp=(u8)((CID_Tab[3]&0x00FF0000)>>16);	 	//��13���ֽ�
	cardinfo->SD_cid.Reserved1|=(tmp&0xF0)>>4;
	cardinfo->SD_cid.ManufactDate=(tmp&0x0F)<<8;    
	tmp=(u8)((CID_Tab[3]&0x0000FF00)>>8);		//��14���ֽ�
	cardinfo->SD_cid.ManufactDate|=tmp;		 	  
	tmp=(u8)(CID_Tab[3]&0x000000FF);			//��15���ֽ�
	cardinfo->SD_cid.CID_CRC=(tmp&0xFE)>>1;
	cardinfo->SD_cid.Reserved2=1;	 
	return errorstatus;
}

/************************************************************
**��ȡSD����������������������   
**����ֵ:0�� ȡ�������� 
**����:SD��������(������/512�ֽ�)
**ÿ�������ֽ�����Ϊ512�ֽڣ���Ϊ�������512�ֽڣ����ʼ������ͨ��.	
*************************************************************/
u32 SD_Get_Sector_Count(void)
{
  u8 csd[16];
  u32 Capacity;  
  u8 n;
  u16 csize;  					    
  if(SD_GetCSD(csd) != 0) return 0;	//ȡCSD��Ϣ������ڼ��������0
  if((csd[0]&0xC0) == 0x40)	        //V2.00�Ŀ�,���ΪSDHC�����������淽ʽ����
  {	
    csize = csd[9] + ((u16)csd[8] << 8) + 1;
    Capacity = (u32)csize << 10;//�õ�������	 		   
  }
  else//V1.XX�Ŀ� 
  {	
    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
    csize = (csd[8] >> 6) + ((u16)csd[7] << 2) + ((u16)(csd[6] & 3) << 10) + 1;
    Capacity= (u32)csize << (n - 9);//�õ�������   
  }
  return Capacity;
}



/**********************************
��ʼ��SD��
***********************************/

u8 SD_Init(void)
{
  u8 r1;      // ���SD���ķ���ֵ
  u16 retry;  // �������г�ʱ����
  u8 buf[4];  
  u16 i;

  SD_SPI_Init();		//��ʼ��IO
  SD_SetLowSpeed();
  for(i=0;i<10;i++)SD_SPI_Read_Write_Byte(0XFF); //��������74������
  retry=20;
  do
  {
    r1 = SD_SendCmd(CMD0,0,0x95);//����IDLE״̬ ����
  }while((r1!=0X01) && retry--);
  CardType=0;   //Ĭ���޿�
  if(r1==0X01)
  {
    if(SD_SendCmd(CMD8,0x1AA,0x87)==1)  //SD V2.0
    {
      for(i=0;i<4;i++)buf[i]=SD_SPI_Read_Write_Byte(0XFF);	//Get trailing return value of R7 resp
      if(buf[2]==0X01&&buf[3]==0XAA)    //���Ƿ�֧��2.7~3.6V
      {
        retry = 0XFFFE;
        do
        {
          SD_SendCmd(CMD55,0,0X01);	    //����CMD55
          r1 = SD_SendCmd(CMD41,0x40000000,0X01);//����CMD41
        }while(r1 && retry--);
        if(retry&&SD_SendCmd(CMD58,0,0X01) == 0)//����SD2.0���汾��ʼ
        {
          for(i=0;i<4;i++)buf[i]=SD_SPI_Read_Write_Byte(0XFF);//�õ�OCRֵ
          if(buf[0]&0x40)CardType=SD_TYPE_V2HC;    //���CCS
          else CardType=SD_TYPE_V2;   
        }
      }
    }
    else//SD V1.x/ MMC	V3
    {
      SD_SendCmd(CMD55,0,0X01);		//����CMD55
      r1 = SD_SendCmd(CMD41,0,0X01);	//����CMD41
      if(r1 <= 1)
      {		
        CardType = SD_TYPE_V1;
        retry = 0XFFFE;
        do //�ȴ��˳�IDLEģʽ
        {
          SD_SendCmd(CMD55,0,0X01);	//����CMD55
          r1 = SD_SendCmd(CMD41,0,0X01);//����CMD41
        }while(r1 && retry--);
      }
      else//MMC����֧��CMD55+CMD41ʶ��
      {
        CardType = SD_TYPE_MMC;//MMC V3
        retry = 0XFFFE;
        do //�ȴ��˳�IDLEģʽ
        {											    
          r1 = SD_SendCmd(CMD1,0,0X01);//����CMD1
        }while(r1 && retry--);  
      }
      if(retry==0 || SD_SendCmd(CMD16,512,0X01)!=0)
        CardType = SD_TYPE_ERR;//����Ŀ�
    }
  }
  SD_Cancel_CS();    //ȡ��Ƭѡ
  SD_SetHighSpeed();
  if(CardType) return 0;
  else if(r1) return r1; 	   
  return 0xaa;//��������
}


/*************************************************************************************
**��SD��
**buf:���ݻ�����
**sector:����
**cnt:������
**����ֵ:0,ok;����,ʧ��.
*************************************************************************************/
u8 SD_Read_Data(u8*buf,u32 sector,u32 cnt)
{
  u8 r1;
  if(CardType != SD_TYPE_V2HC) sector <<= 9;//ת��Ϊ�ֽڵ�ַ
  if(cnt == 1)
  {
    r1 = SD_SendCmd(CMD17, sector, 0X01);//������
    if(r1 == 0)												//ָ��ͳɹ�
    {
      r1 = SD_RecvData(buf,512);			//����512���ֽ�	   
    }
  }else
  {
    r1 = SD_SendCmd(CMD18, sector, 0X01);//����������
    do
    {
      r1 = SD_RecvData(buf, 512);//����512���ֽ�	 
      buf += 512;  
    }while(--cnt && r1==0); 	
    SD_SendCmd(CMD12, 0, 0X01);	//����ֹͣ����
  }   
  SD_Cancel_CS();//ȡ��Ƭѡ
  return r1;//
}

/*************************************************************************************
**дSD��
**buf:���ݻ�����
**sector:��ʼ����
**cnt:������
**����ֵ:0,ok;����,ʧ��.
*************************************************************************************/
u8 SD_Write_Data(u8*buf,u32 sector,u32 cnt)
{
  u8 r1;
  if(CardType!=SD_TYPE_V2HC) sector *= 512;//ת��Ϊ�ֽڵ�ַ
  if(cnt == 1)
  {
    r1 = SD_SendCmd(CMD24, sector, 0X01);//������
    if(r1 == 0)//ָ��ͳɹ�
    {
      r1 = SD_Send_Data(buf, 0xFE);//д512���ֽ�	   
    }
  }else
  {
    if(CardType != SD_TYPE_MMC)
    {
      SD_SendCmd(CMD55, 0, 0X01);	
      SD_SendCmd(CMD23, cnt, 0X01);//����ָ��	
    }
    r1 = SD_SendCmd(CMD25, sector, 0X01);//����������
    if(r1 == 0)
    {
      do
      {
        r1 = SD_Send_Data(buf, 0xFC);//����512���ֽ�	 
        buf += 512;  
      }while(--cnt && r1==0);
      r1 = SD_Send_Data(0, 0xFD);//����512���ֽ� 
    }
  }   
  SD_Cancel_CS();//ȡ��Ƭѡ
  return r1;//
}	
