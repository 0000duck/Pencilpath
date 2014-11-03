#include "StdAfx.H"
#include "SmartNC.H"
#include "SurfGeo.H"
#include "SmartWrap.H"
#include "SmartVoro.H"
#include "SmtPathGen.H"
#include "SmtPathGen2D.H"

#include "dibimage.h"   
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

HDIB JD_ReSizeDibData(LPSTR lpDibData,long lWidth,long lHeight)
{                       
    HDIB hNewDIB;
	long lWidthFact = lWidth;
        
    DWORD dwNewDataSize;
    
    if(lWidthFact%4)
        lWidthFact = (lWidth/4+1)*4;
    
    dwNewDataSize = (DWORD)lWidthFact*(DWORD)lHeight+1064;  
            
    hNewDIB = (HDIB)::GlobalAlloc(GHND,dwNewDataSize);
    if(hNewDIB==NULL)        return NULL;
        
//    DWORD data = ::GlobalSize(hNewDIB);     
    
    
    LPSTR  lpNewDIB = (LPSTR)::GlobalLock(hNewDIB);
    
    ::memcpy((void *)lpNewDIB,(void *)lpDibData,1064);
    
    LPBITMAPINFOHEADER lpbi;
    lpbi = (LPBITMAPINFOHEADER)lpNewDIB;
    lpbi -> biWidth = lWidth;
    lpbi -> biHeight= lHeight;
    
    ::GlobalUnlock((HANDLE)hNewDIB);
             
    return hNewDIB;
}
BYTE JD_Get256Bit(BYTE * PDat,long sx,long sy,long x, long y)
{
    LONG cxFact = sx;
    if(sx%4)
    cxFact= (sx/4+1)*4; 
    if((x > sx)||(y > sy))
            return 0;
    if((x < 1)||(y < 1))
            return 0;                   
        
    PDat +=  cxFact*(sy - y);
    PDat +=  x;   
    
    return *PDat;
}
void JD_Set256Bit(BYTE * PDat,long sx,long sy,long x,long y,BYTE index )
{
    LONG cxFact = sx;
    if(sx%4)
    cxFact= (sx/4+1)*4; 
    
    if((x > sx)||(y > sy))
            return;             
    if((x < 1)||(y <= 1))
            return;                     
            
    PDat +=  cxFact*(sy - y);
    PDat +=  x;   
    *PDat =  index;
    
    return ;
}                       
BYTE JD_GetValue(BYTE * HP,long width,long xoff,long yoff)
{
  long offset;
  offset = yoff*width+xoff;
  return (HP[offset]);
}

BYTE JD_GetValue(BYTE * HP,long width,long height,long xoff,long yoff)
{
  /*long offset;            
  if((0<=xoff<width)&&(0<=yoff<height))
  {
  	offset = yoff*width+xoff;
  	return (HP[offset]);
  } 
  else
  	return 127;
  	//return 250;
  	*/         
  long offset;            
  long xf,yf;
  xf=xoff;yf=yoff;
  if(xoff<0)
  	xf=0;
  if(xoff>width)
  	xf=width-1;
  
  if(yoff<0)
  	yf=0;
  if(yoff>height)
  	yf=height-1;
  	
  	offset = yf*width+xf;
  	return (HP[offset]);
}



void JD_MakeNewDIBData(LPSTR lpNewDibData,LPSTR lpOldDibData)   
{                        
    long lOldWidth,lOldHeight;
    long lNewWidth,lNewHeight; 
    LPBITMAPINFOHEADER lpoldbi = (LPBITMAPINFOHEADER)lpOldDibData;
    LPBITMAPINFOHEADER lpnewbi = (LPBITMAPINFOHEADER)lpNewDibData;
    lOldWidth = lpoldbi->biWidth;
    lOldHeight= lpoldbi->biHeight;
    
    lNewWidth = lpnewbi->biWidth;
    lNewHeight= lpnewbi->biHeight;
    
    long lWcounter,lHcounter;
    float fWAtom,fHAtom;
    fWAtom = (float)(1/(float)lNewWidth );
    fHAtom = (float)(1/(float)lNewHeight);  
    long  lWIndex,lHIndex;
    float fWIndex,fHIndex;
    float fWpart,fHpart;
    BYTE  bTemp[3][3];              
    BYTE  bNewData;
    
    BYTE  * hbOld = (BYTE  *)lpOldDibData + 1064;
    BYTE  * hbNew = (BYTE  *)lpNewDibData + 1064;   
    
    for(lHcounter=1;lHcounter<=lNewHeight;lHcounter++)          
    for(lWcounter=1;lWcounter<=lNewWidth ;lWcounter++)
	{
  	

            fWIndex = (float)(lOldWidth  * lWcounter) * fWAtom;
            fHIndex = (float)(lOldHeight * lHcounter) * fHAtom;
            
            lWIndex = (long)floor(fWIndex);
            lHIndex = (long)floor(fHIndex);
            
            fWpart = fWIndex - (float)lWIndex;
            fHpart = fHIndex - (float)lHIndex;
            
            bTemp[1][1] = JD_Get256Bit(hbOld,lOldWidth,lOldHeight,lWIndex  ,lHIndex  );
            bTemp[2][1] = JD_Get256Bit(hbOld,lOldWidth,lOldHeight,lWIndex+1,lHIndex  );
            bTemp[1][2] = JD_Get256Bit(hbOld,lOldWidth,lOldHeight,lWIndex  ,lHIndex+1);
            bTemp[2][2] = JD_Get256Bit(hbOld,lOldWidth,lOldHeight,lWIndex+1,lHIndex+1);
            
            bNewData = (BYTE)(((1-fWpart)*bTemp[1][1]+(fWpart)*bTemp[2][1])*(1-fHpart)
                             +((1-fWpart)*bTemp[1][2]+(fWpart)*bTemp[2][2])*(fHpart));
            
            JD_Set256Bit(hbNew,lNewWidth,lNewHeight,lWcounter,lHcounter,bNewData);

	}
}
BYTE JD_SetValue(BYTE * HP,long width,long xoff,long yoff,BYTE value)
{
  long offset;
  offset = yoff*width+xoff;
  HP[offset]=value;
  return (HP[offset]);    
}

float JD_Dim(float x1,float y1,float x2,float y2,float x,float y)
{
  if((y1==y2)&&(y1==y))
    return 0;
  
  float a,b,c;
  float aa,bb;       
  float top;
  float boot;
  a = (y2-y1)*0.1f;
  b = (x1-x2)*0.1f;
  c = (x2*y1-x1*y2)*0.01f;
  aa = a*a;
  bb = b*b;          
    
  boot = (float)sqrt(aa+bb);  
  top = (float)fabs(a*x+b*y+c);    
  return ((top/boot)*10.0f);
} 

void JD_MakeMapData(HDIB hdib,HDIB hpos)
//new algorithm for process in line ;
{
   BYTE soure[1024];
  
   int i,j;      
   int h,w; 
   int ww;
   
   float gate  = 50;
      
   float tx,tz;
   float nx,nz;
   float ex,ez;
   float dtemp;
   register int k,s;
     
   BYTE  * pcolor= (BYTE *)::GlobalLock((HGLOBAL)hdib);
   BYTE  * pPos  = (BYTE *)::GlobalLock((HGLOBAL)hpos);
   
   
   h = (int)DIBHeight((LPSTR)pcolor);
   w = ww=(int)DIBWidth((LPSTR)pcolor);
   if(w%4) 
     ww = (w/4+1)*4;
       
   pcolor = pcolor + 1064;
   pPos   = pPos   + 1064;
   
    for(j=1;j<h-1;j++)
     {       
      for(k=1;k<=ww;k++)
          soure[k] = JD_GetValue(pcolor,ww,k,j);
      JD_SetValue(pPos ,(long)ww,(long)2,long(j), 1);
                        
      for(i=1;i<w-1;i++)
         {
            
            tx = (float)i ;
            tz = soure[i];
            
            nx = tx +1;
            ex = nx +1;
            
            nz= soure[i+1];
            ez= soure[i+2];
            
       // for the flat area 
        if((tz==ez)&&(i<w-1))
            { 
             for( k=i+3;k<w-1;k++)
               { 
                 ex= (float)k;      
                 ez= soure[k];
                 if(tz!=ez)
                    {
                        i=k-2;
                        ex= (float)(k-1);
                        ez= soure[k-1];
                        goto wr1;        
                    }   
               }        
              i = k; 
              ex= (float)(k-1);
              ez= soure[k-1];
            }           
        // for mountoin area          
        else
         if(((dtemp=JD_Dim(tx,tz,ex,ez,nx,nz))<=gate)&&(i<w-1))  
            { 
             for( k=i+3;k<w-1;k++)
               { 
                 ex= (float)k;
                 ez= soure[k];
               for( s=i+1;s<k;s++)
                 {
                   nx= (float)s;
                   nz= soure[s];
                    if(((dtemp=JD_Dim(tx,tz,ex,ez,nx,nz))>gate)||(k==w-1))
                        {
                            i=k-2;
                            ex= (float)(k-1);
                            ez= soure[k-1];
                            goto wr1;
                        }
                 }          
               
               }            
            }    
      wr1:  JD_SetValue(pPos ,(long)ww,(long)ex,long(j), 1);
            
         }  
       
     }     
   ::GlobalUnlock((HGLOBAL) hdib);
   ::GlobalUnlock((HGLOBAL) hpos);
} 
void JD_MakePointMapData(HDIB hdib,HDIB hpos,int nLevel)
//new algorithm for process in line ;
{                           
                                    
   BYTE  * spcolor= (BYTE *)::GlobalLock((HGLOBAL)hdib);
   BYTE  * spPos  = (BYTE *)::GlobalLock((HGLOBAL)hpos);
  
   BYTE * pcolor= (BYTE *)spcolor;
   BYTE * pPos  = (BYTE *)spPos;
   
   int h,w,ww;  //define for the source data;
   int maph,mapw,mapww; // define for the map data
   
   h = (int)DIBHeight((LPSTR)pcolor);
   w = ww=(int)DIBWidth((LPSTR)pcolor);
   
   int scale = 3; // the default is 3 by other side if 9 level and 3*3 matrix;
    
   if(nLevel == 4)
    	scale = 2; //  for the 2*2 matrix for 4 level;
   else if(nLevel == 9)
       	scale = 3; //  for the 3*3 matrix for 9 level;
       	else
       	scale = 8;
    // scale the size to level     
    maph 	=	h * scale;
    mapww = mapw = w * scale;
    
    BITMAPINFOHEADER * pdibhead;
    
    ::memcpy((void *)spPos,(void *)spcolor,1064);
    
    pdibhead = (BITMAPINFOHEADER *)spPos;
    pdibhead->biWidth = mapw*1L;
    pdibhead->biHeight= maph*1L;
    
   if(w%4) 
     ww = (w/4+1)*4;
   if(mapw%4) 
     mapww = (mapw/4+1)*4;
       
   pcolor = pcolor + 1064;
   pPos   = pPos   + 1064;
    
   int i,j;
   int index;     
   int room;
   long lPer;            
	               
   if(nLevel==4)
   {
	    
    for(j=0;j<h;j++)
     {      
     	lPer = (long)j *100/h; 
     for(i=0;i<w;i++)
     	{                           
     		
			index = JD_GetValue(pcolor,ww,i,j);
			room = index/51;
			switch(room)
			{
				case 0:
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2),0);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2),0);
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2+1),0);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2+1),0);
						break;
				case 1:
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2),0);
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2+1),0);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2+1),0);
						break;
				case 2: 
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2),0);
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2+1),0);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2+1),250);
						break;
				case 3:
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2+1),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2+1),0);
						break;
				default:
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2,long(j*2+1),250);
						JD_SetValue(pPos ,(long)mapww,(long)i*2+1,long(j*2+1),250);
						break;						
			}
      	}
     }           
   }
   else     
   if(nLevel==9)
   {
    for(j=0;j<h;j++)
     {      
     	lPer = (long)j *100/h; 
     for(i=0;i<w;i++)
     	{     
     		
			index = JD_GetValue(pcolor,ww,i,j);
			room = index/25;                                                     
			switch(room)
			{
				case 0:												
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);

						break;                                               
				case 1:
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);
						
						break;
				case 2:
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);
						break;
				case 3:                                                  
				        JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);
						break;
				case 4:
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);
						break;
				case 5:
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);
						break;
				case 6:       
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);
						break;
				case 7:                                                    
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 0);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 0);
						break;
				case 8:
						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 0);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 250);
						break;
				default:      
				        JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+1), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+1), 250);

						JD_SetValue(pPos ,(long)mapww,(long)(i*3),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+1),long(j*3+2), 250);
						JD_SetValue(pPos ,(long)mapww,(long)(i*3+2),long(j*3+2), 250);
						break;						
			}                                       

      	}
     }           
   }   
   // test for random !
   else
   {
    for(j=0;j<h-1;j++)
     {      
     	lPer = (long)j *100/h; 
     for(i=0;i<w-1;i++)
     	{     
     		
			index = JD_GetValue(pcolor,ww,i,j);
			room = index/4;
			
			                                                     
			int pos[8][8];
			int count = 0;
			int rands;
			for(int k =0;k<8;k++)
			for(int kk=0;kk<8;kk++)
				{	
					pos[k][kk]=0;
				}
				int f1,f2;
				for(f1=0;f1<8;f1++)
				for(f2=0;f2<8;f2++)
				{
					//JD_SetValue(pPos ,(long)mapww,(long)(i+f1),long(j+f2),0); 
				 }
		        
				int col,row;
				
				int test = 0;               
				
				srand((unsigned)time(NULL));
				rand();
				rand();
			while(count<room)
				{                 
					
					rands = (rand()+65)%64;
					
					col = rands/8;
					row = rands%8;
					if(pos[col][row]==0)
					{  
						pos[col][row]=1;
						JD_SetValue(pPos,(long)mapww,(long)(i*8+col),long(j*8+row),250);
						count++;
					}       
					test++;
					if(test>500)break;
						
				}
			                                    
      	}
     }           
   }
   ::GlobalUnlock((HGLOBAL) hdib);
   ::GlobalUnlock((HGLOBAL) hpos);
}  
//#define  DO_MORE8
void JD_MakeDitherPointMapData(HDIB hdib,HDIB hpos,int iMethod)
{                       
   BYTE  * spcolor= (BYTE *)::GlobalLock((HGLOBAL)hdib);
   BYTE  * spPos  = (BYTE *)::GlobalLock((HGLOBAL)hpos);
  
   BYTE * pcolor= (BYTE  *)spcolor;
   BYTE * pPos  = (BYTE  *)spPos;
   
   int h,w,ww;  //define for the source data;
   int maph,mapw,mapww; // define for the map data
   
   int temp[4][4];
#ifndef	DO_MORE8
   // the dither template
   switch( iMethod )
   {
    // the classic pattern
   case 3:	temp[0][0]=0 ;temp[0][1]=8 ;temp[0][2]=2 ;temp[0][3]=10;
   			temp[1][0]=12;temp[1][1]=4 ;temp[1][2]=14;temp[1][3]=6 ;
	 	   	temp[2][0]=3 ;temp[2][1]=11;temp[2][2]=1 ;temp[2][3]=9 ;
		   	temp[3][0]=15;temp[3][1]=7 ;temp[3][2]=13;temp[3][3]=5 ;
			break;
   // the cross pattern
   case 4: temp[0][0]=15;temp[0][1]=4 ;temp[0][2]=5 ;temp[0][3]=6 ;
		   temp[1][0]=14;temp[1][1]=0 ;temp[1][2]=1 ;temp[1][3]=7 ;
		   temp[2][0]=13;temp[2][1]=3 ;temp[2][2]=2 ;temp[2][3]=8 ;
		   temp[3][0]=12;temp[3][1]=11;temp[3][2]=10;temp[3][3]=9 ;
    		break;
   // the shear pattern
   /*
   temp[0][0]=0 ;temp[0][1]=4 ;temp[0][2]=10;temp[0][3]=14 ;
   temp[1][0]=7 ;temp[1][1]=1 ;temp[1][2]=5 ;temp[1][3]=11 ;
   temp[2][0]=12;temp[2][1]=8 ;temp[2][2]=2 ;temp[2][3]=6 ;
   temp[3][0]=15;temp[3][1]=13;temp[3][2]=9 ;temp[3][3]=3 ;
   */
   //dot
   /*
   temp[0][0]=4;temp[0][1]=12 ;temp[0][2]=14 ;temp[0][3]=6 ;
   temp[1][0]=8;temp[1][1]=0 ;temp[1][2]=2 ;temp[1][3]=10 ;
   temp[2][0]=11;temp[2][1]=3 ;temp[2][2]=1 ;temp[2][3]=9 ;
   temp[3][0]=7;temp[3][1]=15;temp[3][2]=13;temp[3][3]=5 ;
   */                                                  
   // horz line pattern
   case 5: temp[0][0]=0;temp[0][1]=4 ;temp[0][2]=8 ;temp[0][3]=12 ;
		   temp[1][0]=1;temp[1][1]=5 ;temp[1][2]=9 ;temp[1][3]=13 ;
		   temp[2][0]=2;temp[2][1]=6 ;temp[2][2]=10 ;temp[2][3]=14 ;
		   temp[3][0]=3;temp[3][1]=7;temp[3][2]=11;temp[3][3]=15 ;
		   		break;
   case 6: temp[0][0]=4;temp[0][1]=12 ;temp[0][2]=14 ;temp[0][3]=6 ;
		   temp[1][0]=8;temp[1][1]=0 ;temp[1][2]=2 ;temp[1][3]=10 ;
		   temp[2][0]=11;temp[2][1]=3 ;temp[2][2]=1 ;temp[2][3]=9 ;
		   temp[3][0]=7;temp[3][1]=15;temp[3][2]=13;temp[3][3]=5 ;
		   		break;
	}		   		
		   		
                                                        
   
   
   
#else   
   int more[8][8];
   more[0][0]=0 ;more[0][1]=32;more[0][2]=8 ;more[0][3]=40;more[0][4]=2 ;more[0][5]=34;more[0][6]=10;more[0][7]=42;
   more[1][0]=48;more[1][1]=16;more[1][2]=56;more[1][3]=24;more[1][4]=50;more[1][5]=18;more[1][6]=58;more[1][7]=26;
   more[2][0]=12;more[2][1]=44;more[2][2]=4 ;more[2][3]=36;more[2][4]=14;more[2][5]=46;more[2][6]=6; more[2][7]=38;
   more[3][0]=60;more[3][1]=28;more[3][2]=52;more[3][3]=20;more[3][4]=62;more[3][5]=30;more[3][6]=54;more[3][7]=22;
   more[4][0]=3 ;more[4][1]=35;more[4][2]=11;more[4][3]=43;more[4][4]=1 ;more[4][5]=33;more[4][6]=9; more[4][7]=41;
   more[5][0]=51;more[5][1]=19;more[5][2]=59;more[5][3]=27;more[5][4]=49;more[5][5]=17;more[5][6]=57;more[5][7]=25;
   more[6][0]=15;more[6][1]=47;more[6][2]=7 ;more[6][3]=39;more[6][4]=13;more[6][5]=45;more[6][6]=5; more[6][7]=37;
   more[7][0]=63;more[7][1]=31;more[7][2]=55;more[7][3]=23;more[7][4]=61;more[7][5]=29;more[7][6]=53;more[7][7]=21;
#endif   
   
   
   
   
   h = (int)DIBHeight((LPSTR)pcolor);
   w = ww=(int)DIBWidth((LPSTR)pcolor);
   
    // scale the size to level     
    maph 	=	h;
    mapww = mapw = w;
    
    BITMAPINFOHEADER * pdibhead;
    
    ::memcpy((void *)spPos,(void *)spcolor,1064);
    
    pdibhead = (BITMAPINFOHEADER *)spPos;
    pdibhead->biWidth = mapw*1L;
    pdibhead->biHeight= maph*1L;
    
   if(w%4) 
     ww = (w/4+1)*4;
   if(mapw%4) 
     mapww = (mapw/4+1)*4;
       
   pcolor = pcolor + 1064;
   pPos   = pPos   + 1064;
    
   int i,j; 
   int x,y;
   int index,codes;        
   long lPer;
   
   if( iMethod !=7 )
	{
    for(j=0;j<h;j++)
     {  
     	lPer = (long)j *100/h; 
     for(i=0;i<w;i++)
     	{               
#ifndef DO_MORE8     	
     		// FOR 4X4 16 COLORS
			index = JD_GetValue(pcolor,ww,i,j);
			x=i%4;y=j%4;
			codes = index/16;
			if(codes>temp[x][y]||codes==15)
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),250);
			else
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),0);
#else				
			
			// FOR THE 8X8 COLORS
			srand((unsigned)time(NULL));
			int	rands = (rand()+10)%9;
			
			index = JD_GetValue(pcolor,ww,i,j);
			x=(i+rands)%8;y=(j+rands)%8;
			codes = index/4;    
			
			/*index = JD_GetValue(pcolor,ww,i,j);
			x=i%8;y=j%8;
			codes = index/4;    
			*/
			if(codes>more[x][y])
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),250);
			else
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),0);
			
#endif
      	}
     }           
    } 
    else        
    {
	for(j=0;j<h;j++)
    {  
     	lPer = (long)j *100/h; 
 	for(i=0;i<w;i++)
     	{               

     		// for test the diffuse dither
			// the template is     	x	7
			//					 3	5  	1
			//
			//
			// 				code flague
			//              s2	s3	s4
			//				s1	x	T
			//				T	T	T
			//             
			//              generation (get diffuse)
			//             	1	5	3
			//				7	x
			//
			//
			// secussful complete for the 16th!   
			
			index = JD_GetValue(pcolor,ww,i,j);
			
			int s1,s2,s3,s4;
			
			s1	=	JD_GetValue(pcolor,ww,h,i-1,j	);
			s2	=	JD_GetValue(pcolor,ww,h,i-1,j-1);
			s3	=	JD_GetValue(pcolor,ww,h,i	,j-1);
			s4	=	JD_GetValue(pcolor,ww,h,i+1,j-1);
			
			int d1,d2,d3,d4;
			
			
			if(s1>=127)d1 = s1-255;
				else	d1 = s1 ;
			if(s2>127)d2 = s2-255;
				else	d2 = s2 ;
			if(s3>=127)d3 = s3-255;
				else	d3 = s3 ;
			if(s4>127)d4 = s4-255;
				else	d4 = s4 ;
						
			x=i%4;y=j%4;
			codes = index;
			
			float f1,f2,f3,f4,tatol;
			
			f1= (float)(d1 *7.0/16.0);
			f2= (float)(d2 *1.0/16.0);
			f3= (float)(d3 *5.0/16.0);
			f4= (float)(d4 *3.0/16.0);
			tatol = f1+f2+f3+f4+(float)codes;     
			int test = (int)tatol;
			
			if(tatol>255)
				test = 255;
			if(tatol<0)
				test = 0;
            
			//JD_SetValue(pcolor,ww,i,j,(BYTE)tatol);
			//JD_SetValue(pcolor,ww,i,j,tatol);
			JD_SetValue(pcolor,ww,i,j,(BYTE)test);
			
			//codes = (int)tatol;
						
			
			if(tatol>127)
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),250);
			else
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),0);
				
			
			// secussful complete end for 16th!	
			
			/*
			// for the better codes!
			//              generation (get diffuse)32
			//            	2	4	8	4	2
			//				4	8	x
			//
			
			index = JD_GetValue(pcolor,ww,i,j);
			
			int s1,s2,s3,s4,s5,s6,s7;
			
			s1	=	JD_GetValue(pcolor,ww,h,i-2,j	);
			s2	=	JD_GetValue(pcolor,ww,h,i-1,j  );
			s3	=	JD_GetValue(pcolor,ww,h,i-2,j-1);
			s4	=	JD_GetValue(pcolor,ww,h,i-1,j-1);
			s5	=	JD_GetValue(pcolor,ww,h,i  ,j-1);
			s6	=	JD_GetValue(pcolor,ww,h,i+1,j-1);
			s7	=	JD_GetValue(pcolor,ww,h,i+2,j-1);
			
			int d1,d2,d3,d4,d5,d6,d7;
			d1 = (s1>128)?s1-255:s1;
			d2 = (s2>128)?s2-255:s2;
			d3 = (s3>128)?s3-255:s3;
			d4 = (s4>128)?s4-255:s4;
			d5 = (s5>128)?s5-255:s5;
			d6 = (s6>128)?s6-255:s6;
			d7 = (s7>128)?s7-255:s7;
			//(val >= 0) val ? -val:
			
			float f1,f2,f3,f4,f5,f6,f7,tatol;
			
			f1= ((float)d1) *4.0/32.0;
			f2= ((float)d2) *8.0/32.0;
			f3= ((float)d3) *2.0/32.0;
			f4= ((float)d4) *4.0/32.0;
			f5= ((float)d5) *8.0/32.0;
			f6= ((float)d6) *4.0/32.0;
			f7= ((float)d7) *2.0/32.0;
			tatol = f1+f2+f3+f4+f5+f6+f7+index;
			
			JD_SetValue(pcolor,ww,(long)i,long(j),tatol);
			
			if(tatol>128)
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),250);
			else
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),0);
			*/
			// for the better codes!
			//              generation (get diffuse)32
			//				1	2	4	2	1
			//            	2	4	8	4	2
			//				4	8	x
			//
			/*
			index = JD_GetValue(pcolor,ww,i,j);
			
			int s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11,s12;
			
			s1	=	JD_GetValue(pcolor,ww,h,i-2,j	);
			s2	=	JD_GetValue(pcolor,ww,h,i-1,j  );
			s3	=	JD_GetValue(pcolor,ww,h,i-2,j-1);
			s4	=	JD_GetValue(pcolor,ww,h,i-1,j-1);
			s5	=	JD_GetValue(pcolor,ww,h,i  ,j-1);
			s6	=	JD_GetValue(pcolor,ww,h,i+1,j-1);
			s7	=	JD_GetValue(pcolor,ww,h,i+2,j-1);
			s8	=	JD_GetValue(pcolor,ww,h,i-2,j-2);
			s9	=	JD_GetValue(pcolor,ww,h,i-1,j-2);
			s10	=	JD_GetValue(pcolor,ww,h,i  ,j-2);
			s11	=	JD_GetValue(pcolor,ww,h,i+1,j-2);
			s12	=	JD_GetValue(pcolor,ww,h,i+2,j-2);
			
			int d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12;
			d1 = (s1>128)?s1-255:s1;
			d2 = (s2>128)?s2-255:s2;
			d3 = (s3>128)?s3-255:s3;
			d4 = (s4>128)?s4-255:s4;
			d5 = (s5>128)?s5-255:s5;
			d6 = (s6>128)?s6-255:s6;
			d7 = (s7>128)?s7-255:s7;
			d8 = (s8>128)?s8-255:s8;
			d9 = (s9>128)?s9-255:s9;
			d10 = (s10>128)?s10-255:s10;
			d11 = (s11>128)?s11-255:s11;
			d12 = (s12>128)?s12-255:s12;
			
			//(val >= 0) val ? -val:
			
			float f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12,tatol;
			
			f1= ((float)d1) *4.0/42.0;
			f2= ((float)d2) *8.0/42.0;
			f3= ((float)d3) *2.0/42.0;
			f4= ((float)d4) *4.0/42.0;
			f5= ((float)d5) *8.0/42.0;
			f6= ((float)d6) *4.0/42.0;
			f7= ((float)d7) *2.0/42.0;
			f8= ((float)d8) *1.0/42.0;
			f9= ((float)d9) *2.0/42.0;
			f10= ((float)d10) *4.0/42.0;
			f11= ((float)d11) *2.0/42.0;
			f12= ((float)d12) *1.0/42.0;
			
			tatol = f1+f2+f3+f4+f5+f6+f7+f8+f9+f10+f11+f12+index;
			
			JD_SetValue(pcolor,ww,(long)i,long(j),tatol);
			
			if(tatol>128)
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),250);
			else
				JD_SetValue(pPos ,(long)mapww,(long)i,long(j),0);
			*/	
			
      		}
     	}            
     }
   
   ::GlobalUnlock((HGLOBAL) hdib);
   ::GlobalUnlock((HGLOBAL) hpos);

}     
HDIB JD_MakeMap(HDIB hdib)
{
    HDIB hpos;   
    BYTE * pData = (BYTE *)::GlobalLock((HGLOBAL)hdib);
      
    int h = (int)DIBHeight((LPSTR)pData);
    int w = (int)DIBWidth((LPSTR)pData);
    int ww = w;
    if(w%4) 
     ww = (w/4+1)*4;
   
    DWORD dsize  = (DWORD)h * (DWORD)ww + 1064;
    hpos = (HDIB)::GlobalAlloc(GHND,dsize);  
    if(hpos==NULL)
        return NULL;
   
    ::GlobalUnlock((HGLOBAL)hdib);    
    return hpos;
}
HDIB JD_MakePointMap(HDIB hdib,int nLevel)
{
    HDIB hpos;   
    BYTE * pData = (BYTE *)::GlobalLock((HGLOBAL)hdib);
      
    int h = (int)DIBHeight((LPSTR)pData);
    int w = (int)DIBWidth((LPSTR)pData);
    
    int scale = 3; // the default is 3 by other side if 9 level and 3*3 matrix;
    
    if(nLevel == 4)
    	scale = 2; //  for the 2*2 matrix for 4 level;
   	else
   		if(nLevel == 9)
    		scale = 3; //  for the 3*3 matrix for 9 level;
    	else
    		scale = 8; //  for the 3*3 matrix for 9 level;
    // scale the size to level     
    h 	=	h * scale;
    w	=	w * scale;
    
    int ww = w;
    if(w%4) 
     ww = (w/4+1)*4;
   
    DWORD dsize  = (DWORD)h * (DWORD)ww + 1064;
    hpos = (HDIB)::GlobalAlloc(GHND,dsize);  
    if(hpos==NULL)
        return NULL;
   
    ::GlobalUnlock((HGLOBAL)hdib);    
    return hpos;
}
void JD_DestoryMap(HDIB hpos)
{
    ::GlobalFree((HGLOBAL)hpos);
}
HDIB JD_MakeLastDib(HDIB hNewDIB,int iMethod)
{   
   HDIB hmap; 
	HDIB tempsrc;
	switch(iMethod)
	{
		case 1:	hmap = JD_MakePointMap(hNewDIB,4);
    			JD_MakePointMapData(hNewDIB,hmap,4);     
    			break;
    	case 2:	hmap = JD_MakePointMap(hNewDIB,9);
	   			JD_MakePointMapData(hNewDIB,hmap,9);     
    			break;
	    case 3: hmap = JD_MakeMap(hNewDIB);
     			JD_MakeDitherPointMapData(hNewDIB,hmap,3);     
	    		break; 
	    case 4: hmap = JD_MakeMap(hNewDIB);
     			JD_MakeDitherPointMapData(hNewDIB,hmap,4);     
	    		break; 
	    case 5: hmap = JD_MakeMap(hNewDIB);
     			JD_MakeDitherPointMapData(hNewDIB,hmap,5);     
	    		break; 
	    case 6: hmap = JD_MakeMap(hNewDIB);
     			JD_MakeDitherPointMapData(hNewDIB,hmap,6);     
	    		break;       
		case 7:	hmap = JD_MakeMap(hNewDIB);
				tempsrc = (HDIB)CopyHandle(hNewDIB);
     			JD_MakeDitherPointMapData(tempsrc,hmap,7);     
     			
     			JD_DestoryMap(tempsrc);       
     			
    			break;	    		                        
	    default:hmap = JD_MakePointMap(hNewDIB,9);
	   			JD_MakePointMapData(hNewDIB,hmap,9);     
	
	}
    return hmap;

}

BOOL JD_GetOnePointPath( float p2dLB[2],HDIB hmap,float fHeight,float fDepth,float fRadius,CPoint *ppos,
                        TPNT3D *buf,int *num)
{

    BYTE  * pmap  = (BYTE  *)GlobalLock(hmap);
    int count = 0;
                     
    int height = (int)DIBHeight((LPSTR)pmap);
    int width  = (int)DIBWidth((LPSTR)pmap);
    
	pmap  = pmap + 1064;                 
    
    int ww = width;
    
    if(width%4) 
      ww = (width/4+1)*4;
    
    BYTE data; 
    BYTE kk;                              
    BOOL bDrill = FALSE;
    BOOL bData;
    
    buf[count][0] = fRadius * ppos->x + p2dLB[0];
    buf[count][1] = fRadius * ppos->y + p2dLB[1];
    data = JD_GetValue(pmap,ww,ppos->x,ppos->y);
    if(data)
	{
	    buf[count][2] = -fDepth;
	    bDrill = TRUE;
	}
    else
    	buf[count][2] = fHeight;
    	
    
    int h , w ;  
    for (h=ppos->y;h<height-2;h++)  
	{ 
      if(h%2)
	  {
		  for (w=ppos->x;w<width-1 ;w++)
          {  
           	kk = JD_GetValue(pmap,ww,w,h);
           	if(!kk)bData = TRUE;
           	else bData = FALSE;          
           	// for the border
           	if((w==width-2)||(w==2))
			{
           	   	count++;        
                buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];
    			
    			if(bDrill)
	    			{
				    	buf[count][2] = -fDepth;
				    	//bDrill = FALSE;
			    	}
			    else 
			    	{
				    	buf[count][2] = fHeight;
				    	//bDrill = TRUE;;
			    	}
			}
           	// for drill    
            //if(bData!=bDrill)
            else if(bData!=bDrill)
			{ 
                count++;        
                if(bDrill)
	    			{                       
	    			//if(buf[count-2].z == depth)
					kk = JD_GetValue(pmap,ww,w-1,h) ;
	    			if( kk == 0 ) 
	    			{              
		    			buf[count][0] = fRadius * (w-1) + p2dLB[0];
	    				buf[count][1] = fRadius * h + p2dLB[1];
		    			buf[count][2] = -fDepth;
				    	count++;
	    			}
	    			buf[count][0] = fRadius * (w-1) + p2dLB[0];
    				buf[count][1] = fRadius * h+ p2dLB[1];
	    			
			    	buf[count][2] = fHeight;
			    	bDrill = FALSE;
			    	}
			    else 
			    	{                    
			    	buf[count][0] = fRadius * w + p2dLB[0];
    				buf[count][1] = fRadius * h + p2dLB[1];
			    	buf[count][2] = fHeight;
			    	bDrill = TRUE;
			    	}
			    	
			    count++;        
                buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];
    			
    			if(bDrill)
	    			{
			    	buf[count][2] = -fDepth;
			    	}
			    else 
			    	{
			    	buf[count][2] = fHeight;
			    	}	
                
			}  
            if(count>1998)   
			{
                ppos->x = w;
                ppos->y = h;
                
                *num = count + 1;
                return  TRUE;
			}  
            ppos->x = w; 
		  }                
	  }    
      else
	  {
         for (w=ppos->x;w>1 ;w--)
		 {    
            kk = JD_GetValue(pmap,ww,w,h);
            if(!kk)bData = TRUE;
            else bData = FALSE;
            // for the border
            if((w==width-2)||(w==2))
			{
           	   	count++;        
                buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];
    			
    			if(bDrill)
	    			{
			    	buf[count][2] = -fDepth;
			    	//bDrill = FALSE;
			    	}
			    else 
			    	{
			    	buf[count][2] = fHeight;
			    	//bDrill = TRUE;;
			    	}
			}
            // for drill    
            //if(bData!=bDrill)
            else if(bData!=bDrill)
			{ 
                count++;        
    			if(bDrill)
				{
	    			//if(buf[count-2].z == -depth)
					kk = JD_GetValue(pmap,ww,w+1,h) ;
	    			if( kk == 0 )
	    			{              
		    			buf[count][0] = fRadius * (w+1) + p2dLB[0];
	    				buf[count][1] = fRadius * h + p2dLB[1];
		    			buf[count][2] = -fDepth;
				    	count++;
	    			}
	    			
	    			buf[count][0] = fRadius * (w+1) + p2dLB[0];
    				buf[count][1] = fRadius * h + p2dLB[1];
			    	bDrill = FALSE;
			    	
				}
			    else 
				{
			    	buf[count][0] = fRadius * w + p2dLB[0];
    				buf[count][1] = fRadius * h + p2dLB[1];
			       	bDrill = TRUE;;
				}
			    buf[count][2] = fHeight;            
			    	
			    count++;        
                buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];
    			
    			if(bDrill)
				{
			    	buf[count][2] = -fDepth;
				}
			    else 
				{
			    	buf[count][2] = fHeight;
				}	
			}
            if(count>1998)   
			{
                ppos->x = w;
                ppos->y = h;
                
               *num = count + 1;
                return  TRUE;
			}            
            ppos->x = w;
		 }                
	  }
    }   
    *num = count + 1;
    ppos->x = 0;
    ppos->y = 0;
    return TRUE;
}

/////////////////////////////////////////////////////////////////////////////////////
// make the depth data!
/////////////////////////////////////////////////////////////////////////////////////
BOOL JD_GetOneDepthPath( float p2dLB[2],HDIB hmap,float fHeight,float fDepth,float fRadius,CPoint *ppos,
                        TPNT3D *buf,int *num)
{

    BYTE  * pmap  = (BYTE  *)GlobalLock(hmap);
    int count = 0;
                     
    int height = (int)DIBHeight((LPSTR)pmap);
    int width  = (int)DIBWidth((LPSTR)pmap);
    
	pmap  = pmap + 1064;                 
    
    int ww = width;
    
    if(width%4) 
      ww = (width/4+1)*4;
    
    float depthAtom = fDepth/256.0f;
    BYTE kk;                              
    
    buf[0][0] = fRadius * ppos->x + p2dLB[0];
    buf[0][1] = fRadius * ppos->y + p2dLB[1];
    buf[0][2] = fHeight;
    
    int h , w ;  
    for (h=ppos->y;h<height-2;h++)  
    { 
      if(h%2)
	  {
            for (w=ppos->x;w<width-1 ;w++)
			{            
            	count++;
           		kk = JD_GetValue(pmap,ww,w,h);
           		buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];     
    			buf[count][2] = -depthAtom*(255-kk);
           
           	   	count++;        
                buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];
    			buf[count][2] = fHeight;

				if(w<width-2)
				{
			   	    count++;        
	        		buf[count][0] = fRadius * (w+1) + p2dLB[0];
		    		buf[count][1] = fRadius * h + p2dLB[1];
			    	buf[count][2] = fHeight;
		    	}
            if(count>1995)   
			{
                ppos->x = w;
                ppos->y = h;
                
                *num = count + 1;
                return  TRUE;
			}  
               ppos->x = w; 
			}
	  }    
      else
	  {
         for (w=ppos->x;w>1 ;w--)
		 {           
            	count++;
           		kk = JD_GetValue(pmap,ww,w,h);
           		buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];     
    			buf[count][2] = -depthAtom*(255-kk);
           
           	   	count++;        
                buf[count][0] = fRadius * w + p2dLB[0];
    			buf[count][1] = fRadius * h + p2dLB[1];
    			buf[count][2] = fHeight;
				    	
				if(w>2)                      
				{
			   	    count++;        
	        		buf[count][0] = fRadius * (w-1) + p2dLB[0];
		    		buf[count][1] = fRadius * h + p2dLB[1];
			    	buf[count][2] = fHeight;
                }
            if(count>1995)   
			{
                ppos->x = w;
                ppos->y = h;
                
                *num = count + 1;
                return  TRUE;
			}  
            ppos->x = w; 
		 }  
	  }
        
    }   
    *num = count+1;
    ppos->x = 0;
    ppos->y = 0;
    return TRUE;
}


CSmartGrayGen::CSmartGrayGen()
{
}
CSmartGrayGen::~CSmartGrayGen() 
{
}
LPSTR Smart_Create256Bitmap( HDIB hDib )    
{
	if( hDib == NULL ) return NULL  ;
    LPSTR lpDIB = (LPSTR)::GlobalLock(hDib);
	if( lpDIB == NULL ) return NULL ; 
	LPBITMAPINFO  lpinfo = (LPBITMAPINFO)lpDIB;
    if(lpinfo->bmiHeader.biCompression!=BI_RGB ||
	   ( lpinfo->bmiHeader.biBitCount  != 8  && 
         lpinfo->bmiHeader.biBitCount  != 24  )  )
    { // 处理256色图片 || 24位图片
		::GlobalUnlock(hDib);
        return  NULL ;
	}
    int nHeight     =  DIBHeight(lpDIB) ;
    int nWidth      =  DIBWidth(lpDIB)  ;
	int nNewStrip   =  WIDTHBYTES( nWidth*8) ;
	int nSize  = *(LPDWORD)lpDIB + 256 *  sizeof(RGBQUAD) + nNewStrip * nHeight ;
	LPSTR  pNewData = new CHAR[ nSize] ;
	LPBITMAPINFO pNewInfo = (LPBITMAPINFO) pNewData ;
	pNewInfo->bmiHeader = lpinfo->bmiHeader ;
	pNewInfo->bmiHeader.biBitCount = 8 ;
	pNewInfo->bmiHeader.biClrUsed  = 256 ;
	pNewInfo->bmiHeader.biSizeImage= WIDTHBYTES( nWidth*8) * nHeight ;
	for( int i = 0 ; i < 256 ; i ++ ) 
	{
		pNewInfo->bmiColors[i].rgbRed     =(byte) i ;
		pNewInfo->bmiColors[i].rgbGreen   =(byte) i ;
		pNewInfo->bmiColors[i].rgbRed     =(byte) i ;
		pNewInfo->bmiColors[i].rgbReserved=(byte) i ;
	}
    LPSTR pBit = FindDIBBits( lpDIB ) ;
	LPSTR pNewBit = pNewData + *(LPDWORD)pNewData + 256 *  sizeof(RGBQUAD) ;
	BYTE bColor[3] ;
	int bBitCount   =  lpinfo->bmiHeader.biBitCount ;
	int nStrip  = WIDTHBYTES( nWidth* bBitCount);
	int nOffset,at; 
    for( i = 0 ; i < nHeight; i ++ )
	{
	  for( int j = 0 ; j < nWidth ; j ++ ) 
	  {
         nOffset =  i * nStrip + (j * bBitCount)/8;
		 if( bBitCount == 8 )
		 {
			 at    = (BYTE) pBit[ nOffset] ;
			 bColor[0] = (BYTE) lpinfo->bmiColors[at].rgbRed     ;
			 bColor[1] = (BYTE) lpinfo->bmiColors[at].rgbGreen   ;
			 bColor[2] = (BYTE) lpinfo->bmiColors[at].rgbBlue    ;
		 }
		 else if( bBitCount == 24 )
		 {
			 bColor[2] =  (BYTE)pBit[nOffset     ] ;
			 bColor[1] =  (BYTE)pBit[nOffset + 1 ] ;
			 bColor[0] =  (BYTE)pBit[nOffset + 2 ] ;
		 }
		 nOffset = i * nNewStrip + j ;
		 pNewBit[nOffset] = BYTE(0.3*bColor[0] + 0.59*bColor[1] + 0.11*bColor[2]) ;
	  }
	}
    GlobalUnlock(hDib);
	return pNewData ;

}
BOOL CSmartGrayGen::GeneratePathEx( CPathGroup& NewPath ,   /*雕刻路径*/ 
		                            CSmartGraphic& Graph)   /*图形函数*/
{
	int bRetFlag = FALSE ;
    float fHeight = 0.2f ;
    int   iMethod = m_cParam.m_nGrayMode+1;
	BITMAP_INFO  BmpInfo ;
	if( ! Graph.GetBitmapInfo( BmpInfo ) ) 
	{
		return FALSE ;
	}
	m_cSetupDef.m_cTolDef.m_nFitType = NCDEF_FITTYPE_LINE ;
    LPSTR lpDibData  = Smart_Create256Bitmap( (HDIB) BmpInfo.m_hDib ) ;   
	if( lpDibData == NULL ) return FALSE   ;
  
	//for test display the source dib data!
	//dsp(hWndParent,hDib);                    
    
    /////////////////////////////////////////
    // STEP 1 :根据亮度修正调色板          //
    ////////////////////////////////////////
    FLOAT Width,Height;
    Width = (float) BmpInfo.m_dSize[0]  ;
    Height= (float) BmpInfo.m_dSize[1] ;

    FLOAT fToolRadius = float(m_cFeedDef.m_cStepDef.m_dOverStep) ;
    long lWidth  = (long)(Width /fToolRadius);  
    long lHeight = (long)(Height/fToolRadius);
	
    
    HDIB hNewDIB;
    
    
    int scale ;
    switch(iMethod)
	{
       	case 1: scale = 2; //  for the 2*2 matrix for 4 level;
       	        break;
       	case 2: scale = 3; //  for the 3*3 matrix for 9 level;
       			break;
       	default:scale = 1;
	}
    lWidth 		=	lWidth	/scale;
	lHeight		=	lHeight /scale;
    
    ///////////////////////////////////////////////////////////////
    // STEP 2: To allocate a memory block for the new DIB data   //
    ///////////////////////////////////////////////////////////////
    hNewDIB = JD_ReSizeDibData(lpDibData,lWidth,lHeight);
    
    if(hNewDIB != NULL)
	{
		LPSTR lpNewDibData = (LPSTR)::GlobalLock(hNewDIB);
		///////////////////////////////////////////////////////////////
		// STEP 3: To product the new DIB data  for the  new size    //
		//         then can use the new DIB data to make the tool    //
		//         path data                                         //
		///////////////////////////////////////////////////////////////
		JD_MakeNewDIBData(lpNewDibData,lpDibData); 
            
		::GlobalUnlock(hNewDIB);
    
		///////////////////////////////////////////////////////////////
		// step 4: To product the lastest image                      //
		///////////////////////////////////////////////////////////////
    
		float 	p2dLB[2] ;
		p2dLB[0] =   (float) BmpInfo.m_dCorner[0] ;
		p2dLB[1] =   (float) BmpInfo.m_dCorner[1] ;

		CPathCombine	*pTComb = new CPathCombine(NC_WPROCESS_ROUGH) ; 
		int TotalPathNum = 0;
		TPNT3D  fBuffer[2002];;
		int num = 0;
		CPoint ppos(2,1);    
		CPoint pend(0,0);                                  

 		if( iMethod!=8 )
 		{
			HDIB  hmap;
			hmap = JD_MakeLastDib(hNewDIB,iMethod);
			while(JD_GetOnePointPath(p2dLB, hmap,fHeight,(float)GetCutDepth(),fToolRadius,&ppos,fBuffer,&num))
			{ 
				if (num)
				{
					CPathPLine3D * pSect = new CPathPLine3D();
					pSect->Copy( fBuffer, num - 1 ) ;
					pTComb->AddEntity( pSect ) ;
					TotalPathNum++;
				} 
				if (ppos == pend) break;
			}                    
			JD_DestoryMap(hmap);       
		}  
		else  
		{
			while(JD_GetOneDepthPath(p2dLB,hNewDIB,fHeight,(float)GetCutDepth(),fToolRadius,&ppos,fBuffer,&num))
			{
				if (num)
				{   
					CPathPLine3D * pSect = new CPathPLine3D();
					pSect->Copy( fBuffer, num - 1 ) ;
					pTComb->AddEntity( pSect ) ;
					TotalPathNum++;
				} 
				if (ppos == pend) break;
			} 
		}
		JD_DestoryMap(hNewDIB);
		NewPath.AddData(0.0, pTComb ) ; 
		bRetFlag = TRUE ;
	}
	delete[] lpDibData ; 
	return bRetFlag ;
}
CSmartBReliefGen::CSmartBReliefGen()
{
}
CSmartBReliefGen::~CSmartBReliefGen() 
{
}
BOOL MathCAM_BuildSmartImage( CSmartImage& Cloud , HDIB& hDib )
{
	BYTE   bColor[3], bBitCount ;
    PALETTEENTRY  *pEntry = NULL;
	int  nNumColor, nWidth, nHeight, i, j; 
	unsigned char nIndex ; 
    //STEP 0 : 准备图像数据 
    LPSTR lpDIB = (LPSTR)::GlobalLock(hDib);
    bBitCount   = (BYTE)((LPBITMAPINFOHEADER)lpDIB)->biBitCount;
	if( bBitCount != 8 && bBitCount != 24 ) return 0 ;
	CPalette  Palette ;
    if( CreateDIBPalette( hDib, &Palette ) )
	{
		nNumColor = Palette.GetEntryCount() ;
	    pEntry  = new PALETTEENTRY[nNumColor] ;
	    Palette.GetPaletteEntries( 0, nNumColor, pEntry ) ;
	}
    LPSTR pData = FindDIBBits( lpDIB ) ;
    nHeight     =  DIBHeight(lpDIB) ;
    nWidth      =  DIBWidth(lpDIB) ;
	// STEP 1 : 初始化三角片
	Cloud.InitBuffer( nWidth-1, nHeight-1) ; 
	int nStrip  = WIDTHBYTES( nWidth * bBitCount ) ;
	int nOffset ; 
    // STEP 2 : 读取图像数据
    for( i = 0 ; i < nHeight; i ++ )
	{
	  for( j = 0 ; j < nWidth ; j ++ ) 
	  {
         nOffset =  i * nStrip + (j * bBitCount)/8;
		 if( bBitCount == 8 )
		 {
			 nIndex    = (unsigned char) pData[ nOffset] ;
			 bColor[0] = (BYTE) pEntry[nIndex].peRed   ;
			 bColor[1] = (BYTE) pEntry[nIndex].peGreen ;
			 bColor[2] = (BYTE) pEntry[nIndex].peBlue  ;
		 }
		 else if( bBitCount == 24 )
		 {
			 bColor[2] =  (BYTE)pData[nOffset     ] ;
			 bColor[1] =  (BYTE)pData[nOffset + 1 ] ;
			 bColor[0] =  (BYTE)pData[nOffset + 2 ] ;
		 }
         Cloud.m_fHeight[i][j] = 0.3f*bColor[0] + 0.59f*bColor[1] + 0.11f*bColor[2]; 
	  }
	}
    GlobalUnlock(hDib);
	if( pEntry ) delete[]  pEntry ;
	return 1 ;
}

BOOL CSmartBReliefGen::GeneratePathEx( CPathGroup& NewPath,   /*雕刻路径*/ 
		                              CSmartGraphic& Graph)   /*图形函数*/
{
	// STEP 1 : 建立图片模型
	BITMAP_INFO  BmpInfo ;
	if( ! Graph.GetBitmapInfo( BmpInfo ) ) 
	{
		return FALSE ;
	}
    if( !BmpInfo.m_hDib) 	return NULL ;   
	CSmartImage  Cloud ;
	Cloud.InitBitmap( (float)(BmpInfo.m_dCorner[0]), 
		              (float)(BmpInfo.m_dCorner[0] + BmpInfo.m_dSize[0]) ,
		              (float)(BmpInfo.m_dCorner[1]) ,
					  (float)(BmpInfo.m_dCorner[1] + BmpInfo.m_dSize[1] ) ) ;
	HDIB   hDib = (HDIB) BmpInfo.m_hDib ;
	if( ! MathCAM_BuildSmartImage( Cloud, hDib ) )
	{
		return FALSE ;
	}
	float fZMin = (float) GetTopHeight() ;
	float fZMax = (float) GetBottomHeight() ;
	Cloud.ScaleToRange( fZMin, fZMax, m_cParam.m_nEffectMode == 0 ) ;
    // STEP 2 : 构建路径点
	CSmartLoop	LoopLib ;
	CSmartLoop* AllLoop = Graph.GetAllContour( m_cSetupDef ) ; 
	if( !AllLoop )
	{
		PNT3D dMin , dMax ;
		dMin[2] = dMax[2] = 0.0 ;
		dMin[0] = BmpInfo.m_dCorner[0] ; 
		dMin[1] = BmpInfo.m_dCorner[1] ; 
		dMax[0] = dMin[0] + BmpInfo.m_dSize[0] ;
		dMax[1] = dMin[1] + BmpInfo.m_dSize[1] ;
	    AllLoop = new CSmartLoop() ;
		AllLoop->CreateLoop( dMin, dMax ) ;
	}
	CSmartLoop *pLoop ;
	CPathCombine  tmpComb( NC_WPROCESS_ROUGH ) ;
	for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
	{
		CreateLinearPath( tmpComb, *pLoop, m_cParam.m_cLinear ) ;
	}
	Mini_DeleteContours( AllLoop ) ;
	CSmtCPathLib  AllPath  ;
	AllPath.CreateCutPath( tmpComb ,GetCurveTol()) ;
    // STEP 3 :干涉检查, 生成无干涉的刀具路径
	int nPnt = AllPath.GetNumPoint() ;
	if( nPnt < 1 )  nPnt = 1      ;
	m_cPrgDef.m_dLimitAt  =  nPnt / 100.0   ;   // 前进一次的计算量
	m_cPrgDef.m_dStepAt   =  0.0            ;   // 当前计算进度
	m_cPrgDef.m_dIncStep  =  1.0            ;   // 每次计算的前进量
    if( !Cloud.VerifyCutPath( AllPath, GetSurfaceTol().m_dArcTol, m_cPrgDef ) )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ;
	}
    // STEP 4 : 加入路径结构
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine( * pPComb ) ;
	NewPath.AddData( 0.0, pPComb ) ;
	return TRUE ;
}