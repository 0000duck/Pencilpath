// RectNetRoughCounter.h : 计算矩形网格等高轮廓函数 的 头文件
//guomin 2012/10
///////////////////////////////////////////////////////////////////

#ifndef __RECTNETROUGHCOUNTER
#define __RECTNETROUGHCOUNTER

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// 加工刀具定义
struct SRectNetRoughTool
{
	int m_nToolType ;		// 刀具类型 
	//				0----ball tool;		球刀
	//				1----flat tool;		平底刀
	//				2----V bit tool		锥度平底刀
	//				3----coned ball		锥度球头刀
	//				4----coned bull nose锥度牛鼻刀
	//				5----bull nose		牛鼻刀

	float m_fDiameter ;		// 刀具直径
	float m_fDiameterBase ;	// 刀具底直径
	float m_fAngleFull    ;	// 锥全角
	float m_fRoundRadius ;	// 圆角半径
} ;

class CToolPathContours 
{
public :
	CToolPathContours():
	   aPts(NULL),
		   anPtsInCurve(NULL),
		   nCurve(0),
		   aReverse(NULL)
	   {
	   }

	   ~CToolPathContours()
	   {

	   }
	   void cleardata()
	   {
		   for( int i=0 ; i<nCurve ; i++ )
		   { 
			   if( aPts[i] )
			   {
				   delete[] aPts[i] ;
				   aPts[i] = NULL ;
			   }
		   }
		   if( aPts ) 
		   {
			   delete []aPts ;
			   aPts = NULL ;
		   }
		   if( anPtsInCurve) 
		   {
			   delete []anPtsInCurve ;
			   anPtsInCurve = NULL ;
		   }
		   if( aReverse )
		   {
			   delete []aReverse ;
			   aReverse = NULL ;
		   }
	   }

public :
	PT_3D	**	aPts ;			// int curves
	int		*	anPtsInCurve ;	// num of points in each int. curve
	int			nCurve ;		// num of int. curves
	BOOL	*	aReverse ;		// Reverse Loop Curves
	float		fZ ;			// the height---Z
};


BOOL VirtualCarving(		int mode,			//1--virtual carving; 0--tip center surf
	CGeoNetSurf* pSurf,
	float diameter,
	float diameterBase,	
	float angleFull,	
	float roundRadius,   
	int   toolType,			//0--ball tool; 1--flat tool; 2--v-bit tool
	JDNC_PRGDEF &PrgDef)		;

BOOL EnlargeRectMesh(CGeoNetSurf*& pSurf,int num);
void CalAllContours(	
	CGeoNetSurf * pCreatedRect ,
	float zValue,
	CToolPathContours & contour );
CSmartLoop *ConvertToSmartLoop(PT_3D pts[],int nPt,float tol);
float CalPtsArea( PT_3D * aPts ,int ai[], int nPts ) ;
float CalPtsArea( PT_3D * aPts , int nPts ) ;
BOOL OffsetRectMesh(CGeoNetSurf* &pSurf,	float radius) ;
BOOL RecreateMesh(CGeoNetSurf *&pSurf,float fToolDia,float fRemainder,float flowz ) ;
void	RectSurfMirror(CGeoNetSurf *&pSurf) ;//用于将曲面Z值反向,便于进行负向等距

#endif // !__RECTNETROUGHCOUNTER