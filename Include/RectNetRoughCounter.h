// RectNetRoughCounter.h : �����������ȸ��������� �� ͷ�ļ�
//guomin 2012/10
///////////////////////////////////////////////////////////////////

#ifndef __RECTNETROUGHCOUNTER
#define __RECTNETROUGHCOUNTER

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// �ӹ����߶���
struct SRectNetRoughTool
{
	int m_nToolType ;		// �������� 
	//				0----ball tool;		��
	//				1----flat tool;		ƽ�׵�
	//				2----V bit tool		׶��ƽ�׵�
	//				3----coned ball		׶����ͷ��
	//				4----coned bull nose׶��ţ�ǵ�
	//				5----bull nose		ţ�ǵ�

	float m_fDiameter ;		// ����ֱ��
	float m_fDiameterBase ;	// ���ߵ�ֱ��
	float m_fAngleFull    ;	// ׶ȫ��
	float m_fRoundRadius ;	// Բ�ǰ뾶
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
void	RectSurfMirror(CGeoNetSurf *&pSurf) ;//���ڽ�����Zֵ����,���ڽ��и���Ⱦ�

#endif // !__RECTNETROUGHCOUNTER