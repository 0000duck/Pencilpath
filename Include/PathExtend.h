#ifndef PATH_EXTEND_H
#define PATH_EXTEND_H


#include "SmartNC.H"

//�Ǳպϵȸ�·������ֱ������
class DLL_EXPORT CHeightExtendOut
{
public:
	CHeightExtendOut();
	~CHeightExtendOut();
private:
	BOOL           m_bChkTHolder;
	CSmartHolder * m_pTHolder;

public:
	void ClearAll();
	void ExtendOutSide(CSmtCPathLib& AllPath, 
				double ExtendDist,  //�������
			   JDNC_SETUP& SetupDef,
			   JDNC_HOLDER & THolderDef,//����
			   CSmtCheckMdl &DriveMdl,
			   BOOL ChkTHolder);        //�Ƿ���е���������
	BOOL ExtendOutOnePath(CSmtCutPath *pPath, double ExtendDist,
			   JDNC_SETUP& SetupDef,CSmtCheckMdl &DriveMdl);
protected:
	//�����Ƿ���ë��(����ģ��)����
	BOOL IsTHolderIntWithStock(CSmtCutPoint * pAtPnt,double dPathHeight,
						JDNC_STOCKEX & StockDef);

	//�����Ƿ����������
	BOOL IsTHolderIntWithSurf(CSmtCutPoint * pAtPnt,
						      CSmtCheckMdl &DriveMdl);
	BOOL IsLocalPathOverCut(double Begin[], double End[],
						CSmtCheckMdl &DriveMdl,double OverCutPt[]);
	BOOL PntHorPrjOnChkMdl(FPNT3D Pnt,PNT3D Dir,
					    CSmtCheckMdl & CheckMdl);
private:
	void CreateToolHolder(JDNC_HOLDER & THolderDef,CSmtCheckMdl &DriveMdl);
};

//·���㣨�������췽��
class CSmtCutPointHx : public CSmtCutPoint
{
public:
	CSmtCutPointHx( CSmtCutPoint& CutAt )
	{ 
		*((CSmtCutPoint*)this) = CutAt ;
		m_vExtendDir[0] = 0.0 ;
		m_vExtendDir[1] = 0.0 ;
		m_vExtendDir[2] = 1.0 ;
		m_bValid = FALSE ;
	} ;
	~CSmtCutPointHx(){} ;
public:
	BOOL    m_bValid     ; //��Ǹõ��Ƿ���Ч
	VEC3D   m_vExtendDir ; //���췽��
public:
	void setValid( BOOL bValid ){ 
		VEC3D vXYDir = {0};
		nc_VectorCopy( vXYDir, m_vExtendDir, 2 ) ;
		nc_Normalize( vXYDir, 3 ) ;
		DOUBLE dAngWithX = mathGetAngleUnit( m_vExtendDir, vXYDir ) ;
		m_bValid = dAngWithX < ANGLE_TO_RADIAN(10) ? FALSE : TRUE ;
		m_bValid = m_bValid && bValid ;
	}
	BOOL isValid(){ return m_bValid ;}
	void setExtendDir(FPNT3D cExtendDir) {
		nc_VectorCopy( m_vExtendDir, cExtendDir , 3 ) ;
	}
};
//��������ȸ�·��
BOOL ExtendUpHeigtPath( CSmtCheckMdl& CheckMdl,
						CSmtCPathLib& AllPath, 
						const double dExtndDist, 
						const double dStep, 
						const BOOL is3DStep ) ;
#endif