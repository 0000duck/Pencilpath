#ifndef PATH_EXTEND_H
#define PATH_EXTEND_H


#include "SmartNC.H"

//非闭合等高路径向外直线延伸
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
				double ExtendDist,  //延伸距离
			   JDNC_SETUP& SetupDef,
			   JDNC_HOLDER & THolderDef,//刀柄
			   CSmtCheckMdl &DriveMdl,
			   BOOL ChkTHolder);        //是否进行刀柄干涉检查
	BOOL ExtendOutOnePath(CSmtCutPath *pPath, double ExtendDist,
			   JDNC_SETUP& SetupDef,CSmtCheckMdl &DriveMdl);
protected:
	//刀柄是否与毛坯(残料模型)干涉
	BOOL IsTHolderIntWithStock(CSmtCutPoint * pAtPnt,double dPathHeight,
						JDNC_STOCKEX & StockDef);

	//刀柄是否与曲面干涉
	BOOL IsTHolderIntWithSurf(CSmtCutPoint * pAtPnt,
						      CSmtCheckMdl &DriveMdl);
	BOOL IsLocalPathOverCut(double Begin[], double End[],
						CSmtCheckMdl &DriveMdl,double OverCutPt[]);
	BOOL PntHorPrjOnChkMdl(FPNT3D Pnt,PNT3D Dir,
					    CSmtCheckMdl & CheckMdl);
private:
	void CreateToolHolder(JDNC_HOLDER & THolderDef,CSmtCheckMdl &DriveMdl);
};

//路径点（包含延伸方向）
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
	BOOL    m_bValid     ; //标记该点是否有效
	VEC3D   m_vExtendDir ; //延伸方向
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
//向上延伸等高路径
BOOL ExtendUpHeigtPath( CSmtCheckMdl& CheckMdl,
						CSmtCPathLib& AllPath, 
						const double dExtndDist, 
						const double dStep, 
						const BOOL is3DStep ) ;
#endif