#ifndef __SMT_NEWUWLINEPATH_GEN_H__
#define __SMT_NEWUWLINEPATH_GEN_H__

// 计算平面雕刻加工路径
#include "SmtAutoFinishGen.H"

class CSmartAutoFinishGen;

///////////////////////////////////////////
// CSmtNewUWLinePathGen: 新曲面流线曲面精雕刻
//
class DLL_EXPORT CSmtNewUWLinePathGen : public CSmartAutoFinishGen
{
public :
	CSmtNewUWLinePathGen() ;
	virtual ~CSmtNewUWLinePathGen() ;
public :
	JDNC_UWLINE m_cParam ;
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath  ,  /*雕刻路径*/
								CSmartGraphic& Graph    ) ; /*雕刻图形*/
	BOOL CreatOneSurfPath(CSmartLoop* AllLoop, CGeoTrmSurf* pSurf,CPathGroup& NewPath);//生成单个面的加工路径
	void CreatParamPath(CGeoTrmSurf* pSurf, CSmtCutPath& ParamPath, float fBox2d[2][2], int pCutDir);//计算参数域路径
	void CreatStepPath(CSmtCutPath* ParamPath, CSmtCutPath& pNewPath);//生成路径节点满足路径间距的参数域路径
	BOOL CreatFinalPath(CSmartLoop* AllLoop, CSmtCPathLib& AllPath, CSmtCutPath* pNewPath, CGeoTrmSurf* pSurf, float fBox2d[2][2], int pCutDir);//生成最终的曲面加工路径
	void PathTrim(CSmartLoop* AllLoop, CGeoTrmSurf* pSurf,CSmtCutPointEx* ptFEnd[2], CSmtCPathLib& AllPath);//生成符合加工曲面边界的路径
	void MappingPath(CSmtCutPath* pCutPath, CGeoTrmSurf* pSurf);//将参数域路径向曲面进行投影
	void MappingPnt(PNT2D ptOr, CGeoTrmSurf* pSurf, PNT3D ptSurf, VEC3D nor);//将参数域点向曲面投影
	void CalCLPt(PNT3D ptSurf, VEC3D nor, PNT3D ptCL);//计算刀尖点
};


#endif // __SMT_NEWUWLINEPATH_GEN_H__