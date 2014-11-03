#ifndef __SMART_AUTOFINISH_GEN_H__
#define __SMART_AUTOFINISH_GEN_H__

// 计算平面雕刻加工路径
#include "SmtPathGen.H"
class CSmtCheckMdl ;
class CSmartAutoFinishGen;
#define MIN_FLT		1.0e-6
// 创建平行截线精加工路径的线程数据
typedef struct CreateParallelPathThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	CSmtCPathLib *TmpPath ;
	LPVOID *	 AllPath ;
	CSmtCheckMdl *DriveMdl;
	JDNC_PRGDEF   PrgDef  ;
    BOOL         bRandMode;
	CSmartAutoFinishGen *pFinishGen;
}CPPT_DATA;

typedef struct TransFrom3AxisTo5AxisThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	CSmtCheckMdl *DriveMdl;
	CSmtCPathLib *OrgPath;
	LPVOID      *NewPath;
	JDNC_PRGDEF ProgDef;
	CNc5DAxisCtrl *pAxisCtrl;
	double dCur;
	CSmartPathGen *pPathGen;
}TF3X5XT_DATA;

// 平行截线线程函数
static UINT MathCAM_CreateParallelPathSubProc(LPVOID lpParam);
// 径向放射线程函数
static UINT MathCAM_CreateOrgEmitPathSubProc(LPVOID lpParam);
// 三轴转五轴线程函数
static UINT MathCAM_TransFrom3AxisTo5AxisSubProc(LPVOID lpParam);
// 得到曲面的法矢走向
int  MathCAM_GetSurfNormalDir(CGeoTrmSurf* pSurf, RFRAME& cRFrame);
// 得到合适的UW参数
double MathCAM_UWParam( double uInit );
//////////////////////////////////////////////////////////////////////////

class DLL_EXPORT CSmartAutoFinishGen : public CSmartPathGen
{
public :
    CSmartAutoFinishGen() ;
    virtual ~CSmartAutoFinishGen() ;
public:
    CSmartGraphic *m_pSmtGraph  ; // 加工图形边界 
    BOOL   m_bTrimByZSize       ; // 使用高度裁减
    BOOL BuildSFinishGen( CPathGroup&   NewPath  ,  /*雕刻路径*/
                          CSmartGraphic& Graph   , 
                          CSmtCheckMdl& DriveMdl , 
                          CSmtCheckMdl& CheckMdl ,
                          CSmartLoop*&   BoundCont,
						  double dAngle = 0. );

    // 根据加工面范围重设保护面范围
    virtual void SetCheckFltBoxByDriveMdl( CSmtCheckMdl& DriveMdl ,
                                           CSmtCheckMdl& CheckMdl );
	friend UINT MathCAM_CreateParallelPathSubProc(LPVOID lpParam);
private :
    CPathGroup*   m_pDebugPath  ;

public :
    // 计算所有区域的加工路径
    BOOL GenerateAllAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath ,
                                 JDNC_PRGDEF&  PrgDef );
    // 计算平坦区域的加工路径
    BOOL GenerateFlatAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath ,
                                 JDNC_PRGDEF&  PrgDef );
    // 计算陡峭区域的加工路径
    BOOL GenerateSteepAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath  ,
                                 JDNC_PRGDEF&  PrgDef );
    // 计算平面区域的加工路径
    BOOL GeneratePlaneAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath ,
                                 JDNC_PRGDEF&  PrgDef );
public :
    // 计算平行截线路径
    int  CreateParallelPath( CSmtCheckMdl& DriveMdl , 
                             DOUBLE  LineAngle      ,
                             CSmartLoop* AllCont    , 
                             CSmtCPathLib& AllPath  ,
                             JDNC_PRGDEF&  PrgDef );
    // 计算3D路径间距的平行截线路径
    int  CreateParallel3DPath( CSmtCheckMdl& DriveMdl , 
                               DOUBLE  LineAngle      ,
                               CSmartLoop* AllCont    , 
                               CSmtCPathLib& AllPath  ,
                               JDNC_PRGDEF&  PrgDef   );
	// 计算导动线3D路径间距的平行截线路径
	int  CreateGuideParallel3DPath( CSmtCheckMdl& DriveMdl , 
                               DOUBLE  LineAngle      ,
                               CSmartLoop* AllCont    , 
                               CSmtCPathLib& AllPath  ,
                               JDNC_PRGDEF&  PrgDef   );

    // 计算平面环绕等距路径
    int  CreateFollowPath ( CSmtCheckMdl& DriveMdl , 
                            JDNC_FOLLOW&  FollowDef,
                            CSmartLoop* AllCont    , 
                            CSmtCPathLib& AllPath  ,
                            JDNC_PRGDEF&  PrgDef   );
    // 计算径向放射加工路径
    int  CreateSEmitPath ( CSmtCheckMdl& DriveMdl, 
                           CSmartLoop* AllCont   , 
                           CSmtCPathLib& AllPath ,
                           JDNC_PRGDEF&  PrgDef );
    // 计算3D环绕等距路径
    int  Create3DStepPath ( CSmtCheckMdl& DriveMdl ,
                            JDNC_FOLLOW&  FollowDef,
                            CSmartLoop* AllCont    ,
                            CSmtCPathLib& AllPath  ,
                            JDNC_PRGDEF&  PrgDef  );
    // 计算等高加工路径
    int  CreateHeightPath ( CSmtCheckMdl& DriveMdl , 
                            CSmartLoop* AllCont    , 
							int 		ValidMode  ,
                            BOOL        IsCutPlane , 
                            DOUBLE      SlopeAngle ,
                            CSmtCPathLib& AllPath  ,
                            JDNC_PRGDEF&  PrgDef  );
    // 计算曲面流线加工路径
    int  CreateUWLinePath ( CSmtCheckMdl& DriveMdl, 
                            CSmartLoop* AllCont, 
                            CSmtCPathLib& AllPath,
                            JDNC_PRGDEF&  PrgDef );
    // 计算无限多笔精加工路径
    int  CreateMPencilPath ( CSmtCheckMdl& DriveMdl, 
                             CSmartLoop* AllCont   , 
                             CSmtCPathLib& AllPath,
                             JDNC_PRGDEF&  PrgDef );
public :
    // 计算平坦加工域边界轮廓
    BOOL CreateFlatAreas( CSmtCheckMdl& DriveMdl,     // 加工模型
                          CSmartLoop*   AllCont ,     // 区域范围
                          DOUBLE SlopeAngle     ,     // 分区角度, if SlopeAngle < 0.0 , 为所有区域, if SlopeAngle == 0.0, 为平面区域, 否则为平坦区域
                          CSmartLoop*& FlatAreaBnd  , // 平坦区域边界
                          DOUBLE     UsedBndTol ,     // 边界偏移量 
                          JDNC_PRGDEF&  PrgDef,
						  double dStep,               // 路径间距
						  BOOL bFollow,               // 平坦部分是否环切
						  CSmartLoop*& SteepAreaBnd , // 峭区域边界 qqs、xiejunxian 2013.04.25
						  DOUBLE dExtLen = 0.0);      // 重叠区域宽度 qqs、xiejunxian 2013.04.25
    // 计算平行截线分区加工边界轮廓
    BOOL CreateXParallelAreas( CSmtCheckMdl& DriveMdl,// 加工模型
                               CSmartLoop*   AllCont ,// 区域范围 
                               DOUBLE SlopeAngle     ,// 分区角度, 5 ~ 85 度
							   DOUBLE dOverLapDist   ,// 重叠区域宽度
                               DOUBLE LineAngle      ,// 行切角度
                               CSmartLoop* AreaBnd[2],// [0], 平坦区域边界,[1]陡峭区域边界
                               DOUBLE  UsedBndTol    ,// 边界偏移量 
                               JDNC_PRGDEF&  PrgDef );
    // 计算角度分区加工域边界轮廓
    BOOL CreateMixtureAreas( CSmtCheckMdl& DriveMdl ,// 加工模型
                             CSmartLoop*   AllCont  ,// 区域范围
                             DOUBLE SlopeAngle,      // 分区角度, 5 ~ 85 度
							 DOUBLE dOverLapDist,	 // 重叠区域宽度
							 BOOL	bFollow	,		 // 平坦部分是否环切
							 double	dStep		 ,	 // 路径间距
                             CSmartLoop* AreaBnd[2], // [0], 平坦区域边界，[1]陡峭区域边界
                             int    AreaType       , // 区域类型
                             JDNC_PRGDEF&  PrgDef );
	// 生成加工区域轮廓
	BOOL CreateNcAreas ( CSmtCheckMdl	& DriveMdl,	// 加工模型
						 CSmartLoop		*& AllCont );	// 区域范围

	void KeepSharpEdges(CSmtCPathLib & AllPath,CSmtCheckMdl & DriveMdl);

	//等高路径延伸（包括向外和向上延伸）
	void ExtendFHeightPath(CSmtCPathLib & AllPath, CSmtCheckMdl& DriveMdl);

	//样条拟合路径
	BOOL PathSplFit(CSmtCPathLib & AllPath);
	BOOL PathSplFit(CSmtCPathLib & AllPath, CPathCombine & PathComb);

protected :
    // 根据边界设置过滤曲面的范围
    void SetCheckFltBoxByContours( CSmartLoop* AllCont, CSmtCheckMdl& DriveMdl, CSmtCheckMdl& CheckMdl ) ;
    // 生成导动加工曲线
    void CreateGuideCurve( CSmartGraphic& Graph, CSmtCheckMdl& DriveMdl, 
                           CSmartLoop*  AllCont, CSmtCPathLib& GuideLib ) ;
    int  CreateOneParallel3DPath( CSmtCheckMdl& DriveMdl, 
                                  CSmartLoop&   BndCont , 
                                  CSmtCPathLib& AllPath ,
                                  JDNC_PRGDEF&  PrgDef  );
    // 计算区域修边路径
    BOOL CreateBoundPath( CSmtCheckMdl& DriveMdl, CSmartLoop* AllCont, CSmtCPathLib& BndPath ) ;
private:
	// 生成初始的截线路径
	void GenOrginalPath(CSmartLoop *AreaCont, CSmtCPathLib &OrgPath);
	// 平行截线线程函数
	BOOL CreateParallelPathSubProc(int nAtCore, int nCoreNum, CSmtCPathLib &TmpPath, LPVOID* AllPath, 
		                           CSmtCheckMdl &DriveMdl, BOOL RandMode , JDNC_PRGDEF &PrgDef);


	// 变间距平行截线单线程和多线程的函数
	void SingleThreadCPPEx(CSmtCheckMdl& DriveMdl, CSmartLoop *AreaCont,
						   CSmtCPathLib &AllPath,double * aY,int nY,JDNC_PRGDEF &PrgDef);		// 单线程计算平行截线路径
protected:
	// 生成曲面边界轮廓
	CSmartLoop* GenSurfBorder(CSmartGraphic *pSmtGraph, CSmtCheckMdl *pDriveMdl, double dAngle);
	// 生成沿边界连刀的路径
	int CreatePathhByBound(CSmartGraphic &Graph, CSmtCheckMdl &DriveMdl, 
						   JDNC_LINEAR& FLinear, JDNC_SETUP& Setup, 
						   DOUBLE dAngle, CSmtCPathLib& AllPath);
};
///////////////////////////////////////////
// CSmartFLinearGen: 曲面平行截线精雕刻
//
class DLL_EXPORT CSmartFLinearGen : public CSmartAutoFinishGen
{
public :
	CSmartFLinearGen() ;
	virtual ~CSmartFLinearGen() ;
public:
	JDNC_LINEAR  m_cParam    ;  // 雕刻参数
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
};

///////////////////////////////////////////
// CNCSFinishBPath: 等高外形曲面精雕刻
//
class DLL_EXPORT CSmartFHeightGen : public CSmartAutoFinishGen
{
public :
	CSmartFHeightGen() ;
	virtual ~CSmartFHeightGen() ;
public :
	JDNC_HEIGHT		m_cParam ; 

public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }

};
///////////////////////////////////////////
// CNCSFinishAPath: 曲面平行截线精雕刻
//
class DLL_EXPORT CSmartMixtureGen : public CSmartAutoFinishGen
{
public :
	CSmartMixtureGen() ;
	virtual ~CSmartMixtureGen() ;
public:
	JDNC_MIXTURE  m_cParam    ;  // 雕刻参数
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	int  GenToolPath(CSmartGraphic& Graph, CMachineMdl& MachMdl, CSmartLoop* AllLoop, CPathGroup& NewPath) ;
	virtual BOOL IsSupportProgress() { return TRUE ; }

};

///////////////////////////////////////////
// CSmartFEmitGen: 径向放射曲面精雕刻
//
class CSmartFEmiterGen;
// 计算径向放射原始加工路径的线程数据
typedef struct CreateOrgEmitPathThreadData  
{
	int nAtCore;
	int nCoreNum;
	double dRadius;
	BOOL DefNull;
	CSmtCheckMdl *pDriveMdl;
	JDNC_PRGDEF pTmpPrg;
	CSmartLoop* BndCont;
	LPVOID* AllPath;
	CSmartFEmiterGen *pFEmiterGen;
}COEPT_DATA;

class DLL_EXPORT CSmartFEmiterGen : public CSmartAutoFinishGen
{
public :
	CSmartFEmiterGen() ;
	virtual ~CSmartFEmiterGen() ;
public:
	JDNC_EMIT  m_cParam    ;  // 雕刻参数
    DOUBLE     m_dMaxRadius;  // 最大半径
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }

	friend UINT MathCAM_TransFrom3AxisTo5AxisSubProc(LPVOID lpParam);
public :
	// 轮廓排序
	CSmartLoop* ResortContour(CSmartLoop* AllLoop );
	// 生成刀具路径
protected :
	// 计算原始加工路径
	int CreateOrgEmitPath( CSmtCheckMdl& DriveMdl, /*网格模型*/ 
		                 FPNT3D MaxBox[2]     , /*包围盒  */
                        CSmartLoop* BndCont   , /*边界裁剪  */
                        BOOL   DefNull       , /*定义无效点 */
                        CSmtCPathLib&  AllPath); /*原始路径 */
	// 计算兜边的加工路径
	int CreateEmitPathBnd( CSmtCheckMdl& DriveMdl, /*网格模型*/ 
		                 FPNT3D MaxBox[2]     , /*包围盒  */
                        CSmartLoop* BndCont   , /*边界裁剪  */
                        CSmtCPathLib&  BndPath); /*陡边路径 */
	// 生成刀具路径的子函数
	BOOL GeneratePathExSubFunc(CSmtCheckMdl &DriveMdl, CSmartLoop *AllLoop, CSmtCPathLib &BndPath, 
		CSmtCPathLib &AllPath);
public:
	// 计算原始加工路径的主函数
	int CreateOrgEmitPathSubProc(int nAtCore, int nCoreNum, double dRadius, BOOL DefNull, 
		CSmtCheckMdl &DriveMdl, JDNC_PRGDEF &TmpPrg, 
		CSmartLoop* BndCont, LPVOID* AllPath);
};
///////////////////////////////////////////
// CSmartFUWLineGen: 曲面流线曲面精雕刻
// 
struct CCombSurf;
typedef CTypedPtrArray<CPtrArray, CCombSurf*>	CCombSurfArray;

class DLL_EXPORT CSmartFUWLineGen : public CSmartAutoFinishGen
{
public :
	CSmartFUWLineGen() ;
	virtual ~CSmartFUWLineGen() ;
public :
	JDNC_UWLINE m_cParam ;
public :
	// 生成刀具路径
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
	
	BOOL Create3AxUWPath( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, CSmartLoop *AllLoop,
						  RFRAME *dNcMtx, BOOL bCheck, double dCur[4], CPathGroup &NewPath ) ;

	// 新三轴算法函数，更改了构造面函数和构造面流线路径的算法 qqs 2013.12.17
	BOOL Create3AxUWPathNew( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, CSmartLoop *AllLoop,
		RFRAME *dNcMtx, BOOL bCheck, double dCur[4], CPathGroup &NewPath ) ;
	// Create3AxUWPath的子函数
	BOOL Create3AxUWPathSubFunc(CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmtCPathLib &AllPath, C3DSurfArray &AllSurf, 
		RFRAME *dNcMtx, double dCur[4], CSmartLoop *AllLoop, BOOL bCheck);
	BOOL Create5AxUWPath( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, 
						  CSmartLoop *AllLoop, RFRAME *dNcMtx, double dCur, CPathGroup &NewPath ) ;
	// 自动干涉检查
	void AutoCheckAllPath( CSmtCheckMdl& DriveMdl, CPathGroup&  NewPath ) ;
	// 把首尾相连的路径合并
	void CombinePath(CSmtCPathList &PathList);
	// 添加进退刀
	void AddAllPathLeadPath( CSmtCheckMdl &DriveMdl, CSmtCPathLib &AllPath, BOOL bCheck, 
		double dLeft, BOOL *arrbSpiral = NULL, BOOL bCombSurfPath = FALSE) ;
	// 如果有非CSmtCutPointEx点,转换点
	void TransfSmtCutPath( CSmtCutPath *pPath ) ;
protected:
	// 清除点数为1的路径
	void DelOnePntPath(CSmtCPathLib &AllPath);
	// 移动一层路径，返回该层号
	int MoveOneLayerPath(CSmtCPathLib &AllPath, CSmtCPathLib &LayerPath);
	// 把同一层号和行号的路径移动到目标路径组
	void MoveSameLinePath(CSmtCPathList &PathFrom, CSmtCPathList &PathTo, POSITION pos);
	// 找到与已知路径最接近的路径位置
	POSITION GetClosestPathPos(const CSmtCPathList &PathFrom, CSmtCutPath *pPath, POSITION pos, BOOL bForward = TRUE, BOOL bCombSurfPath = FALSE);
	// 定义两参数所构成的范围，bEx为真时，该范围被分为两部分，从0到较小值，从较大值到1，
	// 为了给它变成一个范围，把较小值加1，范围变成较大值到较小值加1
	void DefineRange(TFLOAT f1, TFLOAT f2, BOOL bEx, TFLOAT &fMin, TFLOAT &fMax);
	// 定义路径的UW参数包围盒
	void DefineUWBox(CSmtCutPath *pPath, FPNT2D UWBox[2], BOOL &bEx);
	// 两路径UW参数上的公共区域
	TFLOAT TwoPathUWComArea(CSmtCutPath *pPath1, CSmtCutPath *pPath2);
	// 判断路径是否在同一区域内
	BOOL PathInSameArea(CSmtCutPath *pPath1, CSmtCutPath *pPath2, BOOL bCombSurfPath = FALSE);
	// 求两条路径的接近程度，返回值越大，越接近
	TFLOAT TwoPathClose(CSmtCutPath *pPath1, CSmtCutPath *pPath2);
	// 得到与已知路径距离最短的路径位置
	POSITION GetMinDistPathPos(const CSmtCPathList &PathFrom, CSmtCutPath *pPath);
	// 两路径之间的最短距离
	DOUBLE TwoPathMinDis(CSmtCutPath *pPath1, CSmtCutPath *pPath2);
	// 从路径中均匀取nSize(nSize<=5)个点，返回个数
	int SelectPathPoints(CSmtCutPath *pPath, FPNT3D SelPts[]);
	// 得到前一组路径
	void GetPrevLinePaths(CSmtCPathList &PrevPaths, const CSmtCPathList &PathList, POSITION pos);
	// 路径组中存在与输入路径相同区域的路径
	CSmtCutPath *GetSameRegionPath(const CSmtCPathList &PathList,  CSmtCutPath *pPath, BOOL bCombSurfPath = FALSE);
	// 沿相反方向，从输入路径开始，把同区域的路径移到目标路径组中
	POSITION RegionPathBackward(CSmtCPathList &PathList, CSmtCutPath *pPathBegin, BOOL bCombSurfPath = FALSE);
	// 根据区域对路径排序
	void SortPathByRegion(CSmtCPathList &PathList, BOOL bCombSurfPath = FALSE);	
	// 对每一层路径添加连刀
	void AddLayerPathLeadPath(CSmtCheckMdl &DriveMdl, BOOL bCheck, CSmtCPathLib &LayerPath);
	BOOL CreateAllCombSurfPath(CSmartGraphic &Graph, C3DSurfArray &CombSurfs, CSmtCheckMdl &DriveMdl, 
		                       RFRAME *dNcMtx, CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg);
	// 边界构面生成路径
	BOOL CreateCombineSurfPath(CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf *TrimSurf, CSmtCheckMdl& DriveMdl,RFRAME *dNcMtx,
		CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg);
	// 新构面生成路径函数，解决由于投影模型错误引起的生成的路径混乱的问题 qqs 2013.11.15     
	BOOL CreateAllCombSurfPathNew( CSmartGraphic &Graph, CCombSurfArray &CombSurfArray, CSmtCheckMdl &DriveMdl,CSmtCheckMdl &DrMdl,
	                               RFRAME *dNcMtx, CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg,double dCur[4]);
	// 新构造面路径计算函数 qqs 2013.12.17
	BOOL CreateCombineSurfPathNew(CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf *TrimSurf, CSmtCheckMdl& DriveMdl, RFRAME *dNcMtx,
		                       CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg, int nLayerNo, BOOL* arrbSpiral);
	//把一组曲面变成若干张4边面
	BOOL SurfsTo4BndSurf(C3DSurfArray &GeoSurfList, C3DSurfArray &SurfNew, C3DSurfArray &SurfLeft);
	//把一组曲面变成若干张4边面，调用新的构造面函数，可以返回每个构造面对应的原始曲面 qqs 2013.11.15
	BOOL SurfsTo4BndSurfNew(C3DSurfArray &GeoSurfList, CCombSurfArray &CombSurfArray, C3DSurfArray &SurfLeft);
	// 调整合并后的曲面
	void ModifyCombineSurf(RFRAME *dNcMtx, CGeoTrmSurf* pCombineSurf);
	// 生成参数域直线段
	int CreateParamLine(CGeoTrmSurf* TrimSurf, DOUBLE UWStep[2], CSmartCurve& AllLine, int& nMoveDir, RFRAME& dNcMtx, BOOL bSpiral = FALSE);
	// 调整路径的起点
	void AdjustStartAt( CSmartCurve& AllLine, BOOL bSpindle, int nMoveDir );
	// 路径裁剪
	void TrimPath(CSmtCPathLib &AllPath, CSmartLoop *AllLoop);
	
	// 重置顶部与底部路径节点，减少因加工曲面流线不整齐生成的不整齐路径按加工深度裁剪后的过多抬刀 qqs 2013.06.21
	void AdjustTopAndBotPath(CSmtCPathLib &AllPath, double dTopHeight, double dBottomHeight, double dArcTol);
	// 若相邻路径的起末点距离过近，将两点重设为同一个点，
	// 主要用于构造面生成路径时，构造的闭合面闭合边界精度较低时，生成的路径不连续的情况 qqs 2013.06.21
	void ResetSameCutPnt(CSmtCPathLib& TmpPath, BOOL bSpiral, double minDis = 0.01);

	/// 以下函数为创建曲面流线磨削调整半径磨损补偿的函数 qqs
	//////////////////////////////////////////////////////////////////////////
	// 为路径增加三维补偿偏移方向适量
	void Add3DRCompVectorToPath(CSmtCPathLib &AllPath, CPathGroup &NewPath, int nPathType = 0);// 0为普通面生成的三轴路径，1为构造面按五轴投影方法生成的三轴路径，2为五轴路径
	// 开启进退刀时，在进退刀路径上增加进退刀延长线方向的磨损补偿延长路径 qqs 2013.09.25
	CSmtCutPath* CalRCompByLeadPath(CSmtCutPath*& pLead, BOOL bIsOut, BOOL bIs3D = TRUE);
	// 关闭进退刀，或进退刀未生成时，通过切削路径计算于加工曲面点法向垂直的磨损补偿延长路径 qqs 2013.09.25
	CSmtCutPath* CalRCompByRoughPath(CSmtCutPath*& pRough, BOOL bIsOut, BOOL b3D, BOOL bIs5Ax = FALSE);
	// 创建半径磨损补偿建立段路径
	BOOL  CreateRCompPath( JDNC_LEAD&	  LeadDef ,
			 			   CSmtCutPoint*  ptStart ,
						   CSmartLoop*    Contour ,
						   CSmtCutPath*&  RCompLeadPath,
						   VEC3D          Normal,
						   BOOL 		  IsOut);
	// 为五轴方法生成的路径添加磨损补偿建立路径段
	void AddRCompPathTo5AxPathLib(CSmtCPathLib& CutPathLib, BOOL bIs3D = FALSE);
	// 获取半径磨损补偿标记
	int	 Get3DRCompMask ();
	//////////////////////////////////////////////////////////////////////////
};
///////////////////////////////////////////
// CSmartFFollowGen: 环绕等距精雕刻
//
class DLL_EXPORT CSmartFFollowGen : public CSmartAutoFinishGen
{
public :
	CSmartFFollowGen() ;
	virtual ~CSmartFFollowGen() ;
public :
	JDNC_FFOLLOW m_cParam ; 
public :
	// 生成刀具路径
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
public :
	// 轮廓排序
	CSmartLoop* ResortContour(CSmartLoop* AllLoop );

	//环绕等距消除马塞克
	BOOL RemoveMosaic(CSmtCPathLib  &AllPath, CSmtCheckMdl & DriveMdl);
};

#endif // __SMART_AUTOFINISH_GEN_H__