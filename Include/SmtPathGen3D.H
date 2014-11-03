#ifndef __SMART_PATHGEN3D_H__
#define __SMART_PATHGEN3D_H__

// 计算平面雕刻加工路径
#include "SmtPathGen.H"
#include "SmartPathEx.h"
#include "RectNetRoughCounter.h"

class CSmtMeshMdl ;
class CSameZValueSet ;
class CNcSmartPath ;
class CNcZLevelGenMdl ;
#define NCDEF_RPOCKET_DEPTH		0
#define NCDEF_RPOCKET_AREA		1
#define NCDEF_RPOCKET_CAST		2

typedef struct slot_path_info
{
	BOOL bSClose ;
	BOOL bEClose ;
	CSmtCutPath* pCutPath ;
}SlotPath ;
typedef struct surf_facet
{
	CGeoTrmSurf* pTrmSurf ;
	CFacet*     pFacet;
}SurfAndFacet;
class CEttFeatureSlot ;
class CEntity3D ;
class CEttSurface ;
typedef CTypedPtrList< CPtrList, SlotPath*>	SlotPathList ;
typedef CTypedPtrList< CPtrList, SurfAndFacet*>	SurfFacetList ;
typedef CTypedPtrList<CPtrList,CEntity3D*>	CEntList;

class DLL_EXPORT CSmartRPocketGen : public CSmartPathGen
{
public :
	CSmartRPocketGen()          ; 
	virtual ~CSmartRPocketGen() ; 
public :
	JDNC_RPOCKET    m_cParam     ;    // 雕刻参数
	CSmtCurveArr *  m_pCurveArr  ;    // 曲线组,用于层间精加工
	CSmartLoop**    m_pDriveLoop ;    // 留作保存使用的
	CSmtLoopArr *   m_pRoughCastLoopArr ; // 毛坯环组
	CSmartLoop**    m_pRoughLoop ;    // 毛坯的保存环
	int    *    m_pBtwPlgLayer ;  // 保存层间路径的下层层号
	// 以下三个为修剪方式生成路径添加的成员
	CSmtLPathArr*   m_pLPathArr  ;    // 用于保持本次路径环及生成的路径
	CSmartLoop * *  m_pPlunge    ;    // 连到环组
	CSmartCurveLib *m_pCurveLib  ;    // 曲线组

	double      *   m_dZ         ;    // 每层的高度
	BOOL		*	m_bLayer	 ;	  // 应用于层间加工
	CBtwLPathArr *  m_pBtwLPathArr ;   // 保存层间粗加工环相关信息
	CBtwCurveArr *  m_pBtwCurveArr ;  // 保存层间精加工曲线相关信息
	CArray<int,int> m_nPlaneLayer ;   //用于标记哪些层间环为平面环
	int             m_nCount     ;    // 总层数
	double			m_dStepZ     ;	  // 层间距
	double			m_dMaxZ		 ;	  // 过滤平面使用
	BOOL			m_bFirst	 ;	  // 是否第一个
	CSmartLoop *    m_pPlaneLoop ;    // 保存所有的平面环
	CPlaneList		m_cPlaneList ;	  // 记录平面环
	BOOL			m_bAllLoop	 ;	  // 是否有曲线边界
	CSmartLoop*     m_pAllLoop   ;    // 加工边界环
	CSmartTool	  * m_pRoughTool ;	  // 毛坯模型刀具
	BOOL			m_bBndTrim	 ;	  // 边界修剪标记
	JDNC_PRGDEF		m_cTmpPrgDef ;	  // 临时进度条

	double			m_dMinArea	 ;	  // 刀具底面面积

	CGeoNetSurf *m_pRectNetSurf ;	//网格快速开粗勾选后得到的矩形网格,木雕环境用于环的计算 guomin13/02/19

public :
	// 生成刀具路径
	virtual int GeneratePathEx( CPathGroup& NewPath   ,  /*刀具路径*/
				                CSmartGraphic& Graph );  /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
	void	ClearAllLoop() ;
	// 初始化临时进度条
	void	InitTmpProg() ;
	void	CalcTopAndBot( BOX3D &box ) ;
	// 计算分层
	int	GetECutLayers(CSmartLoop *&PlaneLoop ) ;
	// 层间加工
	void	SetCutStepBtwLayer() ;
	void	SetCutStepBtwLayerRough( CSmartLoop *&PlaneLoop ) ;
	BOOL	FilterPlaneZHeight( double *& dPlane_Z, int & nPlaneZCnt ) ;
	BOOL	MarkBottomLayer() ;
	int		FindPrevLayer( int nLayer ) ;
	int		FindNextLayer( int nLayer ) ;
	int		FindBwtLoopIndex(int nLayer) ;
	void    SetBtwLoopInfor() ;
	void    SortLibPathByArea( CSmtCPathLib & AllPath ) ;
	BOOL    IsCutBtwLoop( CSmartLoop * BndCurr, CBtwLPath *& pBtwLPath ) ;
	void	AddLPathToLPathArr( CSmtLPathArr &LPathArr, CSmtLoopPath *LPathHead ) ;
	void    ClearBtwLoop() ;
	void    AddLeadPath(CSmtCheckMdl &DriveMdl, BOOL bCheck, CSmtCPathLib &AllPath) ;
	//  层间精加工所用
	void           GetAllBtwCurves() ;
	CSmtCutPath *  TransfCurveToPath(  CSmartCurve * pCurve , JDNC_TOL &cTol, double dZ, BOOL IsDel ) ;
	BOOL           TransfCurveListToPath( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, CurveList & pCurveList , JDNC_PRGDEF &ProgDef ) ;
	BOOL           SortCurvesByDepth( CPathGroup&  NewPath, CSmtCheckMdl *DriveMdl, int nLayer, CurveList & CurList ) ;
	void           SetBtwCurveInfor() ;
	void           FindAllLayerCurve(int Layer,int Pre, CurveList & pCurveList ) ;
	BOOL           IsCutBtwCurve( CSmartLoop * BndCurr, CBtwCurve * pBtwCurve ) ;
	void           ClearAllBtwCurves() ;
	void           SetCurveListStart( CurveList & CurList ) ;
	// 设置加工环属性
	void	SetAllCutAreaHeadAtBlank() ;
	void    SetAllPlaneLoopHeadAtBlank() ;
	// 判断盲区
	BOOL	IsBlindLoop( CSmartLoop *pLoop ) ;
	// 构建毛坯模型
	BOOL	BuildStockMdl( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box ) ;
public :
	// 生成等高环
	BOOL CreateAllRoughLoop( CSmtCheckMdl *RoughMdl, CPlaneList &cList, JDNC_PRGDEF &ProgDef, double dCur ) ;
	BOOL CreateAllDriveAndPlaneLoop( CSmtCheckMdl *DriveMdl, CPathGroup& NewPath, JDNC_PRGDEF &ProgDef, double dCur ) ;
	void GetAllDriveLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt ) ;
	void GetAllPlaneLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CSmartLoop *PlaneLoop ) ;
	BOOL CreateAllRoughAndPlaneLoop( CSmtCheckMdl *RoughMdl, JDNC_PRGDEF &ProgDef, double dCur ) ;
	void GetAllRoughLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt ) ;
	void TrimAllPlaneLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CPlaneList &cList ) ;
	// 得到一层环
	CSmartLoop *GetOneLayerLoop ( CSmtLoopArr& LoopArr ) ;
	CSmartLoop *GetOneLayerLoop ( CSmtLPathArr &LPathArr ) ;
	CSmartLoop *CopyOneLayerLoop( CSmtLPathArr &LPathArr ) ;
	CSmartLoop *CopyOneLayerLoop( CSmtLoopArr &LoopArr ) ;
	CSmartLoop *RemoveOneLayerLoop( CSmtLoopArr &LoopArr ) ;
	CSmartLoop *RemoveOneLayerLoop( CSmtLPathArr &LPathArr ) ;
	// 用当前区域减去毛坯区域
	CSmartLoop *SubtractCastArea( CSmartLoop *pCutArea, int nLayer ) ;

	// 为所有环添加大环
	BOOL BuildAllDriveLoop( CSmtCheckMdl *DriveMdl, BOX3D &dMaxBox, JDNC_PRGDEF &ProgDef, double dCur ) ;
	// 毛坯外部下刀的时候设置m_pAllLoop的在毛坯外部的属性为blank
	CSmartLoop *CalcAllCutArea(	CSmartLoop *AreaCut,		// <I> 加工区域
								CSmartLoop *AllLoop,		// <I> 原始边界线
								CSmartLoop *RoughLoop ,		// <I> 毛坯区域
								BOOL		bDeleRough) ;	// <I> 删除毛坯
public:
	// 计算平面边界
    int FilterAllPlaneBnd( CSmartLoop *&PlaneHead ) ;
	// 使用毛坯环裁剪平面环
	void TrimPlaneHeadByRough( CNcZLevelGenMdl &heightGen, CPlaneList &cList ) ;
	// 为平面环搜索合适的位置
	void SearchRationalPosition( CPlaneList &cList ) ;
	// 生成平面路径
	void CreatePlanePath( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, CSmartLoop *CurrLoop, int nLayer, int nType ) ;
	void CreatePlanePathBtwRough( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, CSmtLoopPath *&pLPath ) ; //生成层间粗加工平面路径
	// 生成剩余平面路径
	void CreateRemainPlanePath( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath ) ;
	// 生成路径
	void GenPlaneAreaPath( CPathGroup &PGroup, CSmtCheckMdl *DriveMdl, 
						   CPlaneLoop *pPlane, double dLastH, double dDepth ) ;
	// 判断该平面是否加工
	BOOL IsCutPlaneLoop( CPlaneLoop* PlaneLoop, CSmartLoop *CurrLoop, int nLayer, int nType) ;
	// 连接线
	void AddAdsorbPath ( CSmtCheckMdl *DriveMdl, CPathGroup &PGroup, CPathCombine *PComb, double dDepth ) ;
	// 添加开粗路径层间快速连刀路径 qqs 2014.08.21
	void AddQuickPathBetweenLayer ( CSmtCheckMdl *DriveMdl, CPathGroup &PGroup, CPathCombine *PComb, int nLayer ) ;
	// 添加开粗路径层内快速连刀路径 qqs 2014.09.10
	void AddQuickPathInSameLayer ( CSmtCheckMdl *DriveMdl, CPathCombine *PComb, int nLayer ) ;
	// 根据起末点以及路径所在层号，计算快速定位路径 qqs 2014.09.10
	CPathPLine3D * CreateQuickMovePath(CSmtCheckMdl*DriveMdl,PNT3D start,PNT3D end,int nLayer);
public :
	// 曲面分层行切粗雕刻 
	int LinearOneLayer(	CSmtCheckMdl *DriveMdl, 
						CSmartLoop** pLoops,     //粗加工环
						CSmartLoop *pBigLoop,	 //大环
				        int  Layer,				 //分层序号
						BOOL bIsRoughCast ) ;    //是否是毛坯面的环        

	// 计算区域优先的路径
	int GenAreaPath ( CPathGroup &PGroup, CSmtCheckMdl *DriveMdl, CSmartLoop *BndHead, double &dDepth, 
					  int Layer, BOOL bDelete = TRUE, BOOL bAdsorb = FALSE ) ;
	// 按照高度优先对环进行排序
	int SortLoopByDepth( CPathGroup& NewPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	int	SorBtwLoopByDepth( CPathGroup& NewPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	// 按照区域优先的方法对毛坯环进行排序加工
	int SortLoopByArea( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	int	SortBtwLoopByArea( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl,  	
						   CSmartLoop *BndCurr, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	int SortBtwCurveByArea( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl,  	
		                    CSmartLoop *BndCurr, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	// 为防止遗漏,生成其余路径
	int	CreateRemainBtwPath( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	int CreateRemainFBtwPath( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	// 找到同一区域的环
	void FindAllAreaLoop( int nCurr, int nPrev, CPtrArray & pBtwLPathArr ); 
	// 当前层环pCurr与添加包围盒的pNext相减\与毛坯环相减\与边界环求交
	CSmartLoop *CalcCurrBtwLoop( CSmartLoop *pCurr, CSmartLoop *&pDriveLoop, double dTol, int nLayer ) ;
	// 将路径添加到NewPath中
	void AddPCombToNewPath( CPathGroup &NewPath, CSmtCheckMdl *DriveMdl, CPathCombine *PComb, BOOL bAbsorb ) ;

	// 最少抬刀
	BOOL OptimizeGroup( CPathCombine *pTComb, CSmtCheckMdl *DriveMdl, double dDepth, double dH ) ;	

//////////////////////////////////////////////////////////////////////////////////////////////
//毛坯边界修剪方式
///////////////////////////////////////////////////////////////////////////////////////////////
public:
	BOOL SubtractAllLoopNormal ( CSmtCheckMdl *CheckMdl, JDNC_PRGDEF &ProgDef, double dCur ) ;
	BOOL SubtractAllLoopForOpti ( CSmtCheckMdl *CheckMdl, CPathGroup &NewPath, JDNC_PRGDEF &ProgDef, double dCur ) ;
	void CreateAreaPath( CSmtCheckMdl *DriveMdl, CSmtLoopPath *pLPath, int nLayer ) ;
	void AddSectToPath( CSmartSect *pSect, CSmtCutPath *pPath, int nLayer ) ;
	BOOL GetDriveCurvePath( CSmtLoopPath *pLPath, int nLayer ) ;
	void CopyPlungeCurve ( CSmartLoop *pLast, CSmartCurveLib& BndCurve ) ;
	void AddLoopToLPArr( CSmtCheckMdl *CheckMdl,CSmtLPathArr& LPathArr, CSmartLoop* pHead, int Layer, BOOL bAddPath ) ;
	BOOL GetParamZigZag()  ;
	//判断加工路径是否为封闭路径
	BOOL SetPathClosedFlag( CSmtLPathArr& LPathArr, CSmartCurveLib& CurLib ) ;
	//对封闭加工域,用马蹄形路径代替修剪模式路径
	void CreateFollowPath(CSmtCheckMdl *DriveMdl, CPathGroup &NewPath, CSmtLoopPath *&pLPath ) ;

/*以下几个函数为层间粗加工和过滤平面添加下刀环*/
private:
	//得到与层间粗加工环的下刀环
	CSmartLoop * GetBtwPlgLoop ( CSmartLoop * pCurr ) ;
	//得到过滤平面环的下刀环
	CSmartLoop * GetPlanePlgLoop( CPlaneLoop * pPlane ) ;
	//得到下刀环(子函数)
	CSmartLoop * GetPlungeLoop( int nLayer, CSmartLoop * pCurr, DOUBLE dTol = 0.01 ) ;

	//guomin 在生成checkMdl的同时要保存矩形网格，因此重写此函数2013/02/19
	public:
		// 生成加工面和干涉面的混合模型
		BOOL BuildDriveAndCheckMdlnew( CSmartGraphic& Graph , CSmtCheckMdl& DriveMdl, 
			CSmtCheckMdl& CheckMdl, double dAngle = 0. ) ;
		//根据矩形网格生成DriveLoop
		BOOL CreatDriveLoopForRectNetSurf(CSmtCheckMdl *DriveMdl, JDNC_PRGDEF &ProgDef, double dCur) ;
		//刀具类型转换
		BOOL GetToolDef(SRectNetRoughTool &tool) ;
		//为矩形网格计算分层
		int	GetECutLayersForRectSurf(CSmartLoop *&PlaneLoop) ;
};


class DLL_EXPORT CSmartRLinearGen : public CSmartPathGen
{
public :
	CSmartRLinearGen() ;
	virtual ~CSmartRLinearGen() ;
public:
	JDNC_LINEAR      m_cParam    ; // 雕刻参数
	CNcSmartPath *   m_PathNew   ; // 存放路径的数组
	int          m_nCount	 ; // 加工的层数
	double           m_min		 ; // 最小距离(调试时使用)
	int          m_nTotal	 ; // 路径的总数
	int          m_nCur		 ; // 当前路径
	double           m_dDownStep ; // 实际的进刀量
	int              m_nCutMode  ; // 判断加工模式
	CSmartTool *	 m_pRoughTool;
	JDNC_PRGDEF		 m_cTmpPrgDef;
public :
	// 生成刀具路径
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
	//初始化进度和计算进度
	BOOL    ComputeProg( CSmtCPathLib& AllPath, JDNC_PRGDEF &ProgDef ) ;
	BOOL	BuildStockMdl ( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box ) ;
private :
    // 计算平行截线路径
    int  BuildPlanePath( CSmtCPathLib& AllPath ,  //曲面路径组
						 CSmtCheckMdl& DriveMdl,  //加工面模型
						 CSmtCheckMdl& RoughMdl,  //毛坯面模型
						 CSmartLoop* AllLoop,
						 BOX3D& MaxBox, double dBot,
						 JDNC_PRGDEF &ProgDef, 
						 double dCur	)	; 
	// 分层计算刀具路径
    int  LayerAllPath( CSmtCPathLib& AllPath, double dTop, double dBot, BOX3D &sbox, JDNC_PRGDEF &ProgDef, double dCur ); /*所有路径*/
	// 将路径加到CPathGroup中
	int  AddPathToPG(CPathGroup& PGroup, CSmtCPathLib& OldPath ) ;
	// 按照高度优先对路径进行排序
	int  SortPathByDepth( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, JDNC_PRGDEF &ProgDef ) ;
	// 按照区域优先对路径进行排序
	int  SortPathByArea( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, CSmartTool* pMiller, JDNC_PRGDEF &ProgDef ) ;
	// 找到同一层的在同一区域中的路径
	void GetAreaPathInSameLayer( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
								 BOX3D* box, CSmtCPathLib& NewPath ) ;
		 BOOL PathInSameArea( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
							 CSmtCutPath* pPath ) ;
		 BOOL GetDistFromPt ( FPNT3D start, FPNT3D end, 
							 FPNT3D pt, double& dist ) ;
	// 找到下一层在同一区域中的路径
	void GetAreaPathInNextLayer( CSmtCPathLib& PrePathLib , 
								 int i, CSmtCPathLib& CurPathLib ) ;
		 BOOL IsSameAreaNextLayer( CSmtCPathLib& PrePathLib, CSmtCutPath* pPath ) ;

	// 返回值　0:　失败，返回上一层递归 1:　成功返回 2: 中断
	int  GenAreaPath( CPathGroup& PGroup, CSmtCPathLib& PathLib, CSmtCheckMdl& CheckMdl, 
					  CSmartTool* pMiller, int i, JDNC_PRGDEF &ProgDef  ) ;

	void ClearAllPath( CSmtCPathLib& OldPath ) ;

	void SortAreaPath( CSmtCPathLib& OldPath ) ;
		 void FindPrePath( CSmtCPathLib& OldPath, CSmtCPathLib& NewPath ) ;
		 void SortPathByXCoord( CSmtCPathLib& OldPath ) ; 
		 CSmtCutPath* FindLeftPathInSameLine( CSmtCPathLib& OldPath ) ;
public:
	//gm 平行截线
	BOOL CreatePlanePathSubProc(int nAtCore, int nCoreNum, CSmtCPathLib &TmpPath, LPVOID* AllPath, 
								CSmtCheckMdl &DriveMdl,CSmtCheckMdl &RoughMdl, double dBot,JDNC_PRGDEF &PrgDef) ;
};

//多线程传递参数
typedef struct CreatePlanePathThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	CSmtCPathLib *TmpPath ;
	LPVOID *	 AllPath ;
	CSmtCheckMdl *DriveMdl;
	CSmtCheckMdl *RoughMdl;
	double dBot ;
	JDNC_PRGDEF   PrgDef  ;
	CSmartRLinearGen *pPathGen;
}CRLPPT_DATA;
///////////////////////////////////////////
// CSmartREmiterGen: 径向放射曲面粗雕加工

class DLL_EXPORT CSmartREmiterGen : public CSmartPathGen
{
public :
	CSmartREmiterGen() ;
	virtual ~CSmartREmiterGen() ;
public:
	JDNC_EMIT       m_cParam    ;  // 雕刻参数
	CNcSmartPath *  m_PathNew   ;  // 存放路径的数组
	int         m_nCount    ;  // 加工的层数
	double          m_min       ;  // 最小距离(调试时使用)
	int         m_nTotal    ;  // 路径的总数
	int         m_nCur      ;  // 当前路径
	double          m_dDownStep ;  // 实际的进给量
	int             m_nCutMode  ; // 判断加工模式
	CSmartTool *	 m_pRoughTool;
	JDNC_PRGDEF		 m_cTmpPrgDef;

public :
	// 生成刀具路径
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
	//初始化进度和计算进度
	BOOL    ComputeProg( CSmtCPathLib& AllPath, JDNC_PRGDEF &ProgDef ) ;
	void    ReversePath( CSmtCPathLib& PathLib ) ;
	BOOL	BuildStockMdl ( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box ) ;
public :
    // 计算径向放射路径
    int  BuildPlanePath( CSmtCPathLib& AllPath ,  //曲面路径组
						 CSmtCheckMdl& DriveMdl,  //加工面模型
						 CSmtCheckMdl& RoughMdl,  //毛坯面模型
						 CSmartLoop* AllLoop,
				  	     BOX3D& MaxBox, double dBot,
						 JDNC_PRGDEF &ProgDef, 
						 double dCur			) ; 
	int  GetUsedPoint( PNT2D* dPtArr, PNT2D start, 
					   PNT2D end, int nCount ) ;
	DOUBLE GetRadius( BOX3D& MaxBox ) ;
	// 分层计算刀具路径
    int  LayerAllPath( CSmtCPathLib& AllPath, double dTop, double dBot, BOX3D &sbox, JDNC_PRGDEF &ProgDef, double dCur ); /*所有路径*/

	// 将路径加到CPathGroup中
	int  AddPathToPG(CPathGroup& PGroup, CSmtCPathLib& OldPath ) ;
	// 按照高度优先对路径进行排序
	int  SortPathByDepth( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, JDNC_PRGDEF &ProgDef ) ;
	// 按照区域优先对路径进行排序
	int  SortPathByArea( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, CSmartTool* pMiller, JDNC_PRGDEF &ProgDef ) ;
	// 找到同一层的在同一区域中的路径
	void GetAreaPathInSameLayer( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
								 BOX3D* box, CSmtCPathLib& NewPath ) ;
		 BOOL PathInSameArea( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
							 CSmtCutPath* pPath ) ;
		 BOOL GetDistFromPt ( FPNT3D start, FPNT3D end, 
							 FPNT3D pt, double& dist ) ;
	// 找到下一层在同一区域中的路径
	void GetAreaPathInNextLayer( CSmtCPathLib& PrePathLib,	
								 int i, CSmtCPathLib& CurPathLib ) ;
		 BOOL IsSameAreaNextLayer( CSmtCPathLib& PrePathLib, CSmtCutPath* pPath ) ;
	
	// 返回值　0:　失败，返回上一层递归 1:　成功返回 2: 中断
	int  GenAreaPath( CPathGroup& PGroup, CSmtCPathLib& PathLib, CSmtCheckMdl& CheckMdl, 
					  CSmartTool* pMiller, int i, JDNC_PRGDEF &ProgDef ) ;

	void ClearAllPath( CSmtCPathLib& OldPath ) ;

	void SortAreaPath( CSmtCPathLib& OldPath ) ;
		 void FindPrePath( CSmtCPathLib& OldPath, CSmtCPathLib& NewPath ) ;
		 void SortPathByXCoord( CSmtCPathLib& OldPath ) ; 
		 void SortPathByDist( CSmtCPathLib& OldPath ) ;
		 CSmtCutPath* FindNearPathInSameAng( CSmtCPathLib& OldPath ) ;
		 CSmtCutPath* FindLeftPathInSameLine( CSmtCPathLib& OldPath ) ;

public:
	//gm 径向放射的单线程程序
	BOOL CreatePlanePathSubProc(int nAtCore, int nCoreNum, CSmtCPathLib &TmpPath, LPVOID* AllPath, 
								CSmtCheckMdl &DriveMdl,CSmtCheckMdl &RoughMdl, double dBot,JDNC_PRGDEF &PrgDef) ;
};
//多线程传递参数
typedef struct CreateREPlanePathThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	CSmtCPathLib *TmpPath ;
	LPVOID *	 AllPath ;
	CSmtCheckMdl *DriveMdl;
	CSmtCheckMdl *RoughMdl;
	double dBot ;
	JDNC_PRGDEF   PrgDef  ;
	CSmartREmiterGen *pPathGen;
}CRLREPPT_DATA;
///////////////////////////////////////////
// CSmartSRUWLinePath: 曲面流线曲面粗加工
class DLL_EXPORT CSmartRUWLineGen : public CSmartPathGen
{
public :
	CSmartRUWLineGen() ;
	virtual ~CSmartRUWLineGen() ;
public:
	JDNC_UWLINE      m_cParam    ; // 雕刻参数
	CNcSmartPath *   m_PathNew	 ; // 存放路径的数组
	int          m_nCount	 ; // 加工的层数
	double           m_min		 ; // 最小距离(调试时使用)
	int          m_nTotal	 ; // 路径的总数
	int          m_nCur		 ; // 当前路径 
	double           m_dDownStep ; // 实际的进刀量
	int              m_nCutMode  ; // 判断加工模式
	CSmartTool *	 m_pRoughTool;
	JDNC_PRGDEF		 m_cTmpPrgDef;
public :
	// 生成刀具路径
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
	//初始化进度和计算进度
	BOOL    ComputeProg( CSmtCPathLib& AllPath, JDNC_PRGDEF &ProgDef ) ;
	BOOL	BuildStockMdl ( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box ) ;
private :
    // 分层计算刀具路径
    int  LayerAllPath( CSmtCPathLib& AllPath, double dTop, double dBot, BOX3D &sbox, JDNC_PRGDEF &ProgDef, double dCur ); /*所有路径*/
	// 将路径加到CPathGroup中
	int  AddPathToPG(CPathGroup& PGroup, CSmtCPathLib& OldPath ) ;
	// 按照高度优先对路径进行排序
	int  SortPathByDepth( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, JDNC_PRGDEF &ProgDef ) ;
	// 按照区域优先对路径进行排序
	int  SortPathByArea( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, CSmartTool* pMiller, JDNC_PRGDEF &ProgDef ) ;
	// 找到同一层的在同一区域中的路径
	void GetAreaPathInSameLayer( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
								 BOX3D* box, CSmtCPathLib& NewPath ) ;
		 BOOL PathInSameArea( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
							 CSmtCutPath* pPath ) ;
		 BOOL GetDistFromPt ( FPNT3D start, FPNT3D end, 
							 FPNT3D pt, double& dist ) ;
	// 找到下一层在同一区域中的路径
	void GetAreaPathInNextLayer( CSmtCPathLib& PrePathLib,
								 int i, CSmtCPathLib& CurPathLib ) ;
		 BOOL IsSameAreaNextLayer( CSmtCPathLib& PrePathLib, CSmtCutPath* pPath ) ;

	// 返回值　0:　失败，返回上一层递归 1:　成功返回 2: 中断
	int  GenAreaPath( CPathGroup& PGroup, CSmtCPathLib& PathLib, CSmtCheckMdl& CheckMdl, 
					  CSmartTool* pMiller, int i , JDNC_PRGDEF &ProgDef ) ;

	void ClearAllPath( CSmtCPathLib& OldPath ) ;

	void SortAreaPath( CSmtCPathLib& OldPath ) ;
		 void FindPrePath( CSmtCPathLib& OldPath, CSmtCPathLib& NewPath ) ;
		 void SortPathByDist( CSmtCPathLib& OldPath ) ;
		 CSmtCutPath* FindLeftPathInSameLine( CSmtCPathLib& OldPath ) ;
};

// 投影加工
class DLL_EXPORT CSmartProjectGen  : public CSmartPathGen
{
public :
	CSmartProjectGen()          ; 
	virtual ~CSmartProjectGen() ; 
public :
	JDNC_PROJECT  m_cParam ;
public : 
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
public :
	// 分层粗加工路径
	int LayerRoughPath( CPathGroup&   PGroup  , /*路径组　*/
				        CSmtCPathLib& AllPath , /*所有路径*/
					    CSmtCheckMdl& CheckMdl, /*检查模型*/
						JDNC_PROJECT& PrjCut ); /*加工参数*/
	// 分层精加工路径
	int LayerFinishPath(CPathGroup&   PGroup  , /*路径组　*/
				        CSmtCPathLib& AllPath , /*所有路径*/
					    CSmtCheckMdl& CheckMdl, /*检查模型*/
						JDNC_PROJECT& PrjCut ); /*加工参数*/
	// 设置路径高度
    void ResetPathDepth( CSmtCPathLib& AllPath, int Flag ); 
};
// 包裹雕刻加工
class DLL_EXPORT CSmartWrapGen  : public CSmartPathGen
{
public :
	CSmartWrapGen()          ; 
	virtual ~CSmartWrapGen() ; 
public :
	JDNC_WRAP  m_cParam ;
public : 
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
};
// 曲面清根
typedef struct _Clean_Pt CleanPt ;
struct _Clean_Pt
{
	PNT4D left ; // 左点
	PNT4D midd ; // 中点
	PNT4D righ ; // 右点
	int	  nLineNo  ; // 曲线中的点
} ;
typedef CTypedPtrList< CPtrList, CleanPt*>  CleanList  ;
typedef CTypedPtrArray< CPtrArray, CSmtCutPoint*> CSmtPtArray ;
class DLL_EXPORT CSmartCleanupGen  : public CSmartPathGen
{ 
public :
	CSmartCleanupGen()          ; 
	virtual ~CSmartCleanupGen() ; 
public :
	JDNC_CLEANUP   m_cParam ;
protected : // By ZBQ 
	BOOL  IsToolValid();
	// 对于混合清根判断刀具关系
	BOOL JudgeToolValid( CSmartTool *&pLastTool, CSmartTool *&pLastSafe, 
						 double dLastStock, BOOL bMulti ) ;
	// 1. 计算区域行切路径
	int GenerateLinePathEx( CSmartLoop* AllCont   ,
						    CSmtCPathLib&  LvPath );
	// 2. 使用上把刀具裁剪平行截线路径
	int CreateLinearCleanupPath( CSmtCheckMdl& DriveMdl, 
						         CSmtCPathLib&    LvPath,
						         CSmartTool *pLastTool, 
						         CSmartTool *pLastSafe,
                                 JDNC_PRGDEF& PrgDef );

	// 生成行切Cleanup路径
	int GenLinearLvPath(CPathGroup&  NewPath   , /*雕刻路径*/
                        CSmartGraphic& Graph  ,  /*雕刻图形*/
                        CSmtCheckMdl& DriveMdl,  /*加工模型*/
                        CSmartLoop*&  AllLoop,  /*边界轮廓*/
					    CSmartTool * pLastTool,
					    CSmartTool * pLastSafe ) ;
	//生成环绕等距Cleanup路径
	int Gen3DStepLvPath(CPathGroup&    NewPath    ,  /*雕刻路径*/
                        CSmartGraphic& Graph     ,  /*雕刻图形*/
                        CSmtCheckMdl& DriveMdl    , /*加工模型*/
                        CSmartLoop*&  AllLoop,     /*边界轮廓*/
					    CSmartTool *pLastTool, 
					    CSmartTool *pLastSafe ) ;
	//生成多笔清根路径
	int GenMutiplePencilPath(CPathGroup&    NewPath , /*雕刻路径*/
                      CSmartGraphic& Graph     ,  /*雕刻图形*/
                      CSmtCheckMdl& DriveMdl    ,  /*加工模型*/
                      CSmartLoop*&  AllLoop     ) ; /*边界轮廓*/
	//生成角度分区清根路径
    int GenMixtureLvPath(CPathGroup&    NewPath    ,  /*雕刻路径*/
                         CSmartGraphic& Graph	  ,  /*雕刻图形*/ 
                         CSmtCheckMdl& DriveMdl    ,  /*加工模型*/
                         CSmartLoop*&  AllLoop,       /*边界轮廓*/
						 CSmartTool *LastTool, 
						 CSmartTool *LastSafe );
    // 生成自动清根清根路径
	int  GenAutoPath( CPathGroup& NewPath  ,/*雕刻路径*/
                      CSmartGraphic& Graph ,/*雕刻图形*/
                      CSmtCheckMdl& DriveMdl,  /*加工模型*/
                      CSmartLoop*&  AllLoop,   /*边界轮廓*/
                      CSmartTool *pLastTool, 
					  CSmartTool *pLastSafe ) ;
	// 生成自动清根清根路径
	int  GenPencilPath( CPathGroup& NewPath  ,/*雕刻路径*/
		CSmartGraphic& Graph ,/*雕刻图形*/
		CSmtCheckMdl& DriveMdl,  /*加工模型*/
		CSmartLoop*&  AllLoop) ;
	int  GenNetPath( CPathGroup& NewPath  ,/*雕刻路径*/
		CSmartGraphic& Graph ,/*雕刻图形*/
		CSmtCheckMdl& DriveMdl,  /*加工模型*/
		CSmartLoop*&  AllLoop) ;
	void FindPencilPntOnXYDir(CSmtCutPath* pDirPath,CSmtCutPath* pDirPencilPntPath);
public : 
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
private :
	 // 计算局部等高加工路径
    int CreateHeightPath( CSmtCheckMdl& DriveMdl , /*检查模型  */ 
                          CSmtCheckMdl& LastTMdl , /*上把模型  */ 
                          CSmartLoop*  AllCont   , /*边界裁剪  */
                          CSmtCPathLib& AllPath  , /*加工路径  */
                          JDNC_PRGDEF&  PrgDef );
	 // 计算3D环绕等距路径
    int CreateFollow3DStep( CSmtCheckMdl& DriveMdl, 
                            CSmtMeshMdl&  MeshMdl ,
                            CSmtCPathLib& AutoBnd ,
                            CSmtCPathLib& AllPath ,
                            CPathGroup&   NewPath ,
                            JDNC_PRGDEF&  PrgDef  );
    // 计算平坦面环切投影路径
    int CreateFlatAreaPath( CSmtCheckMdl& DriveMdl,
                            CSmtCPathLib& AutoBnd ,
                            CSmtCPathLib& AllPath ,
                            JDNC_PRGDEF& PrgDef  );
};

class DLL_EXPORT CSmartSGuideGen  : public CSmartPathGen
{ 
public :
	CSmartSGuideGen()          ; 
	virtual ~CSmartSGuideGen() ; 
public :
	JDNC_SGUIDE   m_cParam ;
public : 
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }

protected:
	// 生成曲线Z向投影路径
    int CreateZProjectPath( CSmartCurveLib& AllCurve, 
                            CSmtCheckMdl& CheckMdl, 
                            CSmtCPathLib& AllPath   ,
                            CSmartLoop*   AllCont   ) ; 
	// 生成曲线吸附投影路径
    int CreateAdsorbPath(   CSmtCPathLib& AllCurve, 
                            CSmtCheckMdl& CheckMdl, 
                            CSmtCPathLib& AllPath   ,
                            CSmartLoop*   AllCont   ) ; 
	// 生成单轨扫描路径
	int CreateOneSweepPath( CSmartCurveLib& AllCurve, 
                            CSmtCheckMdl& DriveMdl  ,
						    CSmtCPathLib& AllPath   ,
                            CSmartLoop*   AllCont  ) ; 
	// 生成双轨扫描路径
	int CreateTwoSweepPath( CSmartCurveLib& AllCurve, 
                            CSmtCheckMdl& DriveMdl  ,
						    CSmtCPathLib& AllPath   ,
                            CSmartLoop*   AllCont   ) ; 
private:
	// 吸附曲线到面，将空间线转为路径，保留高度
	CSmtCutPath * GetSmtCutPath(   CSmartCurve& Curve, JDNC_TOL& Tol ) ;
};

class DLL_EXPORT CSmartPlungeMillGen  : public CSmartPathGen
{ 
public :
	CSmartPlungeMillGen()          ; 
	virtual ~CSmartPlungeMillGen() ; 
public :
	JDNC_PLUNGEMILL   m_cParam ;
public : 
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }

protected:
	// 生成插槽加工路径

	int		GetPntByLen(PNT4D* dPntArr, int nPntCnt, int nStartPnt, double dLen);
	void	AddSlotCutPathAbovePlunge(PNT4D* dPntArr, int nPnt, int nPnt2, double dScale[2], double dPrevZ);
	int		GenSlotCutPath(PNT4D* dPntArr, int nPntCnt, 
							int nCurrPnt, double dScale[2], PNT3D dCutPnt[3], bool bAddAbovePlunge);

	void	GenSlotMillPath(PNT4D* dPntArr, int nPntCnt, double dScale[2], bool bAddAbovePlunge);

	void	AddBottomSlowPath(CPathGroup& NewPath, double dSlowPathLen, float fFeedScale);
	int		CreateSlotMillPath(CPathGroup& NewPath, CSmartGraphic&  Graph);
};

///////////////////////////////////////////
// CSmartRotate3DGen: 旋转加工路径
//
class DLL_EXPORT CSmartRotate3DGen : public CSmartPathGen
{
public :
	CSmartRotate3DGen() ;
	virtual ~CSmartRotate3DGen() ;
public:
	JDNC_ROTATE3D  m_cParam    ;  // 雕刻参数
public : 
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
protected :
	// 计算 0 度的旋转雕刻路径
	int  CreateRotatePath( CPathGroup& NewPath   , 
							CSmtCheckMdl& CheckMdl,
                         CSmartLoop*  AllCont  ) ;
	// 计算任意角度的的旋转雕刻加工路径
	int  CreateRotatePathEx( CPathGroup& NewPath, 
							 CSmtCheckMdl& CheckMdl,
                          CSmartLoop*  AllCont  ) ;
	// 计算边界加工路径
	int  CreateBoundPath( CPathGroup& NewPath, 
					      CSmtCheckMdl& CheckMdl,
                         CSmtCPathLib& BndPath  ) ;
	// 计算分层加工路径
    int  LayerAllPath( CPathGroup&   PGroup  ,  /*路径组　*/
					   CSmtCPathLib& AllPath ,  /*所有路径*/
					   CSmtCheckMdl& CheckMdl,  /*检查模型*/
					   JDNC_PRGDEF&  ProjDef ); /*进度控制*/
protected :
	// 计算旋转雕刻的边界轮廓, 该轮廓为展开轮廓
    CSmartLoop*  ExtractFairedContours(CSmartGraphic& Graph); /*雕刻图形*/
    BOOL  ExtractFairedBoundPath(CSmartGraphic& Graph, CSmtCPathLib& BoundPath); /*雕刻图形*/
	// 计算旋转雕刻的边界轮廓于直线的交点
    int  LineIntRotateContours( CSmartLoop* AllCont, 
                             PNT2D Start, PNT2D End, 
                             PNT2D Buff[], int Size ) ;
public :
    // 计算旋转雕刻的边界轮廓于直线的交点
    int AddOneBoundPath( CPathCombine& PComb ,  
                         CSmtCutPath& TPath  ,
                         DOUBLE ZMove  )     ;
	// 计算下刀路径
    int CreateRotatePlunge( CSmtCutPath& TPath  ,
                            CSmtCutPath& Plunge ,
                            DOUBLE DepthInc );

};

#define  _NEW_REMAIN_MDL

class DLL_EXPORT CSmartSRemainGen: public CSmartPathGen
{ 
public :
	CSmartSRemainGen()          ; 
	virtual ~CSmartSRemainGen() ; 
public :
	CSmtLoopArr *   m_pLoopArr1  ;    // 保存本次毛坯路径环
	CSmtLPathArr*   m_pLPathArr  ;    // 用于保持本次路径环及生成的路径
	int             m_nCount     ;    // 总层数
	CSmartLoop*     m_pAllLoop   ;    // 边界环	
	double			m_dMaxLen	 ;	  // 删除距离
	double			m_dStepZ	 ;	  // 分层
	double			m_dTopHeight ;
	double			m_dBotHeight ;
	CPlaneList		m_cPlaneList ;	  // 平面链表
	BOOL			m_bAllLoop	 ;	  // 是否有曲线边界
	// 上把刀具参数
	JDNC_SREMAIN    m_cParam     ;    // 上把刀具参数
	CSmtLoopArr*    m_pLastLoopArr ;  // 保持上把刀具的环
	CSmartTool*     m_pLastTool  ;    // 上把刀具
	CSmartTool *	m_pRoughTool ;	  // 毛坯刀具
	BOOL			m_bFirst	 ;	  // 是否第一个

#ifdef _NEW_REMAIN_MDL
	CSmtRemainMdl   *m_pRemainMdl;    // 残料模型
#else 
	CSmartBitmap    *m_pRemainMdl;    // 残料模型
#endif 
	
	JDNC_PRGDEF		m_cTmpPrgDef ;	  // 临时进度条
	// 添加用于连到的曲线组和环组
	CSmartLoop * *  m_pPlunge    ;    // 连到环组
	CSmartCurveLib *m_pCurveLib  ;    // 曲线组
public :
	// 生成刀具路径
	virtual int  GeneratePathEx( CPathGroup& NewPath   ,  /*刀具路径*/
				                CSmartGraphic& Graph );  /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
	void		 ClearAllLoop() ;
	void		 ClearAllCurveLib() ;
	// 初始化临时进度条
	void		 InitTmpProg() ;
	// 初始化普通参数
	BOOL		CalcTopAndBot ( BOX3D &box ) ;
	//计算分层
	int			GetECutLayers() ;
	BOOL		PrepareRemain( CSmartGraphic& Graph ) ;
	// 构建毛坯模型
	BOOL BuildStockMdl( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box ) ;
public :
	
	
	int LinearOneLayer(	CSmartLoop** pDriveLoop,   //加工环组
						CSmartLoop* pBigLoop,     //边界环
						CSmtLoopArr* LoopArr,    //路径数组
				        int  Layer,				 //分层序号
						BOOL bRoughCast	) ;		 //毛坯
						  
	// 为所有环添加大环
	void BuildAllDriveLoop( CSmtLoopArr *pLoopArr, double dMdlTop, int nType, 
							BOX3D &dMaxBox, JDNC_PRGDEF &ProgDef, double dCur ) ;
	// 按照区域优先的方法，对环进行排序
	int SortLoopByArea( CSmtCPathLib &AllPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	// 按照高度优先对环进行排序
	int SortLoopByDepth( CSmtCPathLib &AllPath, CSmtCheckMdl *DriveMdl, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	

/////////////////////保存数据使用/////////////////////////////////////////
public:
	CSmartLoop**    m_pDriveLoop ;
	CSmartLoop**    m_pRoughLoop ;
	CSmartLoop **	m_pRemainLoop;
	BOOL	  *		m_bFlag		 ;
	double    *     m_dZ         ;
public :
	///////////////////整理函数/////////////////////////
	// 获取加工面的环
	BOOL CreateAllDriveLoop( CSmtCheckMdl *DriveMdl, CNcZLevelGenMdl &heightGen, JDNC_PRGDEF &ProgDef, double dCur ) ;
	BOOL CreateAllRoughLoop( CSmartGraphic &Graph, CSmtCheckMdl *RoughMdl, JDNC_PRGDEF &ProgDef, double dCur ) ;

	BOOL CreateRemainAllDriveAndPlaneLoop( CSmtCheckMdl *DriveMdl, JDNC_PRGDEF &ProgDef, double dCur ) ;
	void GetRemainAllDriveLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt ) ;
	void GetRemainAllPlaneLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CSmartLoop *PlaneLoop ) ;
	BOOL CreateRemainAllRoughAndPlaneLoop( CSmartGraphic &Graph, CSmtCheckMdl *RoughMdl, JDNC_PRGDEF &ProgDef, double dCur ) ;
	void GetRemainAllRoughLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt ) ;
	void TrimRemainAllPlaneLoop( CSmartLoop **AllLoop, double *dZ, BOOL *bFlag, int nCnt, CPlaneList &cList ) ;

	// 加工环和毛坯环相减,并且上把刀具的环和当前加工的环相减
	BOOL SubtractLoop( CSmtCheckMdl *CheckMdl, CSmtLoopArr* pTmpArr,
					   JDNC_PRGDEF &ProgDef, double dCur ) ;
	CSmartLoop* GetOneLayerLoop( CSmtLoopArr& LoopArr, BOOL bRough, BOOL bRemove ) ;
	
////////////////////////////////生成残料模型的等高环///////////////////////////////////
public:
	// 比较两把刀的圆角部分
	BOOL CompareToolCorner( CSmartTool *pCurr, CSmartTool *pLast ) ;
	// 生成残料模型
	BOOL CreateAllRemainLoop( CSmartGraphic &Graph, CSmtCheckMdl &DriveMdl,   
				   			  CSmtCheckMdl &StockMdl, int &nType, 
							  JDNC_PRGDEF &ProgDef, double dCur[7],
							  CPathGroup& NewPath ) ;
	// sbg矩形网格
	BOOL BuildRemainZMap( CSmartGraphic& Graph, CSmtCheckMdl *DriveMdl, 
						  CSmtCheckMdl *StockMdl, JDNC_PRGDEF &ProgDef, double dCur ) ;
	// 处理原始残料环
	void CombineRemainLoop( BOX3D dMaxBox, int nType, JDNC_PRGDEF &ProgDef, double dCur ) ;

/////////////////////////////////////////////利用环环求交的方法生成路径/////////////////////////////////////////////////////////
public:
	void GetRemainPath( CSmtCheckMdl *CheckMdl, CSmtLPathArr& LPathArr, CSmartLoop* pCurr, CSmartLoop* pLast, int nLayer ) ;
	void AddLoopToLPArr( CSmtCheckMdl *CheckMdl, CSmtLPathArr& LPathArr, CSmartLoop* pHead, int Layer, BOOL bAddPath ) ;
	int  GenFollowPath ( CSmtCheckMdl *DriveMdl, CSmtLoopPath *pLPath, CSmartLoop *pLast, int Layer ) ;

	///////////////////////////FOR TEST///////////////////////////
	void TestLoopError( CSmartLoop* pCurr, CSmartLoop* pLast, int nLayer ) ;
	void TestLoopCurve( CSmtLPathArr* pLPath, double dZ ) ;
	// 得到下刀点的曲线
	void CopyPlungeCurve( CSmartLoop *pLast, CSmartCurveLib& BndCurve ) ;
	CSmartLoop * CopyPlungeLoop( CSmartLoop *pLast ) ;
	// 判断两点之间直接连接是否可以
	BOOL ConnectLine( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL& cTol ) ;
	// for test///
	int SortLoopByArea( CPathGroup &NewPath, int nLayer, JDNC_PRGDEF &ProgDef ) ;
	void AddTestPathToAllPath( CSmtCPathLib &AllPath, CSmtCPathLib &cPathLib ) ;

public:
	// 计算平面边界
    int	 FilterAllPlaneBnd( CSmartLoop *&PlaneHead ) ;
	// 用组合后的环修剪平面环
	void TrimPlaneHeadByCombineLoop( CPlaneList &cList, BOX3D &MaxBox, int nType ) ;
	// 为平面环过滤刀具盲区
	void FilterAllPlaneBlindArea( CPlaneList &cList ) ;
	// 为平面环搜索合适的位置
	void SearchRationalPosition( CPlaneList &cList ) ;
	// 生成平面路径
	void CreatePlanePath( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath, CSmartLoop *CurrLoop, int nLayer, int nType ) ;
	// 生成剩余平面路径
	void CreateRemainPlanePath( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath ) ;
	// 生成路径
	void GenPlaneAreaPath( CSmtCheckMdl *DriveMdl, CSmtCPathLib &AllPath,
						   CPlaneLoop *pPlane, double dLastH, double dDepth ) ;
	// 判断该平面是否加工
	BOOL IsCutPlaneLoop(CPlaneLoop* & PlaneLoop, CSmartLoop * CurrLoop, int nLayer, int nType) ;
	
protected :
#ifdef _NEW_REMAIN_MDL
    BOOL DebugRemainMdl( CPathGroup& NewPath,  CSmartGraphic& Graph) ;
#endif 
};

///////////////////////////////////////////
// CSmartFPlaneGen: 平面加工精雕刻
//
class DLL_EXPORT CSmartFPlaneGen : public CSmartPathGen
{
public :
	CSmartFPlaneGen() ;
	virtual ~CSmartFPlaneGen() ;
public :
	JDNC_FPLANE  m_cParam ; 
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					            CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
    // 计算平面边界曲线
    int CreatePlaneContours( CSmtCheckMdl& DriveMdl  , // 加工模型
							 CSmartLoop *AllLoop     , // 边界轮廓
                             CNcZLevelGenMdl* MeshSurf  , // 自适应曲面
                             CPlaneList &cList,		// 平面边界环
							 JDNC_PRGDEF& PrgDef ) ; 
	int CreatePlaneContour ( CSmtCheckMdl& DriveMdl  , // 加工模型
                             CNcZLevelGenMdl* MeshSurf  , // 自适应曲面
                             CSmartLoop *&BndCont,	   // 平面环
							 JDNC_PRGDEF& PrgDef ) ; 
protected :
    // 计算平坦区域边界轮廓
    BOOL CreateBoundArea( CSmtCheckMdl& DriveMdl, 
                          CSmartLoop* AllCont   ,
                          CSmartLoop*& BndCont,
						  JDNC_PRGDEF& PrgDef ) ;
   
    // 跟踪精确的平面加工边界
    BOOL TracingPlaneAreaBound( CSmtCheckMdl& DriveMdl, 
                                CSmartLoop*& BndCont  ) ;
	// 修剪无效的平面加工边界
    BOOL ReomvePlaneAreaOnEdge( CSmtCheckMdl& DriveMdl, 
                                CSmartLoop*& BndCont  ) ;
    // 修剪无效的平面加工边界
    CSmartLoop* TrimingInvalidPlaneArea( CSmtCheckMdl& DriveMdl,
                                         DOUBLE BtmRad ,   
                                         CSmartLoop*  BndCont  );
    // 计算平面加工路径
    BOOL CreatePlaneCutPath( CSmtCheckMdl& DriveMdl, 
                             CSmartLoop*  BndCont  ,
                             CPathGroup&  NewPath  );
    // 计算平坦面加工路径
    BOOL CreateFlatAreaPath( CSmtCheckMdl& DriveMdl, 
                             CSmartLoop*  BndCont  ,
                             CPathGroup&  NewPath  );
private :
	// 生成刀具路径
	BOOL GenProjPocketPath(CSmtCheckMdl& DriveMdl, 
                           CSmartLoop* BndCont   ,
                           CPathGroup& PGroup );
    // 区域修剪边界
    void TrimingPlaneAreaByContours( CSmartLoop*& BndCont, CSmartLoop* AllLoop );
	void TrimAllPlaneAreaByContours( CNcZLevelGenMdl* MeshSurf, 
									 CSmartLoop*& BndCont, 
									 CSmartLoop* AllLoop,
									 CPlaneList &cList );
    // 光滑区域边界
    BOOL SmoothPlaneAreaBound( CNcZLevelGenMdl* MeshSurf,CSmartLoop*&  BndCont );
};
///////////////////////////////////////////
// CSmartFeatureSlotGen: 特征槽加工
//
class DLL_EXPORT CSmartFeatureSlotGen : public CSmartPathGen
{
public :
	CSmartFeatureSlotGen() ;
	virtual ~CSmartFeatureSlotGen() ;
public :
	JDNC_FEATURESLOT  m_cParam ; 
	JDNC_PRGDEF		  m_cTmpPrgDef;

public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
								CSmartGraphic& Graph     ) ; /*雕刻图形*/
	virtual BOOL IsSupportProgress() { return TRUE ; }

	BOOL OneSlotPath( CPathCombine& PComb,  /*单个槽路径*/
					  CSmtCheckMdl* pMdl,   /*检查模型*/
					  CSmartCurve* pMidCur, /*槽壁两边界中位线*/
					  BOOL   bIsLine,      /*中位线直线标记*/       
					  DOUBLE dTopWidth,    /*槽顶宽*/
					  DOUBLE dBtmWidth,   /*槽底宽*/
					  JDNC_PRGDEF& cPrgDef );
	BOOL OneDraftSlotPath( CPathCombine& PComb,  /*单个槽路径*/
						   CSmtCheckMdl* pMdl,   /*检查模型*/
						   CSmartCurve* pMidCur,  /*槽壁两边界中位线*/
						   CEntList&  SideList,    /*槽侧壁*/
						   CEttSurface* pSideSurf, /*侧壁虚拟面*/
						   DOUBLE dMaxWidth,
						   JDNC_PRGDEF& cPrgDef );
	BOOL OneDraftSlotPathEx( CPathCombine& PComb,  /*单个槽路径*/
							 CSmtCheckMdl* pMdl,   /*检查模型*/
							 CSmartCurve* pMidCur,  /*槽壁两边界中位线*/
							 CEntList&  SideList1,    /*槽侧壁1*/
							 CEntList&  SideList2,    /*槽侧壁2*/
							 CEttSurface* pSideSurf1,  /*侧壁1虚拟面*/
							 CEttSurface* pSideSurf2,  /*侧壁2虚拟面*/
							 DOUBLE dMaxWidth,
							 JDNC_PRGDEF& cPrgDef );

private:
	//是否有非工件面加入检查模型
	BOOL IsReuildCheckMdl(CEttFeatureSlot* pEntSlot, CSmtCheckMdl* pMdl, RFRAME& frm );
	//将曲面加入检查模型
	void AddSurfToCheckMdl(CSmtCheckMdl* pMdl, CGeoSurf* pSurf, RFRAME& frm);
	//投影路径转化为3D折线
	CGeoPLine3d* ConvertCPathToCur(CSmtCutPath* pCutPath); 
	//平面曲线转换成刀具路径 
	CSmtCutPath* TransCurveToCPath( CSmartCurve&  Curve ,JDNC_TOL& Tol ) ;
	//3D曲线转化成刀具路径
	CSmtCutPath* TransCur3DToCPath(CGeoCurve* pCur3D);
	//裁剪每层路径
	void TrimOneLayer( PNT3D start, PNT3D end, 
					   CGeoCurve* pTopCur,
					   CGeoCurve* pBtmCur, 
					   CSmtCheckMdl* pDriveMdl, 
					   DOUBLE dTopWidth,
					   DOUBLE dBtmWidth,
					   int    nLayer,
					   SlotPathList& layerSlotPathList);
	//每层断点处连接路径
	void ConnectLayerPath(SlotPathList& layerSlotPathList,  CSmtCheckMdl* pDriveMdl);
	//延伸开口路径端点
	void ExtendLayerEnd(SlotPathList& slotPathList, CSmtCheckMdl* pDriveMdl, BOOL bIsLine);
	//生成两条分层路径间的下刀路径,输入两层CutPath和底部投影线,输出生成的下刀路径PathComb
	void PlungeBtwnLayer( SlotPath* pPrePath, 
						  SlotPath* pNxtPath, 
						  CGeoCurve* pBtmCur,    /*槽底部投影曲线*/
						  BOOL bRev,           /*下层路径反向标记*/
						  CPathCombine& pComb);
	//生成两层路径间的下刀，输入两层路径链表和底部投影线，输出PathComb
	void PlungeBtwnLayerEx( SlotPathList& pLayerList1, 
							SlotPathList& pLayerList2, 
							CGeoCurve* pBtmCur,
							BOOL bRev, 
							CPathCombine& pPComb );
	//折线下刀路径
	//输入上下层路径起点和槽底部曲线，pt0 & pt1是下层路径起末点, pt是上层路径末点，输出添加的折线下刀路径
	int  AddRampPlungeEx( CPathCombine& PComb, PNT3D pt0, PNT3D pt1, PNT3D pt, CGeoCurve* pBtmCur, BOOL bVertical );

	//未按区域排序的路径添加到PathCombine中，同时生成下刀(高度优先:分层路径在断点处直接沿曲面连刀)
	BOOL AddPathListToPCombEx( SlotPathList& allSlotPathList, /*输入:原始分层裁剪路径*/
								int nLayerCnt,                /*总层数*/ 
								CGeoCurve* pBtmCurve,          /*底部投影线*/
								JDNC_PRGDEF& cPrgDef,
								double       dCur,
								CPathCombine& PComb);          /*输出:路径PComb*/
	void AddLayerPathToPComb(SlotPathList& layerList, CPathCombine& PComb, BOOL bRev);

	//区域排序后的路径添加到PathCombine中，同时生成下刀
	BOOL AddPathListToPComb(SlotPathList& allSlotPathList,      /*输入:区域排序后的路径*/
							int nLayerCnt,						/*总层数*/ 
							CGeoCurve* pBtmCurve,           /*底部曲线*/
							JDNC_PRGDEF& cPrgDef,
							double       dCur,
							CPathCombine& PComb);           /*输出:路径PComb*/
	//搜索同层路径，输入allPathList和层数nLayer，输出curLayerList
	void SearchOneLayerList(SlotPathList& allPathList, int nLayer, SlotPathList& curLayerList);
	//分层路径按区域优先排序
	void AreaSortPathList( SlotPathList& allSlotPathList,  /*原始分层裁剪路径*/
							int nLayerCnt,                 /*总层数*/
							SlotPathList& areaSortList );   /*输出:按区域排序后的路径*/
	//按区域排序的递归函数
	int GenAreaSlotPath( SlotPathList* pSlotPathGrp,     /*输入:分层路径组*/
						 SlotPath* preSlotPath,         /*上层路径*/ 
						 int nLayerCnt,                /*总层数*/
						 SlotPathList& areaSortList);    /*输出:一个区域内的路径*/
	//搜索下层中的同区域路径
	void GetAreaPathInNextLayer( SlotPathList* pSlotPathGrp,   /*输入:分层路径组*/
								 SlotPath* preSlotPath,       /*上层路径*/ 
								 SlotPathList& CurPathList ); /*输出:下层中同区域路径*/
	//等高分层路径:输入侧壁面和等高截面，输出求得的等高截线路径
	CSmtCutPath* DraftOneLayerPath(CEntList& SideList, CEttSurface* pSideSurf, CGeoTrmSurf* pIntSurf);
	//等高最后一层路径
	CSmtCutPath* DraftLastLayerPath(CEntList& SideList,    /*侧壁面*/
		                         CEttSurface* pSideSurf, /*侧壁虚拟面*/
		                         CGeoTrmSurf* pIntSurf, /*等高截面*/
								    CSmtCheckMdl* pDirveMdl, 
									DOUBLE dBtm ,        /*底面高度*/
								    VEC3D movVec);       /*平移向量*/

	//底部拉伸裁剪面裁剪等高分层路径
	BOOL TrimDraftAllLayer( CSmtCPathLib& cutPathList, /*原始等高截线路径*/
							CGeoTrmSurfList& btmSurfList,   /*底部拉伸裁剪面*/ 
							CSmtCheckMdl* pDriveMdl,  /*检查模型*/
							SlotPathList& layerSlotPathList,
							JDNC_PRGDEF& cPrgDef ,
							double dCur ); 

	//侧壁面组排序
	void SortSideList(CEntList& inList) ;
	//生成裁剪拉伸面
	BOOL CreateDraftSurf( CGeoCurve* pMidCur ,          /*原始曲线*/
						  VEC3D draftDir,              /*拉伸方向*/
						  DOUBLE dDraftWidth,           /*拉伸长度*/
						  CGeoTrmSurfList& draftSurList );/*拉伸面链表*/
	//剔除不合理的底部裁剪面:当裁剪面的法失与给定vec平行时，则剔除掉。
	void SelectValidSurf(CGeoTrmSurfList& btmTrmSurfList, VEC3D vec, CGeoTrmSurfList& validSurfList);
	//3D折线与底部裁剪面的所有交点
	BOOL GetPLineAllSurfIntPnt(CGeoPLine3d* pLine3d, SurfFacetList& allFacet, CCurIntpt& allPntList);
	//3D折线与单个裁剪面的交点
	BOOL GetPLineSurfIntPnt(CGeoPLine3d* pLine3d, SurfAndFacet* pSNode, CCurIntpt& linPntList);
	//找到侧壁组面最高点
	void SideListTopZ(CEntList& SideList, DOUBLE& dTopZ);
	//找到等高路径中同层的位于不同侧壁的对应路径
	SlotPath* FindPathInList(SlotPathList& allSlotPathList, int nIndex);
	//连接同层对应路径
	CSmtCutPath* ConnectTwoCPath(CSmtCutPath* pSidePath1, CSmtCutPath* pSidePath2);
	//等高截槽路径高度优先时，连接对应于槽两侧壁的同一层的路径
	void ConnectLayerSidePath(SlotPathList& allSlotPathList, int nLayerCnt, SlotPathList& sameHhtList);
	//等高截槽路径添加到PathCombine中，同时添加层间下刀(高度优先,下层路径无需反向)
	void AddDraftPathListToPCombEx(SlotPathList& slotPathList, CPathCombine& PComb);          
	//等高截槽路径添加到PathCombine中，同时添加层间下刀
	void AddDraftPathListToPComb(SlotPathList& slotPathList, CPathCombine& PComb);          
	//生成两条分层路径间的下刀路径,输入两层CutPath,输出生成的下刀路径PathComb
	void DraftPlungeBtwnLayer( SlotPath* pPrePath, 
							   SlotPath* pNxtPath, 
							   BOOL bRev,           /*下层路径反向标记*/
							   CPathCombine& pComb);
	//清除临时链表
	void ClearSlotPathList(SlotPathList& slotPathList);
	void ClearCPathLib(CSmtCPathLib& AllPath);
	void ClearFacetList(SurfFacetList& facetList);
	void DeleteSurfList(CGeoTrmSurfList& ResultSide);
};
typedef CSmartCheck* (*CALLBACK_FACETTOCHECK)( FACET& Facet, BOOL ToMix, DOUBLE MaxCos,DOUBLE FltBox[2][3]) ;
typedef CSmartCurve* (*CALLBACK_CURVE3DTO2D)(CGeoCurve* Curve, JDNC_TOL& Tol, RFRAME* NcMtx, BOOL Redepth) ;
DLL_EXPORT void	 MathCAM_RegConvertToCheckEx(CALLBACK_FACETTOCHECK) ;
DLL_EXPORT void  MathCAM_RegCurve3DToCurve2D(CALLBACK_CURVE3DTO2D) ;
#endif // __SMART_PATHGEN3D_H__