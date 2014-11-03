#ifndef __SMART_PATHEX_H__
#define __SMART_PATHEX_H__

#include "SmartNC.h"
#include "SmartCheck.h"

////////////////////////////////////////////////////////////////////////////////////////
// 为CSmtCutPath添加切向进退刀
static int MathCAM_GenLeadPathEx( CSmtCheckMdl&   CheckMdl, JDNC_TOL& Tol, 
								  JDNC_CONNECT3D& ConnectDef, TFLOAT StartAt[3],  
								  TFLOAT TanDir[3], TFLOAT NorDir[3], 
								  CSmtCutPath &CutPath, BOOL bCheck ) ;
static int MathCAM_DefineNormalAt( CSmtCheckMdl& CheckMdl, JDNC_TOL& cTol ,
								   TFLOAT StartAt[3], TFLOAT Normal[3] ) ;
void	CalcSmtCutPathLead( CSmtCheckMdl &CheckMdl, JDNC_TOL &Tol, JDNC_CONNECT3D &Connect, 
						   CSmtCutPath *pPath,  CSmtCutPath *&LeadIn, CSmtCutPath *&LeadOut, BOOL bCheck ) ;
void	CalcCutPathLeadIn( CSmtCheckMdl &CheckMdl, JDNC_TOL &Tol, JDNC_CONNECT3D &Connect, 
						   CSmtCutPath *pPath,  CSmtCutPath *&LeadIn, BOOL bCheck, int nType ) ;
void	CalcCutPathLeadOut( CSmtCheckMdl &CheckMdl, JDNC_TOL &Tol, JDNC_CONNECT3D &Connect, 
						    CSmtCutPath *pPath,  CSmtCutPath *&LeadOut, BOOL bCheck, int nType ) ;

// 为开启半径磨损补偿的曲面磨削路径创建水平面上的进退刀 qqs
void    CalcCutPath2DLeadOut( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						  CSmtCutPath *pPath, CSmtCutPath *&LeadOut, BOOL bCheck );
void    CalcCutPath2DLeadIn( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						  CSmtCutPath *pPath, CSmtCutPath *&LeadOut, BOOL bCheck );
// 用于粗加工连刀

class CRoughLink
{
public:
	JDNC_PRGDEF  m_cPrgDef    ; // 计算进度
	double		 m_dDepth	  ; // 换算深度
	CRoughLink( double dDepth ) ;
	~CRoughLink() ;
	void	InitProg() ;
public:
	// 进行路径连接
	BOOL	AddLinkPath( CSmtCheckMdl* DriveMdl, CPathCombine *pTComb, JDNC_TOL& Tol, double dLastH ) ;
	// 添加Combine路径连接
	BOOL	AddSameLayerLink( CSmtCheckMdl* DriveMdl, CPathCombine *TComb, JDNC_TOL& Tol, double dLastH ) ;
	// 添加两层之间的连接
	BOOL	AddNextLayerLink( CSmtCheckMdl* DriveMdl, CPathCombine *pHead, CPathCombine *pNext, JDNC_TOL& Tol, double dLastH ) ;
	// 为CPathCombine添加元素
	void	InsertBefore( CPathCombine* TComb, CPathEntity* pNew, CPathEntity* pNext ) ;
	void	InsertAfter ( CPathCombine* TComb, CPathEntity* pNew, CPathEntity* pPrev ) ;
	// 向一个pHead中添加元素,构成链表
	CPathEntity* AddPathEntity( CPathEntity* pHead, CPathEntity* pPath ) ;
	CSmartSect*  AddSmartSect( CSmartSect* pHead, CSmartSect* pSect ) ;
	// 由首末端点生成四个节点的CPathPLine3D
	CPathEntity* AddSafeLink( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL &Tol, double dLastH ) ;
	CPathEntity* GetPLineEntity( PNT3D start, PNT3D end, double dH ) ;
	BOOL	AddPCombToHead( CPathCombine *pHead, CPathCombine *TComb ) ;
} ;

///////////////////////////////////////////////////////////////////////////////////
typedef CTypedPtrArray< CPtrArray, CSmartCurveLib* >CCurveLibArr ;
//该类用于残料补加工
class CSmtLoopPath
{
public:
	CSmartLoop		*m_pLoop   ;	// 裁减后的环,排序时适用
	CSmtCPathLib     m_cPathLib;	// 记录该层的路径
	CSmartCurveLib   m_cCLib   ;	// 在该曲线上寻找下刀点
	CSmartLoop	    *m_pPlunge ;	// 在曲线上不能生成下刀的使用这个环
	JDNC_PRGDEF      m_cPrgDef ;	// 进度条
	CCurveLibArr     m_cLibArr ;	// 裁减后得到的曲线
	JDNC_PLUNGE		 m_cPlunge ;	// 下刀设置
	double           m_dStep   ;
	double			 m_dDepth  ;
	double			 m_dRadius ;
	JDNC_FEED		 m_cFeed   ;	// 进退刀设置
	// 定义用于CPathCombine使用的数据
	BOOL			 m_bRemain ;
	CSmtLoopPath *	 next	   ;	
	CSmtLoopPath *	 prev	   ;	

	BOOL			 m_bAddPath;	// 添加路径标记
	CSmartLoop	*	 m_pPtLoop ;	// 等高环
	BOOL			 m_bBoundTol	;
	JDNC_SPEED		 m_cSpeed		;
	TFLOAT			 m_fFeedRatio	;
	double			 m_dBoundTol	;
	BOOL			 m_bUsed		; // 标记是否使用
	BOOL             m_bClosedFlag  ; // 标记加工域是否"封闭"
	CSmtLoopPath( CSmartLoop* pLoop, JDNC_FEED &cFeed, double dDepth, 
					double dRadius, double dStep, BOOL bRemain ) ;
	~CSmtLoopPath() ;
	void RemoveLibArr() ;
public:
	// 设置残补兜边
	void SetBoundInfo( JDNC_SPEED &cSpeed, TFLOAT fRatio, BOOL bBoundTol, double dBoundTol ) ;
	// 设置下刀曲线和环
	void SetPlungeInfo( JDNC_PLUNGE &cPlunge ) ;
	// 将曲线反向
	void ReverseAllCurve() ;
	// 判断连线是否过切
	BOOL ConnectLine( CSmtCheckMdl *CheckMdl, PNT3D start, PNT3D end, double dDepth, JDNC_TOL& cTol ) ;
	
	// 使用区域优先的方法进行连接
	void ConnectAllCurve( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib& BndCurve, 
						  BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE & cPlunge, double dDepth ) ;
	// 更新连接函数,更加节省连刀路径
	BOOL ConnectAllCurve( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib &BndCurve, 
						  BOOL bZigZag, BOOL bPrev, PNT3D prev, JDNC_SETUP &cSetup, double dDepth ) ;
	// 得到第一根曲线
	CSmartCurve *FindFstCurve( CSmartCurveLib &cCurveLib, PNT3D end, BOOL bZigZag ) ;
	// 将曲线添加到路径组中
	void AddCurveToPathLib( CSmartCurve *pCurve, CSmtCPathLib &cPathLib, BOOL bDel, int nLevel, JDNC_TOL &cTol ) ;
	// 将曲线转换成路径
	CSmtCutPath * TransfCurveToPath(  CSmartCurve * pCurve , JDNC_TOL &cTol, BOOL IsDel ) ;
	// 寻找最近层的曲线进行连接
	void FindNearCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, CSmartLoop *pPlunge, CSmartCurveLib* pLib, 
						int i, BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, double dDepth, BOOL &bAddClose ) ;
	// 判断是否是最近的曲线
	BOOL IsNextCurve( CSmartCurve *pCurve, PNT3D p, int i, CSmartSect *&pNearSect, PNT2D nearpt, double dTol ) ;
	// 计算点到曲线的最短距离
	double GetPntToCurveDist( PNT2D p, CSmartCurve *pCurve, CSmartSect *&pNearSect, PNT2D nearpt ) ;
	
	// 得到路径组的端点
	void GetEndPoint( CPathCombine *PComb, BOOL bFlag, PNT3D point ) ;
	void GetEndPoint( CPathEntity *pEntity, BOOL bFlag, PNT3D point ) ;
	CSmtCutPath *CreatePathLine( PNT3D start, PNT3D end, double dDepth ) ;
	// 吸附连接
	CSmtCutPath *CreateAdsorbLine( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL &cTol ) ;
	// 添加下刀路径,并将曲线添加到路径组中
	void AddPlungeCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int nLevel, CSmartLoop *pPlunge, CSmartCurve *pCurve, 
						 JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, BOOL &bAddClose, BOOL bIsDel, BOOL bLine, BOOL bZigZag, BOOL bLeadOut = FALSE ) ;
	void AddPlungeCurveForCloseCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int nLevel, CSmartLoop *pPlunge, CSmartCurve *pCurve, 
									 JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, BOOL &bAddClose, BOOL bIsDel, BOOL bLine, BOOL bLeadOut ) ;
	// 得到所有下刀点
	BOOL FindAllPlungePnt( CPointList &allPnt, PNT2D pos, double dMaxDist ) ;
	// 计算圆弧下刀路径
	BOOL AddArcForCloseCurve( CSmtCheckMdl *DriveMdl, CSmtCPathLib &cPathLib, CSmartCurve *pCurve, 
							  CPointList &allPnt, JDNC_SETUP &cSetup, BOOL bHead  ) ;
	BOOL IsLeadArcPath( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib ) ;
	CSmartCurve *CreateLeadArc( PNT3D pos, PNT3D tan, PNT3D nor, PNT3D bgn, double dAng, BOOL bHead ) ;

	void AddPlungeCurveForOpenCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int nLevel,  
								     CSmartCurve *pCurve, JDNC_SETUP &cSetup, BOOL bIsDel, BOOL bZigZag ) ;
	// 判断闭合曲线方向
	void AdjustCloseCurveDir( CSmartCurve *pCurve, JDNC_SETUP &cSetup ) ;
	// 判断直线是否与未加工的曲线相交
	BOOL LineIntAllCurve( PNT2D p1, PNT2D p2 ) ;
	// 添加直线切入切出
	CSmtCutPath *GetPlungeLine( CSmtCheckMdl *CheckMdl, PNT2D start, PNT2D end, JDNC_TOL& cTol ) ;
	// 添加切入切出路径(圆弧状)
	CSmtCutPath *CreatePlungeArc( CSmtCheckMdl *DriveMdl, CSmartCurve *pCurve, PNT2D p, BOOL bHead, JDNC_SETUP &cSetup ) ; 
	// 得到CSmtCPathLib的起末点
	BOOL GetSmtCPathLibEndPoint( CSmtCPathLib &tmpLib, PNT3D p, BOOL bHead ) ;
	// 为闭合曲线添加下刀路径
	BOOL AddPlungeForClosedCurve( CSmtCPathLib &cPathLib, CSmartCurve *pCurve, CSmartLoop *pCurr, JDNC_PLUNGE &cPlunge ) ;
	// 将路径添加到NewPath中
	void AddPathToGroup( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath ) ;

	/*以下几个函数用于圆弧连接*/
	BOOL ConnectAllCurve_ArcLink( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib &BndCurve, 
		                          BOOL bZigZag, BOOL bPrev, PNT3D prev, JDNC_SETUP &cSetup, double dDepth ) ;
	// 寻找最近层的曲线进行连接
	CSmartCurve* FindNearCurve_ArcLink( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, CSmartLoop *pPlunge, CSmartCurveLib* pLib, CSmartCurve * &pPrev,
						                int i, BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, double dDepth, BOOL &bAddClose ) ;
	// 判断是否是最近的曲线
	BOOL IsNextCurve_ArcLink( CSmartCurve *pCurve, CSmartCurve*pPrev, PNT3D p, int i, CSmartSect *&pNearSect, PNT2D nearpt, double dTol ) ;
	CSmtCutPath * CreateAdsobSpline( CSmtCheckMdl *Drivemdl, PNT3D IntPnt[2], PNT3D TanDir[2], JDNC_TOL &cTol ) ;
	CSmtCutPath * GetLeadoutSplinePath( CSmtCheckMdl *CheckMdl, JDNC_SETUP & cSetup, CSmartCurve * pCurve, CPointList &allPnt ) ;
	// 重新定义进退刀路径的高度
	BOOL       RedefineLeadPathHeight(CSmtCheckMdl *Drivemdl, CSmtCutPath * pPath, BOOL bLeadIn ) ;
	// 把最后一条封闭路径的退刀路径抬高0.05
	BOOL       RedefineLastLeadoutPathH(CSmtCheckMdl *Drivemdl, CSmtCutPath * pPath) ;
} ;
typedef CTypedPtrArray<CPtrArray, CSmtLoopPath*> CSmtLPathArr ;
void	MathCam_GetLPathBox( CSmtLoopPath *LPathHead, BOX3D *box ) ;
CSmtLoopPath *	MathCam_AddLPath( CSmtLoopPath *LPathHead, CSmtLoopPath *LPathNew ) ;
void	MathCam_DeleteLPath( CSmtLoopPath *&LPathHead ) ;
CSmtLoopPath *	MathCam_RemoveLPath( CSmtLoopPath *LPathHead, CSmtLoopPath *pObj ) ;
int		GetLPathCount( CSmtLoopPath *LPathHead ) ;
void	ReverseAllLPath( CSmtLoopPath *&LPathHead ) ;
CSmtLoopPath *	SortLPathByRemainLoop( CSmtLoopPath *LPathHead, double dSpaceTol, double dSafeDist ) ; 
/////////////////////////////////////////////////////////////////////////////
// 以下函数为粗加工和残料补加工公用
// 得到环组的最大边界环
CSmartLoop *MathCam_CreateBigLoop( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, int nCount, double r ) ;  
CSmartLoop *MathCam_CreateBigLoopAndBox( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, BOX3D &dMaxBox, int nCount, double r ) ;
// 将环放到组中
void		AddLoopToLoopArr( CSmtLoopArr& LoopArr, CSmartLoop* pHead, int Layer, double dMinArea, BOOL bDel = TRUE ) ;
// 将路径转换
void		TransfPCombineToCPath( CPathCombine &PComb, CSmtCPathLib &cPathLib, double dFeedRate, double h, JDNC_TOL& Tol ) ;
void		TransfPCombineToSmartCurve( CPathCombine &PComb, CPtrList &SmtCurList, JDNC_TOL& Tol ) ;
void		TransfSmartCurveToCPath( CSmartCurve *pCurve, CSmtCPathLib &cPathLib, double h, JDNC_TOL &Tol ) ;
void		TransfAllLoopToCPath( CSmartLoop *AllBnd, CSmtCPathLib &AllPath, double h, JDNC_TOL &Tol ) ;
// 当前环和上把刀环pLast相剪,然后和边界环AllLoop求交得到有效的线段放到CSmartCurveLib中
BOOL		SubtractContour( CSmartLoop *pCurr, CSmartLoop *pLast, CSmartLoop *AllLoop, CSmartCurveLib& CurLib, double dMinLen, BOOL bBtw = FALSE ) ;
// 用两个环修剪第一个环，落在两个环以外的曲线被剪掉
BOOL        TrimLoopByTwoContours( CSmartLoop *pLoop, CSmartLoop *pContour1, CSmartLoop *pContour2, CSmartCurveLib& CurLib ) ;
// 将裁剪得到的曲线放到LPathArr中
BOOL		AddCurveToLPath( CSmtLPathArr& LPathArr, CSmartCurveLib& CurLib, int nLevel, BOOL bSetClosed = TRUE ) ;
//判断加工路径是否为封闭路径
BOOL		SetPathClosedFlag( CSmtLoopPath *pLPath, CSmartCurveLib *pLib ) ;
// 使用边界ConHead过滤CSmartCurve
int			IsPointOnContours( CSmartLoop* ConHead, PNT2D Point ) ;
// 将CutPath添加到AllPath中
void		AddCutPathToAllPath( CPathGroup *NewPath, CSmtCPathLib &AllPath, CSmtCheckMdl *DriveMdl, 
								 double dMaxDist, CSmtCPathLib &cPathLib, JDNC_TOL &cTol, BOOL bAbsorb ) ;
// 得到一组路径的末点
BOOL		GetAllPathEndPoint( CSmtCPathLib &AllPath, PNT3D end ) ;
//////////////////////////////////////////////////////////////////////////
void		DeleteInvalidLoop( CSmartLoop *&pHead, JDNC_TOL &cTol, double dTol ) ;
// 判断一个残料区域环是否有效
BOOL		IsRemainLoopValid( CSmartLoop *pReLoop, JDNC_TOL &cTol, double dTol ) ;
// 如果最大距离超过dTol返回TRUE 
BOOL		BlankSectDistLoop( CSmartLoop *pReLoop, CSmartSect *pSect, JDNC_TOL &cTol, double dTol ) ;
BOOL		UnBlankSectDistLoop( CSmartLoop *pReloop, CSmartSect *pSect, JDNC_TOL &cTol, double dTol  ) ;
// 该函数为残料补加工的函数,为残补兜边添加数率
void		AddToPathCombine( CSmtCPathLib &AllPath, CPathCombine *PComb, double dFeedRate ) ;
// 设置为边界环Blank
void		SetContourBlank( CSmartLoop *Contour ) ;
BOOL		HasBlankSect( CSmartLoop *AllLoop ) ;
//////////////////////////////////////////////////////////////////////////////////////////
//删除刀具盲区环
void		DeleteBlindLoop( CSmartLoop *&pHead, double dRadius ) ;
BOOL		IsBlindContour( CSmartLoop *Contour, double dRadius ) ;
///////////////////////////////////////////////////////////////////////////////////////////
// 将环的起点设置在距离给定点最近的blank处
BOOL		SetContourHeadAtBlank(	CSmartLoop *AllLoop,	// <I> 输入换
									double	 dist	,		// <I> 最短长度
									PNT3D InOutPnt ) ;		// <IO> 给定点
									
// 判断CSmtCutPath是否同Line相交
BOOL		AllPathIntLine( CSmtCPathLib &AllPath	,	// <I> 输入路径 
							PNT3D start, PNT3D end	,	// <I> 输入直线
							BOOL bIncludeHead )		;	// <I> 是否包括起末点
// 判断直线是否同SmtLoopArr中的loop相交
BOOL		AllLoopIntLine( CSmartLoop *AllCont		,	// <I> 输入区域
							PNT3D start, PNT3D end );	// <I> 输入直线
// 判断直线是否同smartcurve相交
BOOL		AllCurveIntLine(	CPtrList &AllCurve		,	// <I> 输入所有曲线
								PNT3D start, PNT3D end );	// <I> 输入直线

// 判断直线是否同CPathEntity相交
// 下面两个函数用于判断区域加工中间的连刀是否与路径相交(首末点相交除外)
// 只要确保中间部分不与路径相交即可，所以其中的dTol可以设为-0.0001
BOOL		AllPathEntIntLine(	CPathEntity *pEnt		,	// <I> 输入路径
								PNT3D start, PNT3D end	,	// <I> 输入直线
								int nMethodType			,	// <I> 输入加工方法
								BOOL bNext				) ; // <I> 后续标记
BOOL		AllPathEntIntCurve( CPathEntity *pEnt		,	// <I> 输入路径
								CSmartCurve &tmpCurve	,	// <I> 输入直线
								int nMethodType			,	// <I> 输入加工方法
								BOOL bNext				) ; // <I> 后续标记
// 判断单个CSmartCurv是否与直线相交
BOOL		SmartCurveIntLine(	CSmartCurve *pCurve			,	// <I> 输入曲线
								PNT3D start, PNT3D end	)	;	// <I> 输入直线
// 输入曲线，返回环
CSmartLoop *FormAllCurveToContour( CSmartCurveLib &AllLib ) ;	// <I> 输入曲线
// 定义一个结构用于平坦面加工
class CPlaneLoop
{
public:
	CSmartLoop *m_pPlaneLoop ; // 平面环
	double	    m_dHeight	 ; // 高度
	CSmartLoop *m_pOrgLoop	 ; // 原始环
	CSmartLoop *m_pRoughLoop ; // 原始毛坯环
	BOOL		m_bValid	 ; // 有效性
	CSmartLoop *m_pRemainLoop; // 原始残补环
	
public:
    CSmtLoopArr m_PreLoopArr ; // 保存相关联的粗加工环
    BOOL m_bSearch3 ;          // 第三次查找标记

public:
	void AddPreLoop( CSmartLoop * pPreLoop ) ;
  
public:
	CPlaneLoop( CSmartLoop *pLoop, CSmartLoop *pOrgLoop ) ;
	~CPlaneLoop() ;
	
	void	SetRoughLoop( CSmartLoop *pRoughLoop ) ;
	void	SetPlaneLoop( CSmartLoop *pPlaneLoop ) ;
	void	SetRemainLoop( CSmartLoop *pRemainLoop ) ;
	void	SetValid( BOOL bValid ) ;
} ;

typedef CTypedPtrList< CPtrList, CPlaneLoop* > CPlaneList ;
// 清空平面环
void	ClearAllPlaneLoop( CPlaneList *pList ) ;
BOOL	GetAllPlaneHeight( CPlaneList *pList, double *&dLayer, int &nLayer ) ;
BOOL	GetAllBndContHeight( CSmartLoop *BndCont, double *&dLayer, int &nLayer ) ;
// 将nCnt1和nCnt2合并,放到dAlldZ中,bFlag中TRUE为dZ1的,FALSE为dZ2的
void   CombineAllHeight( double *dZ1, int nCnt1, double *dZ2, int nCnt2, double *&dAlldZ, int &nCnt, BOOL *&bFlag ) ;
// 两环之间最近距离
double MinDistBetweenLoop( CSmartLoop *pLoop1, CSmartLoop *pLoop2 ) ;
double MinDistBetweenLoopAndSect( CSmartLoop *pLoop, CSmartSect *pSect ) ;
BOOL   IsBottomPlaneValid( CSmartLoop *pCutLoop, CSmartLoop *PlaneLoop, double dTol, double dRatio ) ;
double MinDistPntAndContour( CSmartLoop *Contour, PNT2D pt ) ;
double MinDistPntAndAllCurve( CPtrList &AllCurve, PNT2D pt ) ;

// 对路径进行高度裁减
void	TrimAllPathByHeight( CSmtCPathLib &AllPath, double dTop, double dBot ) ;
BOOL	TrimPathByZValue( CSmtCutPath *pPath, double dZValue ) ;
BOOL	LineIntLine( CSmtCutPath *pPath, CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double dZValue ) ;
CSmtCutPointEx *GetMiddlePntEx( CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double t ) ;
// 定义一个类,用来合并层
typedef struct _LAYER_FLAG LayerFlag ;
struct _LAYER_FLAG
{
	double m_dCurZ ;
	BOOL   m_bFlag ;
} ;
///////////////////////////////////////////////////////////////////////////////////////////
/// 用于投影加深的路径裁减
class  CNcSmartPath : public CSmtCPathLib
{
public :
	CNcSmartPath() ;
	virtual ~CNcSmartPath() ;
public :
	// 根据 Z 高度修剪路径
    int			 TrimByZValue0 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& TopLib,     /*顶部路径*/
		                         DOUBLE  ZValue ) ;        /*修剪高度*/      
								    
	// 用于一般的分层						 
	CSmtCutPath* TrimPath0     ( CSmtCPathLib& TempLib,
							     CSmtCutPath* pPath, 
							     DOUBLE ZValue ) ;

	// 用于拷贝分层
	int          TrimByZValue1 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& LowLib,     /*底部路径*/
		                         DOUBLE  ZValue,           /*修剪高度*/
								 DOUBLE  h) ; 			   /*路径抬起的高度*/
								               								 
							
	void         TrimPath1     ( CSmtCutPath* pPath, 
							     DOUBLE ZValue,
								 DOUBLE h,
								 CSmtCPathLib& LowLib ) ;



	int			 TrimByZValue2 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& TopLib,     /*顶部路径*/
		                         DOUBLE  ZValue,           /*修剪高度*/
								 int nCutMode ) ;        
								    
	// 用于一般的分层						 
	CSmtCutPath* TrimPath2     ( CSmtCPathLib& TempLib,
							     CSmtCutPath* pPath, 
							     DOUBLE ZValue,
								 int nCutMode ) ;

	// 用于拷贝分层
	int          TrimByZValue3 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& LowLib,     /*底路径*/
		                         DOUBLE  ZValue,           /*修剪高度*/
								 DOUBLE  h) ;			   /*路径抬起的高度*/
								              
							
	void         TrimPath3     ( CSmtCutPath* pPath, 
							     DOUBLE ZValue,
								 DOUBLE h,
								 CSmtCPathLib& LowLib ) ;
	
	// 路径拷贝，并抬高
	CSmtCutPath* CopyPath     ( CSmtCutPath* pPath, 
								DOUBLE h ) ;

	int			 RefineSmtPath( CSmtCutPath* pPath ) ;
};

/////////////////////////////////////////////////////////////
//以下函数为粗加工和残料补加工使用
/////////////////////////////////////////////////////////////
void AddContourToGroup( CSmartLoop *pHead, double h, CPathGroup& NewPath ) ;
void AddCurveLibToGroup( CSmartCurveLib &curveLib, double h, CPathGroup &NewPath ) ;
void AddCurveLibToGroup( CSmtLPathArr &LPathArr, double h, CPathGroup &NewPath ) ;
void AddLoopToGroup( CSmartLoop *pLoop, double h, CPathGroup &NewPath ) ;
void AddLoopToGroup ( CSmtLPathArr& LPathArr, double h, CPathGroup& NewPath ) ;
void AddPlaneLoopToGroup( CSmartLoop *pLoop, CPathGroup &NewPath ) ;
void TestLoopsArr( CSmtLoopArr* LoopArr, double *dZ, int nCount, CPathGroup& NewPath ) ;
void TestLoopArr( CSmartLoop ** DriveLoop, double* dZ, int nCount, CPathGroup& NewPath ) ;
void ClearLoopArr ( CSmtLoopArr& LoopArr ) ;
void ClearLPathArr( CSmtLPathArr& LPathArr ) ;
void ClearAllLPathArr( CSmtLPathArr*& pTmpArr, int nCount ) ;
void ClearAllLoopArr( CSmtLoopArr*& pTmpArr, int nCount ) ;
void ClearAllLoopHead( CSmartLoop**& pLoopArr, int nCount ) ;
void TestRemainMdl( CSmtRemainMdl *pRemainMdl, CPathGroup& NewPath ) ;
void AddAllPathToGroup( CSmtCPathLib &AllPath, CPathGroup &NewPath ) ;
void AddPathArrToGroup( CSmtLPathArr &PathArr, CPathGroup &NewPath ) ;
void AllCutPathToGroup( CSmtCutPath *pPath, CPathGroup &NewPath ) ;
void TestAllPlaneLoop( CPlaneList *pList, CPathGroup &NewPath, int nType ) ;
void TestAllPlaneLoop( CSmartLoop *PlaneLoop, CPathGroup &NewPath ) ;
CPathCombine *AddLoopToPComb( CSmartLoop* pLoop, WORD nType ) ;
void TestDriveMdl( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath ) ;
void TestMeshMdl( CSmtMeshMdl *MeshMdl, CPathGroup &NewPath ) ;
// 检查等高路径环是否安全
BOOL CheckCutLoop ( CSmtLoopArr *pTmpArr, int& nCount ) ;
// 环和环求交
CSmartLoop* LoopLoopInt( CSmartLoop* pHead, CSmartLoop* pLoop ) ;
// 两曲线的距离
double  MinDistBetweenCurve( CSmartCurve * pCurve1, CSmartCurve * pCurve2 ) ;
// 环和曲线的最小距离
double  MinDistBetweenLoopAndCurve( CSmartLoop * pLoop, CSmartCurve * pCurve ) ;
// 得到环组的最大边界环
CSmartLoop *CreateBigLoop( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, int nCount, double r ) ;  
CSmartLoop *CreateBigLoopAndBox( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, 
								 BOX3D &dMaxBox, int nCount, double r ) ;

// 对得到的等高环进行处理,得到组合的等高环组
CSmartLoop* ManageLoop ( CSmartLoop* pHead, CSmartTool* pTool, CSmartLoop* AllLoop, 
						 BOOL bIsRoughCast, BOOL bBtwLayer = FALSE ) ;
// 判断CPathCombine的可靠性
BOOL IsErrorPComb( CPathCombine *PComb ) ;
// 等高排序
int     SortHeight( const void *arg1, const void *arg2 ) ;
// 等高的残料补加工的分层
void GetUsedLayerZ( double *dSurfZ, int nNum, double *dZ, int& nCount, 
				    double dMinStepZ, double dBot, double dTop ) ;
// 将环组反向
void ReverseAllContour( CSmartLoop *&pHead ) ;
// 设置pPath的高度
void SetCutPathZValue( CSmtCutPath *pPath, TFLOAT fZValue ) ;
// 删除pPath中比fZValue还低的点,返回pPath链表
void TrimCutPathZValue( CSmtCutPath *&pPath, TFLOAT fZValue ) ;
// 添加CutPath到pHead
CSmtCutPath *AddSmtCutPath( CSmtCutPath *PathHead, CSmtCutPath *NewPath ) ;
/////////////////////////////////////////////////////////////
//以下函数为路径空间变换使用
/////////////////////////////////////////////////////////////
int  GetLocalFrame( CPathArc3D* pArc,  RFRAME& locMtx );
void RotatePComb( CPathCombine* PComb, PNT3D ptbase, VEC3D dir, double angle ) ;
void MirrorPComb( CPathCombine* PComb, PNT3D ptbase, VEC3D dir ) ;
void ScalePComb( CPathCombine* PComb, PNT3D ptbase, VEC3D scale ) ;
void TransfPath( CPathGroup& NewPath, JDNC_SPACETRAN cSpaceDef ) ;
// 路径变换
void TransLocalGroup( CPathGroup &NewPath, RFRAME *Local ) ;
//计算环链元素的个数
int GetLoopCount( CSmartLoop * pLoop ) ;

////以下为层间加工使用
class CBtwLPath
{// 保存层间粗加工环信息
public:
	CSmtLoopPath *m_pLPath		;	// 层间加工组\链表结构
	CSmtLoopArr m_UnderLoops	;	// 与加工环链表相关的下层粗加工环

public:
	int n_UnderDepth		;	// 下层粗加工环所在的层号
	BOOL m_bSearch3				;	// 第三次查找标记

public:
	CBtwLPath( CSmtLoopPath *pLPath ) ;
	~CBtwLPath() ;

public:
	void AddUnderLoop(CSmartLoop * pLoop ) ;
};
typedef CTypedPtrArray< CPtrArray, CBtwLPath* > CBtwLPathArr ;

typedef CTypedPtrList< CPtrList, CSmartCurve * > CurveList ;
class CBtwCurve
{// 保存层间精加工曲线信息
public:
    CurveList    m_CurveList ;    // 保存加工曲线
    CSmtLoopArr  m_UnderLoops ;   // 与加工曲线相关的精加工环

public:
    int      m_nUnderDepth ;  // 下层粗加工环所在的层号
    
public :
    CBtwCurve(){ m_nUnderDepth = -1 ; } ;
    ~CBtwCurve(){} ;
  
public:
    void AddUnderLoop(CSmartLoop * pLoop ) ;
};
typedef CTypedPtrArray< CPtrArray, CBtwCurve* > CBtwCurveArr ;
typedef CTypedPtrArray< CPtrArray, CSmartCurve*> CSmtCurveArr ;

//粗加工使用
//判断包围盒是否相交
int ChkBox3DInt(BOX3D* box1, BOX3D* box2, double tol) ;
//判断两轮廓是否相交
BOOL Is2ContoursInt( CSmartLoop * pLoop1, CSmartLoop * pLoop2, double dTol );
//判断两轮廓是否在同一区域内
BOOL IsInSameArea( CSmartLoop * pLoop1, CSmartLoop * pLoop2, double dTol ) ;
//把pNewLPath按高度值插入链表pLPathHead中
void InsertLPath( CSmtLoopPath * pNewLPath, CSmtLoopPath * &pLPathHead ) ;
//判断环pLoop和等高环链表pDriveLoop是否相交
BOOL IsRelatedLoop( CSmartLoop *pLoop, CSmartLoop *pDriveLoop, double dTol ) ;
//生成直线圆弧连接两曲线
CSmartCurve *  CreateConnectCurve( CSmartCurve * pCurve, CSmartCurve * pPrev, double dDist ) ;
//把曲线的起点按曲线方向偏移dDist距离
BOOL   ResetCurveStart( CSmartCurve * pCurve, double dDist ) ;
//调整曲线方向
void   AdjustCurveDir( CSmtCPathLib & cPathLib, CSmartCurve *pPrev, CSmartCurve *pCurve ) ;
//查找点from到曲线pCurve的最近点to
BOOL   FindNearestPnt( CSmartCurve *pCurve, PNT3D from, PNT3D to, double dTol, BOOL bFromHead ) ;
#endif // __NC_SMARTPATH_H__