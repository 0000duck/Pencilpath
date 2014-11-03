#ifndef __SMART_PATHGEN_H__
#define __SMART_PATHGEN_H__
#include "global.h"
#include "SurfNC.H"
#include "SurfGeo.H"
#include "SmartNC.H"
#include "SysPrompt.H"

class CContrLoops ;
class CGeoSurf    ;
class CGeoCurve ;
class CGeoNetSurf ;
class CMachineMdl ;
class CMachSurfSet;
class CSmtCheckMdl; 
class CSmtMeshMdl; 
class CNc5DAxisCtrl ;
//路径计算错误提示
//BEGIN
#define  JDERROR_GENPATH_SUCCESS     100001      /*成功计算    　*/
#define  JDERROR_GENPATH_NOGRAPH     100010      /*没有选择图形　*/
#define  JDERROR_GENPATH_NOLOOP      100011      /*没有封闭轮廓　*/
#define  JDERROR_GENPATH_NOSURFACE   100012      /*没有选择曲面　*/ 
#define  JDERROR_GENPATH_NOPOINT     100013      /*没有选择钻孔点*/  
#define  JDERROR_GENPATH_NOPATH      100014      /*没有投影路径　*/   
#define  JDERROR_GENPATH_NOCURVE     100015      /*没有曲线    　*/   
#define  JDERROR_GENPATH_NOAREA      100016      /*没有可加工区域*/
#define  JDERROR_GENPATH_NOROUGHSURF 100017      /*没有选择毛坯面*/
#define  JDERROR_GENPATH_NOCHECKSURF 100018		 /*没有选择干涉面*/      
#define  JDERROR_GENPATH_NOTOOL      100020      /*没有选择刀具  */
#define  JDERROR_GENPATH_STOCK		 100021      /*加工余量和刀具不匹配*/
#define  JDERROR_GENPATH_NOPARAM     100022      /*没有参数      */
#define  JDERROR_GENPATH_NOGEN       100023      /*没有路径发生器*/
#define  JDERROR_GENPATH_ERRDATA     100024      /*路径参数设置错误*/
#define  JDERROR_GENPATH_ERRDIAM     100025      /*扩孔直径小于刀具直径 */
#define  JDERROR_GENPATH_ABORT       100030      /*用户中断      */
#define  JDERROR_GENPATH_NOTGENPATH  100031      /*没有生成路径  */
#define  JDERROR_GENPATH_LOOPERROR   100032      /*路径的环发生错误*/
#define  JDERROR_GENPATH_SURFSTOCK   100033      /*当前表面余量大于上把刀具的表面余量*/
#define  JDERROR_GENPATH_BNDLOOP     100034      /*生成边界环时发生错误*/
#define  JDERROR_GENPATH_PLANARERROR 100035		 //电极平动量设置错误
#define  JDERROR_GENPATH_SPARKERROR  100036      //电极放电间隙设置错误
#define  JDERROR_GENPATH_CLEANUPTOOL 100037      //清根刀具错误
#define  JDERROR_GENPATH_AUTOCLEAN	 100038		 //缝合和混合刀具应为球刀
#define  JDERROR_GENPATH_NETSURFUW   100039      //网格面不能生成曲面流线路径
#define  JDERROR_GENPATH_TAPTOOL     100040      //应选择螺纹铣刀
#define  JDERROR_GENPATH_PITCHDIF    100041      //螺纹的螺距和铣刀的螺距不匹配
#define  JDERROR_GENPATH_NOSTOCK	 100042		 //没有创建毛坯
#define  JDERROR_GENPATH_CLEANERROR	 100043		 //上把刀具要有圆角;当前刀具圆角不应大于上把刀具圆角;当前刀具平底部分不应大于上把刀具平底部分
#define  JDERROR_GENPATH_ERRORDER	 100044		 //组合刀切割顺序定义错误
#define	 JDERROR_GENPATH_NOREMAINMDL 100045		 //没有残料模型
#define	 JDERROR_GENPATH_SUPPORTBALLONLY   100046	// 三轴转五轴路径只支持球头刀和锥球刀，请重新选择刀具
#define  JDERROR_GENPATH5AX_NOAXISCURVE    100100      /*没有刀轴曲线    */
#define  JDERROR_GENPATH5AX_NOAXISCURVE5AX 100101      /*没有五轴刀轴曲线*/
#define  JDERROR_GENPATH5AX_NOGUIDESURF    100102      /*没有导动曲面    */
#define  JDERROR_GENPATH5AX_NOMESHCURVE1   100103      /*没有变形曲线1   */
#define  JDERROR_GENPATH5AX_NOMESHCURVE2   100104      /*没有变形曲线2   */
#define  JDERROR_GENPATH5AX_NOMESHSURF1    100105      /*没有变形曲面1   */
#define  JDERROR_GENPATH5AX_NOMESHSURF2    100106      /*没有变形曲面2   */
#define  JDERROR_GENPATH5AX_NOBLADENUM     100107      /*没有定义叶片个数*/ 
#define  JDERROR_GENPATH5AX_FINDNETSURF    100108      /*直纹面侧铣时挡墙面不应包含网格面*/ 
#define  JDERROR_GENPATH5AX_LASTTOOLTOOBIG 100109      /*上把刀具圆角半径太小*/
#define  JDERROR_GENPATH5AX_SWARFTOOLAXIS  100110      /*侧铣中刀轴控制只能选择自动或沿切削方向倾斜*/
#define  JDERROR_GENPATH5AX_SIDEANGTOOBIG  100111      /*侧倾斜角度太大*/
#define  JDERROR_GENPATH5AX_PLUGTOOLAXIS   100112	   /*三轴授权加密狗刀轴只能竖直*/
#define	 JDERROR_GENPATH5AX_PLUG5AX		   100113      /*三轴授权加密狗不支持多轴路径*/
#define  JDERROR_GENPATH5AX_PLUG5DCLEAN	   100114	   /*授权加密狗不支持多轴清根*/
#define	 JDERROR_GENPATH5AX_PLUGIMPELLER   100115	   /*授权加密狗不支持叶轮加工*/
#define	 JDERROR_GENPATH5AX_NOSWARFBTMCUR  100116	   /*缺少侧铣底部曲线*/
#define  JDERROR_GENPATH5AX_NOSWARFTOPCUR  100117	   /*缺少侧铣顶部曲线*/
#define  JDERROR_GENPATH5AX_MAPFAILURE     100150      /*区域映射失败（多轴区域加工）   */
#define  JDERROR_FOLLOWPATH_NOGUIDEPNT     100160      /*环绕等距：缺少导动点*/ 
#define  JDERROR_FOLLOWPATH_NOGUIDECURVE   100161      /*环绕等距：缺少导动线*/ 
#define  JDERROR_FEATURESLOT_TOOLERROR     100162      /*刀具尺寸错误*/
#define  JDERROR_GENPATH5AX_SUPPORTBALLONLY 100163 // 只支持球头刀和锥球刀，请重新选择刀具
#define  JDERROR_GENPATH5AX_CSGAP		    100164      /*线面间距离过大*/
//END

typedef CTypedPtrArray< CPtrArray , CGeoSurf*>  C3DSurfArray ;
typedef CTypedPtrArray< CPtrArray , CGeoCurve*>	CCurveArray ;


typedef struct smart_bitmap_info
{
	double  m_dCorner[2]  ;   // 图片位置
	double  m_dSize[2]    ;   // 图片尺寸
	HANDLE  m_hDib        ;   // 图片
}BITMAP_INFO ;

DLL_EXPORT void MathCAM_TestDraw();

// 该简单的类用于保留曲线高度的时候进行的精确计算
class DLL_EXPORT CRedepthCurve
{
public:
	CGeoCurve *	m_pCurve3D	;	// 3D曲线
	CGeoCurve *	m_pCurve2D	;	// 2D曲线
	BOOL		m_bIs2D		;	// 是否2D
public:
	CRedepthCurve( CGeoCurve *pCurve3D, CGeoCurve *pCurve2D, BOOL bIs2D ) 
	{
		m_pCurve3D = pCurve3D ;
		m_pCurve2D = pCurve2D ;
		m_bIs2D	   = bIs2D	  ;
	} ;
	~CRedepthCurve()
	{
		delete m_pCurve2D ;
	} ;
};

void MathCam_DeleteAllRedepthCur( CPtrList &cListCur ) ;
void MathCam_ReverseAllRedepthCur(CPtrList &ListCur);

// 该简单类用于三维清角减少抬刀
class CPathLayerLoop
{
public:
	CPathEntity* m_pHead;	// 路径环的第一段
	CPathEntity* m_pTail;	// 路径环的最后一段
	CSmartLoop* m_pLoop;	// 路径环
public:
	CPathLayerLoop(CPathEntity* HPEnt, CPathEntity* TPEnt, CSmartLoop* Loop);
	~CPathLayerLoop();
	BOOL IsValid();
};

void MathCam_GetPathLayerLoop(CPathCombine* PComb, CPtrList &LayerLoop);

// 计算平面雕刻加工路径
class DLL_EXPORT CSmartGraphic
{
public :
	CSmartGraphic() ;
    virtual ~CSmartGraphic() ;
public :
	void	*m_pAtClass   ;		// 保存图形对象的基类
	void	*m_pAtPComb   ;		// 当前计算的路径对象
	//图形属性
	//BEGIN
#define  SMTFIND_NETSURF  0x0001  /*存在网格曲面*/
	//END
	int     m_bFindFlag ;		// 图形属性

	// 提取几何图形 (nGeomType)
	//BEGIN
#define SMARTGRAPH_TYPE_POINT			0  /* CSmartPoint*  */ 
#define SMARTGRAPH_TYPE_CURVE			1  /* CSmartCurve*  */ 
#define SMARTGRAPH_TYPE_LOOP			2  /* CSmartLoop*   */  
#define SMARTGRAPH_TYPE_CONTOUR			3  /* CSmartLoop*   */ 
#define SMARTGRAPH_TYPE_PATHGROUP		4  /* CPathGroup*   */ 
#define SMARTGRAPH_TYPE_FACET			5  /* FACET*        */      
#define SMARTGRAPH_TYPE_GEOSURF			6  /* CGeoSurf*     */      
#define SMARTGRAPH_TYPE_PATHCOMB		7  /* CPathCombine* */      
#define SMARTGRAPH_TYPE_CURVE3D			8  /* CSmartCurve*   保留高度*/ 
#define SMARTGRAPH_TYPE_CONTOUR3D		9  /* CSmartContour* 保留高度 */ 
#define SMARTGRAPH_TYPE_GEOCURLIST		10 /* CGeoCurList*     */      
#define SMARTGRAPH_TYPE_FLOWSURF		11 /* CGeoSurf*        */      
#define SMARTGRAPH_TYPE_5AXISOBJ		12 /* 五轴对象         */      
#define SMARTGRAPH_TYPE_GUIDECURVE		13 /* CSmartCurve*  保留高度*/ 
#define SMARTGRAPH_TYPE_ALLSURF			14 /* 所有曲面     */    
#define SMARTGRAPH_TYPE_ROUTHCONTOUR	15 // 毛坯边界
#define SMARTGRAPH_TYPE_ROUGHSURF		16 // 毛坯面
#define SMARTGRAPH_TYPE_SHARPEDGE		17 // 锋利边界
#define SMARTGRAPH_TYPE_FEATURESLOT		18 // 槽特征
#define SMARTGRAPH_TYPE_FEATUREHOLE		19 // 孔特征
#define SMARTGRAPH_TYPE_CURVE2D			20 // 2D曲线
#define SMARTGRAPH_TYPE_STOCKBND		21 // 毛坯边界
#define SMARTGRAPH_TYPE_PROBEPNT		23 /*CGeoProbePnt*/
	//END
	BOOL  (*m_pGetAllEnt)(   void* AtClass     ,  /*图形对象*/
		                     JDNC_SETUP& Setup ,  /*计算设置*/  
							 int  nGeomType    ,  /*对象类型*/
		                     CPtrList& AllEnt  ,  /*对象列表*/
                             RFRAME*  PrjMtx  );  /*投影变换矩阵*/

public :
	BOOL (* m_pGetFeaturePoint)( void* AtClass     ,     /*图形对象*/
		                          JDNC_SETUP& Setup ,     /*计算设置*/  
		                          CPtrList& AllPoint,     /*点      */
								  JDNC_POINT& PointDef);  /*过滤直径*/

	BOOL (* m_pGet5AxisPoint)( void* AtClass     ,     /*图形对象*/
		                          JDNC_SETUP& Setup ,     /*计算设置*/  
		                          CPtrList& AllPoint,     /*点      */
								  CPtrList& AllPoint5X);  /*5X点    */

	BOOL (* m_pGet5AxisObject)( void* AtClass     ,     /*图形对象*/
		                          JDNC_SETUP& Setup ,     /*计算设置*/  
								   int nType       , 
								   void* pData ) ;

	// 曲面点
	BOOL (* m_pGetSurfPoint )( void* GeoSurf     ,   /*曲面对象*/
		                        double u, double w,   /*参数    */
								PNT3D dPoint      ,   /*点      */
								VEC3D dNormal   ) ;   /*法失    */
	// 询问是否选择干涉检查面
    BOOL (* m_pHasCheckSurf) ( void* AtClass );			 /*图形对象*/
	// 询问是否有毛坯面
	BOOL (* m_pHasRoughSurf) ( void* AtClass ) ;         /*图形对象*/ 
	// 得到毛坯类型
	int  (* m_pGetRoughType) ( void* AtClass ) ;
	int  (* m_pSetRoughType) ( void *AtClass, int nType ) ;
	// 得到毛坯表面余量
	double ( *m_pGetRoughStock ) (void *AtClass ) ;
	// 寻找加工面为平面的面
	BOOL (* m_pHasPlaneSurf) ( void* AtClass,			 /*图形对象*/ 
							   CPtrArray* AllSurf,       /*放置平面*/ 
							   int nType,				 /*曲面类型*/
							   double dTol ) ;    

	// 构建加工模型
    BOOL (* m_pBuildCheckMdl) ( void* AtClass         ,  /*图形对象*/
		                        JDNC_SETUP&  Setup    ,  /*图形对象*/
		                        CSmtCheckMdl& CheckMdl,  /*加工模型*/
								int SurfType 	    ) ;  /*曲面类型*/
    BOOL (* m_pCreateCheckMdl)( void* AtClass        ,  /*图形对象*/
		                        JDNC_SETUP&  Setup    ,  /*图形对象*/
		                        CSmtCheckMdl& CheckMdl,  /*加工模型*/
								int EntTypeMask	      ,  /*曲面掩码*/
                                JDNC_PRGDEF& PrgDef ) ;  /*进度设置*/
	// 构建毛坯模型
	BOOL (* m_pBuildStockMdl) ( void *AtClass,			   // 图形对象
								JDNC_SETUP &Setup,		   // 加工设置
								CSmtCheckMdl &StockMdl,    // 毛坯模型
								BOX3D &box	) ;
	// 构建投影变换模型
	BOOL( *m_pBuildProjectMdl ) (	void *AtClass,				// <I> 图形对象
									JDNC_SETUP &Setup,			// <I> 加工设置
									CSmtCheckMdl &ProjMdl ) ;	// <I> 投影模型
	// 构建叶轮毛坯模型
	BOOL (* m_pBuildImpStockMdl)( void *AtClass,			   // 图形对象
								  JDNC_SETUP &Setup,		   // 加工设置
								  CSmtCheckMdl &StockMdl,      // 毛坯模型
								  BOX3D &box,	
								  CPtrArray &AllSurf) ;
	// 构建直纹面模型（双曲线侧铣时用到2011.01.21）
	BOOL (* m_pBuildRulSurfMdl)( void *AtClass,			   // 图形对象
								 JDNC_SETUP &Setup,		   // 加工设置
								 CSmtCheckMdl &WallMdl,    // 挡墙面模型
								 CGeoCurve* pTopCurve,
								 CGeoCurve* pBtmCurve ) ;
	// 构建延伸面模型(等高精加工使用2011.04.04)
	BOOL(* m_pBuildExtendSurfMdl)( void *AtClass,			   // 图形对象
								   JDNC_SETUP &Setup,		   // 加工设置
								   CSmtCheckMdl &ExtndSurfMdl, // 延伸面模型
								   CPtrArray &ExtndSurf ) ;    // 延伸面
	// 根据类型提取曲面
	BOOL (* m_pGetGeoSurf)( void* AtClass, JDNC_SETUP& Setup, CPtrArray& AllSurf, int nSurfMode ) ;

	// 构建加工模型
    BOOL (* m_pGetBitmapInfo)( void* AtClass         ,  /*图形对象*/
		                       BITMAP_INFO& BmpInfo) ;  /*图片信息*/
	// 计算最大高度
    int (* m_pGetTopHeight)( void* AtClass , int MethodType, int SurfMode, double& TopHeight );         /*图形对象*/
	// 计算投影基面的高度
	BOOL ( *m_pGetProjectSurfTop)( void *AtClass, double &dTop ) ;
	// 计算边界盒
    BOOL (* m_pGetEntBox)( void* AtClass, 
		                   DOUBLE dBox[2][3],
						   int nType );    /*图形对象*/
	// 得到加工面
	BOOL (* m_pGetTPlaneFrame)( void* AtClass,
							    RFRAME& lf ) ;
	// 寻找是否有网格面
	BOOL (* m_pFindNetSurf) ( void* AtClass,
						      int nSurfMode ) ;
	// 寻找是否有网格面
	BOOL (* m_pGetAllPathBefore) ( void* AtClass,
                               JDNC_SETUP& Setup,
                               CPtrList& AllPCom,
						          void* AtPComb, double* dMinStock ) ;
	// 获得毛胚包围盒
	BOOL (* m_pGetStockBox) (void* AtClass,
							    BOX3D& box ) ;
	// 或者加工范围数据
	BOOL (* m_pGetShapeRange) ( void *AtClass, CJDMethod &cParam,
								double &dTop, double &dCutDepth ) ;

	// 得到导动线
	BOOL (* m_pGetGuideCurve )( void * AtClass, 
													   JDNC_SETUP& Setup ,  /*计算设置*/  
													  CPtrList& AllEnt ) ; /*对象列表*/
	// 得到所有曲线和竖直的曲线路径
	BOOL (* m_pGetAllCurveAndVerticalPath)( void* AtClass,
								            JDNC_SETUP& Setup, /*计算设置*/
								            CPtrList&   AllCurve, /*所有曲线*/
								            CPtrList&   VerticalPath) ;/*竖直的曲线路径*/

	//guomin 
	//构建加工模型同时取出网格快速开粗算法中用于加速的矩形网格,大浮雕快速开粗计算中需要使用2013/02/19
	BOOL (* m_pBuildCheckMdlnew) ( void* AtClass         ,  /*图形对象*/
														JDNC_SETUP&  Setup    ,  /*图形对象*/
														CSmtCheckMdl& CheckMdl,  /*加工模型*/
														int SurfType,/*曲面类型*/
														CGeoNetSurf *&pRectSurf) ;
public :
	BOOL FindNetSurf( int nSurfMode ) ;
	BOOL GetAllSurface( JDNC_SETUP& Setup, C3DSurfArray& AllSurf ) ;
	BOOL GetRoughSurface( JDNC_SETUP &Setup, C3DSurfArray &AllSurf ) ;
	BOOL GetAllEntity(  JDNC_SETUP& Setup , CPtrList& AllEnt, int nType, RFRAME* LocMtx=NULL );
	BOOL GetAll5XPoint(  JDNC_SETUP& Setup , CPtrList& AllEnt , CPtrList& AllEnt5X ) ;
	BOOL GetAll5XObject(  JDNC_SETUP& Setup , int nType, void* pData ) ;
	BOOL GetEntBox( DOUBLE dBox[2][3], int nType ) ;
    CSmartLoop* GetAllContour( JDNC_SETUP& Setup , BOOL Redepth = FALSE ) ;
	CSmartLoop *GetAllPartContour( JDNC_SETUP &Setup, BOOL Redepth = FALSE ) ;
	CSmartLoop *GetAllStockBnd( JDNC_SETUP &Setup ) ;
    CSmartLoop* Get5AxTrimContour( JDNC_SETUP& Setup , RFRAME& TrimMtx ) ;
	CSmartLoop *GetRoughContour( JDNC_SETUP &Setup ) ;
	BOOL GetSharpEdges(JDNC_SETUP &Setup, CPtrList& SharpEdges);
	BOOL GetGuideCurve( JDNC_SETUP &Setup, CPtrList &AllEnt ) ;

	BOOL HasCheckSurf() ; //判断是否有干涉检查面
	BOOL HasRoughSurf() ; //判断是否有毛坯面
	int	 GetRoughType() ; // 得到毛坯类型
	int  SetRoughType( int nType ) ; // 设置毛坯类型
	double GetRoughStock() ;// 得到毛坯表面余量
	BOOL HasPlaneSurf( CPtrArray* AllSurf, int nType, double dTol ) ;// 寻找平面
	BOOL GetTPlaneFrame( RFRAME& lf ) ;
	BOOL GetAllRemainPath( JDNC_SETUP& Setup , CPtrList& AllEnt, double* dMinStock=NULL ) ;

	// 构建检查模型
	BOOL BuildCheckModel( JDNC_SETUP& Setup   , CSmtCheckMdl& CheckMdl, int nSurfMode = NCDEF_SURF_DRIVE ) ;
	BOOL BuildCheckModelByMask( JDNC_SETUP& Setup, CSmtCheckMdl& CheckMdl, int JDSelEntMask, JDNC_PRGDEF& PrgDef ) ;
	// 构建毛坯模型
	BOOL BuildStockModel( JDNC_SETUP &Setup, CSmtCheckMdl &StockMdl, BOX3D &box ) ;
	// 构建叶轮毛坯型
	BOOL BuildImpStockModel( JDNC_SETUP &Setup, CSmtCheckMdl &StockMdl, BOX3D &box ) ;
	// 构建投影变换基面模型
	BOOL BuildProjectModel( JDNC_SETUP &Setup, CSmtCheckMdl &ProjMdl ) ;
	// 根据类型提取曲面
	BOOL GetGeoSurf( JDNC_SETUP& Setup, CPtrArray& AllSurf, int nSurfMode ) ;
	// 建立指定曲面的模型（两曲线侧铣加工时使用）
	BOOL BuildWallMdlBy2Curves(  JDNC_SETUP &Setup, CSmtCheckMdl &WallMdl, CGeoCurve*pTopCurve, CGeoCurve*pBtmCurve ) ;
	// 建立加工面的延伸面模型
	BOOL BuildExtendSurfMdl( JDNC_SETUP &Setup, CSmtCheckMdl &ExtndSurfMdl, double dExtndDist ) ;
		// 得到加工面的延伸曲面
		BOOL ExtendDriveSurfaces( CPtrArray &AllSurf, double dExtndDist, CPtrArray &AllExtndSurf ) ;
	// 访问图片信息
	BOOL GetBitmapInfo( BITMAP_INFO& BmpInfo ) ;
	// 获得特征点
	BOOL GetFeaturePoint( JDNC_SETUP& Setup, CPtrList& AllEnt, JDNC_POINT& Point) ;
	// 获得曲面的最大高度
    BOOL GetSurfaceTopHeight( int MethodType, int nSurfMode, double & TopHeight ) ;
	// 获取投影基面的最大高度
	BOOL GetProjectSurfTop( double &dTop ) ;
	// 获得毛胚包围盒
	BOOL GetStockBox ( BOX3D& box );
	// 获得加工范围数据
	BOOL GetShapeRange( CJDMethod &cParam, double &dTop, double &dCutDepth ) ;
	// 生成叶轮毛坯面
	void CreateImpStockSurf( JDNC_SETUP &Setup, CPtrArray & AllSurf ) ;
	// 得到所有曲线和竖直的曲线路径
	BOOL GetAllCurveAndVerticalPath( JDNC_SETUP& Setup, CPtrList& AllCurve, CPtrList& AllComb ) ;

	//guomin
	//构建加工模型同时取出网格快速开粗算法中用于加速的矩形网格,大浮雕快速开粗计算中需要使用2013/02/19
	BOOL BuildCheckModelnew( JDNC_SETUP& Setup   , CSmtCheckMdl& CheckMdl,CGeoNetSurf *&pRectSurf, int nSurfMode = NCDEF_SURF_DRIVE ) ;

};
typedef CTypedPtrList<CPtrList,CGeoTrmSurf*>	CGeoTrmSurfList;
typedef CTypedPtrList<CPtrList,CGeoCurve*>		CGeoCurveList;

///////////////////////////
// 路径发生器
class DLL_EXPORT CSmartPathGen 
{
public :
	CSmartPathGen()          ; 
	virtual ~CSmartPathGen() ; 
	virtual int GetMoveType(){ return -1 ; }

public : // 加工参数
	int			m_bMoveFlag       ; // 曲面精加工中使用的走刀设置
	JDNC_SHAPE  m_cShapeDef       ; // 雕刻形状
	JDNC_STOCKEX m_cStockDef      ;	// 加工余量
	JDNC_TOOLEX m_cToolDef        ; // 刀具设置
	JDNC_SPEED  m_cSpeedDef       ; // 下刀控制
	JDNC_SETUP  m_cSetupDef       ; // 计算设置
	JDNC_FEED   m_cFeedDef        ; // 进给设置 
	JDNC_PATHPNT m_cPathPntDef    ;	// 路径起末点
	JDNC_METHOD m_cMethodDef      ; // 路径参数
	JDNC_PROCESS m_cProcessDef	  ; // 工艺参数
	JDNC_5DCTRL  m_c5DCtrlDef     ; // 多轴控制参数
	JDNC_TRANSFORM m_cTransfDef   ; // 路径变换参数
	JDNC_SLAYER	 m_cSlayerDef	  ;	// 侧向分层参数

public :
    JDNC_PRGDEF m_cPrgDef        ; // 计算进度
	CSmartTool*	m_pTool			 ; // 加工刀具
	int			m_nErrorType     ; // 错误类型
	CSmartTool*	m_pSafeTool		 ; // 检查刀具
    int         m_bFollowOuter   ; // 环切方向
    CSmartTool*  m_pHolderTool   ;

	int			m_nCalcThreadNum ; // 参与计算的线程个数，小于2则无须启动多线程
	double		m_dTopHeight	 ;
	double		m_dBotHeight	 ;
	double		m_dCutDepth		 ;

	CSmartGraphic *m_pGraph		; // 临时，用于计算GetCutDepth\GetTopHeight
	CJDParameter   m_cParamter	; // 同上，用于计算GetCutDepth\GetTopHeight
	int            m_bRComb     ; // 标记半径磨损补偿延长线路径是否完全生成，为2时未完全生成 qqs、xiejunxian 2013.04.28
public : // 加工参数
	RFRAME		m_dNcMtx		;  // 当前加工面
	BOX3D*		m_pStockBox		;  // 毛胚包围盒

	int			m_nData			; // 私有数据传递	
	void*		m_pData			; // 私有数据传递	
	double      m_dMaxStep      ; //保留参数设置中的平坦系数设置，环绕等距消除马赛克中提速用
public :
	// 生成刀具路径
	virtual BOOL GeneratePathEx( CPathGroup& NewPath, CSmartGraphic& Graph ) ;
	// 是否支持计算进度
	virtual BOOL IsSupportProgress() { return FALSE ; }
	//  设置加工参数
	void  SetEngraveParam( CPathGroup& NewPath , CSmartGraphic& Graph ) ;
	// 生成刀具路径
	BOOL GeneratePath( CPathGroup& NewPath, CSmartGraphic& Graph ) ;
	// 根据加工模型自动校正深度范围
	virtual BOOL AutoAdjustCutDepth(CSmtCheckMdl& DriveMdl);
public : //访问参数
	DOUBLE GetTopHeight()   { return m_dTopHeight ;}
	DOUBLE GetCutDepth()    { return m_dCutDepth  ;}
	DOUBLE GetBottomHeight(){ return m_dBotHeight ;}
	int    GetSortType()  { return m_cSetupDef.m_cOrderDef.m_nSortType ; }
	int    GetLayerOrder(){ return m_cSetupDef.m_cOrderDef.m_nLayerOrd ; }
	int    GetMillDir()   { return m_cSetupDef.m_cOrderDef.m_nMillDir;}
	DOUBLE GetSideStock() { return m_cStockDef.m_dSideStock  ; }
	DOUBLE GetDepthStock(){ return m_cStockDef.m_dDepthStock ; }
	JDNC_TOL&   GetCurveTol()  { return m_cSetupDef.m_cTolDef ;  }
	JDNC_TOL&   GetSurfaceTol(){ return m_cSetupDef.m_cTolDef ;  }
	JDNC_COR&   GetCorDef()    { return m_cSetupDef.m_cCorDef ;  }
	JDNC_ORDER& GetOrderDef()  { return m_cSetupDef.m_cOrderDef; }
	JDNC_LAYER*    GetLayerDef() { return & m_cFeedDef.m_cLayerDef ; }
	JDNC_SLOT*     GetSlotDef()  { return & m_cFeedDef.m_cSlotDef  ; }
	JDNC_PLUNGE*   GetPlungeDef(){ return & m_cFeedDef.m_cPlungeDef; }
	JDNC_LEAD*     GetLeadDef(){ return & m_cFeedDef.m_cLeadDef ; }	
	JDNC_CONNECT3D* GetConnecte3DDef(){ return & m_cFeedDef.m_cConnectDef; } 
	void	GetLeadCutShift(JDNC_LEADOUT& out, double dShift[2]);
    BOOL    GetToolHolder( JDNC_HOLDER& Holder ) ;
	CJDTool* GetTool() ;
	virtual CSmartPathGen& operator = ( CSmartPathGen& PathGen ) ;
	double			GetLeadArcLen();
	DOUBLE GetDriveSurfOffset() ;
	DOUBLE GetDriveSurfZMove() ;
	DOUBLE GetCheckSurfOffset() ;
	DOUBLE GetCheckSurfZMove() ;
    DOUBLE GetWedmPlanarGap() ;
	DOUBLE GetToolBotArea( CSmartTool *pTool ) ;
    // 是否删除边界路径点
    BOOL IsDelBndPoint() ;
	// 是否删除平面路径点
    BOOL IsDelFlatPoint() ;
	// 获得层间干涉距离
    DOUBLE GetLoopCheckTol() ;
	// 生成刀柄模拟刀具
    int CreateHolderTool() ;
	// 添加刀杆
	BOOL CreateToolShank( JDNC_HOLDER &Holder ) ;
	// 更新退刀参数
	void UpdateLeadOutParam(JDNC_LEADOUT& leadOut, JDNC_LEAD& Out);
	// 圆弧内切进刀时是否需要外切引入
	BOOL NeedOutArcAtArcIn(JDNC_LEAD& LeadDef, BOOL IsOut);
	// 获取圆弧内切进刀时外切引入参数
	void GetParamOfOutArcAtArcIn(BOOL IsOut, double& dRad, double& dAng);
	// 圆弧外切是否需要直线引入
	BOOL NeedLineAtArcOut( JDNC_LEAD &LeadDef, BOOL IsOut ) ;
public :
	// 定义刀具
	CSmartTool* CreateSmartTool( JDNC_TOOL& Tool, DOUBLE SurfStock, double dPlanar ) ;
	CSmartTool* CreateSmartTool( JDNC_TOOLEX& Tool, DOUBLE SurfStock, double dPlanar ) ;
    // 生成所有的刀具类型,包括成型铣刀
    CSmartTool* CreateAllTypeTool( JDNC_TOOLEX& Tool, DOUBLE SurfStock, double dPlanar ) ;
	int UpdateTool3D(CSmartGraphic& Graph) ;
	int UpdateTool2D() ;
	void SetCheckMachMdlBox(PNT3D dSrcBox[2], PNT3D dDesBox[2]);

	// 定义外形
    BOOL DefineShapeAngle( JDNC_SHAPE& Shape );  /*雕刻形状*/
    // 定义分层　
    BOOL DefineLayerDepth( JDNC_LAYER& Layer ); /*分层参数*/
	// 自定义分层
	void AssignSelfDefineLayerDepth(int nLayerFlag, double dTop, double dBot, 
									double*& dDepth, int& nCount, BOOL bSurfMode);
	// 自定义渐变分层
	void UserDefineGradualLayerDepth(int nLayerFlag, double dTop, double dBot, 
									double*& dDepth, int& nCount, BOOL bSurfMode);
	// 渐变分层
	void DefineGradualLayerDepth(double dTop, double dBot, double &dFirstDepth, double &dLastDepth, 
							double*& dDepth, int& nCount);
	// 计算侧向分层
	BOOL DefineSideLayerInc( JDNC_LAYERDEF& cDef );
    // 定义分层　
    BOOL RepairBlindRadius( JDNC_PLUNGE& Plunge );
    // 定义开槽分层　
    BOOL DefineSlotDepth( JDNC_SLOT& Slot  , /*开槽深度*/
		                  JDNC_TOOLEX *Tool= NULL ); /*雕刻刀具*/ 
	// 校正加工参数
    void ValidGeneralParam() ;
	// 定义置侧面信息
    BOOL  DefineRCompFlag( DOUBLE& dRComp   , 
						   BOOL bSameComp ) ;

	BOOL  OffsetBorder(CSmartLoop*& AllLoop);
	// 初始化
	BOOL  InitPathGen( JDNC_PARAM& PathDef ) ;
	
	// 返回开槽
	DOUBLE GetDownStep()  ;

public :
	// 根据位置计算下刀路径
	int	CreatePlungePath( CPathCombine&  PComb ,   /*路径组  */
						  JDNC_PLUNGE&  Plunge ,   /*下刀方式*/
						  PNT3D   Position     ,   /*刀具位置*/
						  CSmartCurve* Bound    );  /*轮廓曲线*/ 
	// 从偏移的轮廓线上和下刀点查找最近的下刀位置
	// 返回值说明：0,查找失败;1,查找成功;2,查找成功,代表在指定点螺旋下刀
	#define  HELIX_REFF_TRUE 2 //用于标记指定点螺旋下刀
    int FindPlungePoint( JDNC_PLUNGE& Plunge,/*下刀方式*/
		                CSmartLoop&  Contour  ,  /*原始轮廓*/ 
						 CSmartLoop* AllCont   ,  /*边界曲线*/
						 PNT3D     ToPoint     ,  /*目标位置*/
						 PNT3D     AtPoint     , /*下刀位置*/
                         CSmartCurve*  StartCurve=NULL ) ;/*原始曲线*/
	// 从偏移的轮廓线上和下刀点查找最近的下刀位置
    int  FindPlungeCurve( CSmartLoop&   Contour , /*原始轮廓*/
						  PNT3D         Position, /*目标位置*/
						  CSmartCurve*& ObjCurve);/*下刀曲线*/
	// 添加下刀路径
    int InsertPlungePath( CPathCombine&   TComb  ,  /*路径组合*/
						  CSmartLoop&    Contour ,  /*轮廓边界*/
						  JDNC_PLUNGE&  Plunge   ,  /*下刀参数*/
		                  CSmartLoop*   PartBnd  ,  /*模型非切区域*/
		                  CSmartLoop*   BlankBnd ,  /*毛坯边界*/
						  DOUBLE MaxDist[2]		 ,  /*搜索距离[0], 延伸距离[1]*/
						  int nFlag[2]			);  //走刀类型\顺逆铣
	// 添加垂直下刀路径
    int InsertVertPlunge(CPathCombine&    TComb  , /*路径*/ 
				         JDNC_PLUNGE&     Plunge );/*下刀*/ 
    // 计算进刀、退刀路径
	int  CreateBoundLead( JDNC_LEAD&  LeadDef   ,  /*切入切出定义*/
		                  CSmartCurve&  Curve   ,  /*原始曲线    */
		                  CSmartSect*&  LeadIn  ,  /*导入段      */
						  CSmartSect*&  LeadOut ,  /*导出段      */
						  CSmartLoop*  ContHead);  /*干涉检查    */
	//切入切出分别定义的进退刀方式
	int  CreateBoundLeadEx( JDNC_LEAD&  LeadInDef  ,  /*切入定义*/         
		                    JDNC_LEAD&  LeadOutDef ,  /*切出    */
		                    CSmartCurve&  Curve    ,  /*原始曲线*/
		                    CSmartCurve&  LeadInCv ,  /*导入段  */
		                    CSmartCurve&  LeadOutCv,  /*导出段  */
		                    CSmartLoop*   ContHead ,  /*干涉检查*/
		                    DOUBLE     OverlapLen  ,  /*重叠距离*/
                            DOUBLE     DepthAt     ,  /*路径所在高度*/
                            DOUBLE     ZShift[2] ) ;  /*起末点抬高  */  
	//切入切出分别定义的进退刀方式,计算进退刀路径，重复加工路径，2013.12.09 liuxin
	int  CreateBoundLeadExNew( JDNC_LEAD	& LeadInDef  ,  /*切入定义*/         
							   JDNC_LEAD	& LeadOutDef ,  /*切出    */
							   CSmartCurve	& Curve    ,	/*原始曲线*/
							   CSmartCurve	& OverlapCv,	/*重复加工段*/
							   CSmartCurve	& LeadInCv ,	/*导入段  */
							   CSmartCurve	& LeadOutCv,	/*导出段  */
							   CSmartLoop	* ContHead ,	/*干涉检查*/
							   DOUBLE		OverlapLen  ,	/*重叠距离*/
							   DOUBLE		DepthAt     ,	/*路径所在高度*/
							   DOUBLE		ZShift[2] ,     /*起末点抬高  */
							   DOUBLE       DepthInc) ;	    /*层深，用于沿轮廓进刀  */  
	// 计算进刀路径
	CSmartSect* CreateLeadEnt(	JDNC_LEAD& LeadDef, /*结构定义*/
								PNT2D   Point ,      /*端点*/
								VEC2D   Normal,      /*法失*/
								int     IsOut ,      /*切出*/
								CSmartLoop* ContHead);/*干涉检查*/

	// 计算退刀路径
	CSmartSect* CreateLeadOutEnt(	JDNC_LEAD& LeadDef, /*结构定义*/
									PNT2D   Point ,     /*端点*/
									VEC2D   Tangent,    /*切失*/
									VEC2D   Normal,     /*法失*/
									int     IsOut ,      /*切出*/
									CSmartLoop* AllCont);/*干涉检查*/
	// 计算进刀路径
	BOOL CreateLeadEntNew(	JDNC_LEAD& LeadDef,		/*结构定义*/
							PNT2D   Point ,			/*端点*/
							VEC2D   Normal,			/*法失*/
							int     IsOut ,			/*切出*/
							CSmartLoop* AllCont,	/*干涉检查*/
							CSmartCurve& leadCv);	/*退刀路径*/

	// 计算退刀路径
	BOOL CreateLeadOutEntNew(	JDNC_LEAD& LeadDef,	/*结构定义*/
								PNT2D   Point ,		/*端点*/
								VEC2D   Tangent,		/*切失*/
								VEC2D   Normal,		/*法失*/
								int     IsOut ,      /*切出*/
								CSmartLoop* AllCont,	/*干涉检查*/
								CSmartCurve& leadCv);/*退刀路径*/
	// 计算开槽加工路径
	int CreateECutPath( CPathCombine& PComb ,   /*数据*/
					    CSmartLoop& Contour ,   /*轮廓*/
					    JDNC_SLOT&  ECut   ,  /*参数*/
					    JDNC_PLUNGE& Plunge); /*下刀*/
	// 计算行切加工路径
	int CreateLinearPath( CPathCombine& PComb    ,  /*保存路径*/
					      CSmartLoop& Contour    ,  /*轮廓曲线*/
		                  JDNC_LINEAR& Linear );  /*行切参数*/ 
	// 计算环切加工路径
	int  CreateFollowPath( CPathCombine&     PComb ,  /*保存路径*/
						   CSmartLoop&      Contour,  /*轮廓曲线*/
		                   JDNC_FOLLOW&  Follow ,  /*环切参数*/ 
						   bool bSmooth=false);
	// 计算平面环绕等距加工路径
	int  CreateFollowPathEx( CSmartLoop&   Contour,  /*轮廓曲线*/
		                   JDNC_FOLLOW&  Follow ,    /*环切参数*/ 
                          CSmtCPathLib&  AllPath);  /*保存路径*/
	// 计算清角加工；路径
    int CreateCornerPath( CPathCombine&   TComb  ,  /* 路径组  */
				          CSmartLoop&     Contour); /* 轮廓组  */
	// 计算螺旋走刀路径
    int  CreateSpiralPath( CPathCombine& PComb   ,  /*保存路径*/
						   CSmartLoop& Contour   ,  /*轮廓曲线*/
		                   JDNC_SPIRAL&  Spiral); /*行切参数*/ 
	// 在固定位置生成下刀路径
    BOOL AddPlungePathAt( CPathCombine& PComb  ,  /*路径组  */
					      JDNC_PLUNGE& Plunge  ,  /*下刀方式*/
						  PNT2D   ToPoint      ,  /*连接位置*/
						  PNT2D   AtPoint      ,  /*刀具位置*/
						  CSmartLoop* Cont = NULL,/*轮廓边界*/
						  int    nHexRef = 0 ) ; /*指定下刀点标记*/
	// 沿轮廓曲线增加下刀路径
	int  AddPlungePathBy( CPathCombine&   PComb  ,  /*路径组  */
				          JDNC_PLUNGE&    Plunge ,  /*下刀方式*/
						  PNT3D         ToPoint  ,  /*连接位置*/
						  CSmartCurve&    ByCurve ,  /*轮廓曲线*/
						  BOOL         IsStart  );  /*对齐基准*/
	// 沿轮廓折线下刀路径
	int  AddPlungePathRamp( CPathCombine&   PComb  ,  /*路径组  */
				          JDNC_PLUNGE&    Plunge ,  /*下刀方式*/
						  PNT3D         ToPoint  ,  /*连接位置*/
						  CSmartCurve&    ByCurve ,  /*轮廓曲线*/
						  BOOL         IsStart  );  /*对齐基准*/
    // 生成路径端点处的进退刀路径
    BOOL AddPlungeLeadPath( CSmtCheckMdl& DriveMdl    , 
                            JDNC_CONNECT3D& ConnectDef, 
                            CSmtCPathLib& AllPath     );
    //生成路径端点处的进退刀路径, 并转换成PathGroup
    BOOL AddLeadPathAndAddInPGroup(CSmtCheckMdl& DriveMdl, 
                                   CSmtCPathLib& AllPath ,
                                   CPathGroup&   NewPath ) ;
	// 引入线段变成路径单元
	CPathEntity* GetLeadEnt( CSmartSect* LeadEnt, DOUBLE ZValue ) ;
	// 计算最大半径
	DOUBLE  GetMaxRadius( PNT2D dMin, PNT2D dMax, PNT2D Center ) ;  
	// 底面尺寸基准
	CSmartLoop*  DefineAntifaceContours( CSmartLoop* LpHead ) ;
    // 设置半径补偿属性
    int AddRCompMask( CPathCombine& TComb, 
                      int           RCompMask,
					  BOOL bAutoClose = TRUE); // 末段自动关闭补偿

	// 设置三维半径补偿属性 qqs 2013.09.25
	int Add3DRCompMask( CPathCombine& TComb, 
						int           RCompMask,
						BOOL bAutoClose = TRUE); // 末段自动关闭补偿

    // 螺旋连接一组轮廓
    void RemoveNousedLoop( CSmtLoopArr& AllLoop );
    void GetSubLoopGroup( CSmtLoopArr& AllLoop, CSmtLoopArr& SubArr ) ;
    CSmartCurve* SmoothConnectLoops( CSmtLoopArr& LoopArr, DOUBLE CntLen, CSmartLoop& Contour ) ;
	// 非螺旋连接路径
	CSmartCurve *ConnectLoopsByLine( CSmtLoopArr &LoopArr, double CntLen, CSmartLoop &Contour ) ;

public : //曲面加工模型

	// 曲线转换成加工点数据
    CSmtCutPath* CurveToCPath( CSmartCurve&  Curve, 
		                       JDNC_TOL& Tol ) ;
	// 提取曲面的参数边界环，转换成平面轮廓,然后变换到加工平面
    CSmartLoop* ExtractSurfLoop( CGeoSurf* Surf , 
		                         JDNC_TOL& Tol  , 
								 RFRAME *NcMtx=NULL) ;
	// 提取曲面的空间边界环
    CSmartLoop*  ExtractAllBound( C3DSurfArray& AllSurf ,  /*所有曲面*/ 
								  JDNC_TOL&  BoundTol   ,  /*边界误差*/
								  RFRAME*    NcMtx      ); /*加工平面*/

public :
	//根据距离将PathEttity分解,放在不同的PathComb中
	void SeparatePathComb(CPathGroup& cPathGroup);
	//添加3D加工的下刀、进刀和退刀路径
	void AddAuxPathComb(CPathGroup& cPathGroup, JDNC_CONNECT3D& cPlungeRetract);
	//添加3D曲面加工下刀路径
	CPathEntity* AddPlungePath(JDNC_CONNECT3D& cPlungeRetract, CPathEntity* pPathEnt);
	//添加3D曲面加工抬刀路径
	CPathEntity* AddRetractPath(JDNC_CONNECT3D& cPlungeRetract, CPathEntity* pPathEnt);
	void	NcCheckPGroup(CSmartTool* pMiller, RFRAME& cRFrame, double dArcTol, CPathGroup& PGroup);
	void	ConvertPGroupToSmtCutPathLib(CPathGroup& PGroup, CSmtCPathLib& AllPath, JDNC_TOL& Tol);
	CSmartLoop* ResortContour(CSmartLoop* AllLoop ) ;
    // 一组边界环按照高度优先排序, LoopArr 的 m_nDepth必须有效
    BOOL  SortLoopArray( CSmtLoopArr& LoopArr, /*轮廓组  */  
		                 DOUBLE Tol      ) ;   /*轮廓误差*/
	
public : 
	char*	GetErrorMsg();
	// 显示进度条
	void ProcessNewFunc( JDNC_PRGDEF &ProgDef, int nIDS ) ;
protected :
// 计算路径边界
	CSmartLoop* OffsetPathBound( CSmartLoop& Contour    ,  /*原始轮廓*/
		                         DOUBLE Depth           ,  /*雕刻深度*/
								 DOUBLE SideTol =0.0    ,  /*侧边预留*/
                                 BOOL   SortCont=FALSE  ); /*区域排序*/
// 计算路径边界
	CSmartLoop* OffsetPathBound( CSmartLoop& Contour, DOUBLE Depth, 
		                         CSmartTool* Tool , JDNC_LAYER * Layer ,
								 DOUBLE SideStock ) ;
	// 计算路径边界
	CSmartLoop* OffsetPathBoundEx( CSmartLoop& Contour    ,    /*原始轮廓*/
		                       DOUBLE Depth           ,  /*雕刻深度*/
		                       DOUBLE SideTol =0.0    ,  /*侧边预留*/
		                       BOOL   SortCont=FALSE,     /*区域排序*/
							     int* nRev = NULL);/*圆弧延伸输出参数,标记半径补偿方向, add in 2009.8.12*/
	
	// 重新设置高度
    BOOL RedepthNewPath( CPathGroup& NewPath,  /*雕刻路径*/ 
                         CSmartGraphic& Graph,  /*图形函数*/   
						 BOOL	bKeep[2],		// <I> [0]高度,[1]速度
						 DOUBLE& dTop );  /*最大高度*/
	// 计算高度
    DOUBLE SnapNearestPoint( CPtrList& Curve3D, PNT3D Point ) ;
	BOOL   SnapNearestPointNew( CPtrList &CurveList,	//<I> CRedepthCurve 
								PNT3D point ,			//<I> point
								double	t[2],			//<I> 输入初始参数t
								RFRAME &LocFrame ,		//<I> 输入相对坐标系
								BOOL	bKeep[2],		//<1> [0]高度,[1]速度
								double	dKeep[2] )	;	//<O> [0],高度,[1]速度
	// 重新确定边界高度
	CSmartLoop*  RedefineBoundLoops( CSmartLoop* AllCont ) ;
	// 生成外轮廓开阔的下刀路径
    BOOL CreatePlungeAtBlank( CSmartLoop& Contour   , /*所在区域*/
                              PNT3D Positon         , /*下刀位置*/
							  PNT3D dInfo[2]		, //切向法向
						      DOUBLE MaxDist[2]     , /*[0]最大长度,[1]延伸长度*/
                              CSmartLoop* PartBnd   , /*模型边界, 用于检查是否过切*/
                              CSmartLoop* BlankBnd  , /*毛坯边界, 用于查找引入位置*/
							  int	nMoveType		, // 走刀类型
							  CSmartCurve& PlgCurve); /*下刀路径*/
	// 起末点添加平面进退刀路径
    int AddLeadPathAtEnd( JDNC_LEAD& LeadDef, CPathCombine& TComb, CSmartLoop&  Contour ) ;
    int AddLeadSectAtEnd( JDNC_LEAD& LeadDef, CSmartCurve& Curve, CSmartLoop*  AllCont ) ;
	// 将环的起点设置到BLANK_SECT处
	void SetAllLoopHeadAtBlank( CSmartLoop *AllLoop ) ;
	void SetLoopHeadAtBlank( CSmartLoop *pLoop ) ;
	// 查看待加工区域中是否有BLANK_SECT段
	BOOL HasBlankSectInContour( CSmartLoop *BndHead ) ;

	/*切入切出分别定义的进退刀*/
	int AddLeadPathAtEndEx( JDNC_LEAD& LeadInDef, JDNC_LEAD& LeadOutDef,CPathCombine& TComb, CSmartLoop&  Contour,DOUBLE Depth, DOUBLE ZShift[2], DOUBLE DepthInc = 0.) ;
	int AddLeadSectAtEndEx( JDNC_LEAD& LeadInDef, JDNC_LEAD& LeadOutDef,CSmartCurve& Curve, CSmartLoop*  AllCont,DOUBLE Depth, DOUBLE ZShift[2], BOOL bRComp ) ; 
	int GenLeadPathAtEndEx( JDNC_LEAD& LeadDef, BOOL IsOut,CPathCombine& TComb, CSmartLoop&  Contour, DOUBLE Depth, DOUBLE ZShift,CPathCombine& LeadComb, DOUBLE DepthInc = 0.) ;
	int GenLeadSectAtEndEx( JDNC_LEAD& LeadDef, BOOL IsOut,CSmartCurve& Curve, CSmartLoop*  AllCont, DOUBLE Depth, DOUBLE ZShift,CSmartCurve& LeadCv, DOUBLE DepthInc = 0.) ;

	int AddLeadSectAtEndExNew(  JDNC_LEAD& LeadInDef, JDNC_LEAD& LeadOutDef,CSmartCurve& Curve, 
								CSmartCurve &LeadIn, CSmartCurve &LeadOut, CSmartLoop*  AllCont,
								DOUBLE Depth, DOUBLE ZShift[2], BOOL bRComp, DOUBLE DepthInc = 0.) ; 

	//用于生成沿轮廓下刀时的重复加工路径 20140828 xh
	void GenOverLappedPath( CPathCombine& TComb, CPathCombine& overLappedPath,  DOUBLE OverLapLen );
	void GenOverLappedPath( CSmartCurve& Curve, CSmartCurve& overLapped, DOUBLE OverLapLen );
	// 获取CPathCombine起末点的切向方向和法向方向，用于生成进退刀路径 qqs 2014.03.05
	void GetTanAndNorAtPathCombineEnd(CPathCombine& TPComb, VEC3D dTan[2], VEC3D dNor[2]);

	BOOL CreateLeadEntEx( CSmartCurve& OrgCurve,
		                  JDNC_LEAD& LeadDef, /*结构定义*/
                          CSmartCurve& LeadCv,
		                  PNT2D   Point ,      /*端点*/
		                  VEC2D   Normal,      /*法失*/
		                  int     IsOut ,      /*切出*/
		                  CSmartLoop* ContHead);/*干涉检查*/

	int  CreateLocalLead( CSmartCurve&  Curve    , 
	                      JDNC_LEAD&   tmpLeadIn ,
		                  JDNC_LEAD&   tmpLeadOut,
		                  PNT2D        Point[2]  ,
		                  VEC2D        Norm[2]   ,
		                  CSmartCurve&  LeadInCv ,
		                  CSmartCurve&  LeadOutCv,
		                  CSmartLoop*   Contour  ,
                          DOUBLE     DepthAt     ,  /*路径所在高度*/
                          DOUBLE     ZShift[2]   ,  /*起末点抬高  */
						  DOUBLE     DepthInc = 0.);

	double GetPathEntityFeedRate(CPathEntity* PathEnt, JDNC_SPEED SpeedDef);
	BOOL ConnectLeadInOutPos(	CPathCombine &PComb	,	// <I> 输入路径
								CSmartLoop *AllCont ,	// <I> 区域
								double	dRComp[2]	,	// <I> 补偿值
								int	nRCompSide		,	// <I> 补偿方向 
								double pos[2]	) ;		// <I> 进刀点
	
	BOOL ConnectLeadInOutPos(	CPathCombine &PComb	,	// <I> 输入路径
								CSmartLoop *AllCont	,	// <I> 区域
								double	dRComp[2]	,	// <I> 补偿值
								int		nRCompSide	,	// <I> 补偿方向 
								JDNC_LEADOUT &cLead	, 	// <I> 退刀参数
								BOOL		bCnt[2]		) ;	// <O> 成功返回TRUE

	// 如果切入切出有高度，则将圆弧段的高度转为样条高度
	BOOL ChangeLeadArcHeightBySpline(	CSmartCurve *LeadCurve ,// Inn:曲线 
										JDNC_LEAD&   tmpLead   ,
										DOUBLE		ZShift     ,// Inn:高度
										BOOL	bLeadIn ) ;		// Inn:切入

    // 根据加工面范围重设保护面范围
    virtual void SetCheckFltBoxByDriveMdl( CSmtCheckMdl& DriveMdl ,
                                           CSmtCheckMdl& CheckMdl );
// add by yul for srough
public:

	// 将环组投影到模型上，生成投影路径
	void ProjLoopOnMdl( CSmtCheckMdl& DriveMdl, CSmtCheckMdl& RoughMdl, 
						CSmtCPathLib& AllPath, 	CSmartLoop* pHead, 
						JDNC_PRGDEF& PrgDef, int bFlag ) ;
	// 生成干涉检查模型
	int  GenCheckMdl( CSmartGraphic& Graph, CSmtCheckMdl& CheckMdl, int nSurfMode = NCDEF_SURF_DRIVE ) ;
	// 设置路径的Z坐标
	void SetCutPathZ( CSmtCutPath* pPath, int bFlag ) ;
	// 重新设置高度
	BOOL RedepthCPathCombine (CPtrList	&	CurveList,		// <I> CRedepthCurve
                              CPathCombine*	PComb,			// <I> 路径
							  BOOL			bKeep[2],		// <I> [0]高度,[1]速度
							  RFRAME	&	LocFrame);		// <I> 局部坐标系
    BOOL RedepthPathEntity( CPtrList& Curve3D,    /*3D图形 CSmartCurve */ 
                            CPathEntity* Ent ) ;  /*路径单元           */  
	BOOL RedepthPathEntityNew(	CPtrList &CurveList,		// <I> CRedepthCurve
								CPathEntity *pEnt,			// <I> 路径单元
								BOOL		bKeep[2],		// <I> [0]高度,[1]速度
								RFRAME &LocFrame ) ;		// <I> 局部坐标系
	BOOL RedepthPathEntity(CPathEntity	* PathEnt,
						   BOOL			  IsOut,
		                   DOUBLE       & Depth);
	virtual BOOL IsLoopCutSameOffset();
	// 重新设置刀具锥刀的切削深度
    void ResetToolCutDepth( CSmtCheckMdl& DriveMdl,
                            CSmartTool*  Tool ) ;
	// 生成加工面和干涉面的混合模型
	BOOL BuildDriveAndCheckMdl( CSmartGraphic& Graph , CSmtCheckMdl& DriveMdl, 
								CSmtCheckMdl& CheckMdl, double dAngle = 0. ) ;
	// 生成五轴加工面和干涉面的混合模型
    BOOL Build5AxDriveAndCheckMdl( CSmartGraphic& Graph , 
                                   CSmtCheckMdl& DriveMdl, 
						           CSmtCheckMdl& CheckMdl) ;
	// 生成五轴加工面和干涉面的混合模型
    BOOL TransTo5AxDriveAndCheckMdl( CSmtCheckMdl& DriveMdl ) ;
	// 提取加工边界
	CSmartLoop * CreateAreaLoop ( CSmartGraphic &Graph, BOX3D &sbox, double r, BOOL &bAllLoop ) ;
	CSmartLoop * OffsetBndLoop ( CSmartLoop* AllLoop ) ;
	// 添加开阔标记
	void AddOpenFlag( CSmartLoop *AllLoop ) ;
	// 提示错误
	void PromptLoopErr( double h ) ;
	// 得到加工曲面的边界环
	CSmartLoop* GetSurfLoop( CSmartGraphic& Graph, int nSurfaceType = SMARTGRAPH_TYPE_ALLSURF ) ;
	// 初始化进度条
	void InitProg() ;
	// 分层
	void GetAllSurfLayer( double *dSurfZ, INT_PTR nNum, double dStepZ, double*& dZ, INT_PTR& nCount ) ;
	// 添加安全高度路径
	void Add3DSafeQuickPath(CPathGroup &NewPath);
	// 添加下刀路径和相对安全高度路径
	void AddPlungeAndRelativeH( CSmtCheckMdl *DriveMdl, CPathGroup *NewPath, BOOL bConnect = FALSE, double *dZ = NULL, INT_PTR nCount = 0 ) ;
	void AddHeadPlungePath( CPathCombine *PComb ) ;
	double CheckRelativeQuickH( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, double *dZ, INT_PTR nCount, double dTop ) ;
	CPathPLine3D *CreateRelativeQuick( PNT3D start, PNT3D end, double h ) ;
	BOOL AddConnectLine( CSmtCheckMdl *DriveMdl,CPathCombine *PComb, CPathEntity *pHead, PNT3D start, PNT3D end ) ;

	// 添加慢速下刀路径
	void AddSlowPlungePath(CPathGroup *NewPath, CSmartGraphic &Graph);
	BOOL Add5DSafeQuickPath(CPathGroup& pGroup);
	CSmtCutPath* Cal5DSafeQuickPathPoint(PNT3D point,VEC3D dir);
	// 2.5D添加下刀路径和相对安全高度路径
	void AddPlungeAndRelativeH(CPathGroup *NewPath, CSmartGraphic &Graph);

	// 5D添加慢速下刀路径
	void AddSlowPlungePathFor5DPath(CPathGroup * NewPath);
	BOOL ConvertQuickPathToPlungeAndQuickPath(CPathEntity	* &QuickPath,   // 快速下刀路径
											  DOUBLE		  PlungeDist,   // 慢下距离
											  CPtrList		& AllPath);		// 输出慢下路径和快速下刀路径

    // 创建MeshMdl 
    BOOL BuildMeshMdl( CSmtMeshMdl& MeshMdl  ,  /*网格模型*/
                       CSmtCheckMdl& DriveMdl,  /*加工模型*/
                       CSmartLoop* AllCont   ,  /*边界轮廓*/
                       BOOL   TrimByZSize    ); /*高度修剪有效*/  
public :
	BOOL OpenRegenFile() ;
	BOOL CreateRegenFile() ;
	// 生成外轮廓开阔的下刀路径
    BOOL CreatePlungeAtBlankEx( PNT3D Position        , /*下刀位置               */
								PNT3D dInfo[2]		  , //切向法向
						        DOUBLE MaxDist[2]     , /*[0]最大长度,[1]延伸长度*/
                                CSmartLoop* PartBnd   , /*模型边界, 用于检查是否过切*/
                                CSmartLoop* BlankBnd  , /*毛坯边界, 用于查找引入位置*/
								int nMoveType		  , //走刀类型
							    CSmartCurve& PlgCurve); /*下刀路径*/
    ////////////////////////////////////////
    // 在固定的位置生成相切的螺旋下刀路径
    int  CreateHelixPlunge( JDNC_PLUNGE& Plunge  ,  /*下刀方式*/
						    CSmartLoop& Contour  ,  /*边界轮廓*/ 
						    CPathEntity* CurrEnt ,  /*当前路径*/
                            CPathCombine& TComb  ); /*路径组  */
	//三维清角在指定点处生成下刀路径  2011.11.17  LiLin
	int CreateVertPlungeAtPos( CPathCombine& TComb  , /*路径*/ 
		                       JDNC_PLUNGE&  Plunge ); /*下刀*/
	//三维清角路径尖角降速
	BOOL SlowAtCorner( CPathGroup& PathGroup, float dSlowDist, float dSlowRate);
	// 三维清角减少分层抬刀 2013/8/6, liuxin
	void AddLinkPathBetweenLayers(CPathCombine *PComb, CSmartLoop* AllCont, DOUBLE dAngle);
	// 三维清角在指定点处重设起点  2013/8/6, liuxin
	void ReDefinePathStartPntAtPlungePos(CPathCombine *PComb, JDNC_PLUNGE&  Plunge);
public : // 基于虚拟加工模型的计算路径
    /* 构建三轴加工的虚拟加工模型 */
    BOOL BuildVirtualNcMdl3Ax( CSmartGraphic& Graph, CSmtVirtualNcMdl& VNcMdl3Ax ) ;
    /* 构建多轴加工的虚拟加工模型 */
    BOOL BuildVirtualNcMdl5Ax( CSmartGraphic& Graph, CSmtVirtualNcMdl& VNcMdl5Ax ) ;
	
public:  //路径投影和包裹变换
	void ProjAndWrapPath( CPathGroup& NewPath, CSmartGraphic& Graph );
	int ProjPath( CPathGroup& NewPath, CSmartGraphic& Graph );
	int WrapPath( CPathGroup& NewPath, CSmartGraphic& Graph );
	int PlungeMillPath( CPathGroup& NewPath, CSmartGraphic& Graph );
	//磨削调整
	BOOL ModifyGrind( CPathGroup& NewPath, int nType, DOUBLE depthInc, DOUBLE dist );
	BOOL ModifyGrindNew( CPathGroup& NewPath, int nType, DOUBLE depthInc, DOUBLE dist,int SinSegs ); //add by zy 2013.1.17
	void InsertNewCutPoint(CSmtCutPath * CutPath ,int  nPoint, BOOL StartAtExtreme = TRUE);// StartAtExtreme = FALSE表示起点不是波峰波谷, 2014.1.23 liuxin
	void CreateSinWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect);//类正弦波动
	void CreateTriWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect);//三角形波动
	void CreateTrapWave(CSmtCutPath * CutPath,DOUBLE depthInc,int nSect);//梯形波动
	BOOL MatchSpeedInModifyGrind( CSmtCPathLib &AllPath, CPathCombine &PComb );//磨削调整速度重新匹配 add by zy 2013.5.6
	void TransPointToCutPointEx(CSmtCPathLib &AllPath,CSmtCutPath*& pCutPath);//将原有SmtCutPath中的CutPoint转为CutPointEx  add by zy 2013.5.29
	// 3轴路径磨削调整，支持锥刀的锥角补偿 2014.3.5 liuxin
	void MdfyGrndFor3DPath(CPathGroup &NewPath, CSmartGraphic& Graph);
	BOOL MdfyGrndFor3DPath(CPathGroup &NewPath, CSmartGraphic& Graph, int WaveType, int RCompSide, 
						   int MdfyDir, DOUBLE MdfyHeit, DOUBLE Freq, BOOL AlongCone, BOOL ReadTime);
	BOOL TransCPathToCPath5x(CSmtCPathList &AllPath, CPtrList& CurveList, RFRAME& LocFrame, int RCompSide = 2, DOUBLE Angle = 0.0);
	CSmtCutPath* TransCPathToCPath5x(CSmtCutPath* CutPath, CPtrList& CurveList, RFRAME& LocFrame, int RCompSide = 2, DOUBLE Angle = 0.0);
	void InsertCPntExBy2DStep(CSmtCutPath* CPath5x, DOUBLE Step);
	CGeoCurve* SnapNearestPoint( CPtrList& CurveList, RFRAME& LocFrame, PNT3D In_Pnt, DOUBLE &In_t, PNT3D Ot_Pnt, VEC3D Ot_Tan);
	void CreateSinWaveFor3DPath(CSmtCutPath * CutPath5x, int MdfyDir, DOUBLE MdfyHeit, int SectCnt);
	void CreateTriWaveFor3DPath(CSmtCutPath * CutPath5x, int MdfyDir, DOUBLE MdfyHeit, int SectCnt);
	void CreateTrapWaveFor3DPath(CSmtCutPath * CutPath5x, int MdfyDir, DOUBLE MdfyHeit, int SectCnt);
	void CreateTrochoidalWaveFor3DPath(CSmtCutPath * CutPath5x, int MdfyDir, DOUBLE MdfyHeit, int SectCnt);
	double NormalizeLenByCutTime(CSmtCutPath* CutPath5x, int StartIndex, int EndIndex, BOOL ReadFromFile, BOOL Trochoidal = FALSE);
	double NormalizeLenByDefaultCtrlParam(CSmtCutPath* CutPath5x, BOOL Trochoidal = FALSE);
	BOOL MatchSpeedForMdfyGrnd(CSmtCPathLib &AllPath, CPathCombine &PComb);
	// 使用刀触点速度，用于三轴路径
	void MatchSpeedByToolPosFor3DPath(CPathGroup &NewPath, CSmartGraphic& Graph);
	BOOL MatchSpeedByToolPosFor3D(CPathGroup &NewPath, CSmartGraphic& Graph, int RCompSide = 2);
	BOOL MatchSpeedByToolPosFor3D(CSmtCPathLib &AllPath, CPathCombine &PComb);
	// 通过相邻路径匹配进退刀路径速度
	BOOL MatchLeadConnectByAdjacentPath(CPathCombine & PComb);
    // 三轴路径转换成五轴路径
    BOOL TransAllPathFrom3AxisTo5Axis( CSmartGraphic &Graph  ,  
                                       CSmtCheckMdl& DriveMdl,
                                       CSmtCPathLib &AllPath , 
                                       CPathGroup &NewPath  ); 
    // 转换加工模型
	void GetAll5AxisMdl( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl ) ;
	// 生成五轴路径
	int CreateAll5AxisPath( int nAtCore, int nCoreNum, CSmtCheckMdl &DriveMdl, CSmtCPathLib &OrgPath, 
							 LPVOID *AllPath, JDNC_PRGDEF &ProgDef, CNc5DAxisCtrl &axisCtrl, double dCur) ;
	// 五轴的路径和刀轴
	void Create5AxisPath( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &axisCtrl, CSmtCutPath *&pPath ) ;
private :
	// 修改AllPath类型
	void GetAll5AxisPath( CSmtCPathLib &AllPath ) ;
	// 生成五轴路径点
	void Create5AxisPnt( CSmtCutPointEx& Start, CSmtCutPointEx& End,
						 CNc5DAxisCtrl &axisCtrl, CSmtCutPointEx &tmpPnt ) ;
	// 判断插入的路径点是否合理，即满足刀轴偏差且为平滑过渡点
	BOOL IsCenterPointValid( CSmtCutPointEx& Start, CSmtCutPointEx& End,
		CSmtCutPointEx& Center, CNc5DAxisCtrl &AxisCtrl );
	// 刀轴方向投影
	void Define5AxisPnt( CSmtCheckMdl &DriveMdl, CSmtCutPointEx &CutPnt, JDNC_TOL &cTol ) ;
	// 加密有效路径
	void InsertCutPntEx( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &axisCtrl, 
						 CSmtCutPath *pPath, JDNC_TOL &cTol ) ;
	// 精确迭代有效和无效点之间的分界点
	void FindDividePntEx( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &axisCtrl, 
						  CSmtCutPath *pPath, JDNC_TOL &cTol ) ;
	// 五轴路径连接
	void LinkAll5AxisPath( CSmtCheckMdl &DriveMdl, CNc5DAxisCtrl &axisCtrl, CSmtCPathLib &AllPath ) ;
protected:
	// 判断刀轴模型是否合理
	BOOL IsAxisCtrlValid(CSmartGraphic &Graph);

public:
// 多线程计算相关函数
	BOOL IsNeedMultiCopy(CSmtCheckMdl& DriveMdl);
public:
	// 添加进退刀路径 zy 2012.12.10	
	int InsertLeadOutPath( CPathCombine&   TComb  ,  /*路径组合*/
		                   CSmartLoop&    Contour ,  /*轮廓边界*/
		                   JDNC_LEAD&      Leadin ,  /*进刀参数*/
						   JDNC_LEAD&      Leadout,  /*退刀参数*/
		                  DOUBLE     ZShift[2] ) ;  /*起末点抬高  */  

	int CreatePocketLeadPath(CPathCombine&   tmpTComb  ,
		                      JDNC_LEAD&     LeadOut ,  /*退刀参数*/                   	
	                     		PNT2D         Point ,		/*端点*/
		                        VEC2D         Tangent,		/*切失*/
		                        VEC2D          Normal,		/*法失*/
		                        int            IsOut ,      /*切出*/
		                        CSmartLoop&  AllCont,	/*干涉检查*/
		                        DOUBLE     ZShift[2] ) ;  /*起末点抬高  */  
	
	//guomin 使用多线程添加下刀路径和相对安全高度路径
	void AddPlungeAndRelativeHbyMultithread( CSmtCheckMdl *DriveMdl, CPathGroup *NewPath, BOOL bConnect = FALSE, double *dZ = NULL, INT_PTR nCount = 0 ) ;
	BOOL AddRelativeHSubProc(int nAtCore, int nCoreNum, CPathGroup &TmpPath, LPVOID* AllPath, CSmtCheckMdl &DriveMdl,double dTop,BOOL bConnect,double *dZ, INT_PTR nCount ) ;
	void AddRelativeH( CSmtCheckMdl *DriveMdl, CPathCombine *pComb,CPathEntity *pHead,CPathEntity *pNext,double dTop,BOOL bConnect,double *dZ, INT_PTR nCount) ;
	//guomin 
	void PointDistributeFor3DPath(CPathGroup &NewPath,BOOL bMdfyGrnd,BOOL bMaxDist) ;

////////////////////////////基于参数域匹配的保留曲线高度////////////////////////////////////////////////////
public :

	BOOL RedepthPathByParamT(	CPathGroup& NewPath,	/*雕刻路径*/ 
								CSmartGraphic& Graph,	/*图形函数*/   
								DOUBLE& dTop )	;		/*最大高度*/
	BOOL RedepthCPathCombineByT (CPtrList & CurveList, CPathCombine * PComb, RFRAME & LocFrame) ;

	// 计算高度
	DOUBLE SnapNearestPointByT(	CPtrList &CurveList,	//<I> CRedepthCurve 
								PNT3D point		,		//<I> point
								double	&i_t		,	//<I> 输入初始参数t
								RFRAME &LocFrame )	;	//<I> 相对坐标系
	/*转成刮铣路径*/
public:
	BOOL ConvertShavePath(CPathGroup& NewPath, JDNC_TOL & Tol);
	BOOL ConvertShavePath(CPathCombine* PComb, JDNC_TOL & Tol);
	BOOL TreatNonCutShavePath(CPathCombine* PComb);
	void SetShavePathEntDir(CPathEntity* Ent, VEC3D Dir);
};

//guomin 添加相对高度下刀多线程相关内容
//多线程传递参数
typedef struct AddRelativeHThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	CPathGroup *TmpPath ;
	LPVOID *	 AllPath ;
	CSmtCheckMdl *DriveMdl;
	double		dTop ;
	BOOL		bConnect ;
	double *dZ;
	INT_PTR nCount ;
	CSmartPathGen *pPathGen;
}ADRH_DATA;

// 多线程运算线程函数
//////////////////////////////////////////////////////////////////////////
// 是否需要为多线程计算拷贝干涉检查模型
BOOL MathCAM_IsNeedMultiCopy(CSmtCheckMdl& DriveMdl, int nThreadNum);
// 是否支持双核多线程计算
BOOL MathCAM_IsSupportMultiThread();
// 线程函数调用主体
BOOL MathCAM_ThreadMainFunc(AFX_THREADPROC func, LPVOID lpParam[], int nThreadNum);


// 清除计算数据
DLL_EXPORT  void MathCAM_ClearLoopList(  CPtrList& AllLoop ) ;
DLL_EXPORT  void MathCAM_ClearCurveList( CPtrList& AllCurve) ;
DLL_EXPORT  void MathCAM_ClearGeoCurList( CPtrList& AllCurve);
DLL_EXPORT  void MathCAM_ClearPointList( CPtrList& AllPoint) ;
DLL_EXPORT  void MathCAM_ClearFacetList( CPtrList& AllFacet) ;
// 轮廓自交检查
void MathCAm_ConvertSFinishToAutoFinish ( JDNC_METHOD& OldMethod, JDNC_METHOD& NewMethod ) ;
DLL_EXPORT	CSmartPathGen* MathCAM_CreatePathGen( CJDParameter& Param ) ;
DLL_EXPORT void MathCAM_InitRegenHeader( REGEN_HEADER& cHeader ) ;
DLL_EXPORT BOOL MathCAM_ValidRegenHeader( REGEN_HEADER& cHeader );
DLL_EXPORT CSmartLoop* MathCAM_FindOutMostLoop( CSmartLoop* ContHead ) ;
DLL_EXPORT BOOL MathCAM_IsOutMostLoop( CSmartLoop* ContHead, CSmartLoop* pFind,BOOL ValidIsl = TRUE ) ;
DLL_EXPORT void MathCAM_ValidOutMostLoop( CSmartLoop* LpHead ) ;
DLL_EXPORT void MathCAM_AddPathGroup( CSmtCPathLib& AllPath, CPathGroup& OldPath , JDNC_TOL& Tol, BOOL bReDepth ) ;
DLL_EXPORT void MathCAM_ClearErrorZLayer() ;
DLL_EXPORT int MathCAM_AddErrorLayer( DOUBLE ZAt, DOUBLE LostLen );
DLL_EXPORT int MathCAM_GenZDirLeadPathEx(CSmtCheckMdl&   CheckMdl, 
                              JDNC_TOL&       Tol ,
                              JDNC_CONNECT3D& ConnectDef, 
						      TFLOAT  StartAt[3] , 
                              TFLOAT  TanDir[3]  , 
                              CSmtCutPath& CutPath );
DLL_EXPORT int MathCAM_GenHeightLeadPathEx(CSmtCheckMdl&   CheckMdl, 
                              JDNC_TOL&       Tol ,
                              JDNC_CONNECT3D& ConnectDef, 
						      TFLOAT  StartAt[3] , 
                              TFLOAT  TanDir[3]  , 
                              CSmtCutPath& CutPath );

DLL_EXPORT int MathCAM_GenNormalLeadPathEx(CSmtCheckMdl&   CheckMdl, 
                                JDNC_TOL&       Tol ,
                                JDNC_CONNECT3D& ConnectDef, 
						        TFLOAT  StartAt[3] , 
                                TFLOAT  TanDir[3]  , 
                                CSmtCutPath& CutPath );

//外部调用，统计等高路径丢环的层
DLL_EXPORT int  MathCAM_GetErrorZLayer( DOUBLE ErrorZAt[10],DOUBLE LostLen[10] ) ; 
// 注册网格模型的函数
DLL_EXPORT int  MathCAM_RegisterMeshMdlFunc( CSmtMeshMdl& MeshMdl ) ; 
// 注册网格模型的调试对话框函数
DLL_EXPORT void MathCAM_ResisterDebugEditor( void (*EditMeshMdlDebugInfo)( CSmtMeshMdl& Mdl ) ) ;

DLL_EXPORT void	MathCAM_SetCurrToolGroup(CJDToolGroup* pCurrToolGroup);

DLL_EXPORT BOOL MathCAM_IsPntOnFacet ( PNT3D pt[4], int n, PNT3D p );
DLL_EXPORT BOOL MathCAM_CalcDistToFacet ( PNT3D Center, PNT3D p[4], int n, DOUBLE /*XSize*/[2], DOUBLE Radius[2] );


CJDToolGroup* MathCAM_GetCurrToolGroup() ;
void MathCAM_AddPComb(CPathCombine* pPComb);
BOOL MathCAM_HasSameValue(double dValue, int nAxis, int nShowType);
////////////////////////////////////////
// 在固定的位置生成下刀路径
int  MathCAM_AddPlungePathAtPoint( CSmtCPathLib& AllPath,  /*下刀路径组  */
								   JDNC_PLUNGE& Plunge  ,  /*下刀方式    */
								   PNT3D   AtPoint      ,  /*刀具位置    */
								   CSmartCurve* pCurve   , /*下刀曲线    */
								   CSmartLoop* ContHead ); /*边界轮廓    */

void MathCAM_TrimBox3DByContours( CSmartLoop* ContHead, PNT3D Box3D[2] );
void MathCAM_TrimBox3DByContours( CSmartLoop* ContHead, float Box3D[2][3]);


//加入机床检查模型
DLL_EXPORT BOOL MATHCAM_GenVirtualNcMdl(CSmtVirtualNcMdl & VNcMdl);
typedef BOOL (*MATHCAM_GETVIRTUALNCMDL)(CSmtVirtualNcMdl & VNcMdl);
DLL_EXPORT void MATHCAM_SETFP_GETVIRTUALNCMDL(MATHCAM_GETVIRTUALNCMDL GetVNcMdl);

void Math5Ax_ProjectConncetPath5D( CSmtCheckMdl& DriveMdl,
								  CSmtCutPath& CutPath  ,
								  JDNC_TOL& Tol         ,
								  DOUBLE RapidHeight    ,
								  CNc5DAxisCtrl& AxisCtrl) ;
// 使用Quick5Ax对多轴路径连刀
class CNc5DAxisCtrl ;
DLL_EXPORT int Math5Ax_QuickConnectPath5Ax( CSmtCheckMdl& DriveMdl ,
                                            CSmtCPathLib& AllPath  ,
                                            JDNC_TOL&     Tol      ,
                                            DOUBLE RapidHeight     ,
                                            CNc5DAxisCtrl& AxisCtrl,
											DOUBLE SafeHeight = 0. ); // 安全高度，用于确定路径起末点定位路径长度 qqs 2014.02.27
// 进度条前进n次
void MathCAM_MovePrgStep(JDNC_PRGDEF &PrgDef, int n = 1);

//////////////////////////////////////////////////////////////////////////
// 计算路径时报告错误信息
DLL_EXPORT void MathCAM_ReportErrorGenPath(int nResID, int nType = MB_ICONSTOP);
DLL_EXPORT void MathCAM_ReportErrorGenPath(CString strErr, int nType = MB_ICONSTOP);
// 将SmartCurve转为CPathEntity,可以为多个
DLL_EXPORT CPathEntity * MathCam_TransfSmtCurveToPathEntity( CSmartCurve &Curve, double dZ ) ;

/////////////////////////////////////////////////
// 读取控制参数，用于磨削调整
#pragma pack(push)
#pragma pack (2)
////////////////////////////////////////////////
//文件头结构

#define TIME_IDENTIFY_LENG			20
#define TIME_EN3D_VERSION_LENG      200
#define TIME_PNTNUM_LENG			60
#define TIME_AllTIME_LENG			100
#define TIME_FEEDCTL_LENG			200
#define TIME_FEEDCTL_NOTES_LNEG     100
#define TIME_VERSION_1001			1

struct TIME_FileHead
{
#define TIME_IDENTIFY             "%"
	char  szIdentify[TIME_IDENTIFY_LENG];	//文本标识

	char  en3dVersion[TIME_EN3D_VERSION_LENG];  //en3d版本标识
	long  lVersion	;						//文件版本
	long  lTimehDataOffset;					//时间偏移量
	char  szTimePntNum[TIME_PNTNUM_LENG];	//点数
	char  szAllTime[TIME_AllTIME_LENG]	;	//总时间
	char  szFeedCtrl[TIME_FEEDCTL_LENG]	;	//速度控制模式
	char  szNotes[TIME_FEEDCTL_NOTES_LNEG];			//注释
};

struct  TIME_FileBody
{
	int		nIndex ;
	double	dTimeData ;
};
#pragma pack ()

CSmtCutPath* ReadTimeFromFile(int StartIndex, int EndIndex);
CSmtCutPath* ReadTimeFromFileByVersion1001( CFile &File, TIME_FileHead &FileHead ) ;

#endif // __SMART_PATHGEN_H__