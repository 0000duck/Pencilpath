#ifndef __SMART_PATHGEN2D_H__
#define __SMART_PATHGEN2D_H__

// 计算平面雕刻加工路径

class CSmtVoronoiMap ;

#include "SmtPathGen.H"
///////////////////////////
// 路径发生器
class DLL_EXPORT CSmartDrillGen : public CSmartPathGen
{ /*钻孔路径*/
public :
	CSmartDrillGen()          ; 
	virtual ~CSmartDrillGen() ; 
	// 是否支持计算进度
	virtual BOOL IsSupportProgress() { return TRUE ; }
public :
	JDNC_DRILL  m_cParam ;
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup , /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	void		 ConvertDrillPath ( CPathCombine *PComb ) ;
	CPathEntity *CreateQuickLine( CPathCombine *PComb ) ;
	void		 MatchingDrillType( CPathPoint3D *DrillPnt ) ;
	CPathPLine3D*CreateQuickFromDrill( CPathPoint3D *pHead, CPathPoint3D *pNext, double dTop ) ;
	CPathEntity *ConvertDrillPathToLine( CPathPoint3D *PathPnt ) ;
	CPathEntity *TransfCutPathToPathLine( CSmtCutPath *pPath ) ;

	BOOL		ConvertDrillPathToLine( CPathPoint3D *PathPnt, CSmtCPathLib &AllPath ) ;
};

class DLL_EXPORT CSmartBoreGen : public CSmartPathGen
{ /*扩孔路径*/
public :
	CSmartBoreGen()          ; 
	virtual ~CSmartBoreGen() ; 
	// 是否支持计算进度
	virtual BOOL IsSupportProgress() { return TRUE ; }
public :
	JDNC_BORE  m_cParam ;
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup , /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
};

class DLL_EXPORT CSmartCurveGen: public CSmartPathGen
{ /*轮廓切割*/
public :
	CSmartCurveGen()          ; 
	virtual ~CSmartCurveGen() ; 
public :
	JDNC_CURVE	m_cParam	; 
	CPtrList	m_cListCur	;	// 保留曲线高度、匹配曲线速度所采集的原始曲线
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup     ,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
public :
	// 提取曲线切割路径
	int CurveOneLayer(    CPathCombine& PComb   ,  /*路径集合*/
						  CPtrList &AllCurve	,  /*所有曲线*/
						  CPtrList &AllComb		,  /*所有路径*/	
						  CSmartCurve *pCvHead	,  /*偏移曲线*/
						  double		dComp	,  /*偏移距离*/
			              CSmartCurve&  Curve   ,  /*切割曲线*/
						  JDNC_CURVE& CurveCut  ,  /*切割参数*/
						  RFRAME &	LocFrame	,  /*局部坐标系*/
                          DOUBLE  Depth         ,  /*深度位置*/  
			              DOUBLE DepthInc       ,  /*雕刻深度*/  
						  BOOL	bFst			,  /*第一根曲线*/
						  int  Layer            ,  /*层号    */
                          BOOL  RevDir			,  /*是否反向*/
						  int  SideLayer = 0)   ;  /*侧向层号*/	

	int CurveOneLayerByContourPlunge( CPathCombine& PComb   ,  /*路径集合*/
									  CPtrList &AllCurve	,  /*所有曲线*/
									  CPtrList &AllComb		,  /*所有路径*/	
									  CSmartCurve *pCvHead	,  /*偏移曲线*/
									  double		dComp	,  /*偏移距离*/
									  CSmartCurve&  Curve   ,  /*切割曲线*/
									  JDNC_CURVE& CurveCut  ,  /*切割参数*/
									  RFRAME &	LocFrame	,  /*局部坐标系*/
									  DOUBLE  Depth         ,  /*深度位置*/  
									  DOUBLE DepthInc       ,  /*雕刻深度*/  
									  BOOL	bFst			,  /*第一根曲线*/
									  int  Layer            ,  /*层号    */
									  BOOL  RevDir			,  /*是否反向*/
									  int  SideLayer = 0)   ;  /*侧向层号*/	

	BOOL ConnectLeadInOutPosForCurve(	CPathCombine &PComb	,	// <I> 输入路径
										CPtrList &AllCurve	,	// <I> 原始曲线
										CPtrList &AllComb	,	// <I> 所有路径
										CSmartCurve *pCvHead	,	// <I> 偏移曲线
										double		dComp	,	// <I> 偏移距离
										int		nRCompSide	,	// <I> 补偿方向 
										JDNC_LEADOUT &cLead	, 	// <I> 退刀参数
										BOOL		bCnt[2]		) ;	// <O> 成功返回TRUE

	// 定义空间路路径曲线
    int DefineCurve3DPath( CPathCombine& SelComb,  /*切割路径*/
		                   JDNC_CURVE&  CurveCut,  /*路径    */
				           BOOL   Layer        );  /*层号    */ 
    // 计算半径补偿的号码
    int GetCurveRCompMask() ;
	// 延伸曲线和路径
	void ExtendCurveAndPComb(	CPathCombine &PComb	,	/*路径*/ 
								CSmartCurve &Curve	,	/*曲线*/
								JDNC_CURVE&  CurveCut	/*切割参数*/
								) ;
	// 为曲线和路径添加进退刀
	void AddLeadInOutForCurveAndPComb( CPathCombine &PComb,
										CSmartCurve &Curve,
										CSmartCurve *LeadIn,
										CSmartCurve *LeadOut ) ;
	void AddLeadInOutForCurveAndPComb( CPathCombine &PComb,
									   CSmartCurve &Curve,
									   CSmartCurve *LeadIn,
									   CSmartCurve *LeadOut,
									   BOOL			Keep[2],
									   RFRAME		&LocFrame) ;
	// 计算边界修边路径
	BOOL CreateBoundCurve( CPathGroup& NewGroup,  /*雕刻路径*/ 
                             CSmartGraphic& Graph); /*图形函数*/ 
	// 修边切割单条曲线
	int BoundCurveOneLayer(CPathCombine& PComb   ,  /*路径集合*/
                           CSmartCurve&  Curve   ,  /*切割曲线*/
						   JDNC_CURVE& CurveCut  ,  /*切割参数*/
						   RFRAME &LocFrame		 ,	/*局部坐标系*/
                           DOUBLE  Depth         ,  /*深度位置*/  
			               DOUBLE DepthInc       ,  /*雕刻深度*/  
						   int  Layer           );  /*层号    */
	// 计算返回值为偏移后的加工曲线，封闭加工曲线偏移后仍会得到封闭曲线 qqs 2013.12.11
	CSmartCurve* GenerateOffsetCurve(CSmartCurve* pCurve,  // <I> 待偏移曲线 
	                                 double dRComp);       // <I> 偏移距离
	//重设路径高度
	void RedepthPathCombine( CPathCombine* pTComb, CSmartGraphic& Graph, DOUBLE dTop) ;
	
};

class DLL_EXPORT CSmartContourGen: public CSmartPathGen
{ /*轮廓切割*/
public :
	CSmartContourGen()          ; 
	virtual ~CSmartContourGen() ;
public :
	JDNC_LOOP  m_cParam ;
	CSmartLoop* m_cLastLoop; // 存储上一个环，用于和下个环比较判断是否生成连刀路径 qqs 2013.03.26
	CPtrList	m_cListCur ; // 保留曲线高度或者读取曲线速度
public :
	// 计算轮廓加工路径
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
public :
	// 提取轮廓路径
	int LoopOneLayer( CPathCombine& PComb  ,  /*路径集合*/
					  CSmartLoop *AllCont  ,  /*所有区域*/
		              CSmartLoop&   Contour,  /*切割轮廓*/
					  DOUBLE Depth    ,
			          DOUBLE DepthInc ,      /*雕刻深度*/ 
					  JDNC_LAYER &Layer ,		/*分层*/
					  RFRAME &	LocFrame,	/*局部坐标系*/
					  BOOL bFst		   ,	/*是否为第一个环*/
					  BOOL bLast,			/*是否为最后一个环*/
					  int nRev );          /*圆弧延伸标记*/
	int LoopOneLayerByContourPlunge( CPathCombine& PComb  ,  /*路径集合*/
								   CSmartLoop *AllCont  ,  /*所有区域*/
								   CSmartLoop&   Contour,  /*切割轮廓*/
								   DOUBLE Depth    ,
								   DOUBLE DepthInc ,      /*雕刻深度*/ 
								   JDNC_LAYER &Layer ,		/*分层*/
								   RFRAME &	LocFrame,	/*局部坐标系*/
								   BOOL bFst		   ,	/*是否为第一个环*/
								   BOOL bLast,			/*是否为最后一个环*/
								   int nRev );          /*圆弧延伸标记*/
	// 复制轮廓路径
    int CopyOneContour( CPathCombine& PComb,    /*数据*/
						CSmartLoop *AllCont,	/*所有区域*/
				        CSmartLoop& Contour,    /*轮廓*/
						JDNC_LAYER&  Layer ,    /*参数*/
						RFRAME &	LocFrame,	/*局部坐标系*/
						JDNC_PLUNGE& Plunge,  /*下刀*/
						BOOL bFst		   ,	/*是否为第一个环*/
						int nRev);           /*圆弧延伸标记*/
	// 生成沿轮廓下刀分层不抬刀路径
	int CopyOneCurve(CPathCombine	& PComb,
		             CSmartCurve	*&Curve,
	                 JDNC_LAYER		& Layer,
		             JDNC_PLUNGE	& Plunge);
	// 判断偏移值是否相同
	BOOL IsSameOffset() ;
protected :
	CSmartLoop* CreateLoopBound( CSmartLoop& Contour, 
							     DOUBLE TotalDepth  ,
								 DOUBLE Depth,
								 int* nRev) ; /*输出参数,标记半径补偿方向, add in 2009.8.12*/
	// 计算一个环的切割路径
	int ContourOneLoop( CPathCombine& PComb,    /*加工数据*/
						CSmartLoop *AllCont,	/*区域数组*/
				        CSmartLoop& Loop   ,    /*轮廓曲线*/
						DOUBLE  Depth      ,    /*深度位置*/
						DOUBLE  DepthInc,      /*吃刀深度*/
						JDNC_LAYER &Layer ,		/*分层*/
						RFRAME &	LocFrame,	/*局部坐标系*/
						BOOL bFst		   ,	/*是否为第一个环*/
						BOOL bLast,				/*是否为最后一层*/
						int nRev ,              /*圆弧延伸标记*/
						int nLp  );            /*是否为该加工轮廓第一个环 qqs 2013.03.26*/   
	int ContourOneLoopByContourPlunge( CPathCombine& PComb,    /*加工数据*/
										CSmartLoop *AllCont,	/*区域数组*/
										CSmartLoop& Loop   ,    /*轮廓曲线*/
										DOUBLE  Depth      ,    /*深度位置*/
										DOUBLE  DepthInc,      /*吃刀深度*/
										JDNC_LAYER &Layer ,		/*分层*/
										RFRAME &	LocFrame,	/*局部坐标系*/
										BOOL bFst		   ,	/*是否为第一个环*/
										BOOL bLast,				/*是否为最后一层*/
										int nRev ,              /*圆弧延伸标记*/
										int nLp  );            /*是否为该加工轮廓第一个环 qqs 2013.03.26*/         
	// 合并路径
	int AppendCombineEx( CPathCombine& Base, 
		                 CPathCombine& To  ,
					     CSmartLoop*   Contour ) ;
    // 计算半径补偿的号码
    int GetContourRCompMask() ;
	
	BOOL BoundContour( CPathGroup&   NewPath,  /*雕刻路径*/ 
					   CSmartGraphic& Graph);  /*图形函数*/  
	// 计算补偿值
	BOOL GetBoundOffset(	DOUBLE TotalDepth,		// <I> 总深度
							DOUBLE dDepth,			// <I> 当前深度
							DOUBLE dRComp[2] ) ;	// <I> 偏移补偿 
	// 添加路径的连刀
	int AddConnectForNewPath(	CPathGroup &NewPath,		// 路径
								CSmartGraphic &Graph ) ;	// 图形

	// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
	void AddLeadInOutForPComb( CPathCombine &PComb, CSmartCurve *LeadIn, CSmartCurve *LeadOut ) ;
	// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
	void AddLeadInOutAndOverlapForPComb( CPathCombine &PComb,
										 CSmartCurve *Overlap,
										 CSmartCurve *LeadIn, 
										 CSmartCurve *LeadOut,
										 double Depth, 
										 CPtrList &CurveList,
										 BOOL bKeep[2], 
										 RFRAME &LocFrame) ;

 };

class DLL_EXPORT CSmartPocketGen: public CSmartPathGen
{ /*区域加工*/
public :
	CSmartPocketGen()          ; 
	virtual ~CSmartPocketGen() ; 
public : 
	JDNC_POCKET  m_cParam ;
    DOUBLE       m_dBoundTol  ;
    DOUBLE       m_dFeedRatio ;
public : // 计算区域加工路径
	virtual BOOL GeneratePathEx( CPathGroup& NewPath     ,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
	//增加一级函数，方便多轴区域加工调用。（SBG, 2009.10.28 ）
	BOOL		CreatPath ( CSmartLoop  *BndCont,
							CPathGroup	&NewPath,
							CSmartGraphic& Graph ) ;

public :
    // 计算一层图形的粗加工路径
	int PocketOneLayer( CPathCombine&   NewPath ,  /*路径组  */
				        CSmartLoop&     Contour ,  /*轮廓    */
					  	JDNC_POCKET&  PocketCut ,  /*区域参数*/
			            DOUBLE         DepthInc , /*深度进给*/
	                    CSmartLoop   *PartBnd=NULL , /*模型边界,非切区域      */
                        CSmartLoop   *BlankBnd=NULL);/*毛坯边界,搜索下刀引入点*/
	int PocketOneContour( CPathGroup&  PathGroup,
                          CPathCombine&   NewPath ,  /*路径组  */
				          CSmartLoop&     Contour ,  /*轮廓    */
					  	  JDNC_POCKET&  PocketCut ,  /*区域参数*/
			              DOUBLE         DepthInc , /*深度进给*/
	                      CSmartLoop   *PartBnd=NULL , /*模型边界,非切区域      */
                          CSmartLoop   *BlankBnd=NULL);/*毛坯边界,搜索下刀引入点*/
    // 计算斜侧边开槽 区域雕刻路径
    int PocketOneLayerEx( CPathCombine& TComb   ,    /*路径组  */
					      CSmartLoop&   Contour ,    /*轮廓    */
						  JDNC_POCKET&  PocketCut ,  /*区域参数*/
			              DOUBLE         DepthInc ); /*深度进给*/
    // 计算环切清角路径
	int  CreateFCornerPath( CPathCombine& TComb , /*加工路径*/
					        CSmartLoop& Contour , /*加工轮廓*/
	 						JDNC_FOLLOW& Follow); /*走刀方式*/

    // 计算光滑的环切清角路径
	int  CreateFCornerPathSmooth( CPathCombine& TComb , /*加工路径*/
					        CSmartLoop& Contour , /*加工轮廓*/
	 						JDNC_FOLLOW& Follow); /*走刀方式*/
    // 最后一层修边
    int  FinishPocketBound( CPathGroup& NewPath, CSmartLoop* AllCont, RFRAME &LocFrame ) ;

private:
	// 生成二维刀具扫掠环
	CSmartLoop * CreateToolLoop( PNT3D start, PNT3D end, 
                                 double dRComp , CSmartLoop *pLoop) ;
	// 刀具环是否与加工域相交
	BOOL IsContourIntTool( CSmartLoop* pContour, CSmartLoop* pTLoop ) ;
	// 添加连接上下两层的路径
	BOOL AddCnctPathBtwLayers(CPathCombine& tmpComb, CPathCombine* pTComb, 
		                      double dDepth, double dDepthInc, 
							  CSmartLoop* pPreLoop, PNT3D start, PNT3D end ) ;
};
class DLL_EXPORT CSmartBoundGen: public CSmartPathGen
{ /*区域修边*/
public :
	CSmartBoundGen()          ; 
	virtual ~CSmartBoundGen() ; 
public : 
	JDNC_BOUND  m_cParam ;
	BOOL		m_bLoopCut ;	// 是否为轮廓切割的侧向分层
	CPtrList	m_cListCur ;	// 区域修边保留曲线高度
public:
	virtual BOOL GeneratePathEx( CPathGroup& NewPath,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
	//增加一级函数，方便多轴区域加工调用。（SBG, 2009.10.28 ）
	BOOL		CreatPath ( CSmartLoop *& AllCont,
							CPathGroup &NewPath,
							CSmartGraphic& Graph) ;
public :
	// 计算一层修边路径
	int BoundOneLayer(  CPathCombine& PComb   ,  /*路径集合*/
		                CSmartLoop&    Contour,  /*边界轮廓*/
  						JDNC_BOUND&  BoundCut,   /*轮廓参数*/
						RFRAME &	LocFrame  ,	 /*局部坐标系*/
                        DOUBLE  Depth  ,       /*雕刻位置*/   
			            DOUBLE DepthInc,       /*雕刻深度*/  
						int nR );
	int BoundOneLayerByContourPlunge( CPathCombine& PComb   ,  /*路径集合*/
									  CSmartLoop&    Contour,  /*边界轮廓*/
									  JDNC_BOUND&  BoundCut,   /*轮廓参数*/
									  RFRAME &	LocFrame  ,	 /*局部坐标系*/
									  DOUBLE  Depth  ,       /*雕刻位置*/   
									  DOUBLE DepthInc,       /*雕刻深度*/ 
									  int nR );
	// 计算一层修边路径
	int BoundAllCutPath(  CPathCombine& PComb   ,  /*路径集合*/
		                  CSmartLoop&    Contour,  /*边界轮廓*/
						  RFRAME &LocFrame		,	/*局部坐标系*/	
			              DOUBLE Depth );  /*雕刻深度*/  
	// 计算一层修边路径
	int BoundOneLayerEx(  CPathCombine& PComb   ,  /*路径集合*/
                          CSmartLoop&    Contour,  /*边界轮廓*/
                          CSmtLoopArr&   AllLoop,  /*所有边界*/
 					      JDNC_BOUND&   BoundCut,  /*轮廓参数*/
						  RFRAME &		LocFrame,	/*局部坐标系*/
						  BOOL			bSmooth ,  /*是否光滑*/
                          DOUBLE        Depth   ,  /*深度位置*/
			              DOUBLE		DepthInc,  /*雕刻深度*/  
						  DOUBLE	   dSideInc);  /*偏移量  */ 
	int BoundOneLayerExByContourPlunge( CPathCombine& PComb   ,  /*路径集合*/
										CSmartLoop&    Contour,  /*边界轮廓*/
										CSmtLoopArr&   AllLoop,  /*所有边界*/
										JDNC_BOUND&   BoundCut,  /*轮廓参数*/
										RFRAME &		LocFrame,	/*局部坐标系*/
										BOOL			bSmooth ,  /*是否光滑*/
										DOUBLE        Depth   ,  /*深度位置*/
										DOUBLE		DepthInc,  /*雕刻深度*/  
										DOUBLE	   dSideInc);  /*偏移量  */ 
public :  
    // 计算半径补偿的号码
    int GetBoundRCompMask() ;
    // 计算螺旋路径的进退刀 
    int AddLeadSectAtSpiralEnd( JDNC_LEAD& LeadInDef , 
								JDNC_LEAD& LeadOutDef, 
								CSmartCurve& Curve   , 
								CSmartLoop *BndCont[2],
                                DOUBLE Depth, 
                                DOUBLE ZShift[2] ) ;
	// 计算螺旋路径的进退刀 
	int	AddLeadPCombAtSpiralEnd( JDNC_LEAD& LeadInDef , 
								 JDNC_LEAD& LeadOutDef, 
								 CPathCombine &PComb,
								 CSmartCurve& Curve   , 
								 CSmartLoop *BndCont[2],
                                 DOUBLE Depth, 
                                 DOUBLE ZShift[2], 
								 BOOL bKeep[2],
								 RFRAME &LocFrame,
								 DOUBLE dDepthInc = 0.) ;
	// 曲线外一点向指定点光滑进刀
	int AddSmoothLeadPathAtPos(PNT2D OutPos,			/*外部指定点*/
		                       CSmartCurve& atCurve,	/*目标曲线*/ 
							   DOUBLE atParam,			/*曲线上的目标进刀位置*/
							   CPathCombine& LeadPath); /*输出的连刀路径*/ 
	// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
	void AddLeadInOutForPComb( CPathCombine &PComb, CSmartCurve *LeadIn, CSmartCurve *LeadOut ) ;
	// 为路径添加进退刀曲线,该函数为轮廓切割、修边中调用
	void AddLeadInOutAndOverlapForPComb( CPathCombine &PComb,
										 CSmartCurve *Overlap,
										 CSmartCurve *LeadIn,
										 CSmartCurve *LeadOut,
										 double Depth,
										 CPtrList &CurveList, 
										 BOOL bKeep[2], 
										 RFRAME &LocFrame) ;
	// 添加光滑的沿轮廓下刀路径
	int AddContourPlungePath(CPathCombine	& PComb,							 
							 CSmartCurve	& Curve,
							 JDNC_PLUNGE	& Plunge,
							 PNT3D			ToPoint,
							 CSmartCurve	& PlungeCrv,
							 CSmartCurve *   pBoundOrg ) ;

	void ConnectAllPath( CPathCombine& TComb, CSmartLoop* AllBnd, DOUBLE MaxDist );
};
class DLL_EXPORT CSmartRemainGen: public CSmartPathGen
{ /*残料补加工*/
public :
	CSmartRemainGen()          ; 
	virtual ~CSmartRemainGen() ; 
public : 
	JDNC_REMAIN  m_cParam ;
public :
	virtual BOOL GeneratePathEx( CPathGroup&	NewPath,	/*雕刻路径*/ 
                                 CSmartGraphic& Graph);		/*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
	//增加一级函数，方便多轴区域加工调用。（SBG, 2009.10.28 ）
	BOOL		CreatPath ( CSmartLoop* AllCont,
							CPathGroup& NewPath ) ;
public :
	// 计算一层残料补加工路径
	int RemainOneLayer(  CPathCombine& PComb     ,/*路径集合*/
		                 CSmartLoop&   Contour   ,/*边界轮廓*/
					     JDNC_REMAIN& RemainCut  ,/*加工参数*/
			             DOUBLE DepthInc      ) ; /*雕刻深度*/  
	 // 计算残料补加工轮廓
     CSmartLoop* GetRemainContour( CSmartLoop& Contour     ,    /*边界轮廓  */
								   JDNC_REMAIN& RemainCut  ,    /*补加工参数*/
						 	       DOUBLE Depth        ,         /*雕刻深度  */
								   DOUBLE DepthInc    );        /*偏移深度  */
};
class DLL_EXPORT CSmartExpendGen: public CSmartPathGen
{// 膨胀清角
public:
	CSmartExpendGen() ;
	virtual ~CSmartExpendGen() ;

public : 
	JDNC_CORNER  m_cParam ;
public:
	// 计算膨胀清角路径
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
} ;

class DLL_EXPORT CSmartCornerGen: public CSmartPathGen
{ /*三维清角*/
public :
	CSmartCornerGen()          ; 
	virtual ~CSmartCornerGen() ; 
public : 
	JDNC_CORNER  m_cParam ;
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewPath ,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
	//增加一级函数，方便多轴区域加工调用。（SBG, 2009.10.28 ）
	BOOL		CreatPath ( CSmartLoop* AllCont,
							CPathGroup& NewPath ) ;
public :
	// 计算一层清角加工路径
	int CornerOneLayer(  CPathCombine& PComb     ,  /*路径集合*/
		                 CSmtVoronoiMap& Voronoi ,  /*边界轮廓*/
					     JDNC_CORNER&  CornerCut ,  /*加工参数*/
			             int nLayer              ,  /*雕刻层号*/  
			             DOUBLE SideAngle       );  /*侧面角度*/  
	// 计算清角路径
    int CreateCornerPath( CPathCombine&  TComb , /* 路径组  */
		                  CSmartLoop&   Contour, /* 轮廓组  */
						  JDNC_CORNER&  Corner , /* 清角参数*/  
						  DOUBLE        Angle ); /* 清角角度*/  
protected :
	// 模糊清角
	BOOL FuzzyCornerPath( CPathGroup& NewPath,  /*路径组  */ 
		                  CSmartLoop* AllCont,  /*轮廓组  */  
						  JDNC_LAYER* Layer,    /*分层数据*/ 
						  DOUBLE      Angle);   /*侧面弧度*/ 
	// 修边清角
	BOOL BoundCornerPath( CPathGroup& NewPath,  /*路径组  */ 
		                  CSmartLoop* AllCont,  /*轮廓组  */  
						  JDNC_LAYER* Layer,    /*分层数据*/ 
						  DOUBLE      Angle);   /*侧面弧度*/ 
	// 补加工清角
	BOOL RemainCornerPath(CPathGroup& NewPath,  /*路径组  */ 
		                  CSmartLoop* AllCont,  /*轮廓组  */  
						  JDNC_LAYER* Layer,    /*分层数据*/ 
						  DOUBLE      Angle);   /*侧面弧度*/ 
	// 计算一层路径
 	BOOL CFuzzyOneLayer( CPathCombine& TComb, 
		                CSmartLoop&   Contour, 
						JDNC_LAYER* pLayer ,
						int  i,
						DOUBLE  Angle ,
						DOUBLE  SideTol ) ;
	// 计算一层路径
    BOOL CBoundOneLayer( CPathCombine& TComb, 
		                CSmtVoronoiMap& Voronoi, 
						JDNC_LAYER* pLayer ,
						int  i,
						DOUBLE  Angle ) ;
	// 计算一层路径
    BOOL CRemainOneLayer(CPathCombine& TComb, 
		                CSmtVoronoiMap& Voronoi, 
						JDNC_LAYER* pLayer ,
						int  i,
						DOUBLE  Angle ) ;
	// 计算所有层路径
    BOOL CRemainAllLayer(CPathCombine& TComb, 
		                 CSmtVoronoiMap& Voronoi, 
						 JDNC_LAYER* pLayer ,
						 DOUBLE  Angle ) ;
};

class DLL_EXPORT CSmartGrayGen: public CSmartPathGen
{ /*图片加工*/
public :
	CSmartGrayGen()          ; 
	virtual ~CSmartGrayGen() ; 
public :
	 JDNC_GRAY   m_cParam ;
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup     ,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
};
class DLL_EXPORT CSmartBReliefGen: public CSmartPathGen
{ /*图片加工*/
public :
	CSmartBReliefGen()          ; 
	virtual ~CSmartBReliefGen() ; 
public :
	 JDNC_BRELIEF  m_cParam ;
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup     ,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
};

class DLL_EXPORT CSmartCombMillGen: public CSmartPathGen
{ /*组合刀切割*/
public :
	CSmartCombMillGen()          ; 
	virtual ~CSmartCombMillGen() ;
public :
	JDNC_COMBMILL  m_cParam ;
public :
	// 计算组合刀切割路径
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
protected :
	// 设置进给参数
	void ReDefFeedParam(JDNC_FEED& cFeedDef,  /*进给参数*/ 
						  int nIndex,
						  BOOL bFirst); 

	// 处理切割顺序
	void DefineCutOrder(JDNC_COMBMILL& cNewApp, 
		                double* dDepth,
						int* nOrgOrder);
	
	// 计算一把刀加工路径
	BOOL GenOneToolPath( CPathGroup& NewGroup,  /*雕刻路径*/ 
						 CSmartLoop*& AllCont,	/*切割轮廓*/
						 int		 nIndex);

	// 起末点添加平面进退刀路径,避免干涉
	int AddLeadSectAtEndEx( JDNC_LEAD& LeadDef,
						  CSmartCurve& Curve, 
						  CSmartLoop*  AllCont);
	int AddLeadPathAtEndEx( JDNC_LEAD& LeadDef, 
                            CPathCombine& TComb, 
                            CSmartLoop&  Contour );

	// 连接两把刀具路径
	void AddConnectPath( CPathGroup& cFrom,	/*起始路径*/ 
						 CPathGroup& cTo,	/*到达路径*/ 
						 int nToolIndex);	/*所到达路径的刀具层号*/

	// 提取轮廓路径
	int LoopOneLayer( CPathCombine& PComb  ,  /*路径集合*/
		              CSmartLoop&   Contour,  /*切割轮廓*/
					  JDNC_COMBMILL& LoopCut ,/*切割参数*/
			          DOUBLE DepthInc);		  /*雕刻深度*/  
	// 复制轮廓路径
    int CopyOneContour( CPathCombine& PComb,    /*数据*/
				        CSmartLoop& Contour,    /*轮廓*/
						JDNC_LAYER&  Layer ,    /*参数*/
						JDNC_PLUNGE& Plunge);   /*下刀*/
	// 判断偏移值是否相同
	BOOL IsSameOffset() ;
protected :
	CSmartLoop* CreateLoopBound( CSmartLoop& Contour, 
							     DOUBLE TotalDepth  ,
								 DOUBLE Depth       ) ; 
	// 计算一个环的切割路径
	int ContourOneLoop( CPathCombine& PComb,    /*加工数据*/
				        CSmartLoop& Loop   ,    /*轮廓曲线*/
						DOUBLE  Depth      ,    /*深度位置*/
						DOUBLE  DepthInc  );    /*吃刀深度*/
	// 合并路径
	int AppendCombineEx( CPathCombine& Base, 
		                 CPathCombine& To  ,
					     CSmartLoop*   Contour ) ;
    // 计算半径补偿的号码
    int GetContourRCompMask() ;

private:
	int	m_nCurToolIndex;
};
class DLL_EXPORT CSmartCurveSlotGen: public CSmartPathGen
{ /*单线摆槽加工*/
public :
	CSmartCurveSlotGen()          ; 
	virtual ~CSmartCurveSlotGen() ; 
public :
	JDNC_CURVESLOT   m_cParam ; 
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup     ,  /*雕刻路径*/ 
                                 CSmartGraphic& Graph); /*图形函数*/   
	virtual BOOL IsSupportProgress() { return TRUE ; }
};

#endif // __SMART_PATHGEN2D_H__