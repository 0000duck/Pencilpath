#include "StdAfx.H"
#include "SurfGeo.H"
#include "SurfNC.H"
#include "SmartNC.H"
#include "SmtPathGen.h"
#include "SmtPathGen3D.H"
#include "SmartPathEx.H"
#include "Nc5DToolAxis.h"
#include "SmtFlowLineGen.H"
#include "mathcam.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

////////////////////////////////////////////////////////////////////////
void	CalcTopAndBot( JDNC_SHAPE &cShape, BOX3D &box, double &dTop, double &dBot, double dZMove, double dTol )
{
	double dTmpTop = box.max[2] + dTol ;
	double dTmpBot = box.min[2] - dTol ;
	if( dZMove > 0 ) dTmpTop += dZMove ;
	else			 dTmpBot += dZMove ;

	if( cShape.m_bShapeFlag & NCDEF_SHAPE_AUTODEPTH )
	{
		dTop = dTmpTop ;
		dBot = dTmpBot ;
	}
	else
	{
		if( dTop > dTmpTop ) dTop = dTmpTop ;
		if( dBot < dTmpBot ) dBot = dTmpBot ;
	}
}
DOUBLE GetProjDownStep( double dBot, double dTop, JDNC_FEED &cFeedDef )  
{ 
	if( cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT )
	{
	    if( cFeedDef.m_cLayerDef.m_nLayerCount < 1 )
		{
			ASSERT( 0 ) ;
			cFeedDef.m_cLayerDef.m_nLayerCount = 1 ;
		}
		return ( dTop - dBot ) / cFeedDef.m_cLayerDef.m_nLayerCount ;
	}
	else if( cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_DEPTH )
		return cFeedDef.m_cLayerDef.m_dSideDInc ; 
	else 
		return ( dTop - dBot ) ;
}
void TrimPathByZValue ( CSmtCutPath *&pPath, TFLOAT fZValue, BOOL bTop, CSmtCPathLib &AllPath )
{
	///////////////////////////////////////////////////
	pPath->DefineBox () ;
	if( pPath->m_fBox[0][2] > fZValue )
	{
		if( bTop )	AllPath.AddToTail ( pPath ) ;
		else		delete pPath ;
		pPath = NULL ;
		return ;
	}
	if( pPath->m_fBox[1][2] < fZValue )
	{
		if( bTop )	delete pPath ;
		else		AllPath.AddToTail ( pPath ) ;
		pPath = NULL ;
		return ;
	}

	CSmtCutPath * pNewPath = new CSmtCutPath() ;
	pNewPath->m_nLineNo = pPath->m_nLineNo  ;
	pNewPath->m_nLayerNo = pPath->m_nLayerNo ;

	FPNT4D intpt, Start ; 
	double  t = 0 ;
	int i = 0 ;
	CSmtCutPoint *pHead = pPath->m_pHead, *pNext = NULL ;
	while( pHead )
	{
		pNext = pHead->next ;
		pPath->RemovePoint ( pHead ) ;
		mathFCpyPnt4D( pHead->m_fPoint, Start ) ;
		if( pHead->m_fPoint[2] > fZValue && bTop )
			pNewPath->AddTail ( pHead ) ;
		else if( pHead->m_fPoint[2] < fZValue && !bTop )
			pNewPath->AddTail ( pHead ) ;
		else 
			delete pHead ;

		if( !pNext ) break ;
		if( Start[2] > fZValue && pNext->m_fPoint[2] < fZValue ||
			Start[2] < fZValue && pNext->m_fPoint[2] > fZValue )
		{ // 添加中间点
			t = pNext->m_fPoint[2] - Start[2] ;
			if( fabs( t ) > 1.0e-6 )
			{
				t = ( fZValue - Start[2] ) / t ;
				for( i = 0 ; i < 4 ; i++ )
				{
					intpt[i] = float( Start[i] + t * ( pNext->m_fPoint[i] - Start[i] ) ) ;
				}
				intpt[2] = fZValue ;
				pNewPath->AddPoint ( intpt ) ;
				if( pNewPath->m_nNumPnt > 1 )
                    AllPath.AddToTail ( pNewPath ) ;
				else	
					delete pNewPath ;
				// 下一条路径
				pNewPath = new CSmtCutPath() ;
				pNewPath->m_nLineNo = pPath->m_nLineNo  ;
				pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			}
		}
		pHead = pNext ;
	}
	if( pNewPath )
	{
		if( pNewPath->m_nNumPnt > 1 )
			AllPath.AddToTail ( pNewPath ) ;
		else
			delete pNewPath ;
	}
	delete pPath ;
	pPath = NULL ;
}


void	TrimAllPathByZValue( CSmtCPathLib &AllPath, double ZValue, BOOL bTop )
{
	float fZValue = float( ZValue ) ;
	CSmtCutPath *pPath = NULL ;
	CSmtCPathLib ResultPath ;
	POSITION pos, atpos = NULL ;
	pos = AllPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		
		// 裁减路径,结果放到ResultPath中
		TrimPathByZValue( pPath, fZValue, bTop, ResultPath ) ;

		AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		if( pPath ) delete pPath ;
	}

	AllPath.ClearAllPath () ;
	AllPath.AppendCPathLib ( ResultPath ) ;
	ResultPath.ClearAllPath () ;
}
// gm 多线程运算线程函数
static UINT MathCAM_CreatePlanePathSubProc(LPVOID lpParam)
{	
	if (lpParam == NULL)
	{
		return 0;
	}
	CRLPPT_DATA& cData = *(CRLPPT_DATA *)lpParam;	
	if( cData.pPathGen == NULL || 
		cData.TmpPath == NULL    || 
		cData.AllPath == NULL    || 
		cData.DriveMdl == NULL   ) 
	{
		return 0 ;
	}
	return cData.pPathGen->CreatePlanePathSubProc(cData.nAtCore, cData.nCoreNum, *cData.TmpPath, 
		cData.AllPath, *cData.DriveMdl, *cData.RoughMdl,cData.dBot,cData.PrgDef);
}
////////////////////////////////////////////////////////////////////////
CSmartRLinearGen::CSmartRLinearGen()
{
	m_nCutMode           = 0    ;
	m_PathNew			 = NULL ;
	m_nCount			 = 0	;
	m_pRoughTool		 = NULL ;
	m_cTmpPrgDef.m_dLimitAt = 0. ;
	m_cTmpPrgDef.m_dStepAt  = 0 ;
	m_cTmpPrgDef.m_dIncStep = 1. ;
	m_cTmpPrgDef.m_pBrkFunc = NULL ;
	m_cTmpPrgDef.m_pPrgFunc = NULL ;
	m_cTmpPrgDef.m_pPosFunc = NULL ;
}
CSmartRLinearGen::~CSmartRLinearGen()
{
	for(INT_PTR i = 0 ; i < m_nCount ; i++ )
		m_PathNew[i].ClearAllPath() ;
	if( m_nCount > 0 )
        delete []m_PathNew ;
	if( m_pRoughTool ) delete m_pRoughTool ;
	m_pRoughTool = NULL ;
}


int CSmartRLinearGen::BuildPlanePath(	CSmtCPathLib& AllPath ,  //曲面路径组
										CSmtCheckMdl& DriveMdl,  //加工面模型
										CSmtCheckMdl& RoughMdl,  //毛坯面模型
										CSmartLoop* AllLoop,
										BOX3D& MaxBox, double dBot,
										JDNC_PRGDEF &ProgDef	, 
										double dCur			)          
{
	CSmartLoop* pLoop ;
	CSmtCutPath* pNewPath  ;
	DOUBLE  yMin, yMax, yStep, y ;
	PNT4D  dStart, dEnd ;
	PNT2D  dPtArr[200] ;
	int nGroup =  0, nLineNo = 0 ;
	UINT nCount  ; 
	for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
	{
		for( int k = 0 ; k < 2  ; k ++ ) 
		{
			if( pLoop->m_dBox[0][k] < MaxBox.min[k] ) 
				MaxBox.min[k] = pLoop->m_dBox[0][k] ;
			if( pLoop->m_dBox[1][k] > MaxBox.max[k] )
				MaxBox.max[k] = pLoop->m_dBox[1][k] ;
		}
	}
	memcpy( dStart, MaxBox.min, sizeof( PNT3D ) ) ;
	memcpy( dEnd  , MaxBox.max, sizeof( PNT3D ) ) ;
	RoughMdl.m_fBottom = float( dBot - 5 ) ;
	dStart[2] = dEnd[2] = DriveMdl.m_fBottom ;
	dStart[3] = dEnd[3] = 0.0 ;
	yMin = MaxBox.min[1], yMax = MaxBox.max[1], yStep = m_cParam.m_dOverStep;

	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	ProgDef.m_dTotalMove = dCur ;

	for( pLoop = AllLoop, nGroup = 0 ; pLoop ; pLoop = pLoop->next, nGroup++ )
	{
		for( y = yMin, nLineNo = 0 ; y < yMax ; y += yStep, nLineNo ++ ) 
		{
			nCount = pLoop->GetYLineIntContour( y , dPtArr, 200 ) ;
			if( nCount == 0 || nCount % 2 ) continue ;
			for( UINT i = 0 ; i < nCount ; i += 2 )
			{
				dStart[0] = dPtArr[i  ][0], dStart[1] = dPtArr[i ][1] ;
				dEnd[0]   = dPtArr[i+1][0], dEnd[1]   = dPtArr[i+1][1] ;
				pNewPath = new CSmtCutPath() ;
				pNewPath->AddPoint ( dStart ) ;
				pNewPath->AddPoint ( dEnd ) ;
				pNewPath->m_nLineNo  = nLineNo ;
				pNewPath->m_nLayerNo = nGroup;
				AllPath.AddToTail( pNewPath )  ;
			}
		}
	}

	//gm 多线程
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE ;	
	INT_PTR nPathNum = AllPath.m_cAllPath.GetCount();
	if (nPathNum >=2 && IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
		RoughMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	// 支持多线程，即拷贝数据不为空
	if( nPathNum >= 2 && DriveMdl.GetMultiCopy() != NULL && RoughMdl.GetMultiCopy() != NULL )
	{
		CRLPPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = ProgDef;//进度条
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathNum);
		int i=0, k=1;
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].DriveMdl = i == 0 ? &DriveMdl : DriveMdl.GetMultiCopy(i-k);
			ThreadData[i].RoughMdl = i == 0 ? &RoughMdl : RoughMdl.GetMultiCopy(i-k);
			if (ThreadData[i].DriveMdl == NULL || ThreadData[i].RoughMdl == NULL)
			{
				i--, k--, nThreadNum--;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].TmpPath = &AllPath;
			ThreadData[i].AllPath = &NewPath[0];
			ThreadData[i].dBot = dBot ;
			ThreadData[i].PrgDef = i==0 ? ProgDef : tmpPrg;
			ThreadData[i].pPathGen = this;			
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		// 调用多线程函数
		MathCAM_ThreadMainFunc(MathCAM_CreatePlanePathSubProc, lpParam, nThreadNum);
	}
	else
	{
		//如果不是多线程处理
		CreatePlanePathSubProc(0,1,AllPath,&NewPath[0],DriveMdl,RoughMdl,dBot,ProgDef) ;
	}

	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		DriveMdl.DeleteMultiCopy() ;
		RoughMdl.DeleteMultiCopy() ;
	}
	AllPath.m_cAllPath.RemoveAll();
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
	{
		return FALSE ;
	}	
	Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);

	RoughMdl.m_fBottom = float( dBot ) ;
	return 1 ;
}

//gm 平行截线线程函数	
BOOL CSmartRLinearGen::CreatePlanePathSubProc(	int nAtCore, int nCoreNum, CSmtCPathLib &TmpPath, LPVOID* AllPath, 
												CSmtCheckMdl &DriveMdl, CSmtCheckMdl &RoughMdl,double dBot,JDNC_PRGDEF &PrgDef)
{
	INT_PTR nNumPath = TmpPath.m_cAllPath.GetCount();
	if (nNumPath < 1)
	{
		return FALSE;
	}
	PrgDef.m_dIncStep = PrgDef.m_dStepAt = 0.0 ;
	PrgDef.m_dLimitAt = 1.0 ;
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumPath ;

	int nIndex = 0;

	POSITION pos = TmpPath.m_cAllPath.GetHeadPosition();
	while( pos )
	{
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
		{
			break ;
		}
		MathCAM_MovePrgStep(PrgDef);
		CSmtCutPath* pBasePath = TmpPath.m_cAllPath.GetNext(pos) ;
		if (nIndex++ % nCoreNum != nAtCore || pBasePath == NULL)
		{
			continue;
		}

		pBasePath->InsertCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
		pBasePath->VerifyLinePath( DriveMdl,GetSurfaceTol(),m_cTmpPrgDef) ;	
		// 使用毛坯对路径进行裁减,保留毛坯内部路径
		SetCutPathZ( pBasePath, 0 ) ;
		SetCutPathZValue( pBasePath, float( dBot - 5 ) ) ;
		pBasePath->VerifyLinePath( RoughMdl, GetSurfaceTol(), m_cTmpPrgDef ) ;
		int nLineNo = pBasePath->m_nLineNo ;
		int nLayerNo = pBasePath->m_nLayerNo ;
		TrimCutPathZValue( pBasePath, float( dBot - 1 ) ) ;
		CSmtCutPath *pPath = pBasePath ;
		for( ; pPath ; pPath = pPath->next ) 
		{
			SetCutPathZ( pPath, 1 ) ;
			pPath->m_nLineNo = nLineNo ;
			pPath->m_nLayerNo = nLayerNo ;
		}
		AllPath[nIndex-1] = pBasePath;
	}
	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
	{// 中断时删除数据		
		Nc5D_DestroyMultiCalcData(AllPath, nIndex, &TmpPath, pos, nCoreNum, nAtCore);		
		return FALSE;
	}
	return TRUE;
}

int CSmartRLinearGen::LayerAllPath( CSmtCPathLib& AllPath, double dTop, double dBot, BOX3D &sBox, JDNC_PRGDEF &ProgDef, double dCur ) 
{
	if( GetProjDownStep( dBot, dTop, m_cFeedDef ) < 0.002 )
	{
		m_nCount = 0 ;
		return 0 ;
	}
	DOUBLE dZValue   = dTop - GetProjDownStep( dBot, dTop, m_cFeedDef ) ;
	DOUBLE dMaxDepth = dBot ;
	
	double mode = ( dTop - dBot ) / GetProjDownStep( dBot, dTop, m_cFeedDef ) ;
	m_nCount = ( int ) ( mode ) ;
	if( mode - m_nCount > 0. )
		m_nCount += 1 ;
	if( m_cFeedDef.m_cLayerDef.m_nLayerType < 1 )
		m_nCount = 1 ;
	m_dDownStep = ( dTop - dBot ) / m_nCount ;
	dZValue = dTop ;
	if( m_nCount < 1 ) m_nCount = 1 ;

	ProgDef.m_dLimitAt =  m_nCount / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;

	POSITION pos = NULL ;
	CSmtCutPath *pPath = NULL ;
	m_PathNew = new CNcSmartPath[m_nCount] ;
	INT_PTR i = -1 ;
	// 拷贝分层参数, bFlag 1:拷贝分层;
	int bFlag = m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE ;

	while( dZValue >= dMaxDepth )
	{
		dZValue -= m_dDownStep ;
		i++ ;
		if( i == m_nCount )
			break ;	
		//////////////计算进度条////////////////////////
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
		////////////////////////////////////////////////

		CNcSmartPath cs ;
		// 如果是拷贝分层，计算路径抬起的高度
		double h = dZValue - dMaxDepth ;
		if( bFlag )
		{
			if( m_nCutMode )
				cs.TrimByZValue3 ( AllPath, m_PathNew[i], sBox.max[2], h  ) ;
			else
				cs.TrimByZValue1( AllPath, m_PathNew[i], dTop, h ) ;
		}
		else
		{
			if( m_nCutMode ) // 因为路径的高度为相对高度，所以传入的高度也为相对高度
				cs.TrimByZValue2 ( AllPath, m_PathNew[i], dZValue - sBox.max[2] , m_nCutMode ) ;
			else
				cs.TrimByZValue0( AllPath, m_PathNew[i], dZValue ) ;
		}
		pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;	
			pPath->m_nLayerNo = int( i ) ;
		}
	}
	if( bFlag )
		AllPath.ClearAllPath() ;
	return 1 ; 
}
BOOL CSmartRLinearGen::BuildStockMdl ( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box )
{
	////////////////////处理毛坯情况////////////////
	// 首先生产加工面模型
	if( !Graph.BuildStockModel ( m_cSetupDef, RoughMdl, box ) )
	{
		return FALSE ;
	}
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock () ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, dStock, 0 ) ;
		RoughMdl.ZMoveModel ( dStock ) ;
	}
	else
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, 0., 0 ) ;
	}
	RoughMdl.UpdateCurrTool( m_pRoughTool )  ;
	RoughMdl.m_fBottom =  (FLOAT)GetBottomHeight() ;
	return TRUE ;
}
int CSmartRLinearGen::GeneratePathEx( CPathGroup& NewPath    ,  /*加工路径*/ 
						              CSmartGraphic& Graph )  /*雕刻图形*/
{ // 生成刀具路径 
	// STEP 0 : 转换刀具 
	if( !UpdateTool3D(Graph) ) return 0;
	CSmtCheckMdl  DriveMdl, CheckMdl, RoughMdl ;//CheckMdl2 ;
	BOX3D sbox, MaxBox ;
	mathClnBox3D( &sbox ) ;
	if( !BuildStockMdl( Graph, RoughMdl, sbox ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
		return 0 ;
	}
	m_nCutMode = 1 ;
	double dTop = GetTopHeight(), dBot = GetBottomHeight() ;
	CalcTopAndBot( m_cShapeDef, sbox, dTop, dBot, GetDriveSurfZMove(), m_cSetupDef.m_cTolDef.m_dArcTol ) ;
	if( fabs( dTop - dBot ) < 0.003 )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return 0 ;
	}
	if( !BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return 0 ;
	}
	RoughMdl.m_fBottom = DriveMdl.m_fBottom = CheckMdl.m_fBottom = float( dBot ) ;
	DriveMdl.DefineDriveSurfBox( MaxBox.min, MaxBox.max ) ;
	mathCalBox3DUnion( &MaxBox, &sbox, &MaxBox ) ;
	mathExpandBox3D( m_pRoughTool->m_fRadius * 1.5, &MaxBox ) ;
	BOOL bAllLoop = FALSE ;
	// STEP 3 : 提取曲面边界
	CSmartLoop* AllLoop = CreateAreaLoop( Graph, sbox, m_pRoughTool->m_fRadius * 1.5, bAllLoop ) ;
	if( !AllLoop ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}
	CSmartLoop *pLoop = NULL ;
	// 旋转加工模型
	DOUBLE dRotateAngle = 0.0 ;
	if( fabs( m_cParam.m_dLineAngle ) > 0.001 )
	{ 
		dRotateAngle = ANGLE_TO_RADIAN( m_cParam.m_dLineAngle ) ;
		DriveMdl.RotateByZAxis( -dRotateAngle, TRUE ) ;
		RoughMdl.RotateByZAxis( -dRotateAngle ) ;
		
		for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
		{
			pLoop->RotateContour( -dRotateAngle ) ;
		}
	}
	CSmtCPathLib  AllPath  ;
	////////////////////第一条进度条////////////////////////
	ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_PARALLEL ) ;
	double dCur[3] = { 20, 20, 60 } ;
	if( !BuildPlanePath( AllPath, DriveMdl, RoughMdl, AllLoop, MaxBox, dBot, m_cPrgDef, dCur[0] ) )
	{
		Mini_DeleteContours( AllLoop ) ;
        m_nErrorType = JDERROR_GENPATH_ERRDATA  ;
		return 0 ; 
	}
	Mini_DeleteContours( AllLoop ) ;
	// 连刀的时候不投影到曲面上，直接连过去
	AllPath.SetCutMode ( MINI_CONNECT_PATH ) ;
    // STEP 5 : 分层计算路径
    if( !LayerAllPath( AllPath, dTop, dBot, sbox, m_cPrgDef, dCur[1] ) )
	{
        m_nErrorType =  JDERROR_GENPATH_ERRDATA  ;
		return 0 ; 
	}
	////////////////////第三条进度条//////////////////////
	m_nTotal = 0 ;
	for( int j = 0 ; j < m_nCount ; j++ )
	{
		TrimAllPathByZValue( m_PathNew[j], dTop, FALSE ) ;
		int nCount = (int)m_PathNew[j].m_cAllPath.GetCount() ;
		m_nTotal += nCount ;
	}
	if( m_nTotal < 1 )m_nTotal = 1 ;
	m_cPrgDef.m_dLimitAt =  m_nTotal / dCur[2] ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dIncStep = 1.0  ;

	////////////////////////////////////////////////////

	DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;

	if( GetOrderDef().m_nLayerOrd )/* ==1 高度优先 */
	{
		if( !SortPathByDepth( NewPath, DriveMdl, m_cPrgDef ) )
			NewPath.ClearAll() ;
	}
	else /* == 0区域优先 */
	{
		if( !SortPathByArea( NewPath, DriveMdl, m_pTool, m_cPrgDef ) )
		{
			NewPath.ClearAll() ;
		}
	}
    if( SurfNC_IsAbort() )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return 0 ;
    }
/*	
	// for test
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine( *pPComb ) ;
	NewPath.AddData( 0., pPComb ) ;
*/	
	// 添加慢速和相对高度连接
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
		AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE) ;

	// 旋转加工路径组
	if( dRotateAngle != 0.0 )
	{// 旋转路径 
		NewPath.RotateGroup( dRotateAngle ) ;
	} 
	if( AllPath.GetNumPoint() ) 
	{
		AllPath.ClearAllPath() ;
	}
	
	return 1 ;
}

/*********************************************************
 * 按照加工路径的高度对路径进行处理，首先加工同一高度的 *
 * 路径。                                               *
*********************************************************/
int  CSmartRLinearGen::SortPathByDepth( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, JDNC_PRGDEF &ProgDef )
{
	double dDist = 10. * m_cParam.m_dOverStep ;
	if( dDist > 20. ) 
		dDist = 20. ;
	else if( dDist < 3 * m_cParam.m_dOverStep )
		dDist = 3 * m_cParam.m_dOverStep ;
	
	POSITION pos , atpos ;
	CSmtCutPath* pPath ;
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
	{
		if( !ComputeProg( m_PathNew[i], ProgDef ) )
			return 2 ;
		if( m_cParam.m_bLineFlag & NCDEF_LINEAR_ZIGZAG )
			m_PathNew[i].ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE ) ;
		AddPathToPG(PGroup, m_PathNew[i] ) ;
		pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
			m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
			delete pPath ;
		}
	}
	return 1 ;
}
/*********************************************************
 * 将CSmtCutPath类型的路径转换为CPathCombine类型，并放到 *
 * PGroup当中去。                                        *
*********************************************************/
int CSmartRLinearGen::AddPathToPG ( CPathGroup& PGroup,
								  CSmtCPathLib& OldPath )
{
	POSITION pos ;
	CSmtCutPath* pPath ;
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pos = OldPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		pPath = OldPath.m_cAllPath.GetNext( pos ) ;
		pPath->AddToPathCombine ( *pPComb ) ;
	}
	PGroup.AddData( 0.0, pPComb ) ;
	return 1 ;
}

void CSmartRLinearGen::ClearAllPath ( CSmtCPathLib& OldPath )
{
	POSITION pos, atpos ;
	CSmtCutPath* pPath ;
	pos = OldPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = OldPath.m_cAllPath.GetNext( pos ) ;
		OldPath.m_cAllPath.RemoveAt( atpos ) ;
		delete pPath ;
	}
}

void CSmartRLinearGen::GetAreaPathInSameLayer( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
											 BOX3D* box, CSmtCPathLib& NewPath ) 
{
	CSmtCutPath *pNext ;
	POSITION atPos, atCurr ;
	DOUBLE dist = 3.0 * pMiller->m_fRadius ;
	DOUBLE dist1 = 2 * m_cParam.m_dOverStep ;
	dist = max( dist, dist1 ) ;
	if( !OldPath.m_cAllPath.GetCount () )
		return  ;
	while( OldPath.m_cAllPath.GetCount() ) 
	{
		atCurr = OldPath.m_cAllPath.GetHeadPosition() ;
		int nCount = 0 ;
		while( atCurr )
		{
			atPos = atCurr ;
			pNext = OldPath.m_cAllPath.GetNext ( atCurr ) ;
			
			BOX3D box1 ;
			MathCAM_GetPathBox( pNext, box1 ) ;
			if(  pNext->m_nLineNo == -1 )
				continue ;
			//两段路径的包围盒相差小于刀具直径
			//包围盒间距大于刀具直径，肯定处于不同的区域
			if( mathChkBox3DInt( box , &box1 , 2 * dist ) ) 
			{
				if( PathInSameArea( NewPath, pMiller, pNext ) )
				{
					//扩大包围盒
					mathCalBox3DUnion( box, &box1 , box) ;
					NewPath.m_cAllPath.AddTail( pNext ) ;
					OldPath.m_cAllPath.RemoveAt ( atPos ) ;
					nCount++ ;
				}
			}		
 		}
		if( !nCount )
			break ;
	}
	return  ;
}

BOOL CSmartRLinearGen::PathInSameArea( CSmtCPathLib& OldPath, CSmartTool* pMiller, CSmtCutPath* pPath ) 
{
	CSmtCutPath* pNext ;
	DOUBLE dist = 3.0 * pMiller->m_fRadius ;
	DOUBLE dist0 = 2 * m_cParam.m_dOverStep ;
	dist = min( dist, dist0 ) ;
	if( dist < 1. )
		dist = 1. ;

	DOUBLE dist1, dist2, dist3, dist4 ;
	POSITION pos = OldPath.m_cAllPath.GetTailPosition() ;
	while( pos )
	{
		pNext = OldPath.m_cAllPath.GetPrev( pos ) ;
		//if( fabs( pPath->m_nLineNo - pNext->m_nLineNo ) > 2 || 
		//	fabs( pPath->m_nLineNo - pNext->m_nLineNo ) == 0 ) 
		if( fabs( (double)(pPath->m_nLineNo - pNext->m_nLineNo) ) > 2 || 
			fabs( (double)(pPath->m_nLineNo - pNext->m_nLineNo) ) < 1.0e-20 ) 
			continue ;
		if( !GetDistFromPt( pPath->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint, pNext->m_pHead->m_fPoint, dist1 ) )
			dist1 = mathFDist( pPath->m_pHead->m_fPoint, pNext->m_pHead->m_fPoint ) ;
		if( !GetDistFromPt( pPath->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint, pNext->m_pTail->m_fPoint,   dist2 ) )
			dist2 = mathFDist( pPath->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint   ) ;
		if( !GetDistFromPt( pNext->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint, pPath->m_pHead->m_fPoint, dist3 ) )
			dist3 = mathFDist( pNext->m_pHead->m_fPoint, pPath->m_pHead->m_fPoint ) ;
		if( !GetDistFromPt( pNext->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint, pPath->m_pTail->m_fPoint,   dist4 ) )
			dist4 = mathFDist( pNext->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint   ) ;
		
		if( dist1 < dist || dist2 < dist || 
			dist3 < dist || dist4 < dist )
			return 1 ;
	}
	return 0 ;
}
/*******************************************************
 * 将加工路径按照区域进行划分。即按照路径的最大包围盒 *
 * 进行处理。如果同一层中路径之间的最大包围盒距离大于 *
 * 刀具直径，则加工下一层。依次，然后回过来加工距离比 *
 * 较大的路径。                                       *
*******************************************************/
int  CSmartRLinearGen::SortPathByArea( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, 
									 CSmartTool* pMiller, JDNC_PRGDEF &ProgDef ) 
{
	CSmtCutPath * pPath ;
	POSITION pos, atpos ;
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		while( m_PathNew[i].m_cAllPath.GetCount() )
		{
			pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
			while( pos )
			{
				atpos = pos ;
				pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
				BOX3D box ;
				MathCAM_GetPathBox( pPath, box ) ;

				CSmtCPathLib PathLib ;
				PathLib.m_cAllPath.AddTail( pPath ) ;
				// 从这一层中删除该条路径
				m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
				// 从本层中寻找在同一区域中的路径,放到PathLib中
				GetAreaPathInSameLayer( m_PathNew[i], pMiller, &box, PathLib ) ;

				if( GenAreaPath( PGroup, PathLib, CheckMdl, pMiller, i + 1, ProgDef ) == 2 )
					return 0 ;
				break ;
			}
		}
	}
	return 1 ;
}

int  CSmartRLinearGen::GenAreaPath ( CPathGroup& PGroup, CSmtCPathLib& PrePathLib, 
								    CSmtCheckMdl& CheckMdl, CSmartTool* pMiller, int i, JDNC_PRGDEF &ProgDef )
{
	double dDist = 10. * m_cParam.m_dOverStep ;
	if( dDist > 20. ) 
		dDist = 20. ;
	else if( dDist < 3 * m_cParam.m_dOverStep )
		dDist = 3 * m_cParam.m_dOverStep ;

	// 递归退出条件1
	if( i >= m_nCount )
	{
		SortAreaPath( PrePathLib ) ;
		if( !ComputeProg( PrePathLib, ProgDef ) ) return 2 ;
		if( m_cParam.m_bLineFlag & NCDEF_LINEAR_ZIGZAG )
			PrePathLib.ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE ) ;
		AddPathToPG( PGroup, PrePathLib ) ;
		return 0 ;
	}
	// 从下一层中找出与PrePathLib在同一区域中的路径
	CSmtCPathLib CurPathLib ;
	GetAreaPathInNextLayer( PrePathLib, i, CurPathLib ) ;
	// 将上一层路径按照行号和Ｘ坐标排序.
	SortAreaPath( PrePathLib ) ;
	// 连接上一层路径PrePathLib，并加工之
	if( !ComputeProg( PrePathLib,ProgDef ) ) return 2 ;
	if( m_cParam.m_bLineFlag & NCDEF_LINEAR_ZIGZAG )
		PrePathLib.ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE ) ;
	AddPathToPG( PGroup, PrePathLib ) ;
	// 递归退出条件2
	if( CurPathLib.m_cAllPath.GetCount() < 1 )
		return 0 ;
	// 中断
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
	{
		ClearAllPath( CurPathLib ) ;
		return 2 ;
	}
	CSmtCutPath * pPath ;
	POSITION pos, atpos ;
	while( CurPathLib.m_cAllPath.GetCount() )
	{
		pos = CurPathLib.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = CurPathLib.m_cAllPath.GetNext( pos ) ;
			
			BOX3D box ;
			MathCAM_GetPathBox( pPath, box ) ;
			
			CSmtCPathLib AreaPath ;
			AreaPath.m_cAllPath.AddTail( pPath ) ;
			//然后删除之
			CurPathLib.m_cAllPath.RemoveAt ( atpos ) ;
			// 从CurPathLib中寻找在同一区域的路径,放到AreaPath中
			GetAreaPathInSameLayer( CurPathLib, pMiller, &box, AreaPath ) ;
			// 整理路径,加工OldPath中路径
			if( GenAreaPath ( PGroup, AreaPath, CheckMdl, pMiller, i + 1, ProgDef ) == 2 )
				return 2 ;
			break ;
		}
	}
	return 1 ;
}

void CSmartRLinearGen::GetAreaPathInNextLayer ( CSmtCPathLib& PrePathLib, 
											   int i, CSmtCPathLib& CurPathLib )
{
	CSmtCutPath* pPath ;
	POSITION pos, atpos ;
	pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
	while ( pos )
	{
		atpos = pos ;
		pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
		if( IsSameAreaNextLayer( PrePathLib, pPath ) )
		{
			CurPathLib.m_cAllPath.AddTail( pPath ) ;
			m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
		}
	}
}

BOOL CSmartRLinearGen::IsSameAreaNextLayer ( CSmtCPathLib& PrePathLib, CSmtCutPath* pPath ) 
{
	DOUBLE NextMinX, NextMaxX, PathMinX, PathMaxX ;
	CSmtCutPath* pNext ;
	POSITION pos = PrePathLib.m_cAllPath.GetHeadPosition() ;
	while( pos ) 
	{
		pNext = PrePathLib.m_cAllPath.GetNext( pos ) ;
		if( pNext->m_nLineNo != pPath->m_nLineNo )
			continue ;
		PathMinX = min( pPath->m_pHead->m_fPoint[0], pPath->m_pTail->m_fPoint[0] ) ;
		PathMaxX = max( pPath->m_pHead->m_fPoint[0], pPath->m_pTail->m_fPoint[0] ) ;
		NextMinX = min( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ;
		NextMaxX = max( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ; 
		if( PathMinX >= NextMinX && NextMaxX >= PathMaxX )
			return 1 ;
	}
	return 0 ;
}

// 点到线段的最短距离
BOOL CSmartRLinearGen::GetDistFromPt ( FPNT3D start, FPNT3D end, 
								    FPNT3D pt, double& dist )
{
	double d ;
    VEC2D dir ;
	PNT2D o_pt ;

	dir[0] = end[0] - start[0] ;
	dir[1] = end[1] - start[1] ;
	
	d = dir[0] * dir[0] + dir[1]*dir[1] ;
	if (d < 1.0e-10 )	return 0 ;
    
	double t = ( ( pt[0] - start[0] ) * dir[0] + 
				 ( pt[1] - start[1] ) * dir[1] ) / d ;

	if ( t < MIN_DBL )
	{
		t = 0;
		o_pt[0] = start[0] ;
		o_pt[1] = start[1] ;
	}
	else if ( t > 1.0-MIN_DBL )
	{
		t = 1;
		o_pt[0] = end[0] ;
		o_pt[1] = end[1] ;
	}
	else 
	{
		o_pt[0] = start[0] + t * dir[0] ;
		o_pt[1] = start[1] + t * dir[1] ;
	}
	
	dist = sqrt( ( pt[0] - o_pt[0] ) * ( pt[0] - o_pt[0] ) +
				 ( pt[1] - o_pt[1] ) * ( pt[1] - o_pt[1] ) ) ;

	return 1 ;
}

void CSmartRLinearGen::SortAreaPath (CSmtCPathLib& OldPath ) 
{
	POSITION pos, atpos ;
	CSmtCutPath* pPath ;
	CSmtCPathLib PathLib ;
	while( OldPath.m_cAllPath.GetCount() > 0 )
	{
		CSmtCPathLib NewPath ;
		FindPrePath( OldPath, NewPath ) ;
		SortPathByXCoord( NewPath ) ;
		pos = NewPath.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = NewPath.m_cAllPath.GetNext( pos ) ;
			NewPath.m_cAllPath.RemoveAt( atpos ) ;
			PathLib.m_cAllPath.AddTail( pPath ) ;
		}
		NewPath.m_cAllPath.RemoveAll() ;
	}
	pos = PathLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pPath = PathLib.m_cAllPath.GetNext( pos ) ;
		OldPath.m_cAllPath.AddTail( pPath ) ;
		PathLib.m_cAllPath.RemoveAt( atpos ) ;
	}
	PathLib.m_cAllPath.RemoveAll() ;
}

void CSmartRLinearGen::FindPrePath (CSmtCPathLib& OldPath, CSmtCPathLib& NewPath ) 
{
	POSITION pos, atpos, cur=NULL ;
	CSmtCutPath *pNext, *pPath = NULL ;
	// 首先找到最小的行号
	int MinLineNo = 10000000 ;
	pos = OldPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pNext = OldPath.m_cAllPath.GetNext( pos ) ;
		if( pNext->m_nLineNo < MinLineNo )
		{
			MinLineNo = pNext->m_nLineNo ;
			pPath = pNext ;
			cur = atpos ;
		}
	}
	if( pPath && cur)
	{
		OldPath.m_cAllPath.RemoveAt( cur ) ;
		NewPath.m_cAllPath.AddTail( pPath ) ;
		pos = OldPath.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pNext = OldPath.m_cAllPath.GetNext( pos ) ;
			if( pNext->m_nLineNo == MinLineNo )
			{
				OldPath.m_cAllPath.RemoveAt( atpos ) ;
				NewPath.m_cAllPath.AddTail( pNext ) ;
			}
		}
	}
	else
		return ;
}

void CSmartRLinearGen::SortPathByXCoord ( CSmtCPathLib& OldPath ) 
{
	CSmtCPathLib NewPath ;
	CSmtCutPath *pPath = NULL ;
	while( OldPath.m_cAllPath.GetCount() > 0 )
	{
		pPath = FindLeftPathInSameLine( OldPath ) ;
		if( pPath )
			NewPath.m_cAllPath.AddTail( pPath ) ;
	}
	
	POSITION pos, atpos ;
	pos = NewPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pPath = NewPath.m_cAllPath.GetNext( pos ) ;
		NewPath.m_cAllPath.RemoveAt( atpos ) ;
		OldPath.m_cAllPath.AddTail( pPath ) ;
	}
	NewPath.m_cAllPath.RemoveAll() ;
}

CSmtCutPath* CSmartRLinearGen::FindLeftPathInSameLine ( CSmtCPathLib& OldPath )
{
	if( OldPath.m_cAllPath.GetCount() < 1 )
		return NULL ;
	POSITION pos, atpos, cur = NULL;
	CSmtCutPath *pNext, *pPath = NULL ;
	DOUBLE xcoord, MinX = 1.0e10 ;
	// 找到Ｘ坐标最小的条路径
	pos = OldPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pNext = OldPath.m_cAllPath.GetNext( pos ) ;
		xcoord = min( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ;
		if( xcoord < MinX )
		{
			pPath = pNext ;
			MinX = xcoord ;
			cur = atpos ;
		}
	}
	if( pPath && cur )
	{
		OldPath.m_cAllPath.RemoveAt( cur ) ;
		return pPath ;
	}
	else
		return NULL ;
}

BOOL CSmartRLinearGen::ComputeProg( CSmtCPathLib& AllPath, JDNC_PRGDEF &ProgDef ) 
{
	POSITION pos ;
	CSmtCutPath* pPath ;
	pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		///////////////////////////////////////
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
		///////////////////////////////////////
		pPath = AllPath.m_cAllPath.GetNext( pos ) ;
	}
	return TRUE ;
}

// gm 多线程运算线程函数
static UINT MathCAM_CreateREPlanePathSubProc(LPVOID lpParam)
{	
	if (lpParam == NULL)
	{
		return 0;
	}
	CRLREPPT_DATA& cData = *(CRLREPPT_DATA *)lpParam;	
	if( cData.pPathGen == NULL || 
		cData.TmpPath == NULL    || 
		cData.AllPath == NULL    || 
		cData.DriveMdl == NULL   ) 
	{
		return 0 ;
	}
	return cData.pPathGen->CreatePlanePathSubProc(cData.nAtCore, cData.nCoreNum, *cData.TmpPath, 
													cData.AllPath, *cData.DriveMdl, *cData.RoughMdl,cData.dBot,cData.PrgDef);
}

// 径向放射曲面粗雕加工

CSmartREmiterGen::CSmartREmiterGen()
{
	m_nCutMode	= 0 ;
	m_nCount	= 0 ;
	m_PathNew	= NULL ;
	m_pRoughTool = NULL ;
	m_cTmpPrgDef.m_dLimitAt = 0. ;
	m_cTmpPrgDef.m_dStepAt  = 0 ;
	m_cTmpPrgDef.m_dIncStep = 1. ;
	m_cTmpPrgDef.m_pBrkFunc = NULL ;
	m_cTmpPrgDef.m_pPrgFunc = NULL ;
	m_cTmpPrgDef.m_pPosFunc = NULL ;
}
CSmartREmiterGen::~CSmartREmiterGen()
{
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
		m_PathNew[i].ClearAllPath() ;
	if( m_nCount ) delete []m_PathNew ;
	if( m_pRoughTool ) delete m_pRoughTool ;
	m_pRoughTool = NULL ;
}

DOUBLE CSmartREmiterGen::GetRadius ( BOX3D& MaxBox )
{
	PNT2D center, p1, p2, p3, p4 ;
	center[0] = m_cParam.m_dCenter[0] ;
	center[1] = m_cParam.m_dCenter[1] ;

	p1[0] = MaxBox.min[0], p1[1] = MaxBox.max[1] ;
	p2[0] = MaxBox.max[0], p2[1] = MaxBox.min[1] ;
	memcpy( p3, MaxBox.min, sizeof( PNT2D ) ) ;
	memcpy( p4, MaxBox.max, sizeof( PNT2D ) ) ;
	
	DOUBLE r1, r2, r3, r4 ;
	r1 = mathDist2D( center, p1 ) , r2 = mathDist2D( center, p2 ) ;
	r3 = mathDist2D( center, p3 ) , r4 = mathDist2D( center, p4 ) ;
	r1 = max( r1, r2 ) ;
	r2 = max( r3, r4 ) ;
	
	DOUBLE r = max( r1, r2 ) ;
	return r ;
}

int  CSmartREmiterGen::GetUsedPoint( PNT2D* dPtArr, PNT2D start, 
								   PNT2D end, int nCount ) 
{
	PNT2D PtArr[200] ;
	PtArr[0][0] = start[0], PtArr[0][1] = start[1] ;
	int n = 1 ;
	double t[2] ;
	for( int i = 0 ; i < nCount ; i++ )
	{
		double dist  = mathDist2D( start, end ) ;
		t[0] = mathDist2D( start, dPtArr[i] ) / dist ;
		t[1] = mathDist2D( end,   dPtArr[i] ) / dist ;

		if( ( t[0] >= 0 && t[0] <= 1 ) && 
			( t[1] >= 0 && t[1] <= 1 ) )
		{
			int nNum = n ;
			for( int j = 0 ; j < nNum ; j++ )
			{
				// 判断该点是否为重点
				if( mathDist2D( PtArr[j], dPtArr[i] ) > 1.0e-6 )
				{
					n++ ;
					PtArr[n - 1][0] = dPtArr[i][0] ;
					PtArr[n - 1][1] = dPtArr[i][1] ;
				}
				else
					break ;
			}
		}
	}
	nCount = 0 ;
	for( i = 0 ; i < n ; i++ )
	{
		// 防止重点
		if( i > 0 && mathDist2D( PtArr[i -1 ], PtArr[i] ) < 1.e-6 )
			continue ;
		
		dPtArr[nCount][0] = PtArr[i][0], dPtArr[nCount][1] = PtArr[i][1] ;
		nCount++ ;
	}/**/
	return nCount ;
}

int CSmartREmiterGen::BuildPlanePath(	CSmtCPathLib& AllPath ,  //曲面路径组
										CSmtCheckMdl& DriveMdl,  //加工面模型
										CSmtCheckMdl& RoughMdl,  //毛坯面模型
										CSmartLoop* AllLoop,
										BOX3D& MaxBox, double dBot,
										JDNC_PRGDEF &ProgDef	,
										double dCur			)          
{

	CSmartLoop* pLoop ;
	CSmtCutPath* pNewPath  ;
	PNT4D  dStart, dEnd ;
	PNT2D  dPtArr[200] ;
	CSmartSect* dSectList[200] ;
	int nGroup =  0, nLineNo = 0 ;
	DOUBLE dRadius , dRadAng, dAngle, dAngRange ; 

	for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
	{
		for( int k = 0 ; k < 2  ; k ++ ) 
		{
			if( pLoop->m_dBox[0][k] < MaxBox.min[k] ) 
				MaxBox.min[k] = pLoop->m_dBox[0][k] ;
			if( pLoop->m_dBox[1][k] > MaxBox.max[k] )
				MaxBox.max[k] = pLoop->m_dBox[1][k] ;
		}
	}
	MaxBox.min[2] = MaxBox.max[2] = 0. ;
	dRadius = GetRadius( MaxBox )  + 2 * DriveMdl.m_pTool->m_fRadius ;
	dStart[2] = dEnd[2] = DriveMdl.m_fBottom ;
	dStart[3] = dEnd[3] = 0.0 ;
	dStart[0] = m_cParam.m_dCenter[0] ;
	dStart[1] = m_cParam.m_dCenter[1] ;
	RoughMdl.m_fBottom = float( dBot - 5 ) ;

	int n , /*nPnt,*/ i = 0 ;
	dAngRange = m_cParam.m_dAngRange ;
	if( dAngRange > 360. ) dAngRange = 360. ;

	n = (int)( dAngRange / m_cParam.m_dAngStep) + 1 ;
	dAngle = m_cParam.m_dAngFrom ;

	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	ProgDef.m_dTotalMove = dCur ;

	for( pLoop = AllLoop, nGroup = 0 ; pLoop ; pLoop = pLoop->next, nGroup++ )
	{
		for(  i = 0 , nLineNo = 0 ; i < n ; i ++, dAngle += m_cParam.m_dAngStep, nLineNo++ ) 
		{

			dRadAng = ANGLE_TO_RADIAN( dAngle ) ;
			dEnd[0] = dStart[0] + cos( dRadAng ) * dRadius ;
			dEnd[1] = dStart[1] + sin( dRadAng ) * dRadius ;

			UINT nCount = pLoop->GetLineInt ( dStart , dEnd , dPtArr, dSectList, 200 ) ;
			if( nCount == 0 || nCount % 2 ) 
			{
				continue ;
			}

			nCount = GetUsedPoint( dPtArr, dStart, dEnd, nCount ) ;

			UINT i = 0 ;
			if( nCount % 2 ) i = 1 ;
			PNT2D start, end ;
			for( i ; i < nCount ; i += 2 )
			{
				start[0] = dPtArr[i  ][0], start[1] = dPtArr[i ][1] ;
				end[0]   = dPtArr[i+1][0], end[1]   = dPtArr[i+1][1] ;
				pNewPath = new CSmtCutPath() ;
				pNewPath->AddPoint ( start ) ;
				pNewPath->AddPoint ( end ) ;
				pNewPath->m_nLineNo  = nLineNo ;
				pNewPath->m_nLayerNo = nGroup;
				AllPath.AddToTail( pNewPath )  ;
			}
		}
	}

	//gm 多线程
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE ;	
	INT_PTR nPathNum = AllPath.m_cAllPath.GetCount();
	if (nPathNum >=2 && IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
		RoughMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	// 支持多线程，即拷贝数据不为空
	if( nPathNum >= 2 && DriveMdl.GetMultiCopy() != NULL && RoughMdl.GetMultiCopy() != NULL )
	{
		CRLREPPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = ProgDef;//进度条
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathNum);
		int i=0, k=1;
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].DriveMdl = i == 0 ? &DriveMdl : DriveMdl.GetMultiCopy(i-k);
			ThreadData[i].RoughMdl = i == 0 ? &RoughMdl : RoughMdl.GetMultiCopy(i-k);
			if (ThreadData[i].DriveMdl == NULL || ThreadData[i].RoughMdl == NULL)
			{
				i--, k--, nThreadNum--;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].TmpPath = &AllPath;
			ThreadData[i].AllPath = &NewPath[0];
			ThreadData[i].dBot = dBot ;
			ThreadData[i].PrgDef = i==0 ? ProgDef : tmpPrg;
			ThreadData[i].pPathGen = this;			
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		// 调用多线程函数
		MathCAM_ThreadMainFunc(MathCAM_CreateREPlanePathSubProc, lpParam, nThreadNum);
	}
	else
	{
		//如果不是多线程处理
		CreatePlanePathSubProc(0,1,AllPath,&NewPath[0],DriveMdl,RoughMdl,dBot,ProgDef) ;
	}

	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		DriveMdl.DeleteMultiCopy() ;
		RoughMdl.DeleteMultiCopy() ;
	}
	AllPath.m_cAllPath.RemoveAll();
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc() )
	{
		return FALSE ;
	}	
	Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);

	RoughMdl.m_fBottom = float( dBot ) ;
	return 1 ;
}

//gm 平行截线线程函数
BOOL CSmartREmiterGen::CreatePlanePathSubProc(int nAtCore, int nCoreNum, CSmtCPathLib &TmpPath, LPVOID* AllPath, 
												CSmtCheckMdl &DriveMdl, CSmtCheckMdl &RoughMdl,double dBot,JDNC_PRGDEF &PrgDef)
{
	INT_PTR nNumPath = TmpPath.m_cAllPath.GetCount();
	if (nNumPath < 1)
	{
		return FALSE;
	}
	PrgDef.m_dIncStep = PrgDef.m_dStepAt = 0.0 ;
	PrgDef.m_dLimitAt = 1.0 ;
	PrgDef.m_dIncStep = PrgDef.m_dTotalMove / nNumPath ;

	int nIndex = 0;

	POSITION pos = TmpPath.m_cAllPath.GetHeadPosition();
	while( pos )
	{
		if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
		{
			break ;
		}
		MathCAM_MovePrgStep(PrgDef);
		CSmtCutPath* pBasePath = TmpPath.m_cAllPath.GetNext(pos) ;
		if (nIndex++ % nCoreNum != nAtCore || pBasePath == NULL)
		{
			continue;
		}

		pBasePath->InsertCPoint( m_cSetupDef.m_cTolDef.m_dMaxStep ) ;
		pBasePath->VerifyLinePath( DriveMdl,GetSurfaceTol(),m_cTmpPrgDef) ;	
		// 使用毛坯对路径进行裁减,保留毛坯内部路径
		SetCutPathZ( pBasePath, 0 ) ;
		SetCutPathZValue( pBasePath, float( dBot - 5 ) ) ;
		pBasePath->VerifyLinePath( RoughMdl, GetSurfaceTol(), m_cTmpPrgDef ) ;
		int nLineNo = pBasePath->m_nLineNo ;
		int nLayerNo = pBasePath->m_nLayerNo ;
		TrimCutPathZValue( pBasePath, float( dBot - 1 ) ) ;
		CSmtCutPath *pPath = pBasePath ;
		for( ; pPath ; pPath = pPath->next ) 
		{
			SetCutPathZ( pPath, 1 ) ;
			pPath->m_nLineNo = nLineNo ;
			pPath->m_nLayerNo = nLayerNo ;
		}
		AllPath[nIndex-1] = pBasePath;
	}
	if( PrgDef.m_pBrkFunc && PrgDef.m_pBrkFunc() )
	{// 中断时删除数据		
		Nc5D_DestroyMultiCalcData(AllPath, nIndex, &TmpPath, pos, nCoreNum, nAtCore);		
		return FALSE;
	}
	return TRUE;
}

int CSmartREmiterGen::LayerAllPath( CSmtCPathLib& AllPath, double dTop, double dBot, BOX3D &sbox, JDNC_PRGDEF &ProgDef, double dCur )   /*所有路径*/
{
	if( GetProjDownStep( dBot, dTop, m_cFeedDef ) < 0.002 ) 
	{
		m_nCount = 0 ;
		return 0 ;
	}
	DOUBLE dZValue   = dTop - GetProjDownStep( dBot, dTop, m_cFeedDef ) ;
	DOUBLE dMaxDepth = dBot ;
	
	double mode = ( dTop - dBot )  / GetProjDownStep( dBot, dTop, m_cFeedDef ) ;
	m_nCount = ( int ) ( mode ) ;
	if( mode - m_nCount > 0. )
		m_nCount += 1 ;
	if( m_cFeedDef.m_cLayerDef.m_nLayerType < 1 )
		m_nCount = 1 ;
	m_dDownStep = ( dTop - dBot ) / m_nCount ;
	dZValue = dTop ;
	if( m_nCount < 1 ) m_nCount = 1 ;

	ProgDef.m_dLimitAt =  m_nCount / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;

	m_PathNew = new CNcSmartPath[m_nCount] ;
	INT_PTR i = -1 ;
	// 拷贝分层参数, bFlag 1:拷贝分层;
	int bFlag = m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE ;

	while( dZValue >= dMaxDepth )
	{
		dZValue -= m_dDownStep ;
		i++ ;
		if( i == m_nCount )
			break ;	
		//////////////计算进度条////////////////////////
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
				return FALSE ;
			ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
			while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
			{
				ProgDef.m_pPrgFunc(1) ;
				ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
			}
		////////////////////////////////////////////////
		CNcSmartPath cs ;
		// 如果是拷贝分层，计算路径抬起的高度
		double h = dZValue - dMaxDepth ;
		
		if( bFlag )
		{
			if( m_nCutMode )
				cs.TrimByZValue3 ( AllPath, m_PathNew[i], sbox.max[2], h ) ;
			else
				cs.TrimByZValue1( AllPath, m_PathNew[i], dTop, h ) ;
		}
		else
		{
			if( m_nCutMode ) // 因为路径的高度为相对高度，所以传入的高度也为相对高度
				cs.TrimByZValue2 ( AllPath, m_PathNew[i], dZValue - sbox.max[2], m_nCutMode ) ;
			else
				cs.TrimByZValue0( AllPath, m_PathNew[i], dZValue ) ;
		}
		POSITION pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			CSmtCutPath* pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;	
			pPath->m_nLayerNo = int( i ) ;
		}
	}
	return 1 ; 
}
BOOL CSmartREmiterGen::BuildStockMdl ( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box )
{
	////////////////////处理毛坯情况////////////////
	// 首先生产加工面模型
	if( !Graph.BuildStockModel ( m_cSetupDef, RoughMdl, box ) )
	{
		return FALSE ;
	}
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock () ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, dStock, 0 ) ;
		RoughMdl.ZMoveModel ( dStock ) ;
	}
	else
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, 0., 0 ) ;
	}
	RoughMdl.UpdateCurrTool( m_pRoughTool )  ;
	RoughMdl.m_fBottom =  (FLOAT)GetBottomHeight() ;
	return TRUE ;
}
int CSmartREmiterGen::GeneratePathEx( CPathGroup& NewPath  ,  /*加工路径*/ 
						              CSmartGraphic& Graph )  /*雕刻图形*/
{ // 生成刀具路径 
	// STEP 0 : 转换刀具 
	if( !UpdateTool3D(Graph) ) return 0;
	m_nCutMode = 1 ;
	// STEP 1 : 提取曲面图形 , 并声称干涉检查曲面
	CSmtCheckMdl  DriveMdl, CheckMdl, RoughMdl ;
	BOX3D sbox, MaxBox ;
	mathClnBox3D( &sbox ) ;
	if( !BuildStockMdl( Graph, RoughMdl, sbox ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
		return 0 ;
	}
	double dTop = GetTopHeight(), dBot = GetBottomHeight() ;
	CalcTopAndBot( m_cShapeDef, sbox, dTop, dBot, GetDriveSurfZMove(), m_cSetupDef.m_cTolDef.m_dArcTol ) ;
	if( fabs( dTop - dBot ) < 0.003 )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return 0 ;
	}

	if( !BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return 0 ;
	}
	RoughMdl.m_fBottom = DriveMdl.m_fBottom = CheckMdl.m_fBottom = float( dBot ) ;
	DriveMdl.DefineDriveSurfBox( MaxBox.min, MaxBox.max ) ;
	mathCalBox3DUnion( &MaxBox, &sbox, &MaxBox ) ;
	mathExpandBox3D( m_pRoughTool->m_fRadius * 1.5, &MaxBox ) ;
	DOUBLE dRadius =  nc_Distance( MaxBox.min, MaxBox.max, 2 ) / 2.0 ;
	m_cParam.m_dAngStep = m_cFeedDef.m_cStepDef.m_dOverStep / dRadius  * 180 / MiniPai;
	BOOL bAllLoop = FALSE ;
	// STEP 3 : 提取曲面边界
	CSmartLoop* AllLoop = CreateAreaLoop( Graph, sbox, m_pRoughTool->m_fRadius * 1.5, bAllLoop ) ;
	if( !AllLoop ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}
		
	/////////////////////第一条进度条///////////////////
	ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_EMITER ) ;
	double dCur[3] = { 20, 20, 60 } ;
	CSmtCPathLib  AllPath  ;
	if( !BuildPlanePath( AllPath, DriveMdl, RoughMdl, AllLoop, MaxBox, dBot, m_cPrgDef, dCur[0] ) )
	{
		Mini_DeleteContours( AllLoop ) ;
        m_nErrorType =  JDERROR_GENPATH_ERRDATA  ;
		return 0 ; 
	}
	
	Mini_DeleteContours( AllLoop ) ;
	
	// 连刀的时候不投影到曲面上，直接连过去
	AllPath.SetCutMode ( MINI_CONNECT_PATH ) ;
    // STEP 5 : 分层计算路径
	////////////////////第二条进度条/////////////////////
	if( !LayerAllPath( AllPath, dTop, dBot, sbox, m_cPrgDef, dCur[1] ) )
	{
        m_nErrorType =  JDERROR_GENPATH_ERRDATA ;
		return 0 ; 
	}
	
	////////////////////第三条进度条//////////////////////
	m_nTotal = 0 ;
	for( int j = 0 ; j < m_nCount ; j++ )
	{
		TrimAllPathByZValue( m_PathNew[j], dTop, FALSE ) ;
		int nCount = (int)m_PathNew[j].m_cAllPath.GetCount() ;
		m_nTotal += nCount ;
	}
	if( m_nTotal < 1 )m_nTotal = 1 ;
	m_nCur = 1 ;
	m_cPrgDef.m_dLimitAt =  m_nTotal / dCur[2] ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dIncStep = 1.0  ;
	////////////////////////////////////////////////////
	DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
	if( GetOrderDef().m_nLayerOrd )/* ==1 高度优先 */
	{
		if( !SortPathByDepth( NewPath, DriveMdl, m_cPrgDef ) )
			NewPath.ClearAll() ;
	}
	else /* == 0区域优先 */
	{
		if( !SortPathByArea( NewPath, DriveMdl, m_pTool, m_cPrgDef ) )
		{
			NewPath.ClearAll() ;
		}
	}
    if( SurfNC_IsAbort() )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return 0 ;
    }
	if( AllPath.GetNumPoint() ) 
	{
		AllPath.ClearAllPath() ;
	}
	// 添加慢速和相对高度连接
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
		AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE) ;
	return 1 ;
}

/*********************************************************
 * 按照加工路径的高度对路径进行处理，首先加工同一高度的 *
 * 路径。                                               *
*********************************************************/
int  CSmartREmiterGen::SortPathByDepth( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, JDNC_PRGDEF &ProgDef )
{
	double dDist = 10. * m_cFeedDef.m_cStepDef.m_dOverStep ;
	if( dDist > 20. ) dDist = 20. ;
	else if( dDist < 3 * m_cFeedDef.m_cStepDef.m_dOverStep )
		dDist = 3 * m_cFeedDef.m_cStepDef.m_dOverStep ;

	POSITION pos , atpos ;
	CSmtCutPath* pPath ;
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
	{
		if( !ComputeProg( m_PathNew[i], ProgDef ) ) return 2 ;
		if( m_cParam.m_bEmitFlag & NCDEF_EMIT_ZIGZAG )
			m_PathNew[i].ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE,m_cParam.m_dCenter ) ;
		AddPathToPG(PGroup, m_PathNew[i] ) ;
		pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
			m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
			delete pPath ;
		}
	}
	return 1 ;
}
/*********************************************************
 * 将CSmtCutPath类型的路径转换为CPathCombine类型，并放到 *
 * PGroup当中去。                                        *
*********************************************************/
int CSmartREmiterGen::AddPathToPG ( CPathGroup& PGroup,
								  CSmtCPathLib& OldPath )
{
	POSITION pos ;
	CSmtCutPath* pPath ;
	// 只有单向走刀时，才考虑从内向外或者从外向内
	// 系统计算时，生成的路径是从内向外
	if( !(m_cParam.m_bEmitFlag & NCDEF_EMIT_ZIGZAG ) )
	{
		if( !( m_cParam.m_bEmitFlag & NCDEF_EMIT_TOOUTER ) )
			ReversePath( OldPath ) ;
	}
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pos = OldPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		pPath = OldPath.m_cAllPath.GetNext( pos ) ;
		pPath->AddToPathCombine ( *pPComb ) ;
	}
	PGroup.AddData( 0.0, pPComb ) ;
	return 1 ;
}

void CSmartREmiterGen::ClearAllPath ( CSmtCPathLib& OldPath )
{
	POSITION pos, atpos ;
	CSmtCutPath* pPath ;
	pos = OldPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = OldPath.m_cAllPath.GetNext( pos ) ;
		OldPath.m_cAllPath.RemoveAt( atpos ) ;
		delete pPath ;
	}
}

void CSmartREmiterGen::GetAreaPathInSameLayer( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
											 BOX3D* box, CSmtCPathLib& NewPath ) 
{
	CSmtCutPath *pNext ;
	POSITION atPos, atCurr ;
	DOUBLE dist = 4.0 * pMiller->m_fRadius ;
	if( dist > 5 ) 
		dist = 5 ;

	if( !OldPath.m_cAllPath.GetCount () )
		return  ;
	while( OldPath.m_cAllPath.GetCount() ) 
	{
		atCurr = OldPath.m_cAllPath.GetHeadPosition() ;
		int nCount = 0 ;
		while( atCurr )
		{
			atPos = atCurr ;
			pNext = OldPath.m_cAllPath.GetNext ( atCurr ) ;
			if(  pNext->m_nLineNo == -1 )
				continue ;
			BOX3D box1 ;
			MathCAM_GetPathBox( pNext, box1 ) ;
			//两段路径的包围盒相差小于刀具直径
			//包围盒间距大于刀具直径，肯定处于不同的区域
			if( mathChkBox3DInt( box , &box1 , 2 * dist ) ) 
			{
				if( PathInSameArea( NewPath, pMiller, pNext ) )
				{
					//扩大包围盒
					mathCalBox3DUnion( box, &box1 , box) ;
					NewPath.m_cAllPath.AddTail( pNext ) ;
					OldPath.m_cAllPath.RemoveAt ( atPos ) ;
					nCount++ ;
				}
			}		
 		}
		if( !nCount )
			break ;
	}
	return  ;
}

BOOL CSmartREmiterGen::PathInSameArea( CSmtCPathLib& OldPath, CSmartTool* pMiller, CSmtCutPath* pPath ) 
{
	CSmtCutPath* pNext ;
	DOUBLE dist = 3 * pMiller->m_fRadius ;
	if( dist < 1. )
		dist = 1. ;
	DOUBLE dist1, dist2, dist3, dist4 ;
	POSITION pos = OldPath.m_cAllPath.GetTailPosition() ;
	while( pos )
	{
		pNext = OldPath.m_cAllPath.GetPrev( pos ) ;
		//if( fabs( pPath->m_nLineNo - pNext->m_nLineNo ) > 2 || 
		//	fabs( pPath->m_nLineNo - pNext->m_nLineNo ) == 0 )
		if( fabs( (double)(pPath->m_nLineNo - pNext->m_nLineNo) ) > 2 || 
			fabs( (double)(pPath->m_nLineNo - pNext->m_nLineNo) ) < 1.0e-20 )
			continue ;
		if( !GetDistFromPt( pPath->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint, pNext->m_pHead->m_fPoint, dist1 ) )
			dist1 = mathFDist( pPath->m_pHead->m_fPoint, pNext->m_pHead->m_fPoint ) ;
		if( !GetDistFromPt( pPath->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint, pNext->m_pTail->m_fPoint,   dist2 ) )
			dist2 = mathFDist( pPath->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint   ) ;
		if( !GetDistFromPt( pNext->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint, pPath->m_pHead->m_fPoint, dist3 ) )
			dist3 = mathFDist( pNext->m_pHead->m_fPoint, pPath->m_pHead->m_fPoint ) ;
		if( !GetDistFromPt( pNext->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint, pPath->m_pTail->m_fPoint,   dist4 ) )
			dist4 = mathFDist( pNext->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint   ) ;
		
		if( dist1 < dist || dist2 < dist || 
			dist3 < dist || dist4 < dist )
			return 1 ;
	}
	return 0 ;
}
/*******************************************************
 * 将加工路径按照区域进行划分。即按照路径的最大包围盒 *
 * 进行处理。如果同一层中路径之间的最大包围盒距离大于 *
 * 刀具直径，则加工下一层。依次，然后回过来加工距离比 *
 * 较大的路径。                                       *
*******************************************************/
int  CSmartREmiterGen::SortPathByArea( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, 
									   CSmartTool* pMiller, JDNC_PRGDEF &ProgDef ) 
{
	CSmtCutPath * pPath ;
	POSITION pos, atpos ;
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		while( m_PathNew[i].m_cAllPath.GetCount() )
		{
			pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
			while( pos )
			{
				atpos = pos ;
				pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
				BOX3D box ;
				MathCAM_GetPathBox( pPath, box ) ;

				CSmtCPathLib PathLib ;
				PathLib.m_cAllPath.AddTail( pPath ) ;
				// 从这一层中删除该条路径
				m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
				// 从本层中寻找在同一区域中的路径,放到PathLib中
				GetAreaPathInSameLayer( m_PathNew[i], pMiller, &box, PathLib ) ;

				if( GenAreaPath( PGroup, PathLib, CheckMdl, pMiller, i + 1, ProgDef ) == 2 )
					return 0 ;
				break ;
			}
		}
	}
	return 1 ;
}

int  CSmartREmiterGen::GenAreaPath ( CPathGroup& PGroup, CSmtCPathLib& PrePathLib, CSmtCheckMdl& CheckMdl, 
										CSmartTool* pMiller, int i, JDNC_PRGDEF &ProgDef )
{
	double dDist = 10. * m_cFeedDef.m_cStepDef.m_dOverStep ;
	if( dDist > 20. ) 
		dDist = 20. ;
	else if( dDist < 3 * m_cFeedDef.m_cStepDef.m_dOverStep )
		dDist = 3 * m_cFeedDef.m_cStepDef.m_dOverStep ;
	// 递归退出条件1
	if( i >= m_nCount )
	{
		SortAreaPath( PrePathLib ) ;
		if( !ComputeProg( PrePathLib, ProgDef ) ) return 2 ;
		if( m_cParam.m_bEmitFlag & NCDEF_EMIT_ZIGZAG )
			PrePathLib.ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE ) ;
		AddPathToPG( PGroup, PrePathLib ) ;
		return 0 ;
	}
	// 从下一层中找出与PrePathLib在同一区域中的路径
	CSmtCPathLib CurPathLib ;
	GetAreaPathInNextLayer( PrePathLib, i, CurPathLib ) ;
	// 将上一层路径按照行号和Ｘ坐标排序.
	SortAreaPath( PrePathLib ) ;
	// 连接上一层路径PrePathLib，并加工之
	if( !ComputeProg( PrePathLib, ProgDef ) ) return 2 ;
	if( m_cParam.m_bEmitFlag & NCDEF_EMIT_ZIGZAG )
		PrePathLib.ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE,m_cParam.m_dCenter ) ;
	AddPathToPG( PGroup, PrePathLib ) ;
	// 递归退出条件2
	if( CurPathLib.m_cAllPath.GetCount() < 1 )
		return 0 ;
	// 中断
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
	{
		ClearAllPath( CurPathLib ) ;
		return 2 ;
	}
	CSmtCutPath * pPath ;
	POSITION pos, atpos ;
	while( CurPathLib.m_cAllPath.GetCount() )
	{
		pos = CurPathLib.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = CurPathLib.m_cAllPath.GetNext( pos ) ;
			
			BOX3D box ;
			MathCAM_GetPathBox( pPath, box ) ;
			
			CSmtCPathLib AreaPath ;
			AreaPath.m_cAllPath.AddTail( pPath ) ;
			//然后删除之
			CurPathLib.m_cAllPath.RemoveAt ( atpos ) ;
			// 从CurPathLib中寻找在同一区域的路径,放到AreaPath中
			GetAreaPathInSameLayer( CurPathLib, pMiller, &box, AreaPath ) ;
			// 整理路径,加工OldPath中路径
			if( GenAreaPath ( PGroup, AreaPath, CheckMdl, pMiller, i + 1, ProgDef ) == 2 )
				return 2 ;
			break ;
		}
	}
	return 1 ;
}

void CSmartREmiterGen::GetAreaPathInNextLayer ( CSmtCPathLib& PrePathLib, 
											  int i, CSmtCPathLib& CurPathLib )
{
	CSmtCutPath* pPath ;
	POSITION pos, atpos ;
	pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
	while ( pos )
	{
		atpos = pos ;
		pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
		if( IsSameAreaNextLayer( PrePathLib, pPath ) )
		{
			CurPathLib.m_cAllPath.AddTail( pPath ) ;
			m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
		}
	}
}

BOOL CSmartREmiterGen::IsSameAreaNextLayer ( CSmtCPathLib& PrePathLib, CSmtCutPath* pPath ) 
{
	DOUBLE NextMinX, NextMaxX, PathMinX, PathMaxX ;
	CSmtCutPath* pNext ;
	POSITION pos = PrePathLib.m_cAllPath.GetHeadPosition() ;
	while( pos ) 
	{
		pNext = PrePathLib.m_cAllPath.GetNext( pos ) ;
		if( pNext->m_nLineNo != pPath->m_nLineNo )
			continue ;
		PathMinX = min( pPath->m_pHead->m_fPoint[0], pPath->m_pTail->m_fPoint[0] ) ;
		PathMaxX = max( pPath->m_pHead->m_fPoint[0], pPath->m_pTail->m_fPoint[0] ) ;
		NextMinX = min( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ;
		NextMaxX = max( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ; 
		if( PathMinX >= NextMinX && NextMaxX >= PathMaxX )
			return 1 ;
	}
	return 0 ;
}

// 点到线段的最短距离
BOOL CSmartREmiterGen::GetDistFromPt ( FPNT3D start, FPNT3D end, 
								    FPNT3D pt, double& dist )
{
	double d ;
    VEC2D dir ;
	PNT2D o_pt ;

	dir[0] = end[0] - start[0] ;
	dir[1] = end[1] - start[1] ;
	
	d = dir[0] * dir[0] + dir[1]*dir[1] ;
	if (d < 1.0e-10 )	return 0 ;
    
	double t = ( ( pt[0] - start[0] ) * dir[0] + 
				 ( pt[1] - start[1] ) * dir[1] ) / d ;

	if ( t < MIN_DBL )
	{
		t = 0;
		o_pt[0] = start[0] ;
		o_pt[1] = start[1] ;
	}
	else if ( t > 1.0-MIN_DBL )
	{
		t = 1;
		o_pt[0] = end[0] ;
		o_pt[1] = end[1] ;
	}
	else 
	{
		o_pt[0] = start[0] + t * dir[0] ;
		o_pt[1] = start[1] + t * dir[1] ;
	}
	
	dist = sqrt( ( pt[0] - o_pt[0] ) * ( pt[0] - o_pt[0] ) +
				 ( pt[1] - o_pt[1] ) * ( pt[1] - o_pt[1] ) ) ;

	return 1 ;
}

void CSmartREmiterGen::SortAreaPath (CSmtCPathLib& OldPath ) 
{
	POSITION pos, atpos ;
	CSmtCutPath* pPath ;
	CSmtCPathLib PathLib ;
	while( OldPath.m_cAllPath.GetCount() > 0 )
	{
		CSmtCPathLib NewPath ;
		FindPrePath( OldPath, NewPath ) ;
		SortPathByDist ( NewPath ) ;
		pos = NewPath.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = NewPath.m_cAllPath.GetNext( pos ) ;
			NewPath.m_cAllPath.RemoveAt( atpos ) ;
			PathLib.m_cAllPath.AddTail( pPath ) ;
		}
		NewPath.m_cAllPath.RemoveAll() ;
	}
	pos = PathLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pPath = PathLib.m_cAllPath.GetNext( pos ) ;
		OldPath.m_cAllPath.AddTail( pPath ) ;
		PathLib.m_cAllPath.RemoveAt( atpos ) ;
	}
	PathLib.m_cAllPath.RemoveAll() ;
}

void CSmartREmiterGen::FindPrePath (CSmtCPathLib& OldPath, CSmtCPathLib& NewPath ) 
{
	POSITION pos, atpos, cur = NULL ;
	CSmtCutPath *pNext, *pPath = NULL ;
	// 首先找到最小的行号
	int MinLineNo = 10000000 ;
	pos = OldPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pNext = OldPath.m_cAllPath.GetNext( pos ) ;
		if( pNext->m_nLineNo < MinLineNo )
		{
			MinLineNo = pNext->m_nLineNo ;
			pPath = pNext ;
			cur = atpos ;
		}
	}
	if( pPath && cur)
	{
		OldPath.m_cAllPath.RemoveAt( cur ) ;
		NewPath.m_cAllPath.AddTail( pPath ) ;
		pos = OldPath.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pNext = OldPath.m_cAllPath.GetNext( pos ) ;
			if( pNext->m_nLineNo == MinLineNo )
			{
				OldPath.m_cAllPath.RemoveAt( atpos ) ;
				NewPath.m_cAllPath.AddTail( pNext ) ;
			}
		}
	}
	else
		return ;
}

void CSmartREmiterGen::SortPathByDist ( CSmtCPathLib& OldPath )
{
	CSmtCPathLib NewPath ;
	CSmtCutPath *pPath = NULL ;
	while( OldPath.m_cAllPath.GetCount() > 0 )
	{
		pPath = FindNearPathInSameAng( OldPath ) ;
		if( pPath )
			NewPath.m_cAllPath.AddTail( pPath ) ;
	}
	
	POSITION pos, atpos ;
	pos = NewPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pPath = NewPath.m_cAllPath.GetNext( pos ) ;
		NewPath.m_cAllPath.RemoveAt( atpos ) ;
		OldPath.m_cAllPath.AddTail( pPath ) ;
	}
	NewPath.m_cAllPath.RemoveAll() ;
}

CSmtCutPath* CSmartREmiterGen::FindNearPathInSameAng ( CSmtCPathLib& OldPath )
{
	if( OldPath.m_cAllPath.GetCount() < 1 )
		return NULL ;
	POSITION pos, atpos, cur = NULL;
	CSmtCutPath *pNext, *pPath = NULL ;
	DOUBLE dist, MinDist = 1.0e10 ;
	PNT2D from, to, center ;
	memcpy( center, m_cParam.m_dCenter, sizeof( PNT2D ) ) ;
	// 找到Ｘ坐标最小的条路径
	pos = OldPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pNext = OldPath.m_cAllPath.GetNext( pos ) ;
		nc_FloatToDouble( from, pNext->m_pHead->m_fPoint, 2 ) ;
		nc_FloatToDouble( to,   pNext->m_pTail->m_fPoint, 2 ) ;
		dist = min( mathDist2D( from, center ) , mathDist( to, center ) ) ;
		if( dist < MinDist )
		{
			pPath = pNext ;
			MinDist = dist ;
			cur = atpos ;
		}
	}
	if( pPath && cur)
	{
		OldPath.m_cAllPath.RemoveAt( cur ) ;
		return pPath ;
	}
	else
		return NULL ;
}

BOOL CSmartREmiterGen::ComputeProg( CSmtCPathLib& AllPath, JDNC_PRGDEF &ProgDef ) 
{
	POSITION pos ;
	CSmtCutPath* pPath ;
	pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		///////////////////////////////////////
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
		///////////////////////////////////////
		pPath = AllPath.m_cAllPath.GetNext( pos ) ;
	}
	return TRUE ;
}

void CSmartREmiterGen::ReversePath ( CSmtCPathLib& PathLib )
{
	CSmtCutPath* pPath = NULL ;
	POSITION pos = PathLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		pPath = PathLib.m_cAllPath.GetNext( pos ) ;
		pPath->ReverseDirect() ;
		pPath->DefineBox() ;
	}
	PathLib.DefineBox() ;
}

CSmartRUWLineGen::CSmartRUWLineGen()
{
	m_nCutMode = 0 ;
	m_nCount   = 0 ;
	m_PathNew  = NULL ;
	m_pRoughTool = NULL ;
	m_cTmpPrgDef.m_dLimitAt = 0. ;
	m_cTmpPrgDef.m_dStepAt  = 0 ;
	m_cTmpPrgDef.m_dIncStep = 1. ;
	m_cTmpPrgDef.m_pBrkFunc = NULL ;
	m_cTmpPrgDef.m_pPrgFunc = NULL ;
	m_cTmpPrgDef.m_pPosFunc = NULL ;
}


CSmartRUWLineGen::~CSmartRUWLineGen()
{
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
		m_PathNew[i].ClearAllPath() ;
	if( m_nCount ) delete []m_PathNew ;
	if( m_pRoughTool ) delete m_pRoughTool ;
	m_pRoughTool = NULL ;
}

int CSmartRUWLineGen::LayerAllPath( CSmtCPathLib& AllPath, double dTop, double dBot, BOX3D &sbox, JDNC_PRGDEF &ProgDef, double dCur ) /*所有路径*/
{
	if( GetProjDownStep( dBot, dTop, m_cFeedDef ) < 0.002 ) 
	{
		m_nCount = 0 ;
		return 0 ;
	}
	DOUBLE dZValue =  dTop - GetProjDownStep( dBot, dTop, m_cFeedDef ) ;
	DOUBLE dMaxDepth = dBot ;
	
	double mode = ( dTop - dBot )  / GetProjDownStep( dBot, dTop, m_cFeedDef ) ;
	m_nCount = ( int ) ( mode ) ;
	if( mode - m_nCount > 0. )
		m_nCount += 1 ;
	if( m_cFeedDef.m_cLayerDef.m_nLayerType < 1 ) 
		m_nCount = 1 ;
	m_dDownStep = ( dTop - dBot ) / m_nCount ;
	dZValue = dTop ;
	if( m_nCount < 1 ) m_nCount = 1 ;

	ProgDef.m_dLimitAt =  m_nCount / dCur ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	m_PathNew = new CNcSmartPath[m_nCount] ;
	int i = -1 ;

	// 拷贝分层参数, bFlag 1:拷贝分层;
	int bFlag = m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE ;

	while( dZValue >= dMaxDepth )
	{
		dZValue -= m_dDownStep ;
		i++ ;
		if( i == m_nCount )
			break ;
		//////////////计算进度条////////////////////////
		//////////////计算进度条////////////////////////
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
		////////////////////////////////////////////////	
		CNcSmartPath cs ;
		// 如果是拷贝分层，计算路径抬起的高度
		double h = dZValue - dMaxDepth ;
		
		if( bFlag )
		{
			if( m_nCutMode )
				cs.TrimByZValue3 ( AllPath, m_PathNew[i], sbox.max[2], h ) ;
			else
				cs.TrimByZValue1( AllPath, m_PathNew[i], dTop, h ) ;
		}
		else
		{
			if( m_nCutMode ) // 因为路径的高度为相对高度，所以传入的高度也为相对高度
				cs.TrimByZValue2 ( AllPath, m_PathNew[i], dZValue - sbox.max[2], m_nCutMode ) ;
			else
				cs.TrimByZValue0( AllPath, m_PathNew[i], dZValue ) ;
		}
		POSITION pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			CSmtCutPath* pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;	
			pPath->m_nLayerNo = i ;
		}
	}
	return 1 ; 
}
BOOL CSmartRUWLineGen::BuildStockMdl ( CSmartGraphic &Graph, CSmtCheckMdl &RoughMdl, BOX3D &box )
{
	////////////////////处理毛坯情况////////////////
	// 首先生产加工面模型
	if( !Graph.BuildStockModel ( m_cSetupDef, RoughMdl, box ) )
	{
		return FALSE ;
	}
	int nRoughType = Graph.GetRoughType () ;
	double dStock = Graph.GetRoughStock () ;
	if( nRoughType == ncStockShapeSurf && fabs( dStock ) > 0.0001 )
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, dStock, 0 ) ;
		RoughMdl.ZMoveModel ( dStock ) ;
	}
	else
	{
		m_pRoughTool = CreateSmartTool( m_cToolDef, 0., 0 ) ;
	}
	RoughMdl.UpdateCurrTool( m_pRoughTool )  ;
	RoughMdl.m_fBottom =  (FLOAT)GetBottomHeight() ;
	return TRUE ;
}
int CSmartRUWLineGen::GeneratePathEx( CPathGroup& NewPath    ,  /*加工路径*/ 
						              CSmartGraphic& Graph )  /*雕刻图形*/
{ // 生成刀具路径 
	// STEP 0 : 转换刀具 
	if( !UpdateTool3D(Graph) ) return 0;
	if (Graph.FindNetSurf( NCDEF_SURF_DRIVE ))
	{// 网格面不能生成曲面流线路径
		m_nErrorType = JDERROR_GENPATH_NETSURFUW ;
		return FALSE ;
	}

	m_nCutMode = 1 ;
	// STEP 1 : 提取曲面图形 , 并声称干涉检查曲面
	BOOL			bAutoCheck = TRUE ;
	BOOL			bCheck = Graph.HasCheckSurf () ;

	CSmtCheckMdl  DriveMdl, CheckMdl, RoughMdl ; ;
	BOX3D sbox, MaxBox ;
	mathClnBox3D( &sbox ), mathClnBox3D( & MaxBox ) ;

	if( !BuildStockMdl( Graph, RoughMdl, sbox ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSTOCK ;
		return 0 ;
	}
	double dTop = GetTopHeight(), dBot = GetBottomHeight() ;
	CalcTopAndBot( m_cShapeDef, sbox, dTop, dBot, GetDriveSurfZMove(), m_cSetupDef.m_cTolDef.m_dArcTol ) ;
	if( !BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return 0 ;
	}
	DriveMdl.DefineDriveSurfBox( MaxBox.min, MaxBox.max ) ;
	if( MaxBox.max[0] < MaxBox.min[0] || 
		MaxBox.max[1] < MaxBox.min[1] )
	{ // 不能确定加工的范围
        m_nErrorType =  JDERROR_GENPATH_NOSURFACE  ;
		return 0 ;
	}
	mathCalBox3DUnion( &MaxBox, &sbox, &MaxBox ) ;
	mathExpandBox3D( m_pRoughTool->m_fRadius * 1.5, &MaxBox ) ;

	// STEP 2 : 生成曲面流线路径, 不采用往复走刀
	DriveMdl.UpdateCurrTool( m_pTool ) ;
	int				nRet = 0 ;
	CSmtCPathLib	AllPath  ;
	// 生成曲面流线路径
	ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_UWLINE ) ;

	int bFlag = m_cParam.m_bUWLineFlag ;
	double dCur[4] = { 30, 10, 20, 40 } ;
	C3DSurfArray    AllSurf ;
	if( !Graph.GetAllSurface( m_cSetupDef, AllSurf )     )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
		return 0 ;
	}
	double dCur1 = dCur[0] * 0.4, dCur2 = dCur[0] * 0.3, dCur3 = dCur[0] * 0.3 ;
	if( !bCheck && !bAutoCheck ) 
	{
		dCur1 = dCur[0], dCur2 = dCur3 = 0 ;
	}
	else if( bCheck && !bAutoCheck )
	{
		dCur1 = dCur[0] * 0.6, dCur2 = 0, dCur3 = dCur[0] *0.4 ;
	}
	else if( !bCheck && bAutoCheck )
	{
		dCur1 = dCur[0] * 0.6, dCur3 = 0, dCur2 = dCur[0] *0.4 ;
	}
	CFlowlineGen	cNcGen( m_cSetupDef, m_cFeedDef, m_cStockDef, m_cToolDef, m_cSpeedDef, m_pTool, m_nCalcThreadNum, m_cParam) ;
	// 设置逐个加工标记
	RFRAME dNcMtx ;
	if( !Graph.GetTPlaneFrame ( dNcMtx ) )
		mathInitRFrame( &dNcMtx ) ;
	JDNC_UWLINE cUWLine = m_cParam ;
	if( cUWLine.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG )
		cUWLine.m_bUWLineFlag &= ~NCDEF_UWLINE_ZIGZAG ;
	nRet = cNcGen.CreateUWLinePath( AllPath, AllSurf, dNcMtx, cUWLine, m_cPrgDef, dCur1, &m_nErrorType) ;
	DriveMdl.m_fBottom -= float(m_pTool->m_fHeight) ;
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return 0 ;
	}
	// 如果选择了自动干涉检查
	JDNC_TOL Tol = GetSurfaceTol() ;
	if( bAutoCheck )
	{
		DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
		cNcGen.CheckAllPathBySelf( DriveMdl, AllPath, Tol, m_cPrgDef, dCur2, &m_nErrorType) ;
		//cNcGen.CheckAllPathByCheckMdl( DriveMdl, AllPath, Tol, TRUE, TRUE,m_cPrgDef, dCur2, &m_nErrorType) ;
		if( SurfNC_IsAbort() )
		{
			m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return 0 ;
		}
	}

	if( bCheck )
	{
		CheckMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
		CheckMdl.m_fBottom -= 1. ;
		cNcGen.CheckAllPathBySurf( CheckMdl, AllPath, Tol, TRUE, m_cPrgDef, dCur3, &m_nErrorType) ;
		//cNcGen.CheckAllPathByCheckMdl( CheckMdl, AllPath, Tol, TRUE, FALSE, m_cPrgDef, dCur3, &m_nErrorType) ;
		if( SurfNC_IsAbort() )
		{
			m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return 0 ;
		}
	}/**/
	INT_PTR nPnt = AllPath.m_cAllPath.GetCount() ;
	if( nPnt < 1 ) nPnt = 1 ;
	m_cPrgDef.m_dLimitAt =  nPnt / dCur[1] ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dIncStep = 1.0  ;
	RoughMdl.m_fBottom = DriveMdl.m_fBottom = float( dBot ) ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		m_cPrgDef.m_dStepAt += m_cPrgDef.m_dIncStep ;
		while( m_cPrgDef.m_pPrgFunc && m_cPrgDef.m_dStepAt >= m_cPrgDef.m_dLimitAt )
		{
			m_cPrgDef.m_pPrgFunc(1) ;
			m_cPrgDef.m_dStepAt -= m_cPrgDef.m_dLimitAt  ;
		}
		CSmtCutPath* pPath = AllPath.m_cAllPath.GetNext( pos ) ;
		SetCutPathZ( pPath, 0 ) ;
		RoughMdl.LabelCheckByBox ( pPath->m_fBox ) ;
		pPath->VerifyCutPath( RoughMdl, GetSurfaceTol() , m_cTmpPrgDef ) ;
		SetCutPathZ( pPath, 1 ) ;
	}

	m_cParam.m_bUWLineFlag = bFlag ;

	if( !nRet )
	{
        m_nErrorType =  JDERROR_GENPATH_ERRDATA  ;
		return 0 ; 
	}
	// 连刀的时候不投影到曲面上，直接连过去
	AllPath.SetCutMode ( MINI_CONNECT_PATH ) ;
    // STEP 3 : 分层计算路径
	/////////////////////第一条进度条/////////////////////////
    if( !LayerAllPath( AllPath, dTop, dBot, sbox, m_cPrgDef, dCur[2] ) )
	{
        m_nErrorType =  JDERROR_GENPATH_ERRDATA  ;
		return 0 ; 
	}
	////////////////////第二条进度条//////////////////////
	m_nTotal = 0 ;
	for( int j = 0 ; j < m_nCount ; j++ )
	{
		TrimAllPathByZValue( m_PathNew[j], dTop, FALSE ) ;
		int nCount = (int)m_PathNew[j].m_cAllPath.GetCount() ;
		m_nTotal += nCount ;
	}
	if( m_nTotal < 1 )m_nTotal = 1 ;
	m_nCur = 1 ;
	m_cPrgDef.m_dLimitAt =  m_nTotal / dCur[3] ;
	m_cPrgDef.m_dStepAt  = 0.0 ;
	m_cPrgDef.m_dIncStep = 1.0  ;
	////////////////////////////////////////////////////
	DriveMdl.m_bCheckMode = SMART_MODEL_PROJECT ;
	if( GetOrderDef().m_nLayerOrd )/* ==1 高度优先 */
	{
		if( !SortPathByDepth( NewPath, DriveMdl, m_cPrgDef ) )
			NewPath.ClearAll() ;
	}
	else /* == 0区域优先 */
	{
		if( !SortPathByArea( NewPath, DriveMdl, m_pTool, m_cPrgDef ) )
		{
			NewPath.ClearAll() ;
		}
	}
    if( SurfNC_IsAbort() )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT  ;
        return 0 ;
    }
	if( AllPath.GetNumPoint() ) 
	{
		AllPath.ClearAllPath() ;
	}
	// 添加慢速和相对高度连接
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
		AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE) ;
	return 1 ;
}

/*********************************************************
 * 按照加工路径的高度对路径进行处理，首先加工同一高度的 *
 * 路径。                                               *
*********************************************************/
int  CSmartRUWLineGen::SortPathByDepth( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, JDNC_PRGDEF &ProgDef )
{
	double dDist = 10. * m_cParam.m_dOverStep ;
	if( dDist > 20. ) 
		dDist = 20. ;
	else if( dDist < 3 * m_cParam.m_dOverStep )
		dDist = 3 * m_cParam.m_dOverStep ;

	POSITION pos , atpos ;
	CSmtCutPath* pPath ;
	for( INT_PTR i = 0 ; i < m_nCount ; i++ )
	{
		if( !ComputeProg( m_PathNew[i], ProgDef ) ) return 2 ;
		if( m_cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG )
			m_PathNew[i].ConnectPathByLineNo( CheckMdl, dDist, m_cSetupDef.m_cTolDef,m_cTmpPrgDef, TRUE ) ;
		AddPathToPG(PGroup, m_PathNew[i] ) ;
		pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
			m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
			delete pPath ;
		}
	}
	return 1 ;
}
/*********************************************************
 * 将CSmtCutPath类型的路径转换为CPathCombine类型，并放到 *
 * PGroup当中去。                                        *
*********************************************************/
int CSmartRUWLineGen::AddPathToPG ( CPathGroup& PGroup,
								  CSmtCPathLib& OldPath )
{
	POSITION pos ;
	CSmtCutPath* pPath ;
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	pos = OldPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		pPath = OldPath.m_cAllPath.GetNext( pos ) ;
		pPath->AddToPathCombine ( *pPComb ) ;
	}
	PGroup.AddData( 0.0, pPComb ) ;
	return 1 ;
}

void CSmartRUWLineGen::ClearAllPath ( CSmtCPathLib& OldPath )
{
	POSITION pos, atpos ;
	CSmtCutPath* pPath ;
	pos = OldPath.m_cAllPath.GetHeadPosition () ;
	while( pos )
	{
		atpos = pos ;
		pPath = OldPath.m_cAllPath.GetNext( pos ) ;
		OldPath.m_cAllPath.RemoveAt( atpos ) ;
		delete pPath ;
	}
}

void CSmartRUWLineGen::GetAreaPathInSameLayer( CSmtCPathLib& OldPath, CSmartTool* pMiller, 
											 BOX3D* box, CSmtCPathLib& NewPath ) 
{
	CSmtCutPath *pNext ;
	POSITION atPos, atCurr ;
	DOUBLE dist = 3.0 * pMiller->m_fRadius ;
	DOUBLE dist1 = 2 * m_cParam.m_dOverStep ; //
	dist = max( dist, dist1 ) ;

	if( !OldPath.m_cAllPath.GetCount () )
		return  ;
	while( OldPath.m_cAllPath.GetCount() ) 
	{
		atCurr = OldPath.m_cAllPath.GetHeadPosition() ;
		int nCount = 0 ;
		while( atCurr )
		{
			atPos = atCurr ;
			pNext = OldPath.m_cAllPath.GetNext ( atCurr ) ;
			
			BOX3D box1 ;
			MathCAM_GetPathBox( pNext, box1 ) ;
			//两段路径的包围盒相差小于刀具直径
			//包围盒间距大于刀具直径，肯定处于不同的区域
			if( mathChkBox3DInt( box , &box1 , 2 * dist ) ) 
			{
				if( PathInSameArea( NewPath, pMiller, pNext ) )
				{
					//扩大包围盒
					mathCalBox3DUnion( box, &box1 , box) ;
					NewPath.m_cAllPath.AddTail( pNext ) ;
					OldPath.m_cAllPath.RemoveAt ( atPos ) ;
					nCount++ ;
				}
			}		
 		}
		if( !nCount )
			break ;
	}
	return  ;
}

BOOL CSmartRUWLineGen::PathInSameArea( CSmtCPathLib& OldPath, CSmartTool* pMiller, CSmtCutPath* pPath ) 
{
	CSmtCutPath* pNext ;
	DOUBLE dist = 3.0 * pMiller->m_fRadius ;
	DOUBLE dist0 = 2 * m_cParam.m_dOverStep ;
	dist = min( dist, dist0 ) ;
	if( dist < 1. )
		dist = 1. ;

	DOUBLE dist1, dist2, dist3, dist4 ;
	POSITION pos = OldPath.m_cAllPath.GetTailPosition() ;
	while( pos )
	{
		pNext = OldPath.m_cAllPath.GetPrev( pos ) ;
		//if( fabs( pPath->m_nLineNo - pNext->m_nLineNo ) > 2 || 
		//	fabs( pPath->m_nLineNo - pNext->m_nLineNo ) == 0 )
		if( fabs( (double)(pPath->m_nLineNo - pNext->m_nLineNo) ) > 2 || 
			fabs( (double)(pPath->m_nLineNo - pNext->m_nLineNo) ) < 1.0e-20 )
			continue ;
		if( !GetDistFromPt( pPath->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint, pNext->m_pHead->m_fPoint, dist1 ) )
			dist1 = mathFDist( pPath->m_pHead->m_fPoint, pNext->m_pHead->m_fPoint ) ;
		if( !GetDistFromPt( pPath->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint, pNext->m_pTail->m_fPoint,   dist2 ) )
			dist2 = mathFDist( pPath->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint   ) ;
		if( !GetDistFromPt( pNext->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint, pPath->m_pHead->m_fPoint, dist3 ) )
			dist3 = mathFDist( pNext->m_pHead->m_fPoint, pPath->m_pHead->m_fPoint ) ;
		if( !GetDistFromPt( pNext->m_pHead->m_fPoint, pNext->m_pTail->m_fPoint, pPath->m_pTail->m_fPoint,   dist4 ) )
			dist4 = mathFDist( pNext->m_pHead->m_fPoint, pPath->m_pTail->m_fPoint   ) ;
		
		if( dist1 < dist || dist2 < dist || 
			dist3 < dist || dist4 < dist )
			return 1 ;
	}
	return 0 ;
}
/*******************************************************
 * 将加工路径按照区域进行划分。即按照路径的最大包围盒 *
 * 进行处理。如果同一层中路径之间的最大包围盒距离大于 *
 * 刀具直径，则加工下一层。依次，然后回过来加工距离比 *
 * 较大的路径。                                       *
*******************************************************/
int  CSmartRUWLineGen::SortPathByArea( CPathGroup& PGroup, CSmtCheckMdl& CheckMdl, 
									   CSmartTool* pMiller, JDNC_PRGDEF &ProgDef ) 
{
	CSmtCutPath * pPath ;
	POSITION pos, atpos ;
	for( int i = 0 ; i < m_nCount ; i++ )
	{
		while( m_PathNew[i].m_cAllPath.GetCount() )
		{
			pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
			while( pos )
			{
				atpos = pos ;
				pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
				BOX3D box ;
				MathCAM_GetPathBox( pPath, box ) ;

				CSmtCPathLib PathLib ;
				PathLib.m_cAllPath.AddTail( pPath ) ;
				// 从这一层中删除该条路径
				m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
				// 从本层中寻找在同一区域中的路径,放到PathLib中
				GetAreaPathInSameLayer( m_PathNew[i], pMiller, &box, PathLib ) ;

				PathLib.DefineBox() ;
				if( GenAreaPath( PGroup, PathLib, CheckMdl, pMiller, i + 1, ProgDef ) == 2 )
					return 0 ;
				break ;
			}
		}
	}
	return 1 ;
}

int  CSmartRUWLineGen::GenAreaPath ( CPathGroup& PGroup, CSmtCPathLib& PrePathLib, CSmtCheckMdl& CheckMdl, 
										CSmartTool* pMiller, int i, JDNC_PRGDEF &ProgDef  )
{
	double dDist = 10. * m_cParam.m_dOverStep ;
	if( dDist > 20. ) 
		dDist = 20. ;
	else if( dDist < 3 * m_cParam.m_dOverStep )
		dDist = 3 * m_cParam.m_dOverStep ;

	// 递归退出条件1
	if( i >= m_nCount )
	{
		SortAreaPath( PrePathLib ) ;
		if( !ComputeProg( PrePathLib, ProgDef ) ) return 2 ;
		if( m_cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG )
			PrePathLib.ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE ) ;
		AddPathToPG( PGroup, PrePathLib ) ;
		return 0 ;
	}
	// 从下一层中找出与PrePathLib在同一区域中的路径
	CSmtCPathLib CurPathLib ;
	GetAreaPathInNextLayer( PrePathLib, i, CurPathLib ) ;
	// 将上一层路径按照行号和Ｘ坐标排序.
	SortAreaPath( PrePathLib ) ;
	// 连接上一层路径PrePathLib，并加工之
	if( !ComputeProg( PrePathLib, ProgDef ) ) return 2 ;
	if( m_cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG )
		PrePathLib.ConnectPathByLineNo( CheckMdl, dDist,m_cSetupDef.m_cTolDef, m_cTmpPrgDef, TRUE ) ;
	AddPathToPG( PGroup, PrePathLib ) ;
	// 递归退出条件2
	if( CurPathLib.m_cAllPath.GetCount() < 1 )
		return 0 ;
	// 中断
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
	{
		ClearAllPath( CurPathLib ) ;
		return 2 ;
	}
	CSmtCutPath * pPath ;
	POSITION pos, atpos ;
	while( CurPathLib.m_cAllPath.GetCount() )
	{
		pos = CurPathLib.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = CurPathLib.m_cAllPath.GetNext( pos ) ;
			
			BOX3D box ;
			MathCAM_GetPathBox( pPath, box ) ;
			
			CSmtCPathLib AreaPath ;
			AreaPath.m_cAllPath.AddTail( pPath ) ;
			//然后删除之
			CurPathLib.m_cAllPath.RemoveAt ( atpos ) ;
			// 从CurPathLib中寻找在同一区域的路径,放到AreaPath中
			GetAreaPathInSameLayer( CurPathLib, pMiller, &box, AreaPath ) ;
			// 整理路径,加工OldPath中路径
			if( GenAreaPath ( PGroup, AreaPath, CheckMdl, pMiller, i + 1, ProgDef ) == 2 )
				return 2 ;
			break ;
		}
	}
	return 1 ;
}

void CSmartRUWLineGen::GetAreaPathInNextLayer ( CSmtCPathLib& PrePathLib, 
											  int i, CSmtCPathLib& CurPathLib )
{
	CSmtCutPath* pPath ;
	POSITION pos, atpos ;
	pos = m_PathNew[i].m_cAllPath.GetHeadPosition() ;
	while ( pos )
	{
		atpos = pos ;
		pPath = m_PathNew[i].m_cAllPath.GetNext( pos ) ;
		if( IsSameAreaNextLayer( PrePathLib, pPath ) )
		{
			CurPathLib.m_cAllPath.AddTail( pPath ) ;
			m_PathNew[i].m_cAllPath.RemoveAt( atpos ) ;
		}
	}
}

BOOL CSmartRUWLineGen::IsSameAreaNextLayer ( CSmtCPathLib& PrePathLib, CSmtCutPath* pPath ) 
{
	DOUBLE NextMinX, NextMaxX, PathMinX, PathMaxX ;
	CSmtCutPath* pNext ;
	POSITION pos = PrePathLib.m_cAllPath.GetHeadPosition() ;
	while( pos ) 
	{
		pNext = PrePathLib.m_cAllPath.GetNext( pos ) ;
		if( pNext->m_nLineNo != pPath->m_nLineNo )
			continue ;
		PathMinX = min( pPath->m_pHead->m_fPoint[0], pPath->m_pTail->m_fPoint[0] ) ;
		PathMaxX = max( pPath->m_pHead->m_fPoint[0], pPath->m_pTail->m_fPoint[0] ) ;
		NextMinX = min( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ;
		NextMaxX = max( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ; 
		if( PathMinX >= NextMinX && NextMaxX >= PathMaxX )
			return 1 ;
	}
	return 0 ;
}

// 点到线段的最短距离
BOOL CSmartRUWLineGen::GetDistFromPt ( FPNT3D start, FPNT3D end, 
								    FPNT3D pt, double& dist )
{
	double d ;
    VEC2D dir ;
	PNT2D o_pt ;

	dir[0] = end[0] - start[0] ;
	dir[1] = end[1] - start[1] ;
	
	d = dir[0] * dir[0] + dir[1]*dir[1] ;
	if (d < 1.0e-10 )	return 0 ;
    
	double t = ( ( pt[0] - start[0] ) * dir[0] + 
				 ( pt[1] - start[1] ) * dir[1] ) / d ;

	if ( t < MIN_DBL )
	{
		t = 0;
		o_pt[0] = start[0] ;
		o_pt[1] = start[1] ;
	}
	else if ( t > 1.0-MIN_DBL )
	{
		t = 1;
		o_pt[0] = end[0] ;
		o_pt[1] = end[1] ;
	}
	else 
	{
		o_pt[0] = start[0] + t * dir[0] ;
		o_pt[1] = start[1] + t * dir[1] ;
	}
	
	dist = sqrt( ( pt[0] - o_pt[0] ) * ( pt[0] - o_pt[0] ) +
				 ( pt[1] - o_pt[1] ) * ( pt[1] - o_pt[1] ) ) ;

	return 1 ;
}

void CSmartRUWLineGen::SortAreaPath (CSmtCPathLib& OldPath ) 
{
	POSITION pos, atpos ;
	CSmtCutPath* pPath ;
	CSmtCPathLib PathLib ;
	while( OldPath.m_cAllPath.GetCount() > 0 )
	{
		CSmtCPathLib NewPath ;
		FindPrePath( OldPath, NewPath ) ;
		SortPathByDist ( NewPath ) ;
		pos = NewPath.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pPath = NewPath.m_cAllPath.GetNext( pos ) ;
			NewPath.m_cAllPath.RemoveAt( atpos ) ;
			PathLib.m_cAllPath.AddTail( pPath ) ;
		}
		NewPath.m_cAllPath.RemoveAll() ;
	}
	pos = PathLib.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pPath = PathLib.m_cAllPath.GetNext( pos ) ;
		OldPath.m_cAllPath.AddTail( pPath ) ;
		PathLib.m_cAllPath.RemoveAt( atpos ) ;
	}
	PathLib.m_cAllPath.RemoveAll() ;
}

void CSmartRUWLineGen::FindPrePath (CSmtCPathLib& OldPath, CSmtCPathLib& NewPath ) 
{
	POSITION pos, atpos, cur = NULL;
	CSmtCutPath *pNext, *pPath = NULL ;
	// 首先找到最小的行号
	int MinLineNo = 10000000 ;
	pos = OldPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pNext = OldPath.m_cAllPath.GetNext( pos ) ;
		if( pNext->m_nLineNo < MinLineNo )
		{
			MinLineNo = pNext->m_nLineNo ;
			pPath = pNext ;
			cur = atpos ;
		}
	}
	if( pPath && cur)
	{
		OldPath.m_cAllPath.RemoveAt( cur ) ;
		NewPath.m_cAllPath.AddTail( pPath ) ;
		pos = OldPath.m_cAllPath.GetHeadPosition() ;
		while( pos )
		{
			atpos = pos ;
			pNext = OldPath.m_cAllPath.GetNext( pos ) ;
			if( pNext->m_nLineNo == MinLineNo )
			{
				OldPath.m_cAllPath.RemoveAt( atpos ) ;
				NewPath.m_cAllPath.AddTail( pNext ) ;
			}
		}
	}
	else
		return ;
}

void CSmartRUWLineGen::SortPathByDist ( CSmtCPathLib& OldPath )
{
	CSmtCPathLib NewPath ;
	CSmtCutPath *pPath = NULL ;
	while( OldPath.m_cAllPath.GetCount() > 0 )
	{
		pPath = FindLeftPathInSameLine( OldPath ) ;
		if( pPath )
			NewPath.m_cAllPath.AddTail( pPath ) ;
	}
	
	POSITION pos, atpos ;
	pos = NewPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pPath = NewPath.m_cAllPath.GetNext( pos ) ;
		NewPath.m_cAllPath.RemoveAt( atpos ) ;
		OldPath.m_cAllPath.AddTail( pPath ) ;
	}
	NewPath.m_cAllPath.RemoveAll() ;
}

CSmtCutPath* CSmartRUWLineGen::FindLeftPathInSameLine ( CSmtCPathLib& OldPath )
{
	if( OldPath.m_cAllPath.GetCount() < 1 )
		return NULL ;
	POSITION pos, atpos, cur= NULL ;
	CSmtCutPath *pNext, *pPath = NULL ;
	DOUBLE xcoord, MinX = 1.0e10 ;
	// 找到Ｘ坐标最小的条路径
	pos = OldPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		atpos = pos ;
		pNext = OldPath.m_cAllPath.GetNext( pos ) ;
		xcoord = min( pNext->m_pHead->m_fPoint[0], pNext->m_pTail->m_fPoint[0] ) ;
		if( xcoord < MinX )
		{
			pPath = pNext ;
			MinX = xcoord ;
			cur = atpos ;
		}
	}
	if( pPath && cur)
	{
		OldPath.m_cAllPath.RemoveAt( cur ) ;
		return pPath ;
	}
	else
		return NULL ;
}

BOOL CSmartRUWLineGen::ComputeProg( CSmtCPathLib& AllPath, JDNC_PRGDEF &ProgDef ) 
{
	POSITION pos ;
	CSmtCutPath* pPath ;
	pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( pos )
	{
		///////////////////////////////////////
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			return FALSE ;
		ProgDef.m_dStepAt += ProgDef.m_dIncStep ;
		while( ProgDef.m_pPrgFunc && ProgDef.m_dStepAt >= ProgDef.m_dLimitAt )
		{
			ProgDef.m_pPrgFunc(1) ;
			ProgDef.m_dStepAt -= ProgDef.m_dLimitAt  ;
		}
		///////////////////////////////////////
		pPath = AllPath.m_cAllPath.GetNext( pos ) ;
	}
	return TRUE ;
}
