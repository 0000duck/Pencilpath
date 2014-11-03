#include "StdAfx.H"
#include "SmartNC.H"
#include "SurfGeo.H"
#include "SmartBound.H"
#include "SmartVoro.H"
#include "SmtPathGen.H"
#include "SmtThreadMillGen.H"
#include "NcGCode.H"
#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

//////////////////////////////
CSmartThreadMillGen::CSmartThreadMillGen()
{
}
CSmartThreadMillGen::~CSmartThreadMillGen()
{
}

// 计算铣螺纹路径
BOOL CSmartThreadMillGen::GeneratePathEx( CPathGroup& NewPath ,   /*加工路径*/ 
										  CSmartGraphic& Graph )  /*图形*/
{
	if ( m_cParam.m_nDepthType != NCDEF_THREADMILL_DEPTH_THREADLENGTH ) //增加m_cParam.m_nDepthType变量后，以前的文件读进来修正无效值
		m_cParam.m_nDepthType = NCDEF_THREADMILL_DEPTH_PATHDEPTH ;
	
	m_dModiNormDiam = m_cParam.m_dNomDiameter - 2 * m_cStockDef.m_dSideStock ;
	m_dModiMinDiam = m_cParam.m_dMinDiameter + 2 * m_cStockDef.m_dSideStock ;
	m_bNewCutPoint = FALSE;
	// STEP 0 : 校正雕刻环境
	if ( m_cToolDef.m_nToolType != surfncToolTap )
	{
		m_nErrorType = JDERROR_GENPATH_TAPTOOL ;
		return FALSE ;
	}

	if ( fabs ( m_cToolDef.m_dPitch - m_cParam.m_dPitch ) > MIN_DIS )
	{
		m_nErrorType = JDERROR_GENPATH_PITCHDIF ;
		return FALSE ;
	}

	if ( m_cToolDef.m_dLength < GetCutDepth() + 0.01 
		|| ( ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALRIGHT 
				|| m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALLEFT )
			&& m_cToolDef.m_dTopDiam > m_cParam.m_dHoleDiameter - 0.01 ) )
	{
		m_nErrorType = JDERROR_GENPATH_ERRDATA ;
		return FALSE ;
	}

	CPtrList allEnt ;
	Graph.GetAllEntity ( m_cSetupDef, allEnt, SMARTGRAPH_TYPE_POINT ) ; 
	if ( m_cParam.m_cPointDef.m_nPointType != NCDEF_TPOINT_STEP )
	{ // 不是线上取点, 和普通点一样。
        Graph.GetFeaturePoint ( m_cSetupDef, allEnt, m_cParam.m_cPointDef ) ;
	}
	CPointList allPoint ;
	CSmartPoint* pPoint ;
	POSITION pos = allEnt.GetHeadPosition() ;
	while ( pos ) 
	{
		pPoint = ( CSmartPoint* ) allEnt.GetNext( pos ) ;
		if ( !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_REDEPTH ) )
		{ /*不保留孔位高度*/
			pPoint->m_dPoint[2] = 0.0 ;
		}
		if ( /*!( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_FLTSAME ) ||*/
			allPoint.FindPoint ( pPoint->m_dPoint ) < 0  )
		{
			allPoint.InsertAfter ( pPoint, allPoint.GetTail() ) ;  
		}
		else
		{ // 删除重点
			delete pPoint ;
		}
	} 
    allEnt.RemoveAll() ;
    if ( GetSortType() != NCDEF_SORTTYPE_SELECT )
    {
		allPoint.ResortAllPoint ( GetSortType(), TRUE, float( m_cToolDef.m_dTopDiam * 0.5f) ) ;
    }
	if ( m_cParam.m_cPointDef.m_nPointType == NCDEF_TPOINT_STEP )
	{ // 线上取点 , 不排序
        allEnt.RemoveAll() ;
	    Graph.GetFeaturePoint ( m_cSetupDef, allEnt, m_cParam.m_cPointDef ) ;
	    pos = allEnt.GetHeadPosition() ;
	    while ( pos ) 
		{ 
			pPoint = ( CSmartPoint* ) allEnt.GetNext ( pos ) ;
			if ( !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_REDEPTH ) )
			{ /*不保留孔位高度*/
				pPoint->m_dPoint[2] = 0.0 ;
			}
			if ( /*!( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_FLTSAME ) ||*/
				  allPoint.FindPoint( pPoint->m_dPoint ) < 0  )
			{
				allPoint.InsertAfter ( pPoint, allPoint.GetTail() ) ;  
			}
			else
			{ // 删除重点
				delete pPoint ;
			}
		} 
	}
	if ( allPoint.GetNum () == 0  )
	{
		m_nErrorType = JDERROR_GENPATH_NOPOINT ;
		return FALSE ;
	}

	if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALRIGHT 
		|| m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALLEFT )
		return CreatInternalPath ( allPoint, NewPath ) ;
	else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALRIGHT 
		|| m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALLEFT )
		return CreatExternalPath ( allPoint, NewPath ) ;
	
	return FALSE ;
}

BOOL CSmartThreadMillGen::CreatInternalPath ( CSmartPoint & Point, CPathCombine & PComb)
{
	// STEP 1 : 计算路径
	double dNomRad = 0.5 * m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ ;
	double dRadius = dNomRad - 0.5 * m_cToolDef.m_dTopDiam ;
	if ( dRadius < 0.02 || m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ < m_cParam.m_dHoleDiameter + 0.02 )
	{
		m_nErrorType = JDERROR_GENPATH_ERRDATA ;
		return FALSE ;
	}
	double dMatRad = 0.5 * m_cParam.m_dHoleDiameter ;
	double dH = dNomRad - dMatRad ;
	double dMatCutRad = dMatRad - 0.5 * m_cToolDef.m_dTopDiam ;

	//JDNC_LAYER *pLayer = GetLayerDef () ;
	//pLayer->m_dTotalDepth = GetCutDepth() - m_cShapeDef.m_cStock.m_dDepthStock ; 
	//DefineLayerDepth ( *pLayer ) ; 

	bool bLayer = false ;
	int nCnt = 1 ;
	double *pCutDepth = NULL ;
	double dCurRad, dDepth = dH, dMinRad = dRadius ;
	if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT 
		&& m_cFeedDef.m_cLayerDef.m_nLayerCount > 1 )
	{
		bLayer = true ;
		nCnt = m_cFeedDef.m_cLayerDef.m_nLayerCount ;
		if ( !CalCutDepthByLayer ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
			nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
		dMinRad = dMatCutRad + pCutDepth[0] ;
	}
	else if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_DEPTH 
		&& m_cFeedDef.m_cLayerDef.m_dSideDInc < dH )
	{
		bLayer = true ;
		dDepth = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
		if ( !CalCutDepthByMaxDepth ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
			dDepth, nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
		dMinRad = dMatCutRad + pCutDepth[0] ;
	}

	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;

	double dBottomZ = GetCutDepth () ;
	double *pBotZArr = NULL ;
	if ( bLayer )
	{
		pBotZArr = new double [nCnt] ;
		if ( !CalAllBottomZ ( nCnt, pCutDepth, dBottomZ, 
			m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_SIDEINC,pBotZArr ) )
		{
			delete[] pBotZArr ;
			return FALSE ;
		}
	}

	if ( GetLeadType () == NCDEF_LEAD_ARC )
	{
		double dBotLeadAng = CalLeadArcAngle ( 0.5 * dMinRad, 0.5 * dRadius ) ;
		if ( bLayer )
		{
			for ( int i = 0 ; i < nCnt ; i ++ )
			{
				dCurRad = dMatCutRad + pCutDepth[i] ; //i * dDepth ;
				CreatIntPath_ArcLead ( &Point, pTComb, dCurRad, dBotLeadAng, pBotZArr[i] ) ;
			}
		}
		else
		{
			CreatIntPath_ArcLead ( &Point, pTComb, dRadius, dBotLeadAng, dBottomZ ) ;
		}
	}
	else
	{
		double dBotLeadOff = CalLeadLineOff ( dMinRad ) ;
		if ( bLayer )
		{
			for ( int i = 0 ; i < nCnt ; i ++ )
			{
				dCurRad = dMatCutRad + pCutDepth[i] ; //i * dDepth ;
				CreatIntPath_LineLead ( &Point, pTComb, dCurRad, dBotLeadOff, pBotZArr[i] ) ;
			}
		}
		else
		{
			CreatIntPath_LineLead ( &Point, pTComb, dRadius, dBotLeadOff, dBottomZ ) ;
		}
	}
	delete[] pCutDepth ;
	delete[] pBotZArr ;

	if ( !pTComb->m_pHead ) 
	{
		delete pTComb ;
	}
	else
	{
		PComb.AppendCombine(pTComb);
	}	

	return TRUE ;
}

BOOL CSmartThreadMillGen::CreatInternalPath ( CPointList& AllPoint, CPathGroup& NewPath )
{
	// STEP 1 : 计算路径
	double dNomRad = 0.5 * m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ ;
	double dRadius = dNomRad - 0.5 * m_cToolDef.m_dTopDiam ;
	if ( dRadius < 0.02 || m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ < m_cParam.m_dHoleDiameter + 0.02 )
	{
		m_nErrorType = JDERROR_GENPATH_ERRDATA ;
		return FALSE ;
	}
	double dMatRad = 0.5 * m_cParam.m_dHoleDiameter ;
	double dH = dNomRad - dMatRad ;
	double dMatCutRad = dMatRad - 0.5 * m_cToolDef.m_dTopDiam ;

	//JDNC_LAYER *pLayer = GetLayerDef () ;
	//pLayer->m_dTotalDepth = GetCutDepth() - m_cShapeDef.m_cStock.m_dDepthStock ; 
	//DefineLayerDepth ( *pLayer ) ; 

	bool bLayer = false ;
	int nCnt = 1 ;
	double *pCutDepth = NULL ;
	double dCurRad, dDepth = dH, dMinRad = dRadius ;
	if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT 
			&& m_cFeedDef.m_cLayerDef.m_nLayerCount > 1 )
	{
		bLayer = true ;
		nCnt = m_cFeedDef.m_cLayerDef.m_nLayerCount ;
		if ( !CalCutDepthByLayer ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
								   nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
		dMinRad = dMatCutRad + pCutDepth[0] ;
	}
	else if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_DEPTH 
			&& m_cFeedDef.m_cLayerDef.m_dSideDInc < dH )
	{
		bLayer = true ;
		dDepth = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
		if ( !CalCutDepthByMaxDepth ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
									  dDepth, nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
		dMinRad = dMatCutRad + pCutDepth[0] ;
	}

	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;

	double dBottomZ = GetCutDepth () ;
	double *pBotZArr = NULL ;
	if ( bLayer )
	{
		pBotZArr = new double [nCnt] ;
		if ( !CalAllBottomZ ( nCnt, pCutDepth, dBottomZ, 
				m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_SIDEINC,pBotZArr ) )
		{
			delete[] pBotZArr ;
			return FALSE ;
		}
	}

	CSmartPoint* pPoint ;
	if ( GetLeadType () == NCDEF_LEAD_ARC )
	{
		double dBotLeadAng = CalLeadArcAngle ( 0.5 * dMinRad, 0.5 * dRadius ) ;
		for ( pPoint = AllPoint.GetHead() ; pPoint ; pPoint = pPoint->next )
		{
			m_bNewCutPoint = TRUE;
			if ( bLayer )
			{
				for ( int i = 0 ; i < nCnt ; i ++ )
				{
					dCurRad = dMatCutRad + pCutDepth[i] ; //i * dDepth ;
					CreatIntPath_ArcLead ( pPoint, pTComb, dCurRad, dBotLeadAng, pBotZArr[i] ) ;
				}
			}
			else
			{
				CreatIntPath_ArcLead ( pPoint, pTComb, dRadius, dBotLeadAng, dBottomZ ) ;
			}
		}
	}
	else
	{
		double dBotLeadOff = CalLeadLineOff ( dMinRad ) ;
		for ( pPoint = AllPoint.GetHead() ; pPoint ; pPoint = pPoint->next )
		{
			m_bNewCutPoint = TRUE;
			if ( bLayer )
			{
				for ( int i = 0 ; i < nCnt ; i ++ )
				{
					dCurRad = dMatCutRad + pCutDepth[i] ; //i * dDepth ;
					CreatIntPath_LineLead ( pPoint, pTComb, dCurRad, dBotLeadOff, pBotZArr[i] ) ;
				}
			}
			else
			{
				CreatIntPath_LineLead ( pPoint, pTComb, dRadius, dBotLeadOff, dBottomZ ) ;
			}
		}
	}
	delete[] pCutDepth ;
	delete[] pBotZArr ;

	if ( !pTComb->m_pHead ) 
	{
		delete pTComb ;
	}
	else
    {
        NewPath.AddData ( 0.0, pTComb ) ;
    }

	if ( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_REDEPTH )
	{// 忽略表面高度
		DOUBLE dTop = -1.0e6 ;
	    for( pPoint = AllPoint.GetHead(); pPoint ; pPoint = pPoint->next )
	    {
            if( pPoint->m_dPoint[2] > dTop ) 
				dTop = pPoint->m_dPoint[2] ;
        }
        m_dTopHeight = dTop;
	}
	else
	{ // 校正表面高度 /*不保留孔位高度*/
		NewPath.ZMoveGroup ( GetTopHeight() ) ;
	}
	
	return TRUE ;
}

BOOL CSmartThreadMillGen::CreatIntPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
{
	if ( GetThreadMillRCompMask () != 0 )
	{
		if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALRIGHT )
		{
			//if ( GetMillDir() == 1 ) //由上向下走刀方式关闭（暂时没用，侧向进给也不对）
				return CreatIntRitClmPath_Wear_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
			//else
				//return CreatIntRitCvtPath_Wear_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
		}
		else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALLEFT )
		{
			//if ( GetMillDir() == 0 )
				return CreatIntLftCvtPath_Wear_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
			//else 
				//return CreatIntLftClmPath_Wear_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
		}
	}
	else
	{
		if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALRIGHT )
		{
			//if ( GetMillDir() == 1 ) 
				return CreatIntRitClmPath_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
			//else
				//return CreatIntRitCvtPath_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
		}
		else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALLEFT )
		{
			//if ( GetMillDir() == 0 )
				return CreatIntLftCvtPath_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
			//else 
				//return CreatIntLftClmPath_ArcLead ( SmtPoint, TComb, CutRad, BotLeadAng, BottomZ ) ;
		}
	}

	return FALSE ;
}

BOOL CSmartThreadMillGen::CreatIntPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ )
{
	if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALRIGHT )
	{
		//if ( GetMillDir() == 1 ) 
			return CreatIntRitClmPath_LineLead ( SmtPoint, TComb, CutRad, BotLeadOff, BottomZ ) ;
		//else
			//return CreatIntRitCvtPath_LineLead ( SmtPoint, TComb, CutRad, BotLeadOff, BottomZ ) ;
	}
	else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALLEFT )
	{
		//if ( GetMillDir() == 0 )
			return CreatIntLftCvtPath_LineLead ( SmtPoint, TComb, CutRad, BotLeadOff, BottomZ ) ;
		//else 
			//return CreatIntLftClmPath_LineLead ( SmtPoint, TComb, CutRad, BotLeadOff, BottomZ ) ;
	}

	return FALSE ;
}

BOOL CSmartThreadMillGen::CreatIntRitClmPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity* pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dLeadRad = 0.5 * CutRad ;
	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
	param.m_dBotLeadAng = BotLeadAng ;
	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
	if ( m_cParam.m_nDepthType == NCDEF_THREADMILL_DEPTH_THREADLENGTH ) //深度设置方式为“螺纹长度”时，路径长度延长
	{
		dCutDepth += param.m_dBotLeadDepth ;
	}
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;

	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;

		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - param.m_dBotLeadDepth - MIN_LEN )
		{
			GenIntRitClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
			AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	}

	GenIntRitClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenIntRitClmPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) 
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint )
		return FALSE ;

	PNT3D ptCutCent, ptStCent, ptEdCent, ptEnd  ;
	double dt = Param.m_dCutDepth / Param.m_dCutPitch ;
	double dCutEndAng = MiniPai2 * ( dt - floor (dt) ) ;

	ptStCent[0] = SmtPoint->m_dPoint[0] + Param.m_dLeadRad ;
	ptStCent[1] = SmtPoint->m_dPoint[1] ;
	ptStCent[2] = Param.m_dCurZValue ;
	PathArr[0] = CreatHelixPath ( ptStCent, Param.m_dLeadRad, Param.m_dLeadPitch, -Param.m_dBotLeadAng, Param.m_dBotLeadDepth, 1 ) ;
	if ( !PathArr[0] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[0]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dLeadRad, Param.m_dLeadPitch, 0 ) ;
	}
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dBotLeadDepth ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 1 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptEnd ) ;
	nc_GetPointAtLine ( ptEnd, SmtPoint->m_dPoint, 0.5, ptEdCent, 2 ) ;
	ptEdCent[2] = ptEnd[2] ;
	PathArr[2] = CreatHelixPath ( ptEdCent, Param.m_dLeadRad, Param.m_dLeadPitch, dCutEndAng, Param.m_dTopLeadDepth, 1 ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_fFeedScale = PathArr[0]->m_fFeedScale ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dTopLeadDepth ;

	return TRUE ;
}

//BOOL CSmartThreadMillGen::CreatIntRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dLeadRad = 0.5 * CutRad ;
//	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
//	param.m_dBotLeadAng = BotLeadAng ;
//	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
//	
//	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dTopLeadAng = param.m_dBotLeadAng ;
//		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//		//param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;
//
//		double dPeriodLen = param.m_dCutDepth + param.m_dTopLeadDepth + dZMove + param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += ( param.m_dCutDepth + param.m_dTopLeadDepth ) ;
//		}
//		    
//		double dJugZ = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenIntRitCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//			AddCutPathToComb ( pPathArr, 3, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
//		param.m_dTopLeadDepth = min ( 0.05, 0.25 * param.m_dLeadPitch ) ;
//		param.m_dTopLeadAng = MiniPai2 * param.m_dTopLeadDepth / param.m_dLeadPitch ;
//		param.m_dCurZValue = param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dTopLeadAng = param.m_dBotLeadAng ;
//		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dTopLeadAng = param.m_dBotLeadAng ;
//		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth ;
//	}
//	
//	GenIntRitCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
//}

//BOOL CSmartThreadMillGen::GenIntRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) 
//{
//	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
//	if ( !SmtPoint )
//		return FALSE ;
//
//	PNT3D ptCutCent, ptStCent, ptEdCent, ptEnd  ;
//	double dt = Param.m_dCutDepth / Param.m_dCutPitch ;
//	double dCutEndAng = MiniPai2 * ( dt - floor (dt) ) ;
//
//	ptStCent[0] = SmtPoint->m_dPoint[0] + Param.m_dLeadRad ;
//	ptStCent[1] = SmtPoint->m_dPoint[1] ;
//	ptStCent[2] = Param.m_dCurZValue ;
//	PathArr[0] = CreatHelixPath ( ptStCent, Param.m_dLeadRad, Param.m_dLeadPitch, Param.m_dTopLeadAng, -Param.m_dTopLeadDepth, 0 ) ;
//	if ( !PathArr[0] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[0]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dLeadRad, Param.m_dLeadPitch, 0 ) ;
//	}
//	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//	Param.m_dCurZValue -= Param.m_dTopLeadDepth ;
//
//	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
//	ptCutCent[2] = Param.m_dCurZValue ; 
//	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, -Param.m_dCutDepth, 0 ) ;
//	if ( !PathArr[1] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
//	}
//	Param.m_dCurZValue -= Param.m_dCutDepth ;
//	
//	PathArr[1]->GetEndPoint ( 1, ptEnd ) ;
//	nc_GetPointAtLine ( ptEnd, SmtPoint->m_dPoint, 0.5, ptEdCent, 2 ) ;
//	ptEdCent[2] = ptEnd[2] ;
//	PathArr[2] = CreatHelixPath ( ptEdCent, Param.m_dLeadRad, Param.m_dLeadPitch, -dCutEndAng, -Param.m_dBotLeadDepth, 0 ) ;
//	if ( !PathArr[2] )
//		return FALSE ;
//	PathArr[2]->m_fFeedScale = PathArr[0]->m_fFeedScale ;
//	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//	Param.m_dCurZValue -= Param.m_dBotLeadDepth ;
//
//	return TRUE ;
//}

BOOL CSmartThreadMillGen::CreatIntLftCvtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dLeadRad = 0.5 * CutRad ;
	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
	param.m_dBotLeadAng = BotLeadAng ;
	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
	if ( m_cParam.m_nDepthType == NCDEF_THREADMILL_DEPTH_THREADLENGTH ) //深度设置方式为“螺纹长度”时，路径长度延长
	{
		dCutDepth += param.m_dBotLeadDepth ;
	}
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;
	
	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;
	    
		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - param.m_dBotLeadDepth - MIN_LEN )
		{
			GenIntLftCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
			AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		param.m_dTopLeadAng = param.m_dBotLeadAng ;
		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	}
	
	GenIntLftCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenIntLftCvtPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint )
		return FALSE ;

	PNT3D ptCutCent, ptStCent, ptEdCent, ptEnd  ;
	double dt = Param.m_dCutDepth / Param.m_dCutPitch ;
	double dCutEndAng = MiniPai2 * ( dt - floor (dt) ) ;

	ptStCent[0] = SmtPoint->m_dPoint[0] + Param.m_dLeadRad ;
	ptStCent[1] = SmtPoint->m_dPoint[1] ;
	ptStCent[2] = Param.m_dCurZValue ;
	PathArr[0] = CreatHelixPath ( ptStCent, Param.m_dLeadRad, Param.m_dLeadPitch, Param.m_dBotLeadAng, Param.m_dBotLeadDepth, 0 ) ;
	if ( !PathArr[0] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[0]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dLeadRad, Param.m_dLeadPitch, 0 ) ;
	}
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dBotLeadDepth ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 0 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptEnd ) ;
	nc_GetPointAtLine ( ptEnd, SmtPoint->m_dPoint, 0.5, ptEdCent, 2 ) ;
	ptEdCent[2] = ptEnd[2] ;
	PathArr[2] = CreatHelixPath ( ptEdCent, Param.m_dLeadRad, Param.m_dLeadPitch, -dCutEndAng, Param.m_dTopLeadDepth, 0 ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_fFeedScale = PathArr[0]->m_fFeedScale ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dTopLeadDepth ;

	return TRUE ;
}

//BOOL CSmartThreadMillGen::CreatIntLftClmPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dLeadRad = 0.5 * CutRad ;
//	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
//	param.m_dBotLeadAng = BotLeadAng ;
//	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
//	
//	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dTopLeadAng = param.m_dBotLeadAng ;
//		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//		//param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;
//
//		double dPeriodLen = param.m_dCutDepth + param.m_dTopLeadDepth + dZMove + param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += ( param.m_dCutDepth + param.m_dTopLeadDepth ) ;
//		}
//
//		double dJugZ = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenIntLftClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//			AddCutPathToComb ( pPathArr, 3, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
//		param.m_dTopLeadDepth = min ( 0.05, 0.25 * param.m_dLeadPitch ) ;
//		param.m_dTopLeadAng = MiniPai2 * param.m_dTopLeadDepth / param.m_dLeadPitch ;
//		param.m_dCurZValue = param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dTopLeadAng = param.m_dBotLeadAng ;
//		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dTopLeadAng = param.m_dBotLeadAng ;
//		param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth ;
//	}
//	
//	GenIntLftClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
//}

//BOOL CSmartThreadMillGen::GenIntLftClmPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
//{
//	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
//	if ( !SmtPoint )
//		return FALSE ;
//
//	PNT3D ptCutCent, ptStCent, ptEdCent, ptEnd  ;
//	double dt = Param.m_dCutDepth / Param.m_dCutPitch ;
//	double dCutEndAng = MiniPai2 * ( dt - floor (dt) ) ;
//
//	ptStCent[0] = SmtPoint->m_dPoint[0] + Param.m_dLeadRad ;
//	ptStCent[1] = SmtPoint->m_dPoint[1] ;
//	ptStCent[2] = Param.m_dCurZValue ;
//	PathArr[0] = CreatHelixPath ( ptStCent, Param.m_dLeadRad, Param.m_dLeadPitch, -Param.m_dTopLeadAng, -Param.m_dTopLeadDepth, 1 ) ;
//	if ( !PathArr[0] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[0]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dLeadRad, Param.m_dLeadPitch, 0 ) ;
//	}
//	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//	Param.m_dCurZValue -= Param.m_dTopLeadDepth ;
//
//	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
//	ptCutCent[2] = Param.m_dCurZValue ; 
//	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, -Param.m_dCutDepth, 1 ) ;
//	if ( !PathArr[1] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
//	}
//	Param.m_dCurZValue -= Param.m_dCutDepth ;
//
//	PathArr[1]->GetEndPoint ( 1, ptEnd ) ;
//	nc_GetPointAtLine ( ptEnd, SmtPoint->m_dPoint, 0.5, ptEdCent, 2 ) ;
//	ptEdCent[2] = ptEnd[2] ;
//	PathArr[2] = CreatHelixPath ( ptEdCent, Param.m_dLeadRad, Param.m_dLeadPitch, dCutEndAng, -Param.m_dBotLeadDepth, 1 ) ;
//	if ( !PathArr[2] )
//		return FALSE ;
//	PathArr[2]->m_fFeedScale = PathArr[0]->m_fFeedScale ;
//	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//	Param.m_dCurZValue -= Param.m_dBotLeadDepth ;
//
//	return TRUE ;
//}

BOOL CSmartThreadMillGen::CreatIntRitClmPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
	CPathEntity *pAllPathArr[5] = { NULL, NULL, NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dLeadRad = 0.5 * CutRad ;
	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
	param.m_dBotLeadAng = BotLeadAng ;
	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
	param.m_dTopLeadAng = param.m_dBotLeadAng ;
	param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	if ( m_cParam.m_nDepthType == NCDEF_THREADMILL_DEPTH_THREADLENGTH ) //深度设置方式为“螺纹长度”时，路径长度延长
	{
		dCutDepth += param.m_dBotLeadDepth ;
	}
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;

	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;
	
	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;

		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - param.m_dBotLeadDepth - MIN_LEN )
		{
			GenIntRitClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
			AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
			SetRCompMask ( pAllPathArr, 5, GetThreadMillRCompMask() ) ;
			AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}
	else
		return FALSE ;

	GenIntRitClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
	AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
	SetRCompMask ( pAllPathArr, 5, GetThreadMillRCompMask() ) ;
	return AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
}

//BOOL CSmartThreadMillGen::CreatIntRitCvtPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	CPathEntity *pAllPathArr[5] = { NULL, NULL, NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dLeadRad = 0.5 * CutRad ;
//	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
//	param.m_dBotLeadAng = BotLeadAng ;
//	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
//	param.m_dTopLeadAng = param.m_dBotLeadAng ;
//	param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//	
//	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		//param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;
//
//		double dPeriodLen = param.m_dCutDepth + param.m_dTopLeadDepth + dZMove + param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += ( param.m_dCutDepth + param.m_dTopLeadDepth ) ;
//		}
//		    
//		double dJugZ = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenIntRitCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//			AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
//			AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
//		param.m_dCurZValue = param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dCurZValue = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth ;
//	}
//	
//	GenIntRitCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//	AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
//	SetRCompMask ( pAllPathArr, 5, GetThreadMillRCompMask() ) ;
//	return AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
//}

BOOL CSmartThreadMillGen::CreatIntLftCvtPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
	CPathEntity *pAllPathArr[5] = { NULL, NULL, NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dLeadRad = 0.5 * CutRad ;
	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
	param.m_dBotLeadAng = BotLeadAng ;
	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
	param.m_dTopLeadAng = param.m_dBotLeadAng ;
	param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	if ( m_cParam.m_nDepthType == NCDEF_THREADMILL_DEPTH_THREADLENGTH ) //深度设置方式为“螺纹长度”时，路径长度延长
	{
		dCutDepth += param.m_dBotLeadDepth ;
	}
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;
	
	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;
	    
		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - param.m_dBotLeadDepth - MIN_LEN )
		{
			GenIntLftCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
			AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
			SetRCompMask ( pAllPathArr, 5, GetThreadMillRCompMask() ) ;
			AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}
	
	GenIntLftCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
	AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
	SetRCompMask ( pAllPathArr, 5, GetThreadMillRCompMask() ) ;
	return AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
}

//BOOL CSmartThreadMillGen::CreatIntLftClmPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	CPathEntity *pAllPathArr[5] = { NULL, NULL, NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dLeadRad = 0.5 * CutRad ;
//	param.m_dLeadPitch = 0.5 * m_cParam.m_dPitch ;
//	param.m_dBotLeadAng = BotLeadAng ;
//	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
//	param.m_dTopLeadAng = param.m_dBotLeadAng ;
//	param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
//	
//	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		//param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;
//
//		double dPeriodLen = param.m_dCutDepth + param.m_dTopLeadDepth + dZMove + param.m_dBotLeadDepth ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += ( param.m_dCutDepth + param.m_dTopLeadDepth ) ;
//		}
//	    
//		double dJugZ = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenIntLftClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//			AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
//			AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
//		param.m_dCurZValue = param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch + param.m_dTopLeadDepth ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dCurZValue = -dCutDepth + param.m_dBotLeadDepth + param.m_dCutDepth + param.m_dTopLeadDepth ;
//	}
//	
//	GenIntLftClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
//	AddLineLeadPath ( SmtPoint, pPathArr, pAllPathArr ) ;
//	SetRCompMask ( pAllPathArr, 5, GetThreadMillRCompMask() ) ;
//	return AddCutPathToComb ( pAllPathArr, 5, TComb ) ;
//}

BOOL CSmartThreadMillGen::AddLineLeadPath ( CSmartPoint *SmtPoint, CPathEntity *PathArr[3], CPathEntity *AllPathArr[5] )
{
	AllPathArr[0] = AllPathArr[1] = AllPathArr[2] = AllPathArr[3] = AllPathArr[4] = NULL ;
	if ( !SmtPoint || !PathArr[0] || !PathArr[1] || !PathArr[2] )
		return FALSE ;

	double dToolRad = 0.5 * m_cToolDef.m_dTopDiam ;
	double dWearLine = m_cFeedDef.m_cLeadDef.m_dWearLine ;
	if ( dWearLine < MIN_DIS )
		dWearLine = MIN_DIS ;
	double dH = 0.866025404 * m_cParam.m_dPitch ;
	double dHoleRad = min ( 0.5 * m_cParam.m_dHoleDiameter, 0.5 * m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ - dH ) ;
	double dMaxLeft = dHoleRad - dToolRad - 0.1 ;

	double dDis, dt, dLen ;
	PNT3D ptSt, ptEd ;
	PathArr[0]->GetEndPoint ( 0, ptEd ) ;

	dDis = nc_Distance ( ptEd, SmtPoint->m_dPoint, 2 ) ;
	if ( dDis < MIN_LEN )
		return FALSE ;
	dLen = max ( dDis - dMaxLeft, dWearLine ) ;
	dt = min ( 1.0, dLen / dDis ) ;
	nc_GetPointAtLine ( ptEd, SmtPoint->m_dPoint, dt, ptSt, 2 ) ;

	//nc_VectorCopy ( ptSt, SmtPoint->m_dPoint, 2 ) ;
	ptSt[2] = ptEd[2] ;
	AllPathArr[0] = new CPathLine3D ( ptSt, ptEd ) ;
	AllPathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	
	AllPathArr[1] = PathArr[0] ; AllPathArr[2] = PathArr[1] ; AllPathArr[3] = PathArr[2] ;

	PathArr[2]->GetEndPoint ( 1, ptSt ) ;

	dDis = nc_Distance ( ptSt, SmtPoint->m_dPoint, 2 ) ;
	if ( dDis < MIN_LEN )
		return FALSE ;
	dLen = max ( dDis - dMaxLeft, dWearLine ) ;
	dt = min ( 1.0, dLen / dDis ) ;
	nc_GetPointAtLine ( ptSt, SmtPoint->m_dPoint, dt, ptEd, 2 ) ;

	//nc_VectorCopy ( ptEd, SmtPoint->m_dPoint, 2 ) ;
	ptEd[2] = ptSt[2] ;
	AllPathArr[4] = new CPathLine3D ( ptSt, ptEd ) ;
	AllPathArr[4]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	return TRUE ;
}

BOOL CSmartThreadMillGen::CreatIntRitClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity* pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dBotLeadOff = param.m_dTopLeadOff = BotLeadOff ;
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;

	double dThreadDepth = dCutDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;

		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - MIN_LEN )
		{
			GenIntRitClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
			SetRCompMask ( pPathArr, 3, GetThreadMillRCompMask() ) ;
			AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}

	GenIntRitClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
	SetRCompMask ( pPathArr, 3, GetThreadMillRCompMask() ) ;
	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenIntRitClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) 
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
		return FALSE ;

	PNT3D ptCutCent, ptLineSt, ptLineEd ;
	double dt ;
	
	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dBotLeadOff ;
	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
	ptLineSt[2] = Param.m_dCurZValue ;
	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
	ptLineEd[2] = Param.m_dCurZValue ;
	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[0] )
		return FALSE ;
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 1 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
	dt = Param.m_dTopLeadOff / Param.m_dCutRad ;
	dt = min ( dt, 1.0 ) ;
	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
	ptLineEd[2] = ptLineSt[2] ;
	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	return TRUE ;
}

//BOOL CSmartThreadMillGen::CreatIntRitCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dBotLeadOff = param.m_dTopLeadOff = BotLeadOff ;
//	
//	double dThreadDepth = dCutDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;
//
//		double dPeriodLen = param.m_dCutDepth + dZMove ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += ( param.m_dCutDepth ) ;
//		}
//		    
//		double dJugZ = -dCutDepth + param.m_dCutDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenIntRitCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
//			AddCutPathToComb ( pPathArr, 3, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth ;
//		param.m_dCurZValue = 0.0 ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dCurZValue = -dCutDepth + param.m_dCutDepth ;
//	}
//	
//	GenIntRitCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
//	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
//}

//BOOL CSmartThreadMillGen::GenIntRitCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) 
//{
//	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
//	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
//		return FALSE ;
//
//	PNT3D ptCutCent, ptLineSt, ptLineEd ;
//	double dt ;
//	
//	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dTopLeadOff ;
//	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
//	ptLineSt[2] = Param.m_dCurZValue ;
//	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
//	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
//	ptLineEd[2] = Param.m_dCurZValue ;
//	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[0] )
//		return FALSE ;
//	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
//	ptCutCent[2] = Param.m_dCurZValue ; 
//	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, -Param.m_dCutDepth, 0 ) ;
//	if ( !PathArr[1] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
//	}
//	Param.m_dCurZValue -= Param.m_dCutDepth ;
//	
//	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
//	dt = Param.m_dBotLeadOff / Param.m_dCutRad ;
//	dt = min ( dt, 1.0 ) ;
//	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
//	ptLineEd[2] = ptLineSt[2] ;
//	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[2] )
//		return FALSE ;
//	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	return TRUE ;
//}

BOOL CSmartThreadMillGen::CreatIntLftCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dBotLeadOff = param.m_dTopLeadOff = BotLeadOff ;
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;
	
	double dThreadDepth = dCutDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;
	    
		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - MIN_LEN )
		{
			GenIntLftCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
			SetRCompMask ( pPathArr, 3, GetThreadMillRCompMask() ) ;
			AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}
	
	GenIntLftCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
	SetRCompMask ( pPathArr, 3, GetThreadMillRCompMask() ) ;
	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenIntLftCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
		return FALSE ;

	PNT3D ptCutCent, ptLineSt, ptLineEd ;
	double dt ;
	
	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dBotLeadOff ;
	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
	ptLineSt[2] = Param.m_dCurZValue ;
	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
	ptLineEd[2] = Param.m_dCurZValue ;
	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[0] )
		return FALSE ;
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 0 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
	dt = Param.m_dTopLeadOff / Param.m_dCutRad ;
	dt = min ( dt, 1.0 ) ;
	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
	ptLineEd[2] = ptLineSt[2] ;
	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	return TRUE ;
}

//BOOL CSmartThreadMillGen::CreatIntLftClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dBotLeadOff = param.m_dTopLeadOff = BotLeadOff ;
//	
//	double dThreadDepth = dCutDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;
//
//		double dPeriodLen = param.m_dCutDepth + dZMove ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += param.m_dCutDepth ;
//		}
//
//		double dJugZ = -dCutDepth + param.m_dCutDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenIntLftClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
//			AddCutPathToComb ( pPathArr, 3, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth ;
//		param.m_dCurZValue = 0.0 ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dCurZValue = -dCutDepth + param.m_dCutDepth ;
//	}
//	
//	GenIntLftClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
//	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
//}

//BOOL CSmartThreadMillGen::GenIntLftClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
//{
//	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
//	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
//		return FALSE ;
//
//	PNT3D ptCutCent, ptLineSt, ptLineEd ;
//	double dt ;
//	
//	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dTopLeadOff ;
//	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
//	ptLineSt[2] = Param.m_dCurZValue ;
//	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
//	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
//	ptLineEd[2] = Param.m_dCurZValue ;
//	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[0] )
//		return FALSE ;
//	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
//	ptCutCent[2] = Param.m_dCurZValue ; 
//	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, -Param.m_dCutDepth, 1 ) ;
//	if ( !PathArr[1] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
//	}
//	Param.m_dCurZValue -= Param.m_dCutDepth ;
//
//	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
//	dt = Param.m_dBotLeadOff / Param.m_dCutRad ;
//	dt = min ( dt, 1.0 ) ;
//	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
//	ptLineEd[2] = ptLineSt[2] ;
//	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[2] )
//		return FALSE ;
//	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	return TRUE ;
//}

BOOL CSmartThreadMillGen::CreatExternalPath ( CSmartPoint & Point, CPathCombine & PComb)
{
	// STEP 1 : 计算路径
	double dMinorRad = 0.5 * m_dModiMinDiam ;
	double dRadius = dMinorRad + 0.5 * m_cToolDef.m_dTopDiam ;
	if ( m_dModiMinDiam > m_cParam.m_dCylDiameter - 0.02 )
	{
		m_nErrorType = JDERROR_GENPATH_ERRDATA ;
		return FALSE ;
	}
	double dCylRad = 0.5 * m_cParam.m_dCylDiameter ;
	double dH = dCylRad - dMinorRad ;
	double dCylCutRad = dCylRad + 0.5 * m_cToolDef.m_dTopDiam ;

	bool bLayer = false ;
	int nCnt = 1 ;
	double *pCutDepth = NULL ;
	double dCurRad, dDepth = dH ;
	if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT 
		&& m_cFeedDef.m_cLayerDef.m_nLayerCount > 1 )
	{
		bLayer = true ;
		nCnt = m_cFeedDef.m_cLayerDef.m_nLayerCount ;
		if ( !CalCutDepthByLayer ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
			nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
	}
	else if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_DEPTH 
		&& m_cFeedDef.m_cLayerDef.m_dSideDInc < dH )
	{
		bLayer = true ;
		dDepth = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
		if ( !CalCutDepthByMaxDepth ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
			dDepth, nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
	}

	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;

	double dBottomZ = GetCutDepth () ;
	double *pBotZArr = NULL ;
	if ( bLayer )
	{
		pBotZArr = new double [nCnt] ;
		if ( !CalAllBottomZ ( nCnt, pCutDepth, dBottomZ, 
			m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_SIDEINC, pBotZArr ) )
		{
			delete[] pBotZArr ;
			return FALSE ;
		}
	}

	if ( GetLeadType () == NCDEF_LEAD_ARC )
	{
		double dSafeRad = 0.433012702 * m_cParam.m_dPitch + 0.5 ;
		double dLeadRad = m_cFeedDef.m_cLeadDef.m_dRadius ;
		dLeadRad = max ( dSafeRad, dLeadRad ) ;
		double dSafeLeadAng = CalExternalLeadArcAngle ( dLeadRad ) ;
		double dLeadAng = ANGLE_TO_RADIAN ( m_cFeedDef.m_cLeadDef.m_dAngle ) ;
		dLeadAng = max ( dSafeLeadAng, dLeadAng ) ;
		if ( bLayer )
		{
			for ( int i = 0 ; i < nCnt ; i ++ )
			{
				dCurRad = dCylCutRad - pCutDepth[i] ; 
				CreatExtPath_ArcLead ( &Point, pTComb, dCurRad, dLeadRad, dLeadAng, pBotZArr[i] ) ;
			}
		}
		else
		{
			CreatExtPath_ArcLead ( &Point, pTComb, dRadius, dLeadRad, dLeadAng, dBottomZ ) ;
		}
	}
	else
	{
		double dSafeLeadLen = CalExternalLeadLineLen () ;
		double dLeadLen = max ( m_cFeedDef.m_cLeadDef.m_dLength, dSafeLeadLen ) ;
		if ( bLayer )
		{
			for ( int i = 0 ; i < nCnt ; i ++ )
			{
				dCurRad = dCylCutRad - pCutDepth[i] ; 
				CreatExtPath_LineLead ( &Point, pTComb, dCurRad, dLeadLen, pBotZArr[i] ) ;
			}
		}
		else
		{
			CreatExtPath_LineLead ( &Point, pTComb, dRadius, dLeadLen, dBottomZ ) ;
		}
	}
	delete[] pCutDepth ;
	delete[] pBotZArr ;

	if ( !pTComb->m_pHead ) 
	{
		delete pTComb ;
	}
	else
	{
		PComb.AppendCombine(pTComb);
	}

	return TRUE ;
}

BOOL CSmartThreadMillGen::CreatExternalPath ( CPointList& AllPoint, CPathGroup& NewPath )
{
	// STEP 1 : 计算路径
	double dMinorRad = 0.5 * m_dModiMinDiam ;
	double dRadius = dMinorRad + 0.5 * m_cToolDef.m_dTopDiam ;
	if ( m_dModiMinDiam > m_cParam.m_dCylDiameter - 0.02 )
	{
		m_nErrorType = JDERROR_GENPATH_ERRDATA ;
		return FALSE ;
	}
	double dCylRad = 0.5 * m_cParam.m_dCylDiameter ;
	double dH = dCylRad - dMinorRad ;
	double dCylCutRad = dCylRad + 0.5 * m_cToolDef.m_dTopDiam ;

	bool bLayer = false ;
	int nCnt = 1 ;
	double *pCutDepth = NULL ;
	double dCurRad, dDepth = dH ;
	if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_COUNT 
			&& m_cFeedDef.m_cLayerDef.m_nLayerCount > 1 )
	{
		bLayer = true ;
		nCnt = m_cFeedDef.m_cLayerDef.m_nLayerCount ;
		if ( !CalCutDepthByLayer ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
								   nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
	}
	else if ( m_cFeedDef.m_cLayerDef.m_nLayerType == NCDEF_LAYER_DEPTH 
			&& m_cFeedDef.m_cLayerDef.m_dSideDInc < dH )
	{
		bLayer = true ;
		dDepth = m_cFeedDef.m_cLayerDef.m_dSideDInc ;
		if ( !CalCutDepthByMaxDepth ( dH, m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_ISOVOL, 
									  dDepth, nCnt, pCutDepth ) 
			|| !pCutDepth )
			return FALSE ;
	}

	CPathCombine *pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;

	double dBottomZ = GetCutDepth () ;
	double *pBotZArr = NULL ;
	if ( bLayer )
	{
		pBotZArr = new double [nCnt] ;
		if ( !CalAllBottomZ ( nCnt, pCutDepth, dBottomZ, 
				m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_SIDEINC, pBotZArr ) )
		{
			delete[] pBotZArr ;
			return FALSE ;
		}
	}

	CSmartPoint* pPoint ;
	if ( GetLeadType () == NCDEF_LEAD_ARC )
	{
		double dSafeRad = 0.433012702 * m_cParam.m_dPitch + 0.5 ;
		double dLeadRad = m_cFeedDef.m_cLeadDef.m_dRadius ;
		dLeadRad = max ( dSafeRad, dLeadRad ) ;
		double dSafeLeadAng = CalExternalLeadArcAngle ( dLeadRad ) ;
		double dLeadAng = ANGLE_TO_RADIAN ( m_cFeedDef.m_cLeadDef.m_dAngle ) ;
		dLeadAng = max ( dSafeLeadAng, dLeadAng ) ;
		for ( pPoint = AllPoint.GetHead() ; pPoint ; pPoint = pPoint->next )
		{
			m_bNewCutPoint = TRUE;
			if ( bLayer )
			{
				for ( int i = 0 ; i < nCnt ; i ++ )
				{
					dCurRad = dCylCutRad - pCutDepth[i] ; 
					CreatExtPath_ArcLead ( pPoint, pTComb, dCurRad, dLeadRad, dLeadAng, pBotZArr[i] ) ;
				}
			}
			else
			{
				CreatExtPath_ArcLead ( pPoint, pTComb, dRadius, dLeadRad, dLeadAng, dBottomZ ) ;
			}
		}
	}
	else
	{
		double dSafeLeadLen = CalExternalLeadLineLen () ;
		double dLeadLen = max ( m_cFeedDef.m_cLeadDef.m_dLength, dSafeLeadLen ) ;
		for ( pPoint = AllPoint.GetHead() ; pPoint ; pPoint = pPoint->next )
		{
			m_bNewCutPoint = TRUE;
			if ( bLayer )
			{
				for ( int i = 0 ; i < nCnt ; i ++ )
				{
					dCurRad = dCylCutRad - pCutDepth[i] ; 
					CreatExtPath_LineLead ( pPoint, pTComb, dCurRad, dLeadLen, pBotZArr[i] ) ;
				}
			}
			else
			{
				CreatExtPath_LineLead ( pPoint, pTComb, dRadius, dLeadLen, dBottomZ ) ;
			}
		}
	}
	delete[] pCutDepth ;
	delete[] pBotZArr ;

	if ( !pTComb->m_pHead ) 
	{
		delete pTComb ;
	}
	else
    {
        NewPath.AddData ( 0.0, pTComb ) ;
    }
	
	if ( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_REDEPTH )
	{// 忽略表面高度
		DOUBLE dTop = -1.0e6 ;
	    for( pPoint = AllPoint.GetHead(); pPoint ; pPoint = pPoint->next )
	    {
            if( pPoint->m_dPoint[2] > dTop ) 
				dTop = pPoint->m_dPoint[2] ;
        }
        m_dTopHeight = dTop;
	}
	else
	{ // 校正表面高度 /*不保留孔位高度*/
		NewPath.ZMoveGroup ( GetTopHeight() ) ;
	}
	
	return TRUE ;
}

BOOL CSmartThreadMillGen::CreatExtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, 
												DOUBLE CutRad, DOUBLE LeadRad, DOUBLE LeadAng, DOUBLE BottomZ )
{
	if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALRIGHT )
		return CreatExtRitCvtPath_ArcLead ( SmtPoint, TComb, CutRad, LeadRad, LeadAng, BottomZ ) ;
	else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALLEFT )
		return CreatExtLftClmPath_ArcLead ( SmtPoint, TComb, CutRad, LeadRad, LeadAng, BottomZ ) ;

	return FALSE ;
}

BOOL CSmartThreadMillGen::CreatExtRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, 
													  DOUBLE CutRad, DOUBLE LeadRad, DOUBLE LeadAng, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity* pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dLeadRad = LeadRad ;
	param.m_dLeadPitch = m_cParam.m_dPitch * LeadRad / CutRad ;
	param.m_dBotLeadAng = LeadAng ;
	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
	param.m_dTopLeadAng = param.m_dBotLeadAng ;
	param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	if ( m_cParam.m_nDepthType == NCDEF_THREADMILL_DEPTH_THREADLENGTH ) //深度设置方式为“螺纹长度”时，路径总长度延长
		dCutDepth += param.m_dBotLeadDepth ;
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;

	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;

		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - param.m_dBotLeadDepth - MIN_LEN )
		{
			GenExtRitCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
			if ( GetThreadMillRCompMask() != 0 )
			{
				CPathEntity *pCompPath[5] ;
				if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
					AddCutPathToComb ( pCompPath, 5, TComb ) ;
				else
					AddCutPathToComb ( pPathArr, 3, TComb ) ;
			}
			else
				AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}

	GenExtRitCvtPath_ArcLead ( SmtPoint, param, pPathArr ) ;
	if ( GetThreadMillRCompMask() != 0 )
	{
		CPathEntity *pCompPath[5] ;
		if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
			return AddCutPathToComb ( pCompPath, 5, TComb ) ;
		else
			return AddCutPathToComb ( pPathArr, 3, TComb ) ;
	}
	else
		return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenExtRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) 
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint )
		return FALSE ;

	PNT3D ptCutCent, ptStCent, ptEdCent, ptEnd  ;
	double dt = Param.m_dCutDepth / Param.m_dCutPitch ;
	double dCutEndAng = MiniPai2 * ( dt - floor (dt) ) ;

	ptStCent[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad + Param.m_dLeadRad ;
	ptStCent[1] = SmtPoint->m_dPoint[1] ;
	ptStCent[2] = Param.m_dCurZValue ;
	PathArr[0] = CreatHelixPath ( ptStCent, Param.m_dLeadRad, Param.m_dLeadPitch, Param.m_dBotLeadAng - MiniPai, Param.m_dBotLeadDepth, 0 ) ;
	if ( !PathArr[0] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[0]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dLeadRad, Param.m_dLeadPitch, 0 ) ;
	}
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dBotLeadDepth ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 1 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 1 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptEnd ) ;
	dt = ( Param.m_dLeadRad + Param.m_dCutRad ) / Param.m_dCutRad ;
	dt = max ( dt, 1.0 ) ;
	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptEnd, dt, ptEdCent, 2 ) ;
	ptEdCent[2] = ptEnd[2] ;
	PathArr[2] = CreatHelixPath ( ptEdCent, Param.m_dLeadRad, Param.m_dLeadPitch, dCutEndAng - MiniPai, Param.m_dTopLeadDepth, 0 ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_fFeedScale = PathArr[0]->m_fFeedScale ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dTopLeadDepth ;

	return TRUE ;
}

BOOL CSmartThreadMillGen::CreatExtLftClmPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, 
													  DOUBLE CutRad, DOUBLE LeadRad, DOUBLE LeadAng, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity* pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dLeadRad = LeadRad ;
	param.m_dLeadPitch = m_cParam.m_dPitch * LeadRad / CutRad ;
	param.m_dBotLeadAng = LeadAng ;
	param.m_dBotLeadDepth = param.m_dLeadPitch * param.m_dBotLeadAng / MiniPai2 ;
	param.m_dTopLeadAng = param.m_dBotLeadAng ;
	param.m_dTopLeadDepth = param.m_dBotLeadDepth ;
	if ( m_cParam.m_nDepthType == NCDEF_THREADMILL_DEPTH_THREADLENGTH ) //深度设置方式为“螺纹长度”时，路径总长度延长
		dCutDepth += param.m_dBotLeadDepth ;
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;

	double dThreadDepth = dCutDepth - param.m_dBotLeadDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch - param.m_dBotLeadDepth - param.m_dTopLeadDepth ;

		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - param.m_dBotLeadDepth - MIN_LEN )
		{
			GenExtLftClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
			if ( GetThreadMillRCompMask() != 0 )
			{
				CPathEntity *pCompPath[5] ;
				if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
					AddCutPathToComb ( pCompPath, 5, TComb ) ;
				else
					AddCutPathToComb ( pPathArr, 3, TComb ) ;
			}
			else
				AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - param.m_dBotLeadDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}

	GenExtLftClmPath_ArcLead ( SmtPoint, param, pPathArr ) ;
	if ( GetThreadMillRCompMask() != 0 )
	{
		CPathEntity *pCompPath[5] ;
		if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
			return AddCutPathToComb ( pCompPath, 5, TComb ) ;
		else
			return AddCutPathToComb ( pPathArr, 3, TComb ) ;
	}
	else
		return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenExtLftClmPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint )
		return FALSE ;

	PNT3D ptCutCent, ptStCent, ptEdCent, ptEnd  ;
	double dt = Param.m_dCutDepth / Param.m_dCutPitch ;
	double dCutEndAng = MiniPai2 * ( dt - floor (dt) ) ;

	ptStCent[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad + Param.m_dLeadRad ;
	ptStCent[1] = SmtPoint->m_dPoint[1] ;
	ptStCent[2] = Param.m_dCurZValue ;
	PathArr[0] = CreatHelixPath ( ptStCent, Param.m_dLeadRad, Param.m_dLeadPitch, MiniPai-Param.m_dBotLeadAng, Param.m_dBotLeadDepth, 1 ) ;
	if ( !PathArr[0] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[0]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dLeadRad, Param.m_dLeadPitch, 0 ) ;
	}
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dBotLeadDepth ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 0 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 0 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptEnd ) ;
	dt = ( Param.m_dLeadRad + Param.m_dCutRad ) / Param.m_dCutRad ;
	dt = max ( dt, 1.0 ) ;
	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptEnd, dt, ptEdCent, 2 ) ;
	ptEdCent[2] = ptEnd[2] ;
	PathArr[2] = CreatHelixPath ( ptEdCent, Param.m_dLeadRad, Param.m_dLeadPitch, MiniPai-dCutEndAng, Param.m_dTopLeadDepth, 1 ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_fFeedScale = PathArr[0]->m_fFeedScale ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	Param.m_dCurZValue += Param.m_dTopLeadDepth ;

	return TRUE ;
}

BOOL CSmartThreadMillGen::CreatExtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadLen, DOUBLE BottomZ )
{
	if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALRIGHT )
	{
		//if ( GetMillDir() == 0 ) //由上向下走刀方式关闭（暂时没用，侧向进给也不对）
			return CreatExtRitCvtPath_LineLead ( SmtPoint, TComb, CutRad, LeadLen, BottomZ ) ;
		//else
			//return CreatExtRitClmPath_LineLead ( SmtPoint, TComb, CutRad, BotLeadLen, BottomZ ) ;
	}
	else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALLEFT )
	{
		//if ( GetMillDir() == 1 )
			return CreatExtLftClmPath_LineLead ( SmtPoint, TComb, CutRad, LeadLen, BottomZ ) ;
		//else 
			//return CreatExtLftCvtPath_LineLead ( SmtPoint, TComb, CutRad, BotLeadLen, BottomZ ) ;
	}

	return FALSE ;
}

BOOL CSmartThreadMillGen::CreatExtRitCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadLen, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity* pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dBotLeadLen = param.m_dTopLeadLen = LeadLen ;
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;

	double dThreadDepth = dCutDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;

		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - MIN_LEN )
		{
			GenExtRitCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
			if ( GetThreadMillRCompMask() != 0 )
			{
				CPathEntity *pCompPath[5] ;
				if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
					AddCutPathToComb ( pCompPath, 5, TComb ) ;
				else
					AddCutPathToComb ( pPathArr, 3, TComb ) ;
			}
			else
				AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}

	GenExtRitCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
	if ( GetThreadMillRCompMask() != 0 )
	{
		CPathEntity *pCompPath[5] ;
		if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
			return AddCutPathToComb ( pCompPath, 5, TComb ) ;
		else
			return AddCutPathToComb ( pPathArr, 3, TComb ) ;
	}
	else
		return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenExtRitCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
		return FALSE ;

	PNT3D ptCutCent, ptLineSt, ptLineEd ;
	double dt ;
	
	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad + Param.m_dBotLeadLen ;
	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
	ptLineSt[2] = Param.m_dCurZValue ;
	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
	ptLineEd[2] = Param.m_dCurZValue ;
	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[0] )
		return FALSE ;
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 1 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 1 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
	dt = ( Param.m_dTopLeadLen + Param.m_dCutRad ) / Param.m_dCutRad ;
	dt = max ( dt, 1.0 ) ;
	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
	ptLineEd[2] = ptLineSt[2] ;
	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	return TRUE ;
}

//BOOL CSmartThreadMillGen::CreatExtRitClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadLen, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dBotLeadLen = param.m_dTopLeadLen = BotLeadLen ;
//	
//	double dThreadDepth = dCutDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;
//
//		double dPeriodLen = param.m_dCutDepth + dZMove ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += ( param.m_dCutDepth ) ;
//		}
//		    
//		double dJugZ = -dCutDepth + param.m_dCutDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenExtRitClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
//			AddCutPathToComb ( pPathArr, 3, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth ;
//		param.m_dCurZValue = 0.0 ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dCurZValue = -dCutDepth + param.m_dCutDepth ;
//	}
//	
//	GenExtRitClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
//	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
//}

//BOOL CSmartThreadMillGen::GenExtRitClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) 
//{
//	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
//	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
//		return FALSE ;
//
//	PNT3D ptCutCent, ptLineSt, ptLineEd ;
//	double dt ;
//	
//	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad + Param.m_dTopLeadLen ;
//	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
//	ptLineSt[2] = Param.m_dCurZValue ;
//	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
//	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
//	ptLineEd[2] = Param.m_dCurZValue ;
//	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[0] )
//		return FALSE ;
//	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
//	ptCutCent[2] = Param.m_dCurZValue ; 
//	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, -Param.m_dCutDepth, 0 ) ;
//	if ( !PathArr[1] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 1 ) ;
//	}
//	Param.m_dCurZValue -= Param.m_dCutDepth ;
//	
//	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
//	dt = ( Param.m_dBotLeadLen + Param.m_dCutRad ) / Param.m_dCutRad ;
//	dt = max ( dt, 1.0 ) ;
//	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
//	ptLineEd[2] = ptLineSt[2] ;
//	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[2] )
//		return FALSE ;
//	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	return TRUE ;
//}

BOOL CSmartThreadMillGen::CreatExtLftClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadLen, DOUBLE BottomZ )
{
	if ( !SmtPoint || !TComb )
		return FALSE ;

	double dCutDepth = BottomZ ;
	int nPitchRate = m_cToolDef.m_nPitchRate > 0 ? m_cToolDef.m_nPitchRate : 1;
	int nTeeth = GetTeethNum() ;
	if ( nTeeth < 1 )
		return FALSE ;

	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
	THREAD_PARAM param ;
	param.m_dCutRad = CutRad ;
	param.m_dCutPitch = m_cParam.m_dPitch ;
	param.m_dBotLeadLen = param.m_dTopLeadLen = LeadLen ;
	param.m_dCurZValue = SmtPoint->m_dPoint[2] - dCutDepth ;
	
	double dThreadDepth = dCutDepth ;

	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;
	    
		while ( param.m_dCurZValue < SmtPoint->m_dPoint[2] - MIN_LEN )
		{
			GenExtLftClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
			if ( GetThreadMillRCompMask() != 0 )
			{
				CPathEntity *pCompPath[5] ;
				if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
					AddCutPathToComb ( pCompPath, 5, TComb ) ;
				else
					AddCutPathToComb ( pPathArr, 3, TComb ) ;
			}
			else
				AddCutPathToComb ( pPathArr, 3, TComb ) ;
			param.m_dCurZValue += dZMove ;
		}
		return TRUE ;
	}

	if ( nTeeth < 2 )
	{
		param.m_dCutDepth = dCutDepth ;
	}
	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
	{
		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
	}
	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
	{
		param.m_dCutDepth = m_cParam.m_dPitch * nPitchRate ;
	}
	
	GenExtLftClmPath_LineLead ( SmtPoint, param, pPathArr ) ;
	if ( GetThreadMillRCompMask() != 0 )
	{
		CPathEntity *pCompPath[5] ;
		if ( AddExtPathCompSeg ( pPathArr, pCompPath ) )
			return AddCutPathToComb ( pCompPath, 5, TComb ) ;
		else
			return AddCutPathToComb ( pPathArr, 3, TComb ) ;
	}
	else
		return AddCutPathToComb ( pPathArr, 3, TComb ) ;
}

BOOL CSmartThreadMillGen::GenExtLftClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
{
	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
		return FALSE ;

	PNT3D ptCutCent, ptLineSt, ptLineEd ;
	double dt ;
	
	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad + Param.m_dBotLeadLen ;
	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
	ptLineSt[2] = Param.m_dCurZValue ;
	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
	ptLineEd[2] = Param.m_dCurZValue ;
	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[0] )
		return FALSE ;
	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
	ptCutCent[2] = Param.m_dCurZValue ;
	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, Param.m_dCutDepth, 0 ) ;
	if ( !PathArr[1] )
		return FALSE ;
	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
	{
		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 1 ) ;
	}
	Param.m_dCurZValue += Param.m_dCutDepth ;

	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
	dt = ( Param.m_dTopLeadLen + Param.m_dCutRad ) / Param.m_dCutRad ;
	dt = max ( dt, 1.0 ) ;
	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
	ptLineEd[2] = ptLineSt[2] ;
	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !PathArr[2] )
		return FALSE ;
	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;

	return TRUE ;
}

//BOOL CSmartThreadMillGen::CreatExtLftCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadLen, DOUBLE BottomZ )
//{
//	if ( !SmtPoint || !TComb )
//		return FALSE ;
//
//	double dCutDepth = BottomZ ;
//	int nTeeth = GetTeethNum() ;
//	if ( nTeeth < 1 )
//		return FALSE ;
//
//	CPathEntity *pPathArr[3] = { NULL, NULL, NULL } ;
//	THREAD_PARAM param ;
//	param.m_dCutRad = CutRad ;
//	param.m_dCutPitch = m_cParam.m_dPitch ;
//	param.m_dBotLeadLen = param.m_dTopLeadLen = BotLeadLen ;
//	
//	double dThreadDepth = dCutDepth ;
//
//	if ( nTeeth > 1 && m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		double dZMove = m_cToolDef.m_dHeight - m_cParam.m_dPitch ;
//
//		double dPeriodLen = param.m_dCutDepth + dZMove ;
//		param.m_dCurZValue = -dThreadDepth ;
//		while ( param.m_dCurZValue + dPeriodLen < -MIN_LEN )
//		{
//			param.m_dCurZValue += dPeriodLen ;
//		}
//		if ( param.m_dCurZValue < -MIN_LEN )
//		{
//			param.m_dCurZValue += param.m_dCutDepth ;
//		}
//
//		double dJugZ = -dCutDepth + param.m_dCutDepth - MIN_LEN ;
//		while ( param.m_dCurZValue > dJugZ )
//		{
//			GenExtLftCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
//			AddCutPathToComb ( pPathArr, 3, TComb ) ;
//			param.m_dCurZValue -= dZMove ;
//		}
//		return TRUE ;
//	}
//
//	if ( nTeeth < 2 )
//	{
//		param.m_dCutDepth = dCutDepth ;
//		param.m_dCurZValue = 0.0 ;
//	}
//	else if ( m_cToolDef.m_dHeight < dThreadDepth - MIN_LEN 
//		&& !( m_cParam.m_nThreadMillFlag & NCDEF_THREADMILL_AXIALDIVIDE ) )
//	{
//		param.m_dCutDepth = dCutDepth - m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//		param.m_dCurZValue = -m_cToolDef.m_dHeight + m_cParam.m_dPitch ;
//	}
//	else if ( m_cToolDef.m_dHeight >= dThreadDepth - MIN_LEN )
//	{
//		param.m_dCutDepth = m_cParam.m_dPitch ;
//		param.m_dCurZValue = -dCutDepth + param.m_dCutDepth ;
//	}
//	
//	GenExtLftCvtPath_LineLead ( SmtPoint, param, pPathArr ) ;
//	return AddCutPathToComb ( pPathArr, 3, TComb ) ;
//}

//BOOL CSmartThreadMillGen::GenExtLftCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] )
//{
//	PathArr[0] = NULL ; PathArr[1] = NULL ; PathArr[2] = NULL ;
//	if ( !SmtPoint || Param.m_dCutRad < MIN_LEN )
//		return FALSE ;
//
//	PNT3D ptCutCent, ptLineSt, ptLineEd ;
//	double dt ;
//	
//	ptLineSt[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad + Param.m_dTopLeadLen ;
//	ptLineSt[1] = SmtPoint->m_dPoint[1] ;
//	ptLineSt[2] = Param.m_dCurZValue ;
//	ptLineEd[0] = SmtPoint->m_dPoint[0] + Param.m_dCutRad ;
//	ptLineEd[1] = SmtPoint->m_dPoint[1] ;
//	ptLineEd[2] = Param.m_dCurZValue ;
//	PathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[0] )
//		return FALSE ;
//	PathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	nc_VectorCopy ( ptCutCent, SmtPoint->m_dPoint, 2 ) ;
//	ptCutCent[2] = Param.m_dCurZValue ; 
//	PathArr[1] = CreatHelixPath ( ptCutCent, Param.m_dCutRad, Param.m_dCutPitch, 0.0, -Param.m_dCutDepth, 1 ) ;
//	if ( !PathArr[1] )
//		return FALSE ;
//	if ( m_cParam.m_nRateType == NCDEF_THREADMILL_RATE_CC )
//	{
//		PathArr[1]->m_fFeedScale = (float) CalHelixFeedScale ( Param.m_dCutRad, Param.m_dCutPitch, 1 ) ;
//	}
//	Param.m_dCurZValue -= Param.m_dCutDepth ;
//
//	PathArr[1]->GetEndPoint ( 1, ptLineSt ) ;
//	dt = ( Param.m_dBotLeadLen + Param.m_dCutRad ) / Param.m_dCutRad ;
//	dt = max ( dt, 1.0 ) ;
//	nc_GetPointAtLine ( SmtPoint->m_dPoint, ptLineSt, dt, ptLineEd, 2 ) ;
//	ptLineEd[2] = ptLineSt[2] ;
//	PathArr[2] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
//	if ( !PathArr[2] )
//		return FALSE ;
//	PathArr[2]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
//
//	return TRUE ;
//}

CPathArc3D* CSmartThreadMillGen::CreatHelixPath ( PNT3D  Center,  //起点中心点
												  DOUBLE Radius,  //半径
												  DOUBLE Pitch,   //螺距
												  DOUBLE StAngle, //起始角度
												  DOUBLE Depth,   //深度：>0：从下向上；<0：从上向下
												  BOOL   RotDir ) //旋转方向：0：顺时针；1：逆时针
{
	if ( Radius < 0.02 || Pitch < MIN_DIS )
		return NULL ;
	CSmartArc smtArc ;

	double dAngRange = fabs ( MiniPai2 * Depth / Pitch ) ;
	nc_VectorCopy ( smtArc.m_aCenter, Center, 2 ) ;
	smtArc.m_dRadius = Radius ;
	smtArc.m_aAngle[0] = StAngle ;
	if ( RotDir == 0 )
		smtArc.m_aAngle[1] = smtArc.m_aAngle[0] - dAngRange ;
	else
		smtArc.m_aAngle[1] = smtArc.m_aAngle[0] + dAngRange ;
	
	CPathArc3D *pArcPath = new CPathArc3D () ;
	pArcPath->Create ( Center[2], smtArc ) ;
	pArcPath->m_fDepth = Depth ;

	return pArcPath ;
}

DOUBLE CSmartThreadMillGen::CalHelixFeedScale ( DOUBLE PathRad, DOUBLE Pitch, BOOL CutSide ) 
{
	double dToolRad = 0.5 * m_cToolDef.m_dTopDiam ;
	double dScale, dCutRad, dCirclePath, dCircleCut, dHelixPath, dHelixCut ;

	if ( CutSide == 0 )
		dCutRad = PathRad + dToolRad ;
	else
		dCutRad = PathRad - dToolRad ;

	dCirclePath = MiniPai2 * PathRad ;
	dCircleCut = MiniPai2 * dCutRad ;
	dHelixPath = sqrt ( dCirclePath * dCirclePath + Pitch * Pitch ) ;
	dHelixCut = sqrt ( dCircleCut * dCircleCut + Pitch * Pitch ) ;
	if ( dHelixCut < MIN_LEN )
		return 1 ;
	dScale = dHelixPath / dHelixCut ;
	
	return dScale ;
}

DOUBLE CSmartThreadMillGen::CalLeadArcAngle ( DOUBLE MinLeadRad, DOUBLE MaxLeadRad )
{
	double dLeftAng, dToolRad = 0.5 * m_cToolDef.m_dTopDiam ;
	if ( GetThreadMillRCompMask () != 0 )
	{
		double dWearLine = m_cFeedDef.m_cLeadDef.m_dWearLine ;
		if ( dWearLine < MIN_DIS )
			dWearLine = MIN_DIS ;
		
		if ( 1.41421356 * MinLeadRad >= dWearLine )
			return MiniPai1_2 ;
		else if ( dWearLine > 2 * MinLeadRad )
			return 0 ;
		else 
			dLeftAng = 2 * mathASin ( 0.5 * dWearLine / MinLeadRad ) ;
	}
	else
	{
		double dH = 0.866025404 * m_cParam.m_dPitch ;
		double dHoleRad = min ( 0.5 * m_cParam.m_dHoleDiameter, 0.5 * m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ - dH ) ;
		double dDis = dHoleRad - dToolRad - 0.1 ;
		if ( dDis < 0.1 )
			return MiniPai ;

		dLeftAng = 2 * mathASin ( 0.5 * dDis / MaxLeadRad ) ;
	}

	return MiniPai - dLeftAng ;
}

DOUBLE CSmartThreadMillGen::CalLeadLineOff ( DOUBLE MinCutRad ) 
{
	double dToolRad = 0.5 * m_cToolDef.m_dTopDiam ;
	double dLeft = 0 ;
	double dH = 0.866025404 * m_cParam.m_dPitch ;
	double dHoleRad = min ( 0.5 * m_cParam.m_dHoleDiameter, 0.5 * m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ - dH ) ;
	dLeft = dHoleRad - dToolRad - 0.1 ;
	if ( dLeft < 0.1 )
		return 0 ;

	if ( GetThreadMillRCompMask () != 0 )
	{
		double dWearLine = m_cFeedDef.m_cLeadDef.m_dWearLine ;
		if ( dWearLine < MIN_DIS )
			dWearLine = MIN_DIS ;

		double dWearLeft = MinCutRad - dWearLine ;
		if ( dWearLeft < 0.1 )
			return 0 ;
		return min ( dLeft, dWearLeft ) ;
	}

	return dLeft ;
}

DOUBLE CSmartThreadMillGen::CalExternalLeadArcAngle ( DOUBLE MinLeadRad ) 
{
	double dH = 0.5 * ( m_cParam.m_dCylDiameter - m_dModiMinDiam ) ;
	double dLen = dH + 1 ; //进退刀离开圆柱料1mm

	if ( dLen > MinLeadRad )
		return MiniPai ;
	else
	{
		double dSafeRad = 0.5 * ( m_cParam.m_dCylDiameter + m_cToolDef.m_dTopDiam ) + 1 ;
		double dDis = 0.5 * ( m_dModiMinDiam + m_cToolDef.m_dTopDiam ) + MinLeadRad ;
		double dDivisor = dDis * MinLeadRad ;
		if ( dDivisor < MIN_LEN )
			return MiniPai ;
		double dDividend = dDis * dDis + MinLeadRad * MinLeadRad - dSafeRad * dSafeRad ;
		return mathACos ( 0.5 * dDividend / dDivisor ) ;
	}
}

DOUBLE CSmartThreadMillGen::CalExternalLeadLineLen ()
{
	double dH = 0.866025404 * m_cParam.m_dPitch ;
	double dLen = dH + 1 ; //进退刀离开圆柱料1mm

	if ( GetThreadMillRCompMask () != 0 )
	{
		double dWearLine = m_cFeedDef.m_cLeadDef.m_dWearLine ;
		return max ( dWearLine, dLen ) ;
	}

	return dLen ;
}

CPathPLine3D* CSmartThreadMillGen::CreateQuickPath ( PNT3D Start, PNT3D End, DOUBLE H )
{
	if ( nc_Distance ( Start, End, 3 ) < MIN_LEN )
		return NULL ;
	CPathPLine3D *pLine = new CPathPLine3D() ;

	if ( nc_Distance ( Start, End, 2 ) < MIN_LEN 
		|| fabs ( Start[2] - End[2] ) < MIN_LEN && fabs ( Start[2] - H ) < MIN_LEN )
	{
		pLine->m_nCount = 1 ;
		pLine->m_pTAPos = new PNT3D[pLine->m_nCount+1] ;
		mathCpyPnt ( Start, pLine->m_pTAPos[0] ) ;
		mathCpyPnt ( End, pLine->m_pTAPos[1] ) ;
	}
	else if ( Start[2] > End[2] && fabs ( Start[2] - H ) < MIN_LEN )
	{
		pLine->m_nCount = 2 ;
		pLine->m_pTAPos = new PNT3D[pLine->m_nCount+1] ;
		
		mathCpyPnt ( Start, pLine->m_pTAPos[0] ) ;
		nc_VectorCopy ( pLine->m_pTAPos[1], End, 2 ) ;
		pLine->m_pTAPos[1][2] = Start[2] ;
		mathCpyPnt ( End, pLine->m_pTAPos[2] ) ;
	}
	else if ( End[2] > Start[2] && fabs ( End[2] - H ) < MIN_LEN )
	{
		pLine->m_nCount = 2 ;
		pLine->m_pTAPos = new PNT3D[pLine->m_nCount+1] ;

		mathCpyPnt ( Start, pLine->m_pTAPos[0] ) ;
		nc_VectorCopy ( pLine->m_pTAPos[1], Start, 2 ) ;
		pLine->m_pTAPos[1][2] = End[2] ;
		mathCpyPnt ( End, pLine->m_pTAPos[2] ) ;
	}
	else
	{
		pLine->m_nCount = 3 ;
		pLine->m_pTAPos = new PNT3D[pLine->m_nCount+1] ;

		mathCpyPnt ( Start, pLine->m_pTAPos[0] ) ;
		nc_VectorCopy ( pLine->m_pTAPos[1], Start, 2 ) ;
		pLine->m_pTAPos[1][2] = H ;
		nc_VectorCopy ( pLine->m_pTAPos[2], End, 2 ) ;
		pLine->m_pTAPos[2][2] = H ;
		mathCpyPnt ( End, pLine->m_pTAPos[3] ) ;
	}

	pLine->m_bFeedType = JDNC_FEEDTYPE_QUICK ;
	return pLine ;
}

int CSmartThreadMillGen::GetTeethNum ()
{
	/*if ( m_cParam.m_dPitch < MIN_LEN )
		return 0 ;

	double dTeeth = m_cToolDef.m_dHeight / m_cParam.m_dPitch ;
	int nTeeth = int ( dTeeth ) ;
	if ( fabs ( nTeeth - dTeeth ) > MIN_LEN )
	{
		nTeeth = int ( ceil ( dTeeth ) ) ;
		if ( fabs ( nTeeth - dTeeth ) > MIN_LEN )
		{
			nTeeth -- ;
		}
	}

	return nTeeth ;*/
	return m_cToolDef.m_nTeeth ;
}

BOOL CSmartThreadMillGen::AddCutPathToComb ( CPathEntity* PathEnt[], int Num, CPathCombine *TComb )
{
	if ( Num < 1 )
		return FALSE ;
	for ( int i = 0 ; i < Num ; i ++ )
	{
		if ( !PathEnt[i] )
			return FALSE ;
	}

	// 如果加工域为新点，m_bNewCutPoint为TRUE，则与上条路径按照相对定位高度连刀 qqs 2014.03.21
	if ( TComb->m_pTail && !m_bNewCutPoint)
	{
		PNT3D pt1, pt2 ;
		TComb->m_pTail->GetEndPoint ( 1, pt1 ) ;
		PathEnt[0]->GetEndPoint ( 0, pt2 ) ;
		if ( nc_Distance ( pt1, pt2, 2 ) < 0.75 * m_dModiNormDiam /*m_cParam.m_dNomDiameter*/ )
		{
			CPathPLine3D *pPathQuic = CreateQuickPath ( pt1, pt2, max ( pt1[2], pt2[2] ) ) ;
			if ( pPathQuic )
				TComb->AddEntity ( pPathQuic ) ;
		}
	}
	m_bNewCutPoint = FALSE;

	for ( int i = 0 ; i < Num ; i ++ )
	{
		if ( PathEnt[i]->GetType() == NC_PATH_ARC3D )
		{//圆弧路径有的是多于一个周期的螺旋线，违反使用规定，需分段表示
			CPathArc3D *pPathArc3D = (CPathArc3D*) PathEnt[i] ;
			if ( fabs ( pPathArc3D->m_fAngle[1] - pPathArc3D->m_fAngle[0] ) > MiniPai2 + MIN_ANG ) //螺旋线分割成角度在360度以内的子段
			{
				CSmartArc smtArc ;
				double dDepth, dCurZ, dHoleDepth, dAngRange, dAngLeft, dAng ;
				CPathArc3D *pNewArc ;

				nc_VectorCopy ( smtArc.m_aCenter, pPathArc3D->m_fCenter, 2 ) ;
				dCurZ = pPathArc3D->m_fCenter[2] ;
				smtArc.m_dRadius = pPathArc3D->m_fRadius ;
				smtArc.m_aAngle[0] = pPathArc3D->m_fAngle[0] ;

				dAngRange = dAngLeft = pPathArc3D->m_fAngle[1] - pPathArc3D->m_fAngle[0] ;
				double dARanRecip = 1.0 / dAngRange ;
				dHoleDepth = pPathArc3D->m_fDepth ;

				while ( fabs(dAngLeft) > MiniPai2 + MIN_ANG )
				{
					if ( dAngRange > 0 )
						dAng = MiniPai2 ;
					else
						dAng = -MiniPai2 ;

					smtArc.m_aAngle[1] = smtArc.m_aAngle[0] + dAng ;
					dDepth = dHoleDepth * dAng * dARanRecip ;
					pNewArc = new CPathArc3D() ;
					pNewArc->Create ( dCurZ, smtArc ) ;
					pNewArc->m_fDepth = dDepth ;

					pNewArc->m_bFeedType = pPathArc3D->m_bFeedType ;
					pNewArc->m_fFeedScale = pPathArc3D->m_fFeedScale ;
					pNewArc->m_bMoveFlag = pPathArc3D->m_bMoveFlag ;

					TComb->AddEntity (pNewArc) ;

					dAngLeft -= dAng ;
					smtArc.m_aAngle[0] = smtArc.m_aAngle[1] ;
					dCurZ += dDepth ;
				}

				if ( fabs (dAngLeft) > MIN_ANG )
				{
					smtArc.m_aAngle[1] = smtArc.m_aAngle[0] + dAngLeft ;
					dDepth = dHoleDepth * dAngLeft * dARanRecip ;
					pNewArc = new CPathArc3D() ;
					pNewArc->Create ( dCurZ, smtArc ) ;
					pNewArc->m_fDepth = dDepth ;

					pNewArc->m_bFeedType = pPathArc3D->m_bFeedType ;
					pNewArc->m_fFeedScale = pPathArc3D->m_fFeedScale ;
					pNewArc->m_bMoveFlag = pPathArc3D->m_bMoveFlag ;

					TComb->AddEntity (pNewArc) ;
				}

				delete pPathArc3D ;
			}
			else
			{
				TComb->AddEntity ( PathEnt[i] ) ;
			}
		}
		else
		{
			TComb->AddEntity ( PathEnt[i] ) ;
		}
	}
	
	return TRUE ;
}

int CSmartThreadMillGen::GetThreadMillRCompMask ()
{
	//暂时不用顺逆铣控制，统一由下向上走刀，走刀方向已随螺纹类型而定
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC )
    {
		if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALRIGHT 
			|| m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALLEFT )
			return JDNC_FGCODE_RCOMPG42 ;
        //if ( GetMillDir() == 0 ) 
		else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALLEFT 
			|| m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALRIGHT )
			return JDNC_FGCODE_RCOMPG41 ;
    }
    else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC )
    {
		if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALRIGHT 
			|| m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALLEFT )
			return JDNC_FGCODE_RCOMPG41 ;
        //if ( GetMillDir() == 0 ) 
		else if ( m_cParam.m_nThreadMillType == NCDEF_THREADMILL_INTERNALLEFT 
			|| m_cParam.m_nThreadMillType == NCDEF_THREADMILL_EXTERNALRIGHT )
			return JDNC_FGCODE_RCOMPG42 ;
    }
    return 0 ;
}

BOOL CSmartThreadMillGen::SetRCompMask ( CPathEntity* PathArr[], int Num, int RCompMask )
{
	if ( RCompMask == 0 ) 
		return FALSE ;
	int nMaxSer = Num - 1 ;
    for ( int i = 0 ; i < nMaxSer ; i++ ) /*末段关闭半径补偿*/ 
    {
        PathArr[i]->m_bMoveFlag |= RCompMask ;
    }
    return TRUE ;
}

int CSmartThreadMillGen::GetLeadType ()
{
	int nLeadType = m_cFeedDef.m_cLeadDef.m_nLeadType ;
	if ( nLeadType != NCDEF_LEAD_SLINE && nLeadType != NCDEF_LEAD_ARC )
		nLeadType = NCDEF_LEAD_ARC ;

	return nLeadType ;
}

BOOL CSmartThreadMillGen::CalCutDepthByLayer ( DOUBLE ToothH, BOOL IsoVol, int LayerCnt, DOUBLE *&CutDepth )
{
	CutDepth = NULL ;
	if ( ToothH < MIN_LEN || LayerCnt < 1 )
		return FALSE ;

	CutDepth = new double [LayerCnt] ;

	if ( IsoVol )
	{
		double dLayerCnt = LayerCnt ;
		CutDepth[0] = ToothH / sqrt ( dLayerCnt ) ;
		double dSer ;
		for ( int i = 1 ; i < LayerCnt ; i ++ )
		{
			dSer = i + 1 ;
			CutDepth[i] = CutDepth[0] * sqrt ( dSer ) ;
		}
	}
	else
	{
		double dDepth = ToothH / LayerCnt ;
		for ( int i = 0 ; i < LayerCnt ; i ++ )
		{
			CutDepth[i] = ( i + 1 ) * dDepth ;
		}
	}

	return TRUE ;
}

BOOL CSmartThreadMillGen::CalCutDepthByMaxDepth ( DOUBLE ToothH, BOOL IsoVol, DOUBLE MaxDepth, 
												 int &LayerCnt, DOUBLE *&CutDepth )
{
	CutDepth = NULL ;
	if ( ToothH < MIN_LEN || MaxDepth < MIN_LEN )
		return FALSE ;

	if ( IsoVol )
	{
		double dt = ( ToothH * ToothH ) / ( MaxDepth * MaxDepth ) ;
		LayerCnt = int ( ceil ( dt ) ) ;
		CutDepth = new double [LayerCnt] ;

		double dLayerCnt = LayerCnt ;
		CutDepth[0] = ToothH / sqrt ( dLayerCnt ) ;
		double dSer ;
		for ( int i = 1 ; i < LayerCnt ; i ++ )
		{
			dSer = i + 1 ;
			CutDepth[i] = CutDepth[0] * sqrt ( dSer ) ;
		}
	}
	else
	{
		double dDepth = MaxDepth ;
		LayerCnt = int ( ceil ( ToothH / dDepth ) ) ;
		CutDepth = new double [LayerCnt] ;
		
		dDepth = ToothH / LayerCnt ;
		for ( int i = 0 ; i < LayerCnt ; i ++ )
		{
			CutDepth[i] = ( i + 1 ) * dDepth ;
		}
	}

	return TRUE ;
}

BOOL CSmartThreadMillGen::CalAllBottomZ ( int LayerCnt, DOUBLE *CutDepth, DOUBLE &BottomZ, BOOL SideInc, DOUBLE *BotZArr ) 
{
	if ( !CutDepth || !BotZArr )
		return FALSE ;

	if ( SideInc )
	{
		double dZOff ;
		if ( LayerCnt > 2 )
		{
			if ( m_cParam.m_nDepthType == NCDEF_THREADMILL_DEPTH_PATHDEPTH ) //深度设置方式是“路径范围”，
			{                                                                //侧向进给分层路径要留出分层Z向分层的空间
				dZOff = ( CutDepth[LayerCnt-1] - CutDepth[1] ) * 0.57735027 ; //截面60度的螺纹
				BottomZ -= dZOff ;
			}
		}

		for ( int i = 0 ; i < LayerCnt-1 ; i ++ )
		{
			dZOff = 0.95 * ( CutDepth[LayerCnt-1] - CutDepth[i] ) * 0.57735027 ;
			if ( i % 2 )
				BotZArr[i] = BottomZ + dZOff ;
			else
				BotZArr[i] = BottomZ - dZOff ;
		}
		BotZArr[LayerCnt-1] = BottomZ ;
	}
	else
	{
		for ( int i = 0 ; i < LayerCnt ; i ++ )
		{
			BotZArr[i] = BottomZ ;
		}
	}

	return TRUE ;
}

BOOL CSmartThreadMillGen::AddExtPathCompSeg ( CPathEntity* InPathArr[3], CPathEntity* OutPathArr[5] ) 
{
	if ( !InPathArr[0] || !InPathArr[2] )
		return FALSE ;

	OutPathArr[1] = InPathArr[0] ;
	OutPathArr[2] = InPathArr[1] ;
	OutPathArr[3] = InPathArr[2] ;
	double dOffLen = m_cFeedDef.m_cLeadDef.m_dWearLine ;
	VEC3D vTan ;
	PNT3D ptLineSt, ptLineEd ;

	OutPathArr[1]->GetEndPoint ( 0, ptLineEd ) ;
	OutPathArr[1]->GetEndTangent ( 0, vTan ) ;
	mathAddVec ( ptLineEd, vTan, -dOffLen, ptLineSt ) ;
	OutPathArr[0] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !OutPathArr[0] )
		return FALSE ;
	OutPathArr[0]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	OutPathArr[0]->m_fFeedScale = OutPathArr[1]->m_fFeedScale ;

	OutPathArr[3]->GetEndPoint ( 1, ptLineSt ) ;
	OutPathArr[3]->GetEndTangent ( 1, vTan ) ;
	mathAddVec ( ptLineSt, vTan, dOffLen, ptLineEd ) ;
	OutPathArr[4] = new CPathLine3D ( ptLineSt, ptLineEd ) ;
	if ( !OutPathArr[4] )
		return FALSE ;
	OutPathArr[4]->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	OutPathArr[4]->m_fFeedScale = OutPathArr[3]->m_fFeedScale ;

	return SetRCompMask ( OutPathArr, 5, GetThreadMillRCompMask() ) ;
}
