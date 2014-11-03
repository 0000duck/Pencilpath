#include "StdAfx.H"
#include "SmartNC.H"
#include "SurfGeo.H"
#include "SmartBound.H"
#include "SmartVoro.H"
#include "SmtPathGen.H"
#include "SmtPathGen2D.H"
#include "NcGCode.H"
#include "SmartPocket.H"
#include <vector>
#include <algorithm>
using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

double MathCAM_MiniDistToContour( CSmartLoop& Contour, DOUBLE Point[2] );
void MathCAM_SetContoursStart( CSmartLoop* AllCont, JDNC_LEAD& LeadDef,
							  CSmartGraphic& Graph, JDNC_SETUP& Setup );

///////////////////////
// CSmartCombMillGen
CSmartCombMillGen::CSmartCombMillGen()
{
	m_nCurToolIndex = 0;
}
CSmartCombMillGen::~CSmartCombMillGen()
{
}

int CSmartCombMillGen::LoopOneLayer( CPathCombine& PComb,	/*路径集合*/
									CSmartLoop&    Contour, /*切割轮廓*/
									JDNC_COMBMILL&,			/*切割参数*/ 
									DOUBLE DepthInc)		/*切割深度*/
{
	//  STEP 0 : 偏移轮廓, 计算下刀保护边界 
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		Plunge.m_dIncStep =  DepthInc  + Plunge.m_dTopTol ;
	}
	// STEP 1 : 提取所有轮廓 , 并生成路径
	CSmtLoopArr   AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType , AllLoop ) ;
	INT_PTR nLoop = AllLoop.GetSize() ;
	CSmartCurve* pBound  ;
	PNT3D  dPoint, dAtPoint ;
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;

	JDNC_LEAD  cColliLead = rcompLead;
	double dExtLen = m_cParam.m_dSafeDiam[m_nCurToolIndex] - m_cParam.m_dTipDiam[m_nCurToolIndex];
	dExtLen /= 2.0;
	if (dExtLen > 0.0001) cColliLead.m_dLength = dExtLen;
	else cColliLead.m_dLength = -1;
	for( INT_PTR i = 0 ; i < nLoop ; i ++ ) 
	{
		if( fabs( AllLoop[i]->m_dArea ) > 1.0e10 || 
			(AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER)  || 
			AllLoop[i]->m_pCurve == NULL  )
		{
			continue  ;
		}
		pBound = AllLoop[i]->m_pCurve->CopyMyself() ;
		if( ! pBound ) continue ;
		pBound->SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;
		if( m_cFeedDef.m_cLeadDef.m_nLeadType != NCDEF_LEAD_CLOSE )
		{/* 1: 尽可能搜索到切入切出 */
			CSmartSect *pLeadIn = NULL, * pLeadOut= NULL;
			JDNC_LEAD  Lead = m_cFeedDef.m_cLeadDef ;
			CreateBoundLead( Lead ,
				*pBound   , 
				pLeadIn  ,   
				pLeadOut , 
				&Contour ) ;
			if( pLeadIn ) pBound->InsertAfter( pLeadIn, NULL ) ;
			if( pLeadOut) pBound->AddSect( pLeadOut ) ;
		} 
		if( GetMillDir() == 0 ) 
		{
			pBound->Reverse() ; 
		}
		if( nRCompMask != 0 ) 
		{
			AddLeadSectAtEnd( rcompLead, *pBound, &Contour ) ;
		}
		if (cColliLead.m_dLength > 0 && 
			Plunge.m_nPlungeType==NCDEF_PLUNGE_CLOSE)
		{// 沿轮廓下刀时不加进退刀干涉处理
			AddLeadSectAtEndEx( cColliLead, *pBound, &Contour ) ;
		}
		CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
		tmpBase.AddCurve( pBound , FALSE , 0.0, TRUE ) ;
		AddRCompMask( tmpBase,  nRCompMask ) ;
		if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
		{ // 增加下刀方式
			pBound->GetPoint( 0.0, dPoint ) ;
			dPoint[2]  = 0  ;
			CSmartLoop *pChild = NULL ; 
			if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT ||
				Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX    || 
				Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP    )
			{
				DOUBLE dOffset = Plunge.m_dSideTol ; 
				if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX ) 
				{
					double dDist = MathCAM_MiniDistToContour( Contour, dPoint ) ;
					dOffset += max( Plunge.m_dRadius, dDist) ;
				}
				pChild = Contour.OffsetContourEx( dOffset , dOffset, 
					m_cSetupDef.m_cCorDef ) ;
			}
			CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
			if( FindPlungePoint( Plunge, Contour, pChild, dPoint, dAtPoint, pBound ) )
			{ // 在固定位置下刀
				AddPlungePathAt( cTPlunge, Plunge, dPoint, dAtPoint,&Contour ) ;
			}
			if( ! cTPlunge.m_pHead )
			{
				AddPlungePathBy( cTPlunge, Plunge , dPoint, *pBound, TRUE ) ;
			}
			PComb.AppendCombine( cTPlunge ) ;
			Mini_DeleteContours( pChild ) ;
		}
		delete pBound ;
		PComb.AppendCombine( tmpBase ) ;
	}
	return 1 ; 
}

int CSmartCombMillGen::AddLeadSectAtEndEx( JDNC_LEAD& LeadDef, 
										  CSmartCurve& Curve , 
										  CSmartLoop*  AllCont ) 
{
	PNT3D dPoint[2], dTan[2], dNor[2], dMidNor;
	if( ! Curve.m_pHead ) return 0 ;
	Curve.m_pHead->GetStart(dPoint[0] ) ;
	Curve.m_pHead->GetTangent( 0.0, dTan[0] ) ;
	Curve.m_pHead->GetNormal( 0.0, dNor[0] ) ;
	nc_VectorReverse( dTan[0], 2 ) ;
	Curve.m_pTail->GetEnd( dPoint[1] ) ;
	Curve.m_pTail->GetTangent( 1.0, dTan[1] ) ;
	Curve.m_pTail->GetNormal( 1.0, dNor[1] ) ;
	if( GetMillDir() == 0 ) 
	{
		nc_VectorReverse( dNor[0], 2 ) ;
		nc_VectorReverse( dNor[1], 2 ) ;
	}
	BOOL bIsClosed = FALSE ;
	if( nc_Distance( dPoint[0], dPoint[1], 2 ) < 0.01 )
	{
		dMidNor[0] = dNor[0][0] + dNor[1][0] ;
		dMidNor[1] = dNor[0][1] + dNor[1][1] ;
		nc_Normalize( dMidNor, 2 ) ;
		bIsClosed = TRUE ;
	}
	double dTemp, dDist = MathCAM_MiniDistToContour(*AllCont, dPoint[0]);
	dDist = LeadDef.m_dLength - dDist;
	dTemp = LeadDef.m_dLength ;
	if (dDist > 0.001)
	{
		LeadDef.m_dLength = dDist;
		CSmartSect* pSect = CreateLeadOutEnt( LeadDef, dPoint[0], dTan[0], dNor[0], FALSE, AllCont ) ;
		if( pSect == NULL && bIsClosed ) 
		{
			JDNC_LEAD tmpLead = LeadDef ;
			tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
			pSect = CreateLeadOutEnt( tmpLead, dPoint[0], dTan[0], dMidNor, FALSE, AllCont ) ;
		}
		if( pSect )  Curve.InsertAfter( pSect, NULL ) ;
	}
	LeadDef.m_dLength = dTemp ;
	dDist = MathCAM_MiniDistToContour(*AllCont, dPoint[1]);
	dDist = LeadDef.m_dLength - dDist;
	if (dDist > 0.001)
	{
		LeadDef.m_dLength = dDist;
		CSmartSect*  pSect = CreateLeadOutEnt( LeadDef, dPoint[1], dTan[1], dNor[1], TRUE, AllCont ) ;
		if( pSect == NULL && bIsClosed ) 
		{
			JDNC_LEAD tmpLead = LeadDef ;
			tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
			pSect = CreateLeadOutEnt( tmpLead, dPoint[1], dTan[1], dMidNor, TRUE, AllCont ) ;
		}
		if( pSect ) Curve.AddSect( pSect ) ;
	}
	LeadDef.m_dLength = dTemp ;
	return 1 ;
}

int CSmartCombMillGen::AddLeadPathAtEndEx( JDNC_LEAD& LeadDef, 
										  CPathCombine& TComb, 
										  CSmartLoop&  Contour ) 
{
	PNT3D dPoint[2], dTan[2], dNor[2], dMidNor;
	TComb.m_pHead->GetEndPoint( 0, dPoint[0] ) ;
	TComb.m_pHead->GetEndTangent( 0, dTan[0] ) ;
	nc_VectorReverse( dTan[0], 2 ) ;
	TComb.m_pHead->GetEndNormal( 0, dNor[0] ) ;
	TComb.m_pTail->GetEndPoint( 1, dPoint[1] ) ;
	TComb.m_pTail->GetEndTangent( 1, dTan[1] ) ;
	TComb.m_pTail->GetEndNormal( 1, dNor[1] ) ;
	nc_Normalize( dTan[0], 2 ) ;
	nc_Normalize( dTan[1], 2 ) ;
	if( GetMillDir() == 0 ) 
	{
		nc_VectorReverse( dNor[0], 2 ) ;
		nc_VectorReverse( dNor[1], 2 ) ;
	}
	BOOL bIsClosed = FALSE ;
	if( nc_Distance( dPoint[0], dPoint[1], 2 ) < 0.01 )
	{
		dMidNor[0] = dNor[0][0] + dNor[1][0] ;
		dMidNor[1] = dNor[0][1] + dNor[1][1] ;
		nc_Normalize( dMidNor, 2 ) ;
		bIsClosed = TRUE ;
	}
	CSmartLoop * tmpCont = Contour.CopyContour() ;

	double dTemp, dDist = Contour.MinDistPoint(dPoint[0]);
	dDist = LeadDef.m_dLength - dDist;
	dTemp = LeadDef.m_dLength ;
	if (dDist > 0.001)
	{
		LeadDef.m_dLength = dDist;
		CSmartSect* pSect = CreateLeadOutEnt( LeadDef, dPoint[0], dTan[0], dNor[0], FALSE, tmpCont ) ;
		if( pSect == NULL && bIsClosed ) 
		{
			JDNC_LEAD tmpLead = LeadDef ;
			tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
			pSect = CreateLeadOutEnt( tmpLead, dPoint[0], dTan[0], dMidNor, FALSE, tmpCont ) ;
		}
		CPathEntity* pLead = GetLeadEnt( pSect, dPoint[0][2] ) ;
		if( pLead )
		{
			pLead->m_bFeedType = pSect->m_bFeedType ;
			TComb.InsertAfter( pLead, NULL ) ;
		}
		if( pSect)    delete pSect ;
	}
	LeadDef.m_dLength = dTemp ;
	dDist = Contour.MinDistPoint(dPoint[1]);
	dDist = LeadDef.m_dLength - dDist;
	if (dDist > 0.001)
	{
		LeadDef.m_dLength = dDist;
		CSmartSect* pSect = CreateLeadOutEnt( LeadDef, dPoint[1], dTan[1], dNor[1], TRUE, tmpCont ) ;
		if( pSect == NULL && bIsClosed ) 
		{
			JDNC_LEAD tmpLead = LeadDef ;
			tmpLead.m_nLeadType =  NCDEF_LEAD_SLINE ;
			pSect = CreateLeadOutEnt( tmpLead, dPoint[1], dTan[1],dMidNor, TRUE, tmpCont ) ;
		}
		CPathEntity* pLead = GetLeadEnt( pSect, dPoint[1][2] ) ;
		if( pLead )
		{
			pLead->m_bFeedType = pSect->m_bFeedType ;
			TComb.AddEntity( pLead) ;
		}
		if( pSect ) delete pSect ;
	}
	Mini_DeleteContours( tmpCont ) ;
	return 1 ;
}

int CSmartCombMillGen::CopyOneContour( CPathCombine& PComb,    /*数据*/
									  CSmartLoop& Contour,    /*轮廓*/
									  JDNC_LAYER&  Layer ,    /*参数*/
									  JDNC_PLUNGE& Plunge)    /*下刀*/
{
	CSmtLoopArr  AllLoop ;
	Contour.ExtractAllLoop( m_cSetupDef.m_cOrderDef.m_nSortType, AllLoop ) ;
	INT_PTR n = AllLoop.GetSize() ; 
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;

	JDNC_LEAD  cColliLead = rcompLead;
	double dExtLen = m_cParam.m_dSafeDiam[m_nCurToolIndex] - m_cParam.m_dTipDiam[m_nCurToolIndex];
	dExtLen /= 2.0;
	if (dExtLen > 0.0001) cColliLead.m_dLength = dExtLen;
	else cColliLead.m_dLength = -1;
	for( INT_PTR i = 0 ; i < n ; i ++ )
	{
		if( fabs( AllLoop[i]->m_dArea ) > 1.0e10 || 
			(AllLoop[i]->m_bUseFlag & NC_LOOP_OUTER)  || 
			AllLoop[i]->m_pCurve == NULL  )
		{
			continue  ;
		}
		CSmartCurve* pCurve = AllLoop[i]->m_pCurve->CopyMyself()   ;
		if( GetMillDir() == 0 ) pCurve->Reverse() ;
		DOUBLE dAngle = ANGLE_TO_RADIAN( Plunge.m_dAngle);
        if( Plunge.m_dIncStep > Plunge.m_dMaxZInc )
        {
            double dAngDepth = atan2( Plunge.m_dMaxZInc, pCurve->GetLength()) ;
            if( dAngle > dAngDepth ) dAngle = dAngDepth ;
        }
		if( Layer.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN )
		{// 分层不抬刀
			CPathCombine  LyComb( NC_WPROCESS_ROUGH )  ;
			for( int k = 1 ; k <= Layer.m_nLayerCount ; k ++ )
			{
				Plunge.m_dIncStep = Layer.m_dLayerDepth[ k ] - Layer.m_dLayerDepth[ k-1 ] ;
				if(  k == 1 ) Plunge.m_dIncStep += Plunge.m_dTopTol ; 
				CSmartCurve* pNewCurve = NULL  ;
				BOOL bOverLap = (k == Layer.m_nLayerCount) ? TRUE : FALSE ;
				LyComb.AddSlantCurve( *pCurve ,dAngle , 
				                      Plunge.m_dIncStep, 
					                  - Layer.m_dLayerDepth[k], 
					                  bOverLap ,
					                  &pNewCurve ) ;
				if( pNewCurve && pNewCurve != pCurve )
				{
					delete pCurve ;
					pCurve = pNewCurve  ;
				} 
			}
			AddLeadPathAtEnd( m_cFeedDef.m_cLeadDef , LyComb, Contour ) ;
			if( nRCompMask != 0 ) 
			{
				AddLeadPathAtEnd( rcompLead, LyComb, Contour ) ;
				AddRCompMask( LyComb, nRCompMask ) ;
			}
			if (cColliLead.m_dLength > 0 && 
				Plunge.m_nPlungeType==NCDEF_PLUNGE_CLOSE)
			{// 沿轮廓下刀时不加进退刀干涉处理				
				AddLeadPathAtEndEx( cColliLead, LyComb, Contour ) ;
			}
			PComb.AppendCombine( LyComb ) ;
		}
		else
		{
			for( int k = 1 ; k <= Layer.m_nLayerCount ; k ++ )
			{
				CPathCombine  LyComb( NC_WPROCESS_ROUGH )  ;
				Plunge.m_dIncStep = Layer.m_dLayerDepth[ k ] - Layer.m_dLayerDepth[ k-1 ] ;
				if(  k == 1 ) Plunge.m_dIncStep += Plunge.m_dTopTol ; 
				Plunge.m_dIncStep = Layer.m_dLayerDepth[ k ] - Layer.m_dLayerDepth[ k-1 ] ;
				Plunge.m_dIncStep += Plunge.m_dTopTol ; 
				LyComb.AddSlantCurve( *pCurve ,dAngle,
					                  Plunge.m_dIncStep,
					                  - Layer.m_dLayerDepth[k],
					                  TRUE ) ;
				AddLeadPathAtEnd( m_cFeedDef.m_cLeadDef,LyComb,Contour ) ;
				if( nRCompMask != 0 ) 
				{
					AddLeadPathAtEnd( rcompLead, LyComb, Contour ) ;
					AddRCompMask( LyComb, nRCompMask ) ;
				}
				if (cColliLead.m_dLength > 0 && 
					Plunge.m_nPlungeType==NCDEF_PLUNGE_CLOSE)
				{// 沿轮廓下刀时不加进退刀干涉处理						
					AddLeadPathAtEndEx( cColliLead, LyComb, Contour ) ;
				}
				PComb.AppendCombine( LyComb ) ;
			}
		}
		delete pCurve ;
	}
	return TRUE ;
}

BOOL CSmartCombMillGen::IsSameOffset()
{
	DOUBLE dAngle[2]; 
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
	if( m_cParam.m_nRCompSide ==  2 )
	{ /* 关闭补偿 */
		return TRUE ;
	}
	else if(m_cParam.m_bLoopFlag & NCDEF_FLOOP_USERRCOMP ) 
	{ /* 定义补偿值 */
		return TRUE ;
	}
	else if( m_cShapeDef.m_bAntiFace && dAngle[0] > 0.001 )
	{/* 底面效果 */
		if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{
			return TRUE ;
		}
	}
	else if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
	{ /*  拷贝分层  */
		return TRUE ;
	}
	else if( dAngle[0] < 0.001 && m_cToolDef.m_nToolType == surfncToolFlat )
	{
		return TRUE ;
	}
	return FALSE ;
}
CSmartLoop* CSmartCombMillGen::CreateLoopBound( CSmartLoop& Contour, 
											   DOUBLE TotalDepth  ,
											   DOUBLE Depth       ) 
{
	CSmartLoop *ContHead = NULL ; 
	DOUBLE dRComp[2] , dAngle[2]; 
	dAngle[0] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
	dAngle[1] = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dIslandAngle ) ;
	if( m_cParam.m_nRCompSide ==  2 )
	{ /* 关闭补偿 */
		dRComp[0] = dRComp[1] = 0.0 ;
	}
	else if( m_cShapeDef.m_bAntiFace && dAngle[0] > 0.001 )
	{/* 底面效果 */
		if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
		{
		}
		else
		{
			double dMyDepth = (TotalDepth - Depth ) * tan( dAngle[0] ) ;
			if( dMyDepth > 1.0e-3 )
			{
				dRComp[0] = dRComp[1] = dMyDepth ;
				ContHead = Contour.OffsetContour( NCDEF_OFFSET_OUTER,dRComp[0], dRComp[1],GetCorDef() ) ;
			}
		}
		dRComp[0] = m_pTool->GetRadiusComp( 0.0, 0.0) ; 
		dRComp[1] = m_pTool->GetRadiusComp( 0.0, 0.0) ; 
	}
	else if( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_COPYMODE )
	{ /*  拷贝分层  */
		dRComp[0] = m_pTool->GetRadiusComp( TotalDepth, dAngle[0]) ; 
		dRComp[1] = m_pTool->GetRadiusComp( TotalDepth, dAngle[1]) ; 
	}
	else 
	{
		dRComp[0] = m_pTool->GetRadiusComp( Depth, dAngle[0]) ; 
		dRComp[1] = m_pTool->GetRadiusComp( Depth, dAngle[1]) ; 
	}
	dRComp[0] += GetSideStock() ;
	dRComp[1] += GetSideStock() ;
	if( ContHead && fabs(dRComp[0]) > 1.0e-3 )
	{
		CSmartLoop* tmpLoop = ContHead; 
		if( dRComp[0] > 0.0 )
		{
			ContHead = Mini_OffsetContourList( tmpLoop ,NCDEF_OFFSET_INNER, dRComp,GetCorDef() ) ;
		}
		else
		{
			dRComp[0] =- dRComp[0] , dRComp[1] = -dRComp[1] ;
			ContHead = Mini_OffsetContourList( tmpLoop ,NCDEF_OFFSET_OUTER, dRComp,GetCorDef() ) ;
		}
		Mini_DeleteContours( tmpLoop ) ;
	}
	else
	{
		ContHead = Contour.OffsetContourEx( dRComp[0], dRComp[1],GetCorDef() ) ;
	}
	if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_AUTOSTART )
	{ /*优化路径起点*/
		BOOL bAt = 0 ;
		CSmartSect* pSect, *pObj ;
		PNT2D dSeed, dPnt ;
		for( CSmartLoop * pLoop = ContHead ; pLoop ; pLoop = pLoop->next )
		{
			for( CSmartLoop * pLp = pLoop ; pLp ; )
			{
				CSmartCurve * pCurve = pLp->GetCurve() ;
				if( pCurve == NULL ) 
				{
					if( pLp == pLoop ) pLp = pLoop->GetIsland() ;
					else pLp = pLp->next ;
					continue ;
				}
				bAt  = FALSE ;
				if( GetMillDir() == 1 ) bAt = !bAt ;
				if( pLp == pLoop ) bAt = !bAt  ;
				if( m_cParam.m_nRCompSide == 0 ) bAt = !bAt ;
				pObj = pCurve->GetHead() ; 
				pObj->GetPoint( 0.0, dSeed ) ;
				for( pSect = pObj->next ; pSect ; pSect = pSect->next )
				{
					pSect->GetPoint( 0.0, dPnt ) ;
					if( bAt == 0  )
					{/*右下角*/
						if( (dPnt[1] - dSeed[1] ) < 0.01 && 
							(dPnt[0] - dSeed[0] ) > -0.01 )
						{ 
							pObj = pSect ;
							memcpy( dSeed, dPnt, sizeof( PNT2D ) ) ;
						} 
					}
					else 
					{/*左上角*/
						if( (dSeed[1] - dPnt[1]) < 0.01 && 
							(dSeed[0] - dPnt[0]) > -0.01    )
						{  
							pObj = pSect ;
							memcpy( dSeed, dPnt, sizeof( PNT2D ) ) ;
						}  
					}
				}
				if( pObj != pCurve->GetHead() )
				{
					pCurve->SetStartSect( pObj ) ;
				}
				if( pLp == pLoop ) pLp = pLoop->GetIsland() ;
				else pLp = pLp->next ;
			}
		}
	}
	ContHead = Contour.ResortContour( ContHead, m_cSetupDef.m_cOrderDef.m_nSortType, 0 ) ;
	return ContHead ;
}
int CSmartCombMillGen::GetContourRCompMask()
{
	if( m_cParam.m_nRCompSide == 2 ) 
	{
		return 0 ;
	}
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC )
	{
		if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG41 ;
		return JDNC_FGCODE_RCOMPG42 ;
	}
	else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC )
	{
		if( GetMillDir() == 0 ) return JDNC_FGCODE_RCOMPG42 ;
		return JDNC_FGCODE_RCOMPG41 ;
	}
	return 0 ;
}

// 计算一个环的切割路径
int CSmartCombMillGen::ContourOneLoop( CPathCombine& PComb,    /*加工数据*/
									  CSmartLoop& Loop   ,    /*轮廓曲线*/
									  DOUBLE  Depth      ,    /*深度位置*/
									  DOUBLE  DepthInc  )     /*吃刀深度*/
{
	//  STEP 1 : 计算进刀路径
	CPathCombine  tmpComb( NC_WPROCESS_ROUGH ) ;
	CSmartLoop * pContour = &Loop ;
	if( Loop.m_pParent ) pContour = Loop.m_pParent ;
	CSmartCurve * pBound = Loop.m_pCurve->CopyMyself() ;
	pBound->SetFeedType( JDNC_FEEDTYPE_ROUGH ) ;
	int nRCompMask = GetContourRCompMask() ;
	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;
	if( rcompLead.m_nLeadType == 0 ) rcompLead.m_nLeadType = NCDEF_LEAD_SLINE ;
	else   rcompLead.m_nLeadType = NCDEF_LEAD_LINE ;

	JDNC_LEAD  cColliLead = rcompLead;
	double dExtLen = m_cParam.m_dSafeDiam[m_nCurToolIndex] - m_cParam.m_dTipDiam[m_nCurToolIndex];
	dExtLen /= 2.0;
	if (dExtLen > 0.0001) cColliLead.m_dLength = dExtLen;
	else cColliLead.m_dLength = -1;

	if( m_cFeedDef.m_cLeadDef.m_nLeadType != NCDEF_LEAD_CLOSE )
	{/* 1: 尽可能搜索到切入切出 */
		CSmartSect* pLeadIn = NULL , *pLeadOut = NULL ; 
		JDNC_LEAD  Lead = m_cFeedDef.m_cLeadDef ;
		CreateBoundLead( Lead  ,
			*pBound, 
			pLeadIn  ,   
			pLeadOut , 
			pContour ) ;
		if( pLeadIn ) pBound->InsertAfter( pLeadIn, NULL ) ;
		if( pLeadOut) pBound->AddSect( pLeadOut ) ;
	} 
	if( GetMillDir() == 0 ) 
	{
		pBound->Reverse() ; 
	}
	if( nRCompMask != 0 ) 
	{
		AddLeadSectAtEnd( rcompLead, *pBound , pContour ) ;
	}
	if (cColliLead.m_dLength > 0 && 
		m_cFeedDef.m_cPlungeDef.m_nPlungeType==NCDEF_PLUNGE_CLOSE)
	{// 沿轮廓下刀时不加进退刀干涉处理
		AddLeadSectAtEndEx(cColliLead, *pBound, pContour ) ;
	}
	CPathCombine tmpBase( NC_WPROCESS_ROUGH ) ;
	tmpBase.AddCurve( pBound , FALSE , -Depth, TRUE ) ;
	AddRCompMask( tmpBase, nRCompMask ) ;
	// STEP 2 :  计算下刀路径
	JDNC_PLUNGE  Plunge = m_cFeedDef.m_cPlungeDef ;
	if( Plunge.m_nPlungeType != NCDEF_PLUNGE_CLOSE )
	{
		PNT3D dPoint , dAtPoint;
		pBound->GetPoint( 0.0, dPoint ) ;
		dPoint[2]  = 0  ;
		CSmartLoop *pChild = NULL ; 
		Plunge.m_dIncStep =  DepthInc  + Plunge.m_dTopTol ;
		if( Plunge.m_nPlungeType == NCDEF_PLUNGE_VERT ||
			Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX    || 
			Plunge.m_nPlungeType == NCDEF_PLUNGE_RAMP    )
		{
			DOUBLE dOffset = Plunge.m_dSideTol ; 
			if( Plunge.m_nPlungeType == NCDEF_PLUNGE_HELIX ) 
			{
				double dDist = MathCAM_MiniDistToContour( *pContour, dPoint ) ;
				dOffset += max( Plunge.m_dRadius, dDist) ;
			}
			pChild = pContour->OffsetContourEx( dOffset , dOffset, 
				m_cSetupDef.m_cCorDef ) ;
		}
		CPathCombine cTPlunge( NC_WPROCESS_PLUNGE ) ;
		if( FindPlungePoint( Plunge, *pContour, pChild, dPoint, dAtPoint, pBound ) )
		{ // 在固定位置下刀
			AddPlungePathAt( cTPlunge, Plunge, dPoint, dAtPoint, pContour ) ;
		}
		if( ! cTPlunge.m_pHead )
		{
			AddPlungePathBy( cTPlunge, Plunge , dPoint, *pBound, TRUE ) ;
		}
		cTPlunge.MoveCombine( 0.0, 0.0, -Depth ) ;
		tmpComb.AppendCombine( cTPlunge ) ;
		Mini_DeleteContours( pChild ) ;
	}
	delete pBound ;
	tmpComb.AppendCombine( tmpBase ) ;
	if( PComb.m_pHead && 
		( m_cFeedDef.m_cLayerDef.m_bLayerFlag & NCDEF_LAYER_KEEPDOWN ) )
	{
		TPNT3D  dStart, dEnd, dMid ;
		if( PComb.GetEndPoint( 1, dStart ) && 
			tmpComb.GetEndPoint( 0, dEnd )   )
		{
			BOOL bFindErr = FALSE ;
			if( nc_Distance( dStart, dEnd, 2) > 0.001) 
			{
				nc_GetPointAtLine( dStart, dEnd, 0.5, dMid, 2 ) ;
				double dAngle = ANGLE_TO_RADIAN( m_cShapeDef.m_cTaper.m_dBoundAngle  ) ;
				double dMinLen = m_pTool->GetRadiusComp(  fabs( dEnd[2]-dStart[2]) , dAngle ) - 
					m_pTool->GetRadiusComp(  0.0 , 0.0 )  ;
				if( fabs( dMinLen - nc_Distance( dStart, dEnd, 2) ) < 0.001 )
				{
				}
				else if( m_cParam.m_nRCompSide == 2 ) 
				{
					bFindErr = TRUE ;
				}
				else if( pContour->IntLineContour( dStart, dEnd ) ||
					pContour->IsPtOnContour( dMid ) == 0  )
				{
					bFindErr = TRUE ;
				}
			}
			if( bFindErr == FALSE && nc_Distance( dStart, dEnd, 3 ) > 2.0e-4 )
			{
				CPathLine3D* pLine3D  = new CPathLine3D( dStart, dEnd ) ;
				pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
				tmpComb.InsertAfter( pLine3D, NULL ) ;
			}
		}
	}
	PComb.AppendCombine( tmpComb ) ;
	return TRUE ; 
}
int  CSmartCombMillGen::AppendCombineEx( CPathCombine& Base, CPathCombine& To , CSmartLoop* Contour ) 
{
	if( To.m_pHead == NULL ) return 1;
	if( Base.m_pHead == NULL )
	{
		Base.AppendCombine( To ) ;
		return 1 ; 
	}
	PNT3D dStart , dEnd, dMid ;
	Base.GetEndPoint( 1, dStart ) ;
	To.GetEndPoint( 0, dEnd ) ;
	BOOL bValid = TRUE ;
	if( nc_FabsDist( dStart , dEnd , 3 ) < 1.0e-3 )
	{
		bValid = FALSE ;
	}
	if( Contour )
	{
		if( Contour->IntLineContour( dStart, dEnd )  )
		{
			bValid = FALSE ;
		}
	}
	if( bValid )
	{
		CPathLine3D* pLine3D = NULL ; 
		if( dStart[2] - dEnd[2] > 1.0e-3 )
		{
			dMid[0] = dEnd[0], dMid[1] = dEnd[1] , dMid[2] = dStart[2] ;
			pLine3D = new CPathLine3D(dStart, dMid) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
			Base.AddEntity( pLine3D ) ;
			pLine3D = new CPathLine3D(dMid , dEnd) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
			Base.AddEntity( pLine3D ) ;
		}
		else
		{
			pLine3D = new CPathLine3D(dStart, dEnd) ;
			pLine3D->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
			Base.AddEntity( pLine3D ) ;
		}
	}
	Base.AppendCombine( To ) ;
	return 1 ;
}

void	UnitPathGroup(CPathGroup* pGroup)
{
	if (pGroup == NULL) return;

	CPathCombine* PCombNext = NULL ;
	while( pGroup->m_pHead && pGroup->m_pHead->next )
	{
		PCombNext = pGroup->m_pHead->next ;
		pGroup->RemoveCombine ( PCombNext ) ;
		pGroup->m_pHead->AppendCombine ( PCombNext ) ;
	}
}

void CSmartCombMillGen::ReDefFeedParam(JDNC_FEED& cFeedDef, int nIndex, BOOL bFirst)  /*进给参数*/  
{
	// 分层定义
	JDNC_LAYER& cLayer = cFeedDef.m_cLayerDef;
	if (m_cParam.m_nShowMode == 0)
	{// 基本模式
		cLayer.m_nLayerType = NCDEF_LAYER_CLOSE;
	}
	else
	{
		cLayer.m_nLayerType = NCDEF_LAYER_DEPTH;
		cLayer.m_dSideDInc = m_cParam.m_dDownStep[nIndex];
		cLayer.m_dTotalDepth = GetCutDepth() - m_cStockDef.m_dDepthStock; 
	}
	DefineLayerDepth(cLayer) ; 

	// 进刀设置
	JDNC_LEAD& cLeadDef = cFeedDef.m_cLeadDef;
	if (m_cParam.m_nShowMode == 0)
	{// 基本模式
		if (bFirst)
		{// 第一层关闭进刀
			cLeadDef.m_nLeadType = NCDEF_LEAD_CLOSE;
		}
		else
		{// 其余层默认圆弧进刀
			cLeadDef.m_nLeadType = NCDEF_LEAD_ARC;
			cLeadDef.m_dRadius = m_cParam.m_dTipDiam[nIndex]*0.5;
			cLeadDef.m_dAngle = 90;
		}
		cLeadDef.m_nPositionType = NCDEF_LEADPOS_AUTO;
	}
	else
	{
		cLeadDef.m_nLeadType = m_cParam.m_nLeadType[nIndex];
		if (cLeadDef.m_nLeadType==NCDEF_LEAD_SLINE ||
			cLeadDef.m_nLeadType==NCDEF_LEAD_LINE)
		{
			cLeadDef.m_dLength = m_cParam.m_dLeadLen[nIndex];
		}
		else if (cLeadDef.m_nLeadType==NCDEF_LEAD_ARC ||
			cLeadDef.m_nLeadType==NCDEF_LEAD_INARC)
		{
			cLeadDef.m_dRadius = m_cParam.m_dLeadLen[nIndex];
			cLeadDef.m_dAngle = 90;
		}
	}

	// 下刀设置
	JDNC_PLUNGE&  cPlungeDef = cFeedDef.m_cPlungeDef;;
	if (m_cParam.m_nShowMode == 0)
	{// 基本模式
		if (bFirst)
		{// 第一层沿轮廓下刀
			cPlungeDef.m_nPlungeType = NCDEF_PLUNGE_CONTOUR;
			cPlungeDef.m_dAngle = 15;
		}
		else
		{// 其余层关闭下刀
			cPlungeDef.m_nPlungeType = NCDEF_PLUNGE_CLOSE;
		}
	}
	else
	{
		cPlungeDef.m_nPlungeType = m_cParam.m_nPlungeType[nIndex];
		if (cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR)
		{
			cPlungeDef.m_dAngle = m_cParam.m_dPlungeAngle[nIndex];
		}	
		else
		{
			cPlungeDef.m_nPlungeType = NCDEF_PLUNGE_CLOSE;
		}
	}
}

void CSmartCombMillGen::DefineCutOrder(JDNC_COMBMILL& cNewApp, double* dDepth, int* nOrgOrder)
{
	if (dDepth==NULL || nOrgOrder==NULL) return;

	int nLevelNum = m_cParam.m_nLevelNum;
	nLevelNum -- ; // based 0 !!

	if (m_cParam.m_nToolOrder == 1) //1(从上往下) 
	{
		for (int i=nLevelNum, j=0; i>=0; i--, j++)
		{
			nOrgOrder[j] = i;
			cNewApp.m_dTipDiam[j] = m_cParam.m_dTipDiam[i];
			cNewApp.m_dSafeDiam[j] = m_cParam.m_dSafeDiam[i];
			dDepth[j] = 0;
			for (int k=i; k>0; k--)
			{
				dDepth[j] -= m_cParam.m_dCutLen[k-1];
			}
		}
	}
	else if (m_cParam.m_nToolOrder == 2) //2(用户自定义)
	{
		for (int i=nLevelNum; i>=0; i--)
		{
			int nOrder = m_cParam.m_nCutOrder[i]-1;
			nOrgOrder[nOrder] = i;
			cNewApp.m_dTipDiam[nOrder] = m_cParam.m_dTipDiam[i];
			cNewApp.m_dSafeDiam[nOrder] = m_cParam.m_dSafeDiam[i];
			dDepth[nOrder] = 0;
			for (int k=i; k>0; k--)
			{
				dDepth[nOrder] -= m_cParam.m_dCutLen[k-1];
			}
		}
	}
	else //0(从下往上)
	{
		for (int i=nLevelNum; i>=0; i--)
		{
			nOrgOrder[i] = i;
			cNewApp.m_dTipDiam[i] = m_cParam.m_dTipDiam[i];
			cNewApp.m_dSafeDiam[i] = m_cParam.m_dSafeDiam[i];
			dDepth[i] = 0;
			for (int k=i; k>0; k--)
			{
				dDepth[i] -= m_cParam.m_dCutLen[k-1];
			}
		}
	}
}

void SetNewPathFeedRate(CPathCombine* pComb, double dFeedRatio)
{
	if (pComb == NULL) return;

	CPathEntity *pHead = pComb->m_pHead ;
	while( pHead )
	{
		pHead->m_fFeedScale = (float)( dFeedRatio * 0.01) ; 
		pHead = pHead->next ;
	}
}

typedef	std::vector<int>	CIntVector;

BOOL CSmartCombMillGen::GeneratePathEx( CPathGroup& NewPath,  /*雕刻路径*/ 
									   CSmartGraphic& Graph)  /*图形函数*/   
{
	int nLevelNum = m_cParam.m_nLevelNum;
	nLevelNum -- ; // based zero !!
	if (nLevelNum < 0)
		return FALSE;

	// O) 检查切割顺序定义
	if (m_cParam.m_nToolOrder == 2) //2(用户自定义)
	{
		BOOL bErr = FALSE;
		CIntVector cVector;
		for (int i=nLevelNum; i>=0; i--) cVector.push_back(i+1);
		for (int i=nLevelNum; i>=0; i--)
		{
			CIntVector::iterator iter = cVector.begin(),
				iter_end = cVector.end();
			iter = std::find(iter, iter_end, m_cParam.m_nCutOrder[i]);
			if (iter != iter_end)
			{
				cVector.erase(iter);
			}
			else
			{
				bErr = TRUE;
				break;
			}
		}

		if (bErr)
		{
			m_nErrorType = JDERROR_GENPATH_ERRORDER ;
			return FALSE;
		}
	}
	double dMaxDiam = 0.;
	for (int i=nLevelNum; i>=0; i--)
	{// 检查安全半径设置
		if (dMaxDiam < m_cParam.m_dTipDiam[i])
			dMaxDiam = m_cParam.m_dTipDiam[i];

		if (dMaxDiam > m_cParam.m_dSafeDiam[i])
			m_cParam.m_dSafeDiam[i] = dMaxDiam;
	}

	JDNC_TOOLEX cTempTool = m_cToolDef; // 备份刀具
	JDNC_FEED cTempFeed = m_cFeedDef;

	// A) 处理轮廓信息
	BOOL bRedepth = FALSE ;
	if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH )
	{
		bRedepth = TRUE ; 
	}
	CSmartLoop* AllCont = Graph.GetAllContour( m_cSetupDef , bRedepth) ;
	if(  !AllCont ) 
	{ // 校正图形
		m_nErrorType = JDERROR_GENPATH_NOLOOP ;
		return FALSE ;
	}
	MathCAM_SetContoursStart( AllCont, m_cFeedDef.m_cLeadDef,Graph, m_cSetupDef ) ;

	CSmartLoop LoopTmp, *pLoop, *pContour;
	if( m_cParam.m_nRCompSide == 0 )
	{ // 向外切割
		PNT3D  dBox[2] ; 
		nc_CleanBox3D( dBox ) ;
		for( pContour = AllCont ; pContour ; pContour = pContour->next )
		{
			nc_ClipBox3D(  dBox, pContour->m_dBox ) ;
		}
		dBox[0][0] -= 1.0e6, dBox[0][1] -= 1.0e6 ;
		dBox[1][0] += 1.0e6, dBox[1][1] += 1.0e6 ;
		pContour = new CSmartLoop() ;
		pContour->UpdateSect( Mini_CreateBoxList( dBox[0] , dBox[1] ) ) ;
		AllCont = Mini_AddContours( AllCont, pContour ) ;
		for( pContour = AllCont ; pContour ; pContour = pContour->next )
		{
			if( pContour->m_pIsland == NULL ) continue ;
			pLoop = pContour->m_pIsland ;
			pContour->m_pIsland = NULL  ;
			AllCont = Mini_AddContours( AllCont, pLoop ) ;
			while( pLoop ) 
			{
				pLoop->m_pParent = NULL  ; 
				pLoop = pLoop->next ;
			}
		}
		AllCont = LoopTmp.BuildContour( AllCont ) ;
		AllCont = LoopTmp.ResortContour( AllCont, m_cSetupDef.m_cOrderDef.m_nSortType , FALSE) ;
	}

	// B) 计算每把刀路径
	CPathGroup cTempPath(0);
	CJDTool  cCalTool(surfncToolFlat);
	int* nOrgOrder = new int[m_cParam.m_nLevelNum]; // 刀具原始顺序
	double* dDepth = new double[m_cParam.m_nLevelNum]; // 距最底层刀具距离
	memset(nOrgOrder, 0, sizeof(int)*m_cParam.m_nLevelNum);
	memset(dDepth, 0, sizeof(double)*m_cParam.m_nLevelNum);

	JDNC_COMBMILL temComb;
	SurfNC_InitPathParam(temComb);
	DefineCutOrder(temComb, dDepth, nOrgOrder);// 处理切割顺序

	for (int i=0; i<=nLevelNum; i++)
	{
		cCalTool.m_cToolParam.m_dTopDiam = temComb.m_dTipDiam[i];
		m_cToolDef = cCalTool.m_cToolParam;

		if (m_pTool != NULL) 
		{
			delete m_pTool; 
			m_pTool = NULL;
		}

		m_nCurToolIndex = nOrgOrder[i];

		// 进给参数设置
		ReDefFeedParam (m_cFeedDef, nOrgOrder[i], i==0);

		if (!GenOneToolPath(cTempPath, AllCont, i))	continue;

		cTempPath.MoveGroup(0, 0, dDepth[i]);

		// 合并路径组
		UnitPathGroup (&NewPath);
		UnitPathGroup (&cTempPath);

		// 设置该层路径走刀速度
		SetNewPathFeedRate(cTempPath.m_pHead, m_cParam.m_dFeedRatio[nOrgOrder[i]]);

		if (i<=nLevelNum && i>0) 
		{
			AddConnectPath(NewPath, cTempPath, nOrgOrder[i]);
		}

		// 转移生成的零时路径
		NewPath.AddData(0.0, cTempPath.m_pHead);
		cTempPath.RemoveCombine(cTempPath.m_pHead);
	}

	delete[] dDepth;
	dDepth = NULL;
	delete[] nOrgOrder;
	nOrgOrder = NULL;

	// C) 路径修正
	Mini_DeleteContours( AllCont ) ;
	if( ! NewPath.m_pHead )
	{
		m_nErrorType = JDERROR_GENPATH_NOAREA ;
		return FALSE ; 
	}
	BOOL bKeep[2] = { FALSE, FALSE } ;
	if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_REDEPTH )
	{// 保留原有高度，忽略表面高度
		bKeep[0] = TRUE ;
	}
	if( m_cProcessDef.m_bProcessFlag & NCDEF_CURVE_SPEED )
	{
		bKeep[1] = TRUE ;
	}
	if( bKeep[0] || bKeep[1] )
	{
		DOUBLE dTop = GetTopHeight() ;
		RedepthNewPath( NewPath,  Graph, bKeep, dTop ) ;
		if( bKeep[0] )
		{
			m_dTopHeight = dTop ;
		}
	}
	if( !bKeep[0] )
	{
		// 校正表面高度
		NewPath.ZMoveGroup( GetTopHeight() ) ;
	}

	m_cToolDef = cTempTool;
	m_cFeedDef = cTempFeed;

	return TRUE;		 
}

void CSmartCombMillGen::AddConnectPath(CPathGroup& cFrom, CPathGroup& cTo, int nToolIndex)
{
	UNUSED_ALWAYS(nToolIndex);

	PNT3D start, end, end1 ;
	CPathEntity *pHead = NULL, *pEnd = NULL ;
	CPathCombine *PCombHead = NULL, *PCombEnd = NULL ;

	// 找首末点
	PCombHead = cFrom.m_pTail;	
	PCombEnd = cTo.m_pHead;

	pHead = PCombHead->m_pHead ;
	while( pHead )
	{
		pEnd = pHead->next ;
		if( !pEnd ) break ;
		pHead = pEnd ;
	}
	pEnd = PCombEnd->m_pHead ;

	pHead->GetEndPoint ( 1, start ) ;
	pEnd->GetEndPoint ( 0, end ) ;

	double dist = mathDist( start, end ) ;
	if( dist >= 0.0001 ) 
	{
		// 添加慢速下刀
		CPathPLine3D*pPLine = NULL;
		CPathLine3D *pLine = NULL;
		if( m_cSpeedDef.m_dPlungeDist > 0.01 )
		{
			mathCpyPnt( end, end1 ) ;
			end[2] += m_cSpeedDef.m_dPlungeDist ;
			pLine = new CPathLine3D( end, end1 ) ;
			pLine->m_bFeedType = JDNC_FEEDTYPE_PLUNGE ;
			PCombHead->InsertAfter ( pLine, pHead ) ;
		}
		// 添加连刀
		double h = max(start[2], end[2]);
		h = max(h, GetTopHeight() + m_cSpeedDef.m_dRapidHeight);
		pPLine = CreateRelativeQuick( start, end, h ) ;
		if( pPLine ) PCombHead->InsertAfter ( pPLine, pHead ) ;
	}
}

BOOL CSmartCombMillGen::GenOneToolPath( CPathGroup& NewPath,    /*雕刻路径*/ 
									   CSmartLoop*& AllCont,    /*切割轮廓*/
									   int			nIndex)
{
	if( !UpdateTool2D() ) return FALSE ;

	// 分层设置
	JDNC_LAYER * pLayer   = GetLayerDef() ;
	pLayer->m_dTotalDepth = GetCutDepth()  - m_cStockDef.m_dDepthStock; 
	DefineLayerDepth( * pLayer) ; 

	DOUBLE dDepth , dDepthInc ;
	CSmartLoop* pLpHead , *pChildHead = NULL, *pLoop, *pContour;
	CPathCombine* pTComb = NULL ; 

	// 轮廓顺序向前递推（减少定位长度）
	// 1->2->3->4  => 4->1->2->3
	if (nIndex != 0)
	{
		for( pLpHead = AllCont ; pLpHead ; )
		{
			pContour = pLpHead->next;
			if (pContour == NULL) break;			
			pLpHead = pContour;
		}

		pLpHead->next = AllCont;
		AllCont->prev = pLpHead;
		if (pLpHead->prev != NULL) pLpHead->prev->next = NULL;
		pLpHead->prev = NULL;

		AllCont = pLpHead;
	}

	if( GetLayerOrder() == 1 )
	{ // 高度优先
		for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
		{
			dDepth = pLayer->m_dLayerDepth[i] ; 
			dDepthInc = dDepth - pLayer->m_dLayerDepth[i-1] ;
			for( pContour = AllCont ; pContour ; pContour = pContour->next )
			{
				CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				pLpHead = CreateLoopBound( *pContour, pLayer->m_dTotalDepth, dDepth ) ;
				for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
				{ 
					LoopOneLayer( *pTComb , *pLoop, m_cParam, dDepthInc ) ;
				}  
				Mini_DeleteContours( pLpHead ) ;
				if( pTComb->m_pHead )	NewPath.AddData(dDepth, pTComb ) ;
				else delete pTComb ;
			}
		}
	}
	else 
	{ // 区域优先 
		if( IsSameOffset() && 
			(m_cFeedDef.m_cPlungeDef.m_nPlungeType == NCDEF_PLUNGE_CONTOUR) )
		{//偏移值相同 && 沿轮廓下刀 , 使用拷贝切割  
			dDepth = 0.0 ; 
			CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
			for( pContour = AllCont ; pContour ; pContour = pContour->next )
			{
				pLpHead = CreateLoopBound( *pContour, pLayer->m_dTotalDepth, dDepth ) ;
				for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
				{ 
					CopyOneContour( *pTComb    , 
						*pLoop     , 
						*pLayer    , 
						m_cFeedDef.m_cPlungeDef ) ;
				}  
				Mini_DeleteContours( pLpHead ) ;
			}
			if( pTComb->m_pHead )	NewPath.AddData( 0.0, pTComb ) ;
			else delete pTComb ;
		}
		else
		{
			CSmtLoopArr LoopArr ;
			for( pContour = AllCont ; pContour ; pContour = pContour->next )
			{
				for( int i = 1 ; i <= pLayer->m_nLayerCount ;  i ++ ) 
				{
					dDepth = pLayer->m_dLayerDepth[i] ; 
					pLpHead = CreateLoopBound( *pContour, pLayer->m_dTotalDepth, dDepth ) ;
					for( pLoop = pLpHead ; pLoop ; pLoop = pLoop->next )
					{
						pLoop->m_nDepth = i , pLoop->m_pParent = NULL ;
						CSmartLoop *pIsland ; 
						for( pIsland = pLoop->GetIsland() ; pIsland ; pIsland = pIsland->next )
						{
							pIsland->m_nDepth = i ;
						}
						CSmtLoopArr tmpArr ;
						pLoop->ExtractAllLoop( GetSortType() , tmpArr ) ;
						INT_PTR nCount = tmpArr.GetSize() ;
						for( INT_PTR k = 0 ; k < nCount ; k ++ ) 
						{
							pIsland = tmpArr.GetAt( k ) ;
							if( fabs( pIsland->m_dArea ) < 1.0e10 && 
								(!(pIsland->m_bUseFlag & NC_LOOP_OUTER)) && 
								pIsland->m_pCurve != NULL  )
							{ /*无效轮廓曲线*/
								LoopArr.Add( pIsland ) ;
							}
						}
					}
					pChildHead = Mini_AddContours( pChildHead, pLpHead ) ;
				}
			}
			BOOL bMoveUp = FALSE ;
			if( pLayer->m_nLayerCount > 1 )
			{
				double dRadius = m_pTool->m_fRadius ;
				if( m_pTool->GetType() == surfncToolAFlat )
				{
					dRadius = m_pTool->m_fBottom ;
				}
				SortLoopArray( LoopArr, dRadius ) ;
			} 
			if( m_cParam.m_bLoopFlag & NCDEF_FLOOP_MOVEUP  ) 
			{
				bMoveUp = TRUE ;
			}
			INT_PTR nCount = LoopArr.GetSize() ; 
			CSmtLoopArr   curList ;
			int nLastDepth = 0 ;
			for( INT_PTR nAt = 0 ; nAt < nCount ;  nAt ++ ) 
			{
				pLoop = LoopArr.GetAt( nAt ) ;
				if( ( pLoop->m_nDepth == nLastDepth + 1 ) || 
					curList.GetSize() == 0 )
				{
					if( bMoveUp )  curList.InsertAt( 0 , pLoop ) ;
					else curList.Add( pLoop ) ;
					if( nAt <  nCount - 1  )
					{
						nLastDepth = pLoop->m_nDepth ;
						continue ;
					} 
				}
				else 
				{
					nAt -- ;
				}
				if( pTComb ) NewPath.AddData( 0.0, pTComb ) ;
				pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
				for( int nLp = 0 ; nLp < curList.GetSize() ; nLp ++ ) 
				{
					pLoop = curList.GetAt( nLp ) ;
					if( nLp != 0  &&  pLoop->m_pCurve )
					{ /*重设轮廓起点, 减少定位路径*/
						PNT2D dSeed[2];
						DOUBLE dDist ;
						CSmartLoop* pLp = curList.GetAt( nLp-1 ) ;
						pLp->m_pCurve->GetPoint( 1.0, dSeed[0] ) ;
						CSmartSect* pSect = pLoop->m_pCurve->FindNearSect( dSeed[0],dSeed[1], dDist ) ;
						if( pSect && pSect != pLoop->GetSectHead() )
						{
							pLoop->m_pCurve->SetStartSect( pSect ) ;
						}
					}
					dDepth = pLayer->m_dLayerDepth[pLoop->m_nDepth] ; 
					dDepthInc = dDepth - pLayer->m_dLayerDepth[pLoop->m_nDepth-1] ;
					ContourOneLoop( *pTComb , *pLoop, dDepth, dDepthInc ) ;
				}
				nLastDepth = 0 ;
				curList.RemoveAll() ;
			}
			if( pTComb ) NewPath.AddData( 0.0, pTComb ) ;
		}
	}

	if( pLayer->m_dLayerDepth ) delete[] pLayer->m_dLayerDepth ;
	pLayer->m_dLayerDepth = NULL ;

	Mini_DeleteContours( pChildHead ) ;

	return TRUE ;
}

