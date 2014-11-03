#include "StdAfx.H"
#include "SmartNC.H"
#include "PathExtend.h"
#include "SurfNC.h"
#include "Nc3DStepAndSpiral.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char BASED_CODE THIS_FILE[] = __FILE__;
#endif

static void MathCam_MovePtAlongVec2D(FPNT3D OrigPt,FPNT3D DestPt,
									   VEC3D Dir,double Dist)
{
	for(int i=0; i<2; i++)
	{
		DestPt[i] = TFLOAT( OrigPt[i] + Dir[i]*Dist );
	}
}
CHeightExtendOut::CHeightExtendOut()
{
	m_pTHolder = NULL;
	m_bChkTHolder = TRUE;
}
CHeightExtendOut::~CHeightExtendOut()
{
	ClearAll();
}
void CHeightExtendOut::ClearAll()
{
	if(m_pTHolder)
	{
		delete m_pTHolder;
		m_pTHolder = NULL;
	}
}

//刀柄是否与毛坯(残料模型)干涉
#if 0
BOOL CHeightExtendOut::IsTHolderIntWithStock(CSmtCutPoint * pAtPnt,double dPathHeight,
											 CSmtCheckMdl &DriveMdl)
{

	return FALSE;
}
#endif

//刀柄是否与曲面干涉
BOOL CHeightExtendOut::IsTHolderIntWithSurf(CSmtCutPoint * pAtPnt,
											CSmtCheckMdl &DriveMdl)
{
	BOOL bRet = FALSE;
	if(m_pTHolder && pAtPnt)
	{
		TFLOAT fHeight = pAtPnt->m_fPoint[2];
		DriveMdl.Define3AxHolder(m_pTHolder, *pAtPnt);
		if( fabs(fHeight - pAtPnt->m_fPoint[2]) > 0.002)
		{
			bRet = TRUE;
		}
		pAtPnt->m_fPoint[2] = fHeight;
	}
	return bRet;
}

BOOL CHeightExtendOut::PntHorPrjOnChkMdl(FPNT3D Pnt,PNT3D Dir,
									 CSmtCheckMdl & CheckMdl)
{
	Dir[2] = 0;
	CSmtCutPoint dBegin, dEnd;
	nc_VectorCopy(dBegin.m_fPoint, Pnt, 3);
	double PrjDir[3];
	nc_VectorCopy(PrjDir, Dir, 3);
	BOOL bOverCut = CheckMdl.Is3AxPointOvercut(dBegin, 0);
	if(!bOverCut)	nc_VectorReverse(PrjDir, 3);
	double dDist = 0.005;
	dEnd.m_fPoint[2] = (TFLOAT)Pnt[2];
	MathCam_MovePtAlongVec2D(dBegin.m_fPoint, dEnd.m_fPoint, PrjDir, dDist);
	if( bOverCut == CheckMdl.Is3AxPointOvercut(dEnd, 0) )
	{
		MathCam_MovePtAlongVec2D(dBegin.m_fPoint, dEnd.m_fPoint, PrjDir, -dDist);
		if( bOverCut == CheckMdl.Is3AxPointOvercut(dEnd, 0) )
		{
			return FALSE;
		}
	}
	return TRUE;
}
BOOL CHeightExtendOut::IsLocalPathOverCut(double Begin[], double End[],
										CSmtCheckMdl &DriveMdl,double OverCutPt[])
{
	double dLen = nc_Distance(Begin, End, 3);
	int nPoint = int(dLen / 0.015) + 2;
	double n = nPoint - 1 ;
	CSmtCutPoint dCurrPt;
	FPNT3D fPrjPt;
	double vLineDir[2], vHorPrj[3];
	nc_VectorMinus(Begin, End, vLineDir, 2);
	nc_Normalize( vLineDir, 2 ) ;
	mathRotVec2D( PI1_2, vLineDir, vHorPrj ) ;
	for(int i=1; i<nPoint; i++)
	{
		double u = double(i) / n;
		dCurrPt.m_fPoint[0] = TFLOAT( (1.0-u)*Begin[0] + u*End[0] );
		dCurrPt.m_fPoint[1] = TFLOAT( (1.0-u)*Begin[1] + u*End[1] );
		dCurrPt.m_fPoint[2] = TFLOAT( (1.0-u)*Begin[2] + u*End[2] );
		nc_VectorCopy(fPrjPt, dCurrPt.m_fPoint, 2);
		fPrjPt[2] = DriveMdl.m_fBottom;
#if 1
		DriveMdl.DefineHeight(fPrjPt);
		if( dCurrPt.m_fPoint[2] >= fPrjPt[2] || 
			fabs(dCurrPt.m_fPoint[2]-fPrjPt[2])<0.002 || 
			PntHorPrjOnChkMdl(dCurrPt.m_fPoint, vHorPrj, DriveMdl))
		{
			continue;
		}
		else
		{
			nc_VectorCopy(OverCutPt, dCurrPt.m_fPoint, 3);
			return TRUE;
		}
#else
		if(DriveMdl.Is3AxPointOvercut(dCurrPt, 0))
		{
			nc_VectorCopy(OverCutPt, dCurrPt.m_fPoint, 3);
			return TRUE;
		}
#endif
	}
	return FALSE;
}
BOOL CHeightExtendOut::ExtendOutOnePath(CSmtCutPath *pPath, double ExtendDist,
			   JDNC_SETUP& /*SetupDef*/,CSmtCheckMdl &DriveMdl)
{
	if(pPath==NULL || pPath->IsClosed() || pPath->GetCutMode()!=MINI_MILL_PATH ||
		pPath->m_nNumPnt < 2 || pPath->m_pHead==NULL || pPath->m_pTail==NULL ||
		DriveMdl.m_pTool==NULL)
	{
		return FALSE;
	}

	CSmtCutPoint * pHead = pPath->m_pHead;
	CSmtCutPoint * pTail = pPath->m_pTail;
	TFLOAT dPathHeight = pHead->m_fPoint[2];
	BOOL bSameZ = ( pHead->m_fPoint[2]==pTail->m_fPoint[2]) ? TRUE : FALSE ;
	double dExtendDir[3];
	double dBeginPnt[3], dEndPnt[3], dOverCutPnt[3];
	FPNT3D  dLBox[2] ;
	dLBox[0][2] = (TFLOAT)DriveMdl.m_fBottom ;
	dLBox[1][2] = 1.0e6;
	for(int n=0; n<2; n++)
	{
		if(n==0)
		{
			for(int i=0; i<3; i++)
			{
				dExtendDir[i] = pHead->m_fPoint[i] - pHead->next->m_fPoint[i];
			}
			nc_VectorCopy(dBeginPnt, pHead->m_fPoint, 3);
		}
		else
		{
			for(int i=0; i<3; i++)
			{
				dExtendDir[i] = pTail->m_fPoint[i] - pTail->prev->m_fPoint[i];
			}
			nc_VectorCopy(dBeginPnt, pTail->m_fPoint, 3);
		}
		mathUniVec(dExtendDir, 1.0e-8);
		for(int i=0; i<3; i++)
		{
			dEndPnt[i] = dBeginPnt[i] + ExtendDist*dExtendDir[i];
		}
		dLBox[0][0] = TFLOAT( min(dBeginPnt[0], dEndPnt[0]) - 1);
		dLBox[0][1] = TFLOAT( min(dBeginPnt[1], dEndPnt[1]) - 1);
		dLBox[1][0] = TFLOAT( max(dBeginPnt[0], dEndPnt[0]) + 1);
		dLBox[1][1] = TFLOAT( max(dBeginPnt[1], dEndPnt[1]) + 1);
		DriveMdl.LabelCheckByBox( dLBox );

		if(IsLocalPathOverCut(dBeginPnt, dEndPnt , DriveMdl, dOverCutPnt))
		{
			for(int i=0; i<3; i++)
			{
				dEndPnt[i] = dOverCutPnt[i] - 0.02*dExtendDir[i];
			}
		}
		if(nc_Distance(dBeginPnt, dEndPnt, 3)>0.01)
		{
			if(n==0)
				pPath->AddHead(new CSmtCutPoint(dEndPnt));
			else
				pPath->AddTail(new CSmtCutPoint(dEndPnt));
		}
	}
	if(bSameZ)
	{
		pHead->m_fPoint[2] = pTail->m_fPoint[2] = dPathHeight;
	}
	return TRUE;
}

void CHeightExtendOut::CreateToolHolder(JDNC_HOLDER & THolderDef,CSmtCheckMdl &DriveMdl)
{
#if 1
	if(m_bChkTHolder)
	{
		m_pTHolder = new CSmartHolder() ;
		m_pTHolder->CreateHolder( THolderDef,
			DriveMdl.m_pTool->m_fLength, 0.1 ) ;
		if( m_pTHolder->m_nNumLevel <= 0 ) 
		{
			delete m_pTHolder ;
			m_pTHolder = NULL;
		}
	}
#else
	if(DriveMdl.m_pTool && m_bChkTHolder)
	{
		CJDToolLib* pTLib = SurfNC_GetSysToolLib();
		if(pTLib)
		{
			CJDTool* pJDTool = pTLib->GetByID(DriveMdl.m_pTool->m_nToolID);
			if(pJDTool && pJDTool->m_pHolderDef)
			{
				JDNC_HOLDER holderDef ;
				m_pTHolder = new CSmartHolder() ;
				m_pTHolder->CreateHolder( *pJDTool->m_pHolderDef,
					DriveMdl.m_pTool->m_fLength, 0.1 ) ;
				if( m_pTHolder->m_nNumLevel <= 0 ) 
				{
					delete m_pTHolder ;
					m_pTHolder = NULL;
				}
			}
		}
	}
#endif
}
extern CSmartTool*  MathCam_GetInnerOffsetTool(CSmartTool & SrcTool, DOUBLE Offset);
void CHeightExtendOut::ExtendOutSide(CSmtCPathLib& AllPath, double ExtendDist,
			   JDNC_SETUP& SetupDef,JDNC_HOLDER & THolderDef,
			   CSmtCheckMdl &DriveMdl, BOOL ChkTHolder)
{
	if(fabs(ExtendDist)<1.0e-5 || DriveMdl.m_pTool==NULL)
	{
		return;
	}

	ClearAll();
	m_bChkTHolder = ChkTHolder;
	CreateToolHolder(THolderDef, DriveMdl);
	POSITION pAtPos = AllPath.m_cAllPath.GetHeadPosition();
	while(pAtPos)
	{
		CSmtCutPath * pCurrPath = AllPath.m_cAllPath.GetNext(pAtPos);
		ExtendOutOnePath(pCurrPath, ExtendDist, SetupDef, DriveMdl );
	}
	AllPath.DelPointOnLine(1.0e-4);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// 作用：路径向上延伸dExtndDist距离
// 目的：解决待高路径球头刀第一刀吃刀量过大问题
static BOOL is2PntsClosed(FPNT3D pnt, FPNT3D snapAt, DOUBLE dStep, DOUBLE dDist, DOUBLE dTDiameter,BOOL is3DStep );
static BOOL AddPathToAllPath( CSmtCutPath* pNewPath, CSmtCPathLib& AllPath );
BOOL ExtendUpHeigtPath(CSmtCheckMdl& DriveMdl,
					   CSmtCPathLib& AllPath, 
					   const double dExtndDist, 
					   const double dStep,
					   const BOOL is3DStep ) 
{
	if ( AllPath.GetNumPath() == 0 || dExtndDist < 1.0e-3 || dStep < 1.0e-3 )
	{
		return FALSE ;
	}
	// STEP 1:查找需要延伸的路径
	//(1)查找行号最小的层
	CSmtCutPath * pPath = AllPath.m_cAllPath.GetHead() ;
	const INT MinLineNum = pPath->m_nLineNo ;
	CSmtCPathList PathList ;
	POSITION Pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( Pos )
	{
		pPath = AllPath.m_cAllPath.GetNext( Pos ) ;
		if ( pPath->m_nLineNo == MinLineNum )
		{
			CSmtCutPath *pNewPath = new CSmtCutPath ;
			pNewPath->m_nLineNo = pPath->m_nLineNo ;
			pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			pNewPath->m_bFeedType = pPath->m_bFeedType ;
			pNewPath->m_bPathFlag = pPath->m_bPathFlag ;
			pNewPath->SetCutMode( pPath->GetCutMode() ) ;
			for ( CSmtCutPoint* pHead = pPath->m_pHead; pHead; pHead = pHead->next )
			{
				CSmtCutPointHx * pNewPoint = new CSmtCutPointHx( *pHead ) ;
				pNewPath->AddTail( pNewPoint ) ;
			}
			PathList.AddTail( pNewPath ) ;
		}
	}
	//(2)查找上方没路径的层
	DOUBLE dTDiameter = 2 * DriveMdl.m_pTool->m_fRadius ;
	Pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( Pos )
	{
		pPath = AllPath.m_cAllPath.GetNext( Pos ) ;
		INT nLineNum = pPath->m_nLineNo ;
		if ( nLineNum == MinLineNum ) continue ;
		//(i) 查找上层路径
		CSmtCPathList UpPathList ;
		POSITION Pos2 = AllPath.m_cAllPath.GetHeadPosition() ;
		while( Pos2 )
		{
			CSmtCutPath *pPath2 = AllPath.m_cAllPath.GetNext( Pos2 ) ;
			if ( pPath2->m_nLineNo == nLineNum - 1 )
			{
				UpPathList.AddTail( pPath2 ) ;
			}
		}
		//(ii) 查找上层没有关联的层
		pPath->NormalizeLen() ;
		FPNT3D start, mid, end ;
		pPath->GetPointAt( 0.0, start ) ;
		pPath->GetPointAt( 0.5, mid ) ;
		pPath->GetPointAt( 1.0, end ) ;
		Pos2 = UpPathList.GetHeadPosition() ;
		BOOL bNoUpLayer = TRUE ;
		while( Pos2 )
		{//判断起始点、中点、末点上面是否都无路径
			CSmtCutPath* pPath2 = UpPathList.GetNext( Pos2 ) ;
			FPNT3D snapAt ;
			TFLOAT dDist = 0.0, t = 0.0 ;
			//判断中点
			pPath2->SnapPointOnPath( mid, snapAt, dDist, t ) ;
			if ( is2PntsClosed( mid, snapAt, dStep, dDist, dTDiameter ,is3DStep ) )
			{
				bNoUpLayer = FALSE ;
				break;
			}
			//判断起点
			pPath2->SnapPointOnPath( start, snapAt, dDist, t ) ;
			if ( is2PntsClosed( start, snapAt, dStep, dDist, dTDiameter, is3DStep ) )
			{
				bNoUpLayer = FALSE ;
				break;
			}
			//判断末点
			pPath2->SnapPointOnPath( end, snapAt, dDist, t ) ;
			if ( is2PntsClosed( end, snapAt, dStep, dDist, dTDiameter, is3DStep ) )
			{
				bNoUpLayer = FALSE ;
				break;
			}
		}
		if ( bNoUpLayer )
		{
			CSmtCutPath *pNewPath = new CSmtCutPath ;
			pNewPath->m_nLineNo = pPath->m_nLineNo ;
			pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			pNewPath->m_bFeedType = pPath->m_bFeedType ;
			pNewPath->m_bPathFlag = pPath->m_bPathFlag ;
			pNewPath->SetCutMode( pPath->GetCutMode() ) ;
			for ( CSmtCutPoint* pHead = pPath->m_pHead; pHead; pHead = pHead->next )
			{
				CSmtCutPointHx * pNewPoint = new CSmtCutPointHx( *pHead ) ;
				pNewPath->AddTail( pNewPoint ) ;
			}
			PathList.AddTail( pNewPath ) ;
		}
	}
	// STEP 2:设置延伸方向
	Pos = PathList.GetHeadPosition() ;
	while( Pos )
	{
		pPath = PathList.GetNext( Pos ) ;
		//(1) 得到下层路径链表
		CSmtCPathList BelowPathList ;
		INT nLineNum = pPath->m_nLineNo + 1 ; //得到下层路径层号
		POSITION Pos2 = AllPath.m_cAllPath.GetHeadPosition() ;
		while( Pos2 )
		{
			CSmtCutPath *pPath2 = AllPath.m_cAllPath.GetNext( Pos2 ) ;
			if ( pPath2->m_nLineNo == nLineNum )
			{
				BelowPathList.AddTail( pPath2 ) ;
			}
		}
		if( BelowPathList.GetSize() == 0 ) 
			continue ;
		//(2) 计算延伸方向
		CSmtCutPointHx *pNext = NULL ;
		for ( CSmtCutPointHx* pHead = (CSmtCutPointHx*)pPath->m_pHead; pHead ; pHead = pNext )
		{
			pNext = (CSmtCutPointHx*)(pHead->next) ;
			//得到延伸方向
			FPNT3D bgn, end ;
			nc_VectorCopy( end, pHead->m_fPoint, 3 ) ;
			double dDist = 1.0e6 ;
			POSITION TmpPos = BelowPathList.GetHeadPosition() ;
			while( TmpPos )
			{
				CSmtCutPath *pTmpPath = BelowPathList.GetNext( TmpPos ) ;
				FPNT3D SnapAt; 
				TFLOAT dTmpDist, t ;
				pTmpPath->SnapPointOnPath( end, SnapAt, dTmpDist, t ) ;
				if ( dTmpDist < dDist )
				{
					dDist = dTmpDist ;
					nc_VectorCopy( bgn, SnapAt, 3 ) ;
				}
			}
			if( dDist < 1.0e6 )
			{
				dDist = 1.0e6 ;
				FPNT3D SnapAt ;
				TFLOAT dTmpDist, t ;
				pPath->SnapPointOnPath( bgn, SnapAt, dTmpDist,t ) ;
				dDist = nc_DistanceSquare( end, SnapAt, 3 ) ;
			}
			if( dDist > 1.0e-4 ) {
				pHead->setValid( FALSE );
			}else {
				FPNT3D dVect ;
				nc_VectorMinus( bgn, end, dVect, 3 ) ;
				nc_Normalize( dVect, 3 ) ;
				pHead->setExtendDir( dVect ) ;
				pHead->setValid( TRUE ) ;
			}
		}
	}
	//STEP 3: 生成延伸路径
	INT nCpyTimes = max( static_cast<int>(dExtndDist / dStep), 1 ) ;
	for( int i = 1; i <= nCpyTimes; i ++ )
	{
		Pos = PathList.GetHeadPosition() ;
		while ( Pos )
		{
			pPath = PathList.GetNext( Pos ) ;
			CSmtCutPath *pNewPath = new CSmtCutPath() ;
			pNewPath->m_nLineNo = pPath->m_nLineNo - i;
			pNewPath->m_nLayerNo = pPath->m_nLayerNo ;
			pNewPath->m_bFeedType = pPath->m_bFeedType ;
			pNewPath->m_bPathFlag = pPath->m_bPathFlag ;
			pNewPath->m_bPathFlag |= SMARTNC_CUTPATH_MOVEUP ;
			pNewPath->SetCutMode( pPath->GetCutMode() ) ;
			CSmtCutPointHx *pNext = NULL ;
			for ( CSmtCutPointHx *pPoint = (CSmtCutPointHx*)pPath->m_pHead ; pPoint; pPoint = pNext )
			{
				pNext = (CSmtCutPointHx*)pPoint->next ;
				if( pPoint->isValid() == FALSE) {
					CSmtCutPoint *pTmpP = new CSmtCutPoint();
					pTmpP->m_bType = 0 ;
					pNewPath->AddTail( pTmpP) ;
					continue ;
				}
				PNT3D point ;
				nc_FloatToDouble( point, pPoint->m_fPoint, 3 ) ;
				for ( int j = 0; j < 3; j++ )
				{
					point[j] += pPoint->m_vExtendDir[j] *( dStep ) * i ;
				}
				CSmtCutPoint *pNewPoint = new CSmtCutPoint ;
				nc_DoubleToFloat( pNewPoint->m_fPoint, point, 3 ) ;
				if( DriveMdl.Is3AxPointOvercut(*pNewPoint, 0) == FALSE )
				{
					pNewPoint->m_bType = 1 ;
				}
				else
				{
					pNewPoint->m_bType = 0 ;
				}
				pNewPath->AddTail( pNewPoint ) ;
			}
			//路径加入路径组
			AddPathToAllPath( pNewPath, AllPath ) ;
		}
	}
	//STEP 4: 整体平移行号,保证均为正值
	
	Pos = AllPath.m_cAllPath.GetHeadPosition() ;
	while( Pos )
	{
		pPath = AllPath.m_cAllPath.GetNext( Pos ) ;
		pPath->m_nLineNo += nCpyTimes ;
	}
	//STEP 5:清除内存
	Pos = PathList.GetHeadPosition() ;
	while( Pos )
	{
		pPath = PathList.GetNext( Pos ) ;
		delete pPath ;
	}
	return TRUE ;
}
/*等高路径中，判断两个点是否靠近*/
static BOOL is2PntsClosed(FPNT3D pnt, FPNT3D snapAt, DOUBLE dStep, 
						  DOUBLE dDist, DOUBLE dDiameter, BOOL is3DStep )
{
	if( is3DStep == FALSE  )
	{
		if( dDist < dDiameter ) {
			DOUBLE zDet = fabs( pnt[2] - snapAt[2] ) ;
			if( zDet < dStep *1.5 ) return TRUE ;
		}
	}
	else if( dDist < dStep * 1.5 )
	{
		return TRUE ;
	}
	return FALSE ;
}
/*把延伸路径加入到路径组内*/
static BOOL AddPathToAllPath( CSmtCutPath* pCutPath, CSmtCPathLib& AllPath )
{
	if( !pCutPath ) return TRUE;
	if( pCutPath->NumPoint() < 2 || pCutPath->GetLength() < 0.02 ){
		delete pCutPath ;
		return TRUE;
	}
	if( pCutPath->IsClosed () )
	{// 优先将无效的点设置为起点,避免封闭路径出现断开现象
		for( CSmtCutPoint *p = pCutPath->m_pHead ; p ; p = p->next )
		{
			if( p->m_bType == 0 )
			{
				pCutPath->SetClosedPathHead ( p ) ;
				break ;
			}
		}
	}
	int nLineNo = pCutPath->m_nLineNo ;
	CSmtCutPath* pNewTPath = pCutPath->BreakAtNullPoint( FALSE, 0.0 ) ;
	while( pNewTPath )
	{
		CSmtCutPath* pPath = pNewTPath ;
		pNewTPath = pNewTPath->next ;
		pPath->m_nLineNo = nLineNo ;
		pPath->next = pPath->prev = NULL ; 
		AllPath.m_cAllPath.AddHead( pPath ) ;
	}
	delete pCutPath ;
	return TRUE ;
}