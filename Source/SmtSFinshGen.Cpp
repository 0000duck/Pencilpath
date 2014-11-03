#include "StdAfx.H"
#include "SurfGeo.H"
#include "SurfNC.H"
#include "SmartNC.H"
#include "SmtPathGen.H"
#include "SmtPathGen2D.H"
#include "SmtPathGen3D.H"
#include "SmtAutoFinishGen.H"
#include "SmtFlowLineGen.H"
#include "mathcam.h"
#include "SurfNC.h"			//为使用SurfNC_GetJDCalSetup()而添加
#include "Nc3DStepAndSpiral.h"
#include "NcHeightMdl.h"
#include "NcBndAutoFinish.h"
#include "SurfEditor.h"
#include "Nc5DToolAxis.h"
#include "Nc5DSGuide.h"
#include "FFollowNoMosaic.h"
#include "productDef.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

void MathCAM_AppendBoundPath( CSmtCPathLib& AllPath, 
                              CSmtCPathLib& bndPath )
{
    while( bndPath.m_cAllPath.GetCount() )
    {
        CSmtCutPath* pTTail , *pTPath = NULL ; 
        CSmtCutPoint* objTPnt = NULL , *pTPnt ; 
        POSITION atDel = bndPath.m_cAllPath.GetHeadPosition() ;
        if( AllPath.m_cAllPath.GetCount())
        {
            pTTail = AllPath.m_cAllPath.GetTail() ;
            DOUBLE fDist , fMinDist = 1.0e6 ;
            POSITION atPos = bndPath.m_cAllPath.GetHeadPosition() ;
            while( atPos && pTTail->m_pTail )
            {
                POSITION atCur = atPos ;
                pTPath =  bndPath.m_cAllPath.GetNext( atPos ) ;
                for( pTPnt = pTPath->m_pHead ; pTPnt ; pTPnt = pTPnt->next )
                { 
                    fDist = nc_Distance( pTPnt->m_fPoint, pTTail->m_pTail->m_fPoint, 3 ) ;
                    if( fDist < fMinDist  )
                    {
                        fMinDist = fDist ;
                        objTPnt  = pTPnt  ;
                        atDel = atCur ;
                    }
                }
            }
        }
        pTPath = bndPath.m_cAllPath.GetAt( atDel ) ;
        bndPath.m_cAllPath.RemoveAt( atDel ) ;
        if( pTPath->IsClosed( 1.0e-3f ) && objTPnt )
        {
            pTPath->SetClosedPathHead( objTPnt ) ;
        }
        AllPath.AddToTail( pTPath ) ;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////
//CSmartFLinearGen 
CSmartFLinearGen::CSmartFLinearGen()
{
}
CSmartFLinearGen::~CSmartFLinearGen()
{
}
void MathCAM_RotateByZAxis( CSmtCPathLib& AllPath, DOUBLE Angle ) ;


int CSmartFLinearGen::GeneratePathEx( CPathGroup&   NewPath  ,  /*雕刻路径*/
									  CSmartGraphic& Graph    ) /*雕刻图形*/
{ // 生成刀具路径 
    // STEP 1 : 构建加工模型
	if (!IsAxisCtrlValid(Graph))	// 刀轴控制模型判断
		return FALSE;

    CSmtCheckMdl DriveMdl, CheckMdl ;
    CSmartLoop* BoundCont = NULL    ; 
	double dRotAngle = 0 ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == 0 )
	{ // 如果三轴,旋转模型,三转五轴,不需要旋转.
		dRotAngle = ANGLE_TO_RADIAN( m_cParam.m_dLineAngle ) ;
		m_cParam.m_dLineAngle = 0 ;
	}
    if( ! BuildSFinishGen( NewPath, Graph, DriveMdl, CheckMdl, BoundCont, -dRotAngle ) )
    {
        return FALSE ;
    }
    // STEP 2 : 计算兜边路径
    SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
	// 需要多线程的拷贝，则拷贝一份DriveMdl
    m_cPrgDef.m_dTotalMove = 100.0 ;
    CSmartLoop *allArea[2] = { NULL, NULL } ;
    CSmtCPathLib flatPath, steepPath, bndPath ;
	CSmartLoop* tempLoop = NULL; //临时参数，用于函数传参，无实际意义 qqs 2013.04.25 
    BOOL bRetFlag = TRUE ;
    DOUBLE dTotalMove = 100.0 ;
    DOUBLE dLineAngle =  m_cParam.m_dLineAngle ;
    DOUBLE dSlopeAngle= m_cParam.m_cSlope.m_dSlopeAngle ;
    DOUBLE dSideTol = -1.0 ;
	if( m_cParam.m_bLineFlag & NCDEF_LINEAR_ZIGZAG )
    {
        m_cFeedDef.m_cConnectDef.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
    }
    else
    {
        m_cFeedDef.m_cConnectDef.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    }
  	if( m_cParam.m_bLineFlag & NCDEF_LINEAR_SIDETOL )
    {
        dSideTol = m_cParam.m_dSideTol ;
    }
    BOOL bLead3DByBound = FALSE;	// 沿边界连刀的标志
    if( m_cParam.m_cSlope.m_nAddType == NCDEF_ADDTYPE_XYALL )
    {/*双向混合走刀*/
        m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove ;
        bRetFlag = CreateXParallelAreas( DriveMdl, BoundCont, dSlopeAngle, m_cParam.m_cSlope.m_dExtLen , 
										 dLineAngle, allArea, dSideTol,m_cPrgDef ) ; 
        if( bRetFlag && allArea[0] )
        {
            if( allArea[0] && allArea[1] ) m_cPrgDef.m_dTotalMove = 0.25 * dTotalMove ;
            else m_cPrgDef.m_dTotalMove =  0.5 * dTotalMove ;
            bRetFlag = CreateParallelPath( DriveMdl, dLineAngle, allArea[0], flatPath, m_cPrgDef ) ;
        }
        if( bRetFlag && allArea[1] )
        {
            if( allArea[0] && allArea[1] ) m_cPrgDef.m_dTotalMove = 0.25 * dTotalMove ;
            else m_cPrgDef.m_dTotalMove =  0.5 * dTotalMove ;
            bRetFlag = CreateParallelPath( DriveMdl,dLineAngle+90.0, allArea[1],steepPath, m_cPrgDef ) ;
        }
    }
    else if( m_cParam.m_cSlope.m_nAddType == NCDEF_ADDTYPE_XFLAT )
    {/*单方向加工平坦面*/
		
		m_cPrgDef.m_dTotalMove = 0.6 * dTotalMove ;
        bRetFlag = CreateFlatAreas( DriveMdl, BoundCont, dSlopeAngle, allArea[0], dSideTol,m_cPrgDef, 0, FALSE, tempLoop) ; 
        if( bRetFlag && allArea[0] )
        {
            m_cPrgDef.m_dTotalMove = 0.4 * dTotalMove ;
            bRetFlag = CreateParallelPath( DriveMdl, dLineAngle, allArea[0], flatPath,m_cPrgDef ) ;
        }
    }
    else
    {
		if( m_cFeedDef.m_cConnectDef.m_nLead3DType == NCDEF_LEAD3D_BYBOUND )
		{// 如果是沿边界连刀，则单独处理
			bLead3DByBound = TRUE;
			bRetFlag = CreatePathhByBound(Graph, DriveMdl, m_cParam, m_cSetupDef, -dRotAngle, flatPath);
		}
		else
		{
			if( !( m_cParam.m_bLineFlag & NCDEF_LINEAR_SIDETOL ) )
			{
				m_cPrgDef.m_dTotalMove = dTotalMove ;
				if(m_cMethodDef.m_cSFinishCut.m_bMoveFlag & NCDEF_SFINISH_GUIDESTEP3D)
				{
					CreateGuideParallel3DPath( DriveMdl, dLineAngle, 
						BoundCont, flatPath, m_cPrgDef ) ;
				}
				else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_3DSTEP )
				{
					bRetFlag = CreateParallel3DPath( DriveMdl, dLineAngle, BoundCont, flatPath, m_cPrgDef ) ;
				}
				else
				{
					bRetFlag = CreateParallelPath( DriveMdl, dLineAngle, BoundCont, flatPath, m_cPrgDef ) ;
				}
			}
			else
			{
                if( bRetFlag )
                {
				    m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove ;
				    bRetFlag = CreateFlatAreas( DriveMdl, BoundCont, -1.0, allArea[0], dSideTol,m_cPrgDef,0,FALSE, tempLoop) ; 

                }
				if( bRetFlag && allArea[0] )
				{
					m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove ;
					if(m_cMethodDef.m_cSFinishCut.m_bMoveFlag & NCDEF_SFINISH_GUIDESTEP3D)
					{
						CreateGuideParallel3DPath( DriveMdl, dLineAngle,  
							BoundCont, flatPath, m_cPrgDef ) ;
					}
					else if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_3DSTEP )
					{
						bRetFlag = CreateParallel3DPath( DriveMdl, dLineAngle, allArea[0], flatPath,m_cPrgDef ) ;
					}
					else
					{
						bRetFlag = CreateParallelPath( DriveMdl, dLineAngle, allArea[0], flatPath,m_cPrgDef ) ;
					}
				}
			}
		}        
    }
   	if( bRetFlag && !bLead3DByBound && 
        !(m_cParam.m_cSlope.m_nAddType == NCDEF_ADDTYPE_XFLAT ) &&
        (m_cParam.m_bLineFlag & NCDEF_LINEAR_SIDETOL) )
	{/*生成修边路径*/
        bRetFlag = CreateBoundPath( DriveMdl, BoundCont, bndPath ) ;
    }
    Mini_DeleteContours( allArea[0] ) ;
    Mini_DeleteContours( allArea[1] ) ;
	Mini_DeleteContours( BoundCont  ) ;
	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bRetFlag != TRUE )
    {
        m_nErrorType = JDERROR_GENPATH_ABORT ;
        return FALSE ;
    }
    CSmtCPathLib AllPath ;
	if( m_cParam.m_cSlope.m_nAddType == NCDEF_ADDTYPE_XYALL )
	{
		AllPath.AppendCPathLib ( flatPath ) ;
		AllPath.AppendCPathLib ( steepPath ) ;
	}
	else
	{
		AllPath.AppendCPathLib ( steepPath ) ;
		AllPath.AppendCPathLib ( flatPath ) ;
	}
    MathCAM_AppendBoundPath( AllPath, bndPath ) ;

	// STEP 5 : 路径连接＼转换
	if (!bLead3DByBound)
		AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef,AllPath ) ; /*添加下刀进退刀路径*/
	if( AllPath.GetNumPath()== 0  )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType > 0 )
	{
		if(!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl,AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
	}
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}

	CPathCombine* pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine( *pPComb ) ;
    NewPath.AddData( 0.0, pPComb ) ;

	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
        AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
	
	if( fabs( dRotAngle ) > 1.0e-4 )
	{
		NewPath.RotateGroup ( dRotAngle ) ;
	}
    return TRUE  ;
}


//////////////////////////////////////////////////////////////////////////////////////////
//CSmartFHeightGen : 曲面外形等高精雕刻
//
CSmartFHeightGen::CSmartFHeightGen()
{
}

CSmartFHeightGen::~CSmartFHeightGen()
{
}


int CSmartFHeightGen::GeneratePathEx( CPathGroup&    NewPath   ,  //雕刻路径
					                  CSmartGraphic&  Graph     )  //雕刻图形
{
    // STEP 1 : 构建加工模型
	if (!IsAxisCtrlValid(Graph))	// 刀轴控制模型判断
		return FALSE;

    CSmtCheckMdl DriveMdl, CheckMdl ;
    CSmartLoop* BoundCont = NULL    ; 
    if( ! BuildSFinishGen( NewPath, Graph, DriveMdl, CheckMdl, BoundCont ) )
    {
        return FALSE ;
    }
    // STEP 2 : 计算原始路径
    SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
    m_cPrgDef.m_dTotalMove = 100.0 ;
    CSmtCPathLib AllPath ;
    BOOL bRetFlag = TRUE ;
	if( m_cParam.m_bHeightFlag & NCDEF_HEIGHT_ZIGZAG )
    {
        m_cFeedDef.m_cConnectDef.m_bConnect3DFlag  |= NCDEF_FCONNECT3D_ZIGZAG ;
    }
    else
    {
        m_cFeedDef.m_cConnectDef.m_bConnect3DFlag  &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    }
    m_bTrimByZSize = FALSE ;
    m_cPrgDef.m_dTotalMove = 100.0 ;
    DOUBLE dSlopeAngle = -1.0 ;
    if( m_cParam.m_bHeightFlag & NCDEF_HEIGHT_DELFLAT)
    {
        dSlopeAngle = m_cParam.m_dSlopeAngle ; 
    }
    int nValidMode = NCZLEVEL_VALID_ALLPOS ;
    if( DriveMdl.m_bCheckFlag & SMARTNC_CHECKMDL_TOUCHMODE )
    {/*刀触点计算模式*/
        nValidMode = NCZLEVEL_VALID_TOUCHPOS ;
    }
    else if( ( m_cParam.m_bHeightFlag & NCDEF_HEIGHT_DELFLAT ) || 
              DriveMdl.m_pCheckMdl || IsDelBndPoint() )
    {/*删除平坦区域，只生成陡峭区域路径*/
        nValidMode = NCZLEVEL_VALID_TOOLPOS ;
    }
    BOOL bCutPlane = (m_cParam.m_bHeightFlag & NCDEF_HEIGHT_CUTPLANE ) ? TRUE : FALSE ;
    bRetFlag = CreateHeightPath( DriveMdl, BoundCont, nValidMode,bCutPlane,dSlopeAngle,AllPath, m_cPrgDef ) ;
	Mini_DeleteContours( BoundCont  ) ;
    if( bRetFlag != TRUE )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }

    // STEP 3 : 路径连接＼转换
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef,AllPath ) ; /*添加下刀进退刀路径*/
	if( AllPath.GetNumPath()== 0  )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}
	// 三轴转五轴
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType > 0 )
	{
		if(!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl,AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
	}
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}
	
	CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
	AllPath.AddToPathCombine( *pPComb ) ;
	NewPath.AddData( 0.0, pPComb  ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
        AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
	
    return TRUE  ;
}

//////////////////////////////////////////////////////////////////////////////////////////
//CSmartMixtureGen 
CSmartMixtureGen::CSmartMixtureGen()
{
}


CSmartMixtureGen::~CSmartMixtureGen()
{
}

int CSmartMixtureGen::GeneratePathEx( CPathGroup&   NewPath  ,  /*雕刻路径*/
									  CSmartGraphic& Graph    ) /*雕刻图形*/
{
    // STEP 1 : 构建加工模型
	if (!IsAxisCtrlValid(Graph))	// 刀轴控制模型判断
		return FALSE;

    CSmtCheckMdl DriveMdl, CheckMdl ;
    CSmartLoop* BoundCont = NULL    ; 
    if( ! BuildSFinishGen( NewPath, Graph, DriveMdl, CheckMdl, BoundCont ) )
    {
        return FALSE ;
    }
    // STEP 2 : 计算兜边路径
    SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
    m_cPrgDef.m_dTotalMove = 100.0 ;
    CSmtCPathLib AllPath ;
	CSmartLoop*  allArea[2] = {NULL,NULL}  ;
    CSmtCPathLib   flatPath, steepPath;
    DOUBLE dSlopeAngle = m_cParam.m_cSlope.m_dSlopeAngle ;
    BOOL bRetFlag = FALSE ;
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_3DSTEP )
    {
        m_cSetupDef.m_cModelTol.m_nMdlFlag &=  ~NCDEF_SETUP_3DSTEP ;
    }
	int nAreaType = m_cParam.m_cSlope.m_nAreaType ;
    double dTotalMove = m_cPrgDef.m_dTotalMove  ;
    m_cPrgDef.m_dTotalMove = dTotalMove * 0.25 ;
	BOOL bFollow = FALSE ;
	if( m_cParam.m_bMixtureFlag & NCDEF_MIXTURE_FOLLOW ) bFollow = TRUE ;
    if( DriveMdl.m_bCheckFlag & SMARTNC_CHECKMDL_TOUCHMODE )
    {	/*刀触点计算模式*/
		// 计算出按重叠区域值偏移后的陡峭环allArea[1] xiejunxian、qqs 2013.04.25
	    bRetFlag = CreateFlatAreas( DriveMdl, BoundCont, dSlopeAngle, allArea[0],
									0.0,m_cPrgDef,m_cParam.m_dOverStep,bFollow,allArea[1],
									m_cParam.m_cSlope.m_dExtLen) ; 
	}
    else
	{		
		bRetFlag = CreateMixtureAreas( DriveMdl, BoundCont, dSlopeAngle, m_cParam.m_cSlope.m_dExtLen,
			bFollow, m_cParam.m_dOverStep, allArea, nAreaType ,m_cPrgDef) ; 
	}
	if( bRetFlag && allArea[0] && ( nAreaType != NCDEF_FSLOPE_SLOPE ) )
	{
		if( m_cParam.m_bMixtureFlag & NCDEF_MIXTURE_HZIGZAG )
        {
            m_cFeedDef.m_cConnectDef.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
        }
        else
        {
            m_cFeedDef.m_cConnectDef.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
        }
        if( allArea[0] && allArea[1] )  
		{
			if( nAreaType == NCDEF_FSLOPE_ALL ) m_cPrgDef.m_dTotalMove = dTotalMove * 0.25 ;
			else								m_cPrgDef.m_dTotalMove = dTotalMove * 0.75 ;
		}
        else m_cPrgDef.m_dTotalMove = dTotalMove * 0.75 ;

        if( m_cParam.m_bMixtureFlag & NCDEF_MIXTURE_FOLLOW )
        {/*螺旋走刀*/
            JDNC_FOLLOW followDef ;
            SurfNC_InitPathParam( followDef ) ;
            DOUBLE dSmoothTol = min( 0.5 * m_cFeedDef.m_cStepDef.m_dOverStep, 0.005 ) ;
			CMixtureContourSmoothMdl cCurveSmoothMdl;
			CSmartLoop * pSmoothedLoop = NULL;
			//角度分区线光顺
			if(cCurveSmoothMdl.Smooth(allArea[0],DriveMdl,m_cSetupDef,
				dSmoothTol,pSmoothedLoop))
			{
				bRetFlag = CreateFollowPath( DriveMdl,followDef, pSmoothedLoop,
					flatPath, m_cPrgDef )  ;
				Mini_DeleteContours(pSmoothedLoop);
			}
			else
			{
				bRetFlag = CreateFollowPath( DriveMdl,followDef, allArea[0],
					flatPath, m_cPrgDef )  ;
			}
        }
        else
        {
            bRetFlag = CreateParallelPath( DriveMdl, m_cParam.m_dLineAngle, allArea[0], flatPath, m_cPrgDef ) ;
        }
    }
    if( bRetFlag && allArea[1] && ( nAreaType != NCDEF_FSLOPE_FLAT ) )
    {
        if( allArea[0] && allArea[1] )  
		{
			if( nAreaType == NCDEF_FSLOPE_ALL )	m_cPrgDef.m_dTotalMove = dTotalMove * 0.5 ;
			else								m_cPrgDef.m_dTotalMove = dTotalMove * 0.75 ;
		}
        else m_cPrgDef.m_dTotalMove = dTotalMove * 0.75 ;
        JDNC_STEP  cOldStep = m_cFeedDef.m_cStepDef ;
        m_cFeedDef.m_cStepDef.m_dOverStep = m_cFeedDef.m_cStepDef.m_dZIncStep ;
		// 为角度分区添加增加平面分层
	    if( m_cParam.m_bMixtureFlag & NCDEF_MIXTURE_VZIGZAG )
        {
            m_cFeedDef.m_cConnectDef.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
        }
        else
        {
            m_cFeedDef.m_cConnectDef.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
        }
        int nValidMode = NCZLEVEL_VALID_ALLPOS ;
        DOUBLE dSlopeBrk = -1.0;
        if( DriveMdl.m_bCheckFlag & SMARTNC_CHECKMDL_TOUCHMODE )
        {   /*刀触点计算模式*/ 
			// xiejunxian、qqs 2013.04.25
			// 在开启刀触点后，此处不将分区角度传入计算等高路径的函数中，而是通过
			// CreateHeightPath该函数计算出所有等高路径，然后通过之前求出的陡峭环
			// 进行裁剪，从而解决了在刀触点计算模式下，重叠区域功能不起作用的问题
            nValidMode = NCZLEVEL_VALID_TOUCHPOS ;
        }
        bRetFlag = CreateHeightPath( DriveMdl, allArea[1], nValidMode,
                                     TRUE, dSlopeBrk, steepPath, m_cPrgDef ) ;
        m_cFeedDef.m_cStepDef = cOldStep ;
    }
	Mini_DeleteContours( allArea[0] ) ;
    Mini_DeleteContours( allArea[1] ) ;
    Mini_DeleteContours( BoundCont ) ;
    if( bRetFlag != TRUE )
    {
        m_nErrorType = JDERROR_GENPATH_NOTGENPATH  ;
        return FALSE ;
    }
	if( m_cParam.m_bMixtureFlag & NCDEF_MIXTURE_PARFIRST )
	{ // 平坦区域先加工
		AllPath.AppendCPathLib ( flatPath ) ;
		AllPath.AppendCPathLib ( steepPath ) ;
	}
	else
	{
		AllPath.AppendCPathLib ( steepPath ) ;
		AllPath.AppendCPathLib ( flatPath ) ;
	}
    // STEP 5 : 路径连接＼转换
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef,AllPath ) ; /*添加下刀进退刀路径*/
	if( AllPath.GetNumPath()== 0  )
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}
	// 三轴转五轴
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType != NCDEF_AXIS_VERT )
	{
		if(!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl,AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
		NewPath.m_nType = NC_PATHTYPE_ENGRAVE5AX ;
	}
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}
	
	CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
	AllPath.AddToPathCombine( *pPComb ) ;
	NewPath.AddData( 0.0, pPComb  ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
        AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
	
    return TRUE  ;
}

static UINT MathCAM_CreateOrgEmitPathSubProc(LPVOID lpParam)
{
	if (lpParam == NULL)
		return 0;
	COEPT_DATA *pData = (COEPT_DATA *)lpParam;
	return pData->pFEmiterGen->CreateOrgEmitPathSubProc(pData->nAtCore, pData->nCoreNum, pData->dRadius, pData->DefNull, *pData->pDriveMdl, 
		pData->pTmpPrg, pData->BndCont, pData->AllPath);
}
//////////////////////////////////////////////////////////////////////////////////////////
//CSmartFEmiterGen : 径向放射曲面精雕刻
//
CSmartFEmiterGen::CSmartFEmiterGen()
{
    memset( &m_cParam, 0, sizeof( m_cParam ) ) ;
    m_dMaxRadius = 1.0 ;
}

CSmartFEmiterGen::~CSmartFEmiterGen()
{
}


CSmartLoop* CSmartFEmiterGen::ResortContour(CSmartLoop* AllLoop )  
{
	CSmartLoop LoopLib ;
	if( ! AllLoop ) return FALSE ;
    CSmartLoop *pLoop ;
	for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
	{
		if( pLoop->m_dArea < 0.0 ) pLoop->ReverseLoop() ;
	}
    AllLoop = LoopLib.BuildContour( AllLoop ) ;
	AllLoop = LoopLib.ResortContour( AllLoop, GetSortType(), TRUE ) ;
	return AllLoop ; 
}

/////////////////////////////////////
// 计算不带边界环的刀具路径
int CSmartFEmiterGen::CreateOrgEmitPath( CSmtCheckMdl& DriveMdl,
									     FPNT3D MaxBox[2]     ,
                                         CSmartLoop* BndCont  ,
                                         BOOL     DefNull    ,
                                         CSmtCPathLib& AllPath )
{
	// STEP 1 : 计算中心半径
    UNUSED_ALWAYS( MaxBox ) ;
    DOUBLE dRadius = m_dMaxRadius + DriveMdl.m_pTool->m_fRadius ;
    int nPathNum = (int)fabs(m_cParam.m_dAngRange / m_cParam.m_dAngStep) ;
	if( nPathNum < 1 ) nPathNum = 1 ;

    // STEP 2 :计算平面路径
	CVecPointer NewPath;
	NewPath.resize(nPathNum + 5);
	ZeroMemory(&NewPath[0], sizeof(LPVOID) * (nPathNum + 5));
	// 支持多线程
	BOOL bRet = TRUE;
	if (DriveMdl.GetMultiCopy() != NULL && nPathNum > 1)
	{
        COEPT_DATA ThreadData[NC_CFG_CPU_MAXNUM];
		LPVOID lpParam[NC_CFG_CPU_MAXNUM] = {NULL};
		JDNC_PRGDEF tmpPrg = m_cPrgDef;
		tmpPrg.m_pPrgFunc = NULL ; 
		tmpPrg.m_pPosFunc = NULL ; 
		int nThreadNum = min(m_nCalcThreadNum, (int)nPathNum), i=0, k=1;
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].pDriveMdl = i == 0 ? &DriveMdl : DriveMdl.GetMultiCopy(i-k); 
			if (ThreadData[i].pDriveMdl == NULL)
			{
				i--, k--, nThreadNum--;
				continue;
			}
			ThreadData[i].nAtCore = i;
			ThreadData[i].dRadius = dRadius;
			ThreadData[i].DefNull = DefNull;
			ThreadData[i].pTmpPrg = i == 0 ? m_cPrgDef : tmpPrg;
			ThreadData[i].BndCont = BndCont;
            ThreadData[i].pFEmiterGen = this;
			ThreadData[i].AllPath = &NewPath[0];
			lpParam[i] = &ThreadData[i];
		}
		for (i=0; i<nThreadNum; i++)
		{
			ThreadData[i].nCoreNum = nThreadNum;
		}
		// 调用多线程函数
		bRet = MathCAM_ThreadMainFunc(MathCAM_CreateOrgEmitPathSubProc, lpParam, nThreadNum);
	}
	else
	{
        bRet = CreateOrgEmitPathSubProc(0, 1, dRadius, DefNull, DriveMdl, m_cPrgDef, BndCont, &NewPath[0]);
	}
	if (bRet)
	{
		Nc5D_AddPathArrayToLib(&NewPath[0], nPathNum, AllPath);
	}
	return bRet  ; 
}

int CSmartFEmiterGen::CreateEmitPathBnd( CSmtCheckMdl& DriveMdl, /*网格模型*/ 
		                                 FPNT3D MaxBox[2]        , /*包围盒  */
                                         CSmartLoop* BndCont   , /*边界裁剪  */
                                         CSmtCPathLib&  BndPath) /*陡边路径 */
{
    UNUSED_ALWAYS( MaxBox ) ;
    CSmartLoop *pTrimCont = BndCont ;
    if( fabs( m_cParam.m_dAngRange) < 359 ) 
    {// 角度范围限定
        CSmartLoop *pNewLoop = new CSmartLoop() ;
        CSmartCurve *pCurve = new CSmartCurve() ;
        DOUBLE dRadius = m_dMaxRadius + DriveMdl.m_pTool->m_fRadius ;
        DOUBLE dAngle[2], dStart[2], dEnd[2] ;
        if( m_cParam.m_dAngRange > 0.0 )
        {
            dAngle[0] = ANGLE_TO_RADIAN( m_cParam.m_dAngFrom ) ;
            dAngle[1] = ANGLE_TO_RADIAN( m_cParam.m_dAngFrom + m_cParam.m_dAngRange ) ;
        }
        else
        {
            dAngle[1] = ANGLE_TO_RADIAN( m_cParam.m_dAngFrom ) ;
            dAngle[0] = ANGLE_TO_RADIAN( m_cParam.m_dAngFrom + m_cParam.m_dAngRange) ;
        }
        CSmartArc *pArc = new CSmartArc( m_cParam.m_dCenter, dAngle, dRadius ) ;
        pCurve->AddSect( pArc ) ;
        pArc->GetStart( dStart ) ;
        pArc->GetEnd( dEnd ) ;
        pCurve->AddSect( new CSmartLine( dEnd, m_cParam.m_dCenter ) ) ;
        pCurve->AddSect( new CSmartLine( m_cParam.m_dCenter, dStart )) ;
        pNewLoop->UpdateSect( pCurve ) ;
        if( BndCont == NULL ) pTrimCont = pNewLoop ;
        else
        {
           pTrimCont = Mini_OperateContours( BndCont, pNewLoop,0 ) ;
           delete pNewLoop ;
        }
    }
    CSmtMeshMdl   MeshMdl ;
	double dTotalMove = m_cPrgDef.m_dTotalMove ;
	m_cPrgDef.m_dTotalMove = dTotalMove * 0.8 ;
    if( BuildMeshMdl( MeshMdl,DriveMdl , pTrimCont, TRUE ) == FALSE )
    {
       if( pTrimCont != BndCont ) Mini_DeleteContours( pTrimCont ) ;
       return FALSE ; 
    }
    if( pTrimCont != BndCont ) Mini_DeleteContours( pTrimCont ) ;
	m_cPrgDef.m_dTotalMove = dTotalMove * 0.2 ;
    MeshMdl.GenMeshSelfBound( BndPath, m_cPrgDef ) ;
    return TRUE ;
}

// 生成径向放射加工路径
int CSmartFEmiterGen::GeneratePathEx( CPathGroup& NewPath     ,  /*加工路径*/ 
					                  CSmartGraphic& Graph     )  /*雕刻图形*/
{
    // STEP 1 : 构建加工模型
	if (!IsAxisCtrlValid(Graph))	// 刀轴控制模型判断
		return FALSE;

    CSmtCheckMdl DriveMdl, CheckMdl ;
    CSmartLoop* BoundCont = NULL    ; 
    if( ! BuildSFinishGen( NewPath, Graph, DriveMdl, CheckMdl, BoundCont ) )
    {
        return FALSE ;
    }
    // STEP 2 : 计算最大半径
    m_dMaxRadius = 0.0 ;
    FPNT2D fCenter = { (TFLOAT)m_cParam.m_dCenter[0],(TFLOAT)m_cParam.m_dCenter[1]} ;
    int nNumChk = (int)DriveMdl.m_cAllChk.GetSize() ;
    for( int i = 0 ;i < nNumChk ; i ++ )
    {
        CSmartCheck* pChk = DriveMdl.m_cAllChk.GetAt( i ) ;
        for( int k = 0 ; k < pChk->m_nNumVert ; k ++ ) 
        {
            double dRad = nc_Distance( pChk->m_aVertex[k], fCenter, 2 ) ;
            if( dRad > m_dMaxRadius )
            {
                m_dMaxRadius = dRad ;
            }
        }
    }
    if( BoundCont )
    {
        CSmtCutPath tmpTPath ;
        for( CSmartLoop* pLp = BoundCont ; pLp ; pLp = pLp->next )
        {
            tmpTPath.AddCurve( *pLp->m_pCurve, m_cSetupDef.m_cTolDef ) ;
            for( CSmartLoop* pIsl = pLp->m_pIsland ; pIsl ; pIsl = pIsl->next )
            {
                tmpTPath.AddCurve( *pIsl->m_pCurve, m_cSetupDef.m_cTolDef ) ;
            }
        }
        double dMaxRad = 0.0 ;
        for( CSmtCutPoint* pTPnt = tmpTPath.m_pHead ; pTPnt ; pTPnt = pTPnt->next )
        {
            double dRad = nc_Distance( pTPnt->m_fPoint, fCenter, 2 ) ;
            if( dRad > dMaxRad )
            {
                dMaxRad = dRad ;
            }
        }
        if( dMaxRad < m_dMaxRadius ) m_dMaxRadius = dMaxRad ;
    }
	m_cParam.m_dAngStep = m_cFeedDef.m_cStepDef.m_dOverStep / m_dMaxRadius * 180 / MiniPai;
    // STEP 3 : 计算加工路径
	BOOL bZigZag = m_cParam.m_bEmitFlag & NCDEF_EMIT_ZIGZAG ? TRUE : FALSE ;
	double dTotalMove = 100. ;
	CSmtCPathLib BndPath, AllPath ;
	m_cPrgDef.m_dTotalMove = 0.7 * dTotalMove ;
	if( !bZigZag )
		m_cPrgDef.m_dTotalMove = dTotalMove ;
	BOOL bRet = GeneratePathExSubFunc(DriveMdl, BoundCont, BndPath, AllPath);
    Mini_DeleteContours( BoundCont ) ;
	if (!bRet)
    {
		return FALSE;
    }
    // STEP 4 : 路径连接＼转换
    if( SurfNC_IsAbort() )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }
	
    m_cPrgDef.m_dTotalMove = dTotalMove * 0.3 ;
	if( AllPath.GetNumPath() )
	{
		SmartNC_RegCreateConnect3DPath( MathCAM_GenZDirLeadPathEx );
		if( bZigZag  )
		{ /*路径连接*/
            INT_PTR nPnt = AllPath.m_cAllPath.GetCount() ;
            if( nPnt < 1 ) nPnt = 1 ;
            m_cPrgDef.m_dLimitAt =   ( nPnt * 2 )/ 100.0;
            DOUBLE dMaxDist = m_cFeedDef.m_cConnectDef.m_dMaxDist ;
            AllPath.ConnectPathByLineNo( DriveMdl, dMaxDist, m_cSetupDef.m_cTolDef, m_cPrgDef, TRUE, m_cParam.m_dCenter,&(m_cFeedDef.m_cConnectDef)) ;
		}
		else 
		{
			if( !( m_cParam.m_bEmitFlag & NCDEF_EMIT_TOOUTER ))
			{ /*从外向内*/
                AllPath.ReverseDirect() ;
			}
			AllPath.CreateLeadPath( DriveMdl, m_cSetupDef.m_cTolDef, m_cFeedDef.m_cConnectDef) ;
		}
		SmartNC_RegCreateConnect3DPath( NULL );
	}
	if( m_cParam.m_bEmitFlag & NCDEF_EMIT_SIDETOL  )
	{/*兜边一次*/
		if( GetMillDir() == 1 )
		{ /*顺铣方向*/
			BndPath.ReverseDirect() ;
		}
		MathCAM_AppendBoundPath( AllPath, BndPath ) ;
	}
	// 转化路径
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef ,AllPath );	
	
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType )
	{
		if(!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl,AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
	}
	
    if( SurfNC_IsAbort() )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }
    CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
    AllPath.AddToPathCombine( *pPComb ) ;
    NewPath.AddData( 0.0, pPComb  ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
    {
        AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
    }
    return TRUE ;
}

// 生成刀具路径的子函数
BOOL CSmartFEmiterGen::GeneratePathExSubFunc(CSmtCheckMdl &DriveMdl, CSmartLoop *BndCont, 
                                             CSmtCPathLib &BndPath, CSmtCPathLib &AllPath)
{
	FPNT3D fBox3D[2] ;
	DriveMdl.CalcBox( fBox3D[0], fBox3D[1]) ;
	fBox3D[0][2] = fBox3D[1][2] = 0.0f ;
	BOOL bDefNull = TRUE ;
	// 进度条
	SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
	double dTotalMove = m_cPrgDef.m_dTotalMove ;
	if( ( m_cParam.m_bEmitFlag & NCDEF_EMIT_SIDETOL ) ||
		m_cParam.m_dSideTol >  1.0e-3 )
	{
		m_cPrgDef.m_dTotalMove = 0.5 * dTotalMove ;
		dTotalMove	-= m_cPrgDef.m_dTotalMove ;
	}
	if( ( m_cParam.m_bEmitFlag & NCDEF_EMIT_SIDETOL ) ||
		m_cParam.m_dSideTol >  1.0e-3 )
	{ /*不需生成边界轮廓*/
		CreateEmitPathBnd( DriveMdl, fBox3D, BndCont, BndPath ) ;
		if( SurfNC_IsAbort() )
		{

			m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return FALSE ;
		}
	}
    CSmartLoop* trimBnd = BndCont ; 
	if( m_cParam.m_dSideTol >= 1.0e-3  )
	{/* 校正轮廓*/
		DOUBLE dOffDist = max( m_cParam.m_dSideTol, m_cSetupDef.m_cTolDef.m_dArcTol * 2.0 ) ; 
		CSmartLoop* pNewBnd = BndPath.ConvertToContours( FALSE, GetCorDef(), dOffDist ) ;
		if( pNewBnd != NULL ) 
		{
			trimBnd = pNewBnd ;

			bDefNull = FALSE ;
		}
	}
	// STEP 3 : 计算原始路径
	m_cPrgDef.m_dTotalMove = dTotalMove ;
	CreateOrgEmitPath( DriveMdl, fBox3D, trimBnd, bDefNull, AllPath ) ;
    if( trimBnd != BndCont )
    {
        Mini_DeleteContours( trimBnd ) ;
    }
	if( SurfNC_IsAbort() )
	{
		m_nErrorType = JDERROR_GENPATH_ABORT  ;
		return FALSE ;
	}
	return TRUE ;
}

// 计算原始加工路径的主函数
int CSmartFEmiterGen::CreateOrgEmitPathSubProc(int nAtCore, int nCoreNum, double dRadius, BOOL DefNull, 
												CSmtCheckMdl &DriveMdl, JDNC_PRGDEF &ProgDef, 
												CSmartLoop* BndCont, LPVOID* AllPath)
{
	// 根据角度范围和角度间距得到循环次数
	int nCount = (int)fabs(m_cParam.m_dAngRange / m_cParam.m_dAngStep) ;
	if( nCount < 1 )	nCount = 1 ;

	double dAngleInc = m_cParam.m_dAngRange / nCount;			// 每步循环增加的角度步长
	double dAngle = m_cParam.m_dAngFrom + nAtCore * dAngleInc;	// 起始角度，要加nStart个角度步长
	dAngleInc = dAngleInc * nCoreNum;								// 角度步长与循环步长同步
	FPNT3D fStart, fEnd ;
	fStart[2] = fEnd[2] = DriveMdl.m_fBottom;
	fStart[0] = (TFLOAT)m_cParam.m_dCenter[0];
	fStart[1] = (TFLOAT)m_cParam.m_dCenter[1];
	JDNC_TOL TmpTol = GetSurfaceTol();
    DOUBLE dLineTol = max( 2.0e-4,(0.1 * m_cSetupDef.m_cTolDef.m_dArcTol) ) ;
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC )
    {
        dLineTol = 2.0e-6 ;
    }
	TFLOAT fZMin, fZMax ;
	fZMin = TFLOAT( GetBottomHeight()) ;
	fZMax = TFLOAT( GetTopHeight() ) ;
	ProgDef.m_dLimitAt =  nCount / ( ProgDef.m_dTotalMove ) ;
	ProgDef.m_dStepAt  = 0.0 ;
	ProgDef.m_dIncStep = 1.0  ;
	// 临时进度
	JDNC_PRGDEF cTmpProg ;
	cTmpProg.m_dLimitAt = 0. ;
	cTmpProg.m_dStepAt  = 0 ;
	cTmpProg.m_dIncStep = 1. ;
	cTmpProg.m_pBrkFunc = NULL ;
	cTmpProg.m_pPrgFunc = NULL ;
	cTmpProg.m_pPosFunc = NULL ;
	int i=0, k=0;
	for( i = nAtCore ; i < nCount ; i += nCoreNum, dAngle += dAngleInc ) 
	{
		CSmtCutPath tmpPath, *pTNext;
		MathCAM_MovePrgStep(ProgDef);
		if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
			break ;
		double dRadAng = ANGLE_TO_RADIAN( dAngle ) ;
		fEnd[0] = fStart[0] + TFLOAT( cos( dRadAng ) * dRadius) ;
		fEnd[1] = fStart[1] + TFLOAT( sin( dRadAng ) * dRadius) ;
		tmpPath.ClearAllPoint() ;
		tmpPath.AddPoint( fStart ) ;
		tmpPath.AddPoint( fEnd ) ;
		tmpPath.InsertCPoint( TmpTol.m_dMaxStep ) ;
		tmpPath.VerifyLinePath( DriveMdl, TmpTol, cTmpProg ) ;
		if( DefNull ) tmpPath.DefineEdgePoint(DriveMdl,TRUE ) ;
		tmpPath.LabelNullPointByContour( BndCont ) ;
		if( DefNull ) tmpPath.LabelNullPointByZValue(fZMin, fZMax) ;
		CSmtCutPath *pNewPath = tmpPath.BreakAtNullPoint( FALSE,m_cFeedDef.m_cConnectDef.m_dDelShort ) ;
		AllPath[i] = pNewPath;
		for(  pTNext = NULL ; pNewPath ; pNewPath = pTNext  )
		{
			pTNext = pNewPath->next ;
			pNewPath->m_nLineNo = i + 1;
			pNewPath->m_nLayerNo = 0 ;
            pNewPath->DelPointOnLine( dLineTol ) ;
		}
	}
	if( ProgDef.m_pBrkFunc && ProgDef.m_pBrkFunc () )
	{// 中断时清除已生成路径
		for( k = nAtCore ; k < i ; k += nCoreNum) 
		{
			if (AllPath[k] != NULL)
			{
				Nc5D_DeleteSmtCutPath((CSmtCutPath*)AllPath[k]);
			}
		}
		return FALSE;
	}

	return TRUE;
}

//////////////////////////////////////////////////////////////////////////////////////////
//CSmartFUWLineGen : 曲面流线曲面精雕刻

// 该结构体用于存储构造面以及构造面的原始面，用于构造面法生成曲面流线中 qqs 2013.07.01
struct CCombSurf
{
	CGeoSurf* m_pCombSurf;
	C3DSurfArray m_pSourceSurf;

	void ClearAllSurf()
	{
		delete m_pCombSurf;
		m_pCombSurf = NULL;
	}

	CCombSurf(CGeoSurf* pCombSurf, CEntList* pSourceSurfList)
	{
		if (pCombSurf)
		{
			m_pCombSurf = pCombSurf->CopyMyself();
		}		
		POSITION pos = pSourceSurfList->GetHeadPosition();
		while(pos)
		{
			CEttSurface* pEttSurf = (CEttSurface*) pSourceSurfList->GetNext(pos);
			if (NULL == pEttSurf) continue;
			m_pSourceSurf.Add(pEttSurf->GetSurf());
		}
	}
};

CSmartFUWLineGen::CSmartFUWLineGen()
{
}

CSmartFUWLineGen::~CSmartFUWLineGen()
{
}
int CSmartFUWLineGen::GeneratePathEx( CPathGroup&    NewPath    ,  /*雕刻路径*/
					                  CSmartGraphic& Graph     )  /*雕刻图形*/
{ // 生成刀具路径 
	if( !UpdateTool3D(Graph) )
    {
        return FALSE ;
    }
	if (Graph.FindNetSurf( NCDEF_SURF_DRIVE ))
	{// 网格面不能生成曲面流线路径
		m_nErrorType = JDERROR_GENPATH_NETSURFUW ;
		return FALSE ;
	}

	// 当非DEBUG版本时，去掉磨削调整标记
#ifndef _DEBUG
	m_cParam.m_bUWLineFlag &= ~NCDEF_UWLINE_GRNDMDFY;
	m_cParam.m_bUWLineFlag &= ~NCDEF_UWLINE_TOOLPOS;	
#endif

	BOOL bCheck = Graph.HasCheckSurf () ;
	// 判断是否有干涉模型
	CSmtCheckMdl DriveMdl, CheckMdl ;
	m_cSetupDef.m_cModelTol.m_nMdlFlag |= NCDEF_SETUP_DELBNDPNT ;
	// 如果有干涉面或者要求自动干涉检查,生成干涉模型
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType > 0 )
	{
        DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE;
        CheckMdl.m_bCheckMode = SMART_MODEL_ROTATE;
    }
	if( !BuildDriveAndCheckMdl( Graph, DriveMdl, CheckMdl ) )
	{
		m_nErrorType = JDERROR_GENPATH_NOSURFACE  ;
		return 0 ; 
	}
	DriveMdl.m_fBottom -= (TFLOAT) m_pTool->m_fHeight ;
	// 生成曲面流线路径
	ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_STEP_UWLINE ) ;
	double dCur[4] = { 30., 30., 30., 10. } ;
	// 得到轮廓线
	CSmartLoop *AllLoop = Graph.GetAllContour( m_cSetupDef ) ;
    OffsetBorder( AllLoop ) ;
	RFRAME dNcMtx ;
	if( !Graph.GetTPlaneFrame ( dNcMtx ) )
		mathInitRFrame( &dNcMtx ) ;
	if( m_c5DCtrlDef.m_cToolAxisApp.m_nFrameFlag )
	{
		mathMultRFrame( &dNcMtx, &m_c5DCtrlDef.m_cToolAxisApp.m_dIndexFrame, &dNcMtx ) ;
	}
	BOOL bRet = TRUE ;
	// 五轴加工
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType > 0 )
	{
		bRet = Create5AxUWPath( DriveMdl, CheckMdl, Graph, AllLoop, &dNcMtx, 100, NewPath ) ;
	}
    else
	{
		if( !( m_cParam.m_bUWLineFlag & NCDEF_UWLINE_CHECK ) )
		{
			dCur[0] += dCur[2] * 0.5, dCur[1] += dCur[2] * 0.5 ; dCur[2] = 0.;
		}
		if( !bCheck )
		{
			dCur[0] += dCur[1] ; dCur[1] = 0.;
		}
		// 三轴加工
	    bRet = Create3AxUWPathNew( DriveMdl, CheckMdl, Graph, AllLoop, &dNcMtx, bCheck, dCur, NewPath );
	}

	Mini_DeleteContours( AllLoop ) ;
	return bRet ;
}

BOOL CSmartFUWLineGen::Create3AxUWPathNew ( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, CSmartLoop *AllLoop,
										 RFRAME *dNcMtx, BOOL bCheck, double dCur[4], CPathGroup &NewPath ) 
{
	C3DSurfArray AllSurf, NormalSurf;
	CCombSurfArray CombSurfArray;
	Graph.GetAllSurface( m_cSetupDef, AllSurf ) ;
	BOOL bGenNormalPath = FALSE, bGenCombSurfPath = FALSE;
	double dTotalMove = 0.;
	if (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_ONEBYONE)	
	{
		bGenNormalPath = TRUE;
		bGenCombSurfPath = FALSE;
		NormalSurf.Append(AllSurf);
	}
	else
	{
		// 构造四边面
	    SurfsTo4BndSurfNew(AllSurf, CombSurfArray, NormalSurf);

		// 进度条分配
        double dProportion = (double)NormalSurf.GetSize() / (double)AllSurf.GetSize();
		dTotalMove = (dCur[0] + dCur[1] + dCur[2] + dCur[3]) * (1.0 - dProportion);
		for (int i=0; i<4; i++)
			dCur[i] *= dProportion;

		bGenCombSurfPath = CombSurfArray.GetSize() > 0;
		bGenNormalPath = NormalSurf.GetSize() > 0;		
	}

	CSmtCPathLib	NormalSurfPath ;
	// 先生成一般面的流线路径
	BOOL bRetForNormalSurf = FALSE;
	if (bGenNormalPath)
	{
        bRetForNormalSurf = Create3AxUWPathSubFunc(DriveMdl, CheckMdl, NormalSurfPath, NormalSurf, dNcMtx, dCur, AllLoop, bCheck);
		// 进度条中断
		if (SurfNC_IsAbort())
		{
			NormalSurfPath.ClearAllPath();
			return FALSE;
		}
	}
	CSmtCPathLib	CombSurfPath ;
	BOOL bRetForCombSurf = FALSE;
	CSmtCheckMdl DrMdl; // 因构造面算法会将DriveMdl修改，所以需要重建DrMdl返回用于添加慢速下刀路径 qqs 2014.03.13
	// 生成边界构面的流线路径
	if (bGenCombSurfPath)
	{
 		bRetForCombSurf = CreateAllCombSurfPathNew(Graph, CombSurfArray, DriveMdl,DrMdl, dNcMtx, AllLoop, CombSurfPath, dTotalMove, dCur);
		if( CheckMdl.m_cAllChk.GetSize() > 0 ) 
		{// 恢复DrMdl的CheckMdl
			DrMdl.SetCheckMdl( & CheckMdl) ;
		}
		// 进度条中断
 		if (SurfNC_IsAbort())
 		{
 			CombSurfPath.ClearAllPath();
 			return FALSE;
 		}
	}

	// 当构造面和非构造面均未生成路径时，返回提示“未生成路径” qqs 2013.07.31
	if (!(bRetForCombSurf || bRetForNormalSurf))
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE;
	}

	if (Get3DRCompMask())
	{
		Add3DRCompVectorToPath(NormalSurfPath, NewPath, 0);
		Add3DRCompVectorToPath(CombSurfPath, NewPath, 1);
	}
	else
	{
		////////////////////连刀///////////////
		CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		NormalSurfPath.AddToPathCombine ( *PComb ) ;
		CombSurfPath.AddToPathCombine ( *PComb ) ;
		NewPath.AddData ( 0, PComb ) ;
	}	
	// 添加慢速和相对高度连接
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
	{
		if (bGenCombSurfPath)
		{
			AddPlungeAndRelativeH( &DrMdl, &NewPath, TRUE) ;
		}
		else
		{
			AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE) ;
		}
	}
		
	return TRUE ;
}

BOOL CSmartFUWLineGen::Create3AxUWPath ( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, CSmartLoop *AllLoop,
										 RFRAME *dNcMtx, BOOL bCheck, double dCur[4], CPathGroup &NewPath ) 
{
	C3DSurfArray AllSurf, NormalSurf, CombSurf;
	Graph.GetAllSurface( m_cSetupDef, AllSurf ) ;
	BOOL bGenNormalPath = FALSE, bGenCombSurfPath = FALSE;
	double dTotalMove = 0.;
	if (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_ONEBYONE)	
	{
		bGenNormalPath = TRUE;
		bGenCombSurfPath = FALSE;
		NormalSurf.Append(AllSurf);
	}
	else
	{
		// 构造四边面
		SurfsTo4BndSurf(AllSurf, CombSurf, NormalSurf);
		// 进度条分配
        double dProportion = (double)NormalSurf.GetSize() / (double)AllSurf.GetSize();
		dTotalMove = (dCur[0] + dCur[1] + dCur[2] + dCur[3]) * (1.0 - dProportion);
		for (int i=0; i<4; i++)
			dCur[i] *= dProportion;

		bGenCombSurfPath = CombSurf.GetSize() > 0;
		bGenNormalPath = NormalSurf.GetSize() > 0;		
	}
	CSmtCPathLib	NormalSurfPath ;
	// 先生成一般面的流线路径
	BOOL bRetForNormalSurf = FALSE;
	if (bGenNormalPath)
	{
        bRetForNormalSurf = Create3AxUWPathSubFunc(DriveMdl, CheckMdl, NormalSurfPath, NormalSurf, dNcMtx, dCur, AllLoop, bCheck);
		// 进度条中断
		if (SurfNC_IsAbort())
		{
			NormalSurfPath.ClearAllPath();
			return FALSE;
		}
	}
	CSmtCPathLib	CombSurfPath ;
	BOOL bRetForCombSurf = FALSE;
	// 生成边界构面的流线路径
	if (bGenCombSurfPath)
	{
		bRetForCombSurf = CreateAllCombSurfPath(Graph, CombSurf, DriveMdl,dNcMtx, AllLoop, CombSurfPath, dTotalMove);
		// 进度条中断
		if (SurfNC_IsAbort())
		{
			CombSurfPath.ClearAllPath();
			return FALSE;
		}
	}

	// 当构造面和非构造面均未生成路径时，返回提示“未生成路径” qqs 2013.07.31
	if (!(bRetForCombSurf || bRetForNormalSurf))
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE;
	}

	if (Get3DRCompMask())
	{
		Add3DRCompVectorToPath(NormalSurfPath, NewPath, 0);
		Add3DRCompVectorToPath(CombSurfPath, NewPath, 1);
	}
	else
	{
		////////////////////连刀///////////////
		CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		NormalSurfPath.AddToPathCombine ( *PComb ) ;
		CombSurfPath.AddToPathCombine ( *PComb ) ;
		NewPath.AddData ( 0, PComb ) ;
	}	

	return TRUE ;
}

void CSmartFUWLineGen::Add3DRCompVectorToPath(CSmtCPathLib &AllPath, CPathGroup &NewPath, int nPathType)
{
	struct SurfNor
	{
		PNT3D m_pPnt;
		SurfNor(FPNT3D pnt)
		{
			m_pPnt[0] = pnt[0];
			m_pPnt[1] = pnt[1];
			m_pPnt[2] = pnt[2];
		}
	};
	std::vector<SurfNor*> surfNorList;

	// 1.提取路径点对应曲面点的曲面法向
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition();
	while (pos)
	{
		CSmtCutPath* pCut = AllPath.m_cAllPath.GetNext(pos);
		if ( pCut->m_bFeedType != JDNC_FEEDTYPE_ROUGH ) continue;

		CSmtCutPointEx* pCutPt = (CSmtCutPointEx*)pCut->m_pHead;
		while (pCutPt)
		{
			SurfNor* tmp = NULL;
			
			if (nPathType == 0) // 普通曲面生成的三轴路径，曲面法向存储在m_fSurfNor中
			{
				tmp = new SurfNor(pCutPt->m_fSurfNor);			
			}
			else if(nPathType == 1) // 构造曲面按照五轴投影方法生成的三轴路径，曲面法向存储在m_fSurfPos中
			{
				tmp = new SurfNor(pCutPt->m_fSurfPos);
			}
			else if (nPathType == 2) // 五轴路径曲面法向存储在m_fSurfPos中
			{
				tmp = new SurfNor(pCutPt->m_fSurfPos);
			}
			mathUniVec(tmp->m_pPnt, MIN_LEN);
			pCutPt = (CSmtCutPointEx*)pCutPt->next;
			surfNorList.push_back(tmp);
		}
	}

	////////////////////连刀///////////////
	CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine ( *PComb ) ;
	Add3DRCompMask(*PComb,Get3DRCompMask());

	CPathEntity* pHead = NULL;
	pHead = PComb->m_pHead;
	
	// 2.将曲面法向赋值给CPathEntity中的m_pProjDir
	int nCount = 0;
	while (pHead)
	{
		FPNT3D LeadVec;
		switch (pHead->m_bFeedType)
		{
		case JDNC_FEEDTYPE_LEAD:
			if (pHead->prev && pHead->prev->m_bFeedType == JDNC_FEEDTYPE_ROUGH)
			{
				for (int i = 0; i < 3; i++)
					LeadVec[i] = (float)surfNorList[nCount-1]->m_pPnt[i];	
			}
			else if (pHead->next && pHead->next->m_bFeedType == JDNC_FEEDTYPE_ROUGH)
			{
				for (int i = 0; i < 3; i++)
					LeadVec[i] = (float)surfNorList[nCount]->m_pPnt[i];	
			}
			else 
				break;

			if(pHead->GetType() == NC_PATH_LINE3D)
			{
				if (nPathType != 2 || !pHead->m_pTDir)
					pHead->m_pTDir = new CNcToolDir;

				pHead->m_pTDir->m_nNumDir = 2;
				pHead->m_pTDir->m_pProjDir = new double[2][3];

				for (int i = 0; i < 3; i++)
				{
					pHead->m_pTDir->m_pProjDir[0][i] = LeadVec[i];
					pHead->m_pTDir->m_pProjDir[1][i] = LeadVec[i];
				}
			}
			else if(pHead->GetType() == NC_PATH_PLINE3D)
			{
				CPathPLine3D* pPLine3D = (CPathPLine3D*)pHead;

				if (nPathType != 2 || !pHead->m_pTDir)
					pHead->m_pTDir = new CNcToolDir;

				pHead->m_pTDir->m_nNumDir = pPLine3D->m_nCount + 1;
				pHead->m_pTDir->m_pProjDir = new double[pPLine3D->m_nCount + 1][3];
				for (int i=0; i<=pPLine3D->m_nCount; i++)
					for (int j = 0; j < 3; j++)
						pHead->m_pTDir->m_pProjDir[i][j] = LeadVec[j];
			}
			break;
		case JDNC_FEEDTYPE_RCOMPOPEN:
			for (int i = 0; i < 3; i++)
				LeadVec[i] = (float)surfNorList[nCount]->m_pPnt[i];

			if (nPathType != 2 || !pHead->m_pTDir)
				pHead->m_pTDir = new CNcToolDir; 

			if (nPathType == 2 && !pHead->m_pTDir->m_pTDir)
			{
				pHead->m_pTDir->m_pTDir = new double[2][3];

				for (int i = 0; i < 3; i++)
				{
					pHead->m_pTDir->m_pTDir[0][i] = pHead->next->m_pTDir->m_pTDir[0][i];
					pHead->m_pTDir->m_pTDir[1][i] = pHead->next->m_pTDir->m_pTDir[0][i];
				}
			}

			pHead->m_pTDir->m_nNumDir = 2;
			pHead->m_pTDir->m_pProjDir = new double[2][3];
			for (int i = 0; i < 3; i++)
				pHead->m_pTDir->m_pProjDir[1][i] = LeadVec[i];
			pHead->m_bFeedType = JDNC_FEEDTYPE_LEAD;
			break;
		case JDNC_FEEDTYPE_RCOMPCLOSE:
			for (int i = 0; i < 3; i++)
				LeadVec[i] = (float)surfNorList[nCount-1]->m_pPnt[i];

			if (nPathType != 2 || !pHead->m_pTDir)
				pHead->m_pTDir = new CNcToolDir; 

			if (nPathType == 2 && !pHead->m_pTDir->m_pTDir)
			{				
				pHead->m_pTDir->m_pTDir = new double[2][3];
				CPathPLine3D* pPLine3D = (CPathPLine3D*)pHead->prev;
				for (int i = 0; i < 3; i++)
				{
					pHead->m_pTDir->m_pTDir[0][i] = pHead->prev->m_pTDir->m_pTDir[pPLine3D->m_nCount][i];
					pHead->m_pTDir->m_pTDir[1][i] = pHead->prev->m_pTDir->m_pTDir[pPLine3D->m_nCount][i];
				}
			}

			pHead->m_pTDir->m_nNumDir = 2;
			pHead->m_pTDir->m_pProjDir = new double[2][3];
			for (int i = 0; i < 3; i++)
				pHead->m_pTDir->m_pProjDir[0][i] = LeadVec[i];
			pHead->m_bFeedType = JDNC_FEEDTYPE_LEAD;
			break;
		case JDNC_FEEDTYPE_ROUGH:
			if(pHead->GetType() == NC_PATH_LINE3D)
			{
				if (nPathType != 2 || !pHead->m_pTDir)
					pHead->m_pTDir = new CNcToolDir;

				pHead->m_pTDir->m_nNumDir = 2;
				pHead->m_pTDir->m_pProjDir = new double[2][3];

				for (int i = 0; i < 3; i++)
					pHead->m_pTDir->m_pProjDir[0][i] = surfNorList[nCount]->m_pPnt[i];

				nCount++;
				for (int i = 0; i < 3; i++)
					pHead->m_pTDir->m_pProjDir[1][i] = surfNorList[nCount]->m_pPnt[i];

				nCount++;
			}
			else if(pHead->GetType() == NC_PATH_PLINE3D)
			{
				CPathPLine3D* pPLine3D = (CPathPLine3D*)pHead;
				if (nPathType != 2 || !pHead->m_pTDir)
					pHead->m_pTDir = new CNcToolDir;
				pHead->m_pTDir->m_nNumDir = pPLine3D->m_nCount + 1;
				pHead->m_pTDir->m_pProjDir = new double[pPLine3D->m_nCount + 1][3];
				for (int i=0; i<=pPLine3D->m_nCount; i++)
				{
					for (int j = 0; j < 3; j++)
						pHead->m_pTDir->m_pProjDir[i][j] = surfNorList[nCount]->m_pPnt[j];
					nCount++;
				}
			}
			break;
		default:
			break;
		}
		pHead = pHead->next;
	}

	int nSize = (int)surfNorList.size();
	for (int i = 0; i < nSize; i++)
	{
		if(surfNorList[i] != NULL)
		{
			delete surfNorList[i];
			surfNorList[i] = NULL;
		}
	}

	NewPath.AddData ( 0, PComb ) ;
}

// Create3AxUWPath的子函数
BOOL CSmartFUWLineGen::Create3AxUWPathSubFunc(CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmtCPathLib &AllPath, C3DSurfArray &AllSurf, 
											  RFRAME *dNcMtx, double dCur[4], CSmartLoop *AllLoop, BOOL bCheck)
{
	INT_PTR nSize = AllSurf.GetCount();
	if (nSize <= 0)
		return FALSE;
	BOOL *arrbSpiral = new BOOL[nSize];
	memset(arrbSpiral, FALSE, nSize * sizeof(BOOL));
	CFlowlineGen	cNcGen( m_cSetupDef, m_cFeedDef, m_cStockDef, m_cToolDef, m_cSpeedDef, m_pTool, m_nCalcThreadNum, m_cParam) ;

	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE;
	if (IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}

	if( !cNcGen.CreateUWLinePath( AllPath, AllSurf, *dNcMtx, m_cParam, m_cPrgDef, dCur[0], &m_nErrorType, arrbSpiral) )
	{
		// DriveMdl的数据在本函数拷贝，则在本函数删除
		if( bLocalCopy == TRUE ) 
			DriveMdl.DeleteMultiCopy() ;
		delete arrbSpiral;
		return FALSE ;
	}

	// 路径裁剪
	TrimPath(AllPath, AllLoop);

	// 如果选择了自动干涉检查
	JDNC_TOL Tol = GetSurfaceTol() ;
	if( bCheck )
	{
		CheckMdl.m_fBottom -= 1. ;
		if( !cNcGen.CheckAllPathByCheckMdl( CheckMdl, AllPath, Tol, 
			m_cParam.m_bUWLineFlag & NCDEF_UWLINE_CHECK,FALSE, 
			m_cPrgDef, dCur[1], &m_nErrorType) )
		{
			// DriveMdl的数据在本函数拷贝，则在本函数删除
			if( bLocalCopy == TRUE ) 
				DriveMdl.DeleteMultiCopy() ;
			delete arrbSpiral;
			return FALSE ;
		}
	}

	BOOL bLeadPathCheck = FALSE;
	if( m_cParam.m_bUWLineFlag & NCDEF_UWLINE_CHECK )
	{
		if( !cNcGen.CheckAllPathByCheckMdl( DriveMdl, AllPath, Tol, TRUE, TRUE,
			m_cPrgDef, dCur[2], &m_nErrorType) )
		{
			// DriveMdl的数据在本函数拷贝，则在本函数删除
			if( bLocalCopy == TRUE ) 
				DriveMdl.DeleteMultiCopy() ;
			delete arrbSpiral;
			return FALSE ;
		}
		bLeadPathCheck = TRUE;
	}


	// 把首尾相连的路径合并
	CombinePath(AllPath.m_cAllPath);
	AllPath.DeleteShortPath( m_cFeedDef.m_cConnectDef.m_dDelShort ) ;
	// 添加进退和连刀
	AddAllPathLeadPath( DriveMdl, AllPath, bLeadPathCheck, dCur[3], arrbSpiral ) ;
	delete arrbSpiral;

	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
		DriveMdl.DeleteMultiCopy() ;

	if (AllPath.m_cAllPath.IsEmpty())
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH;
		return FALSE;
	}
	return TRUE;
}

BOOL CSmartFUWLineGen::Create5AxUWPath ( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, 
										 CSmartLoop *AllLoop, RFRAME *dNcMtx, double dCur, CPathGroup &NewPath ) 
{
	C5XGraph allGraph ; // 五轴信息
	
	DriveMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
	CheckMdl.m_bCheckMode = SMART_MODEL_ROTATE ;
	Graph.GetAll5XObject( m_cSetupDef, 0, &allGraph ) ;
	CSmartTool *pDrvTool = m_pTool->Offset ( -2 * m_cSetupDef.m_cTolDef.m_dArcTol ) ;
	if( pDrvTool )
		DriveMdl.UpdateRotateTool ( pDrvTool ) ;
	else
		DriveMdl.UpdateRotateTool ( DriveMdl.m_pTool ) ;
	CSmartTool *pChkTool = m_pSafeTool->Offset( -2 * m_cSetupDef.m_cTolDef.m_dArcTol ) ;
	if( pChkTool)
		CheckMdl.UpdateRotateTool( pChkTool ) ;
	else
		CheckMdl.UpdateRotateTool( CheckMdl.m_pTool ) ;
	// 构建包围盒曲面，用于路径计算提速
	DriveMdl.UpdateBoundSurf() ;
	CheckMdl.UpdateBoundSurf() ;
	DriveMdl.m_pHolderShape = m_pHolderTool ;
	CheckMdl.m_pHolderShape = m_pHolderTool ;
	
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	BOOL bLocalCopy = FALSE ;	
	if (IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}
	
	// 生成曲面流线路径
	C3DSurfArray    AllSurf;
	Graph.GetAllSurface ( m_cSetupDef, AllSurf ) ;

	CSmtCPathLib	AllPath ;
	CFlowlineGen	cNcGen( m_cSetupDef, m_cFeedDef, m_cStockDef, m_cToolDef, m_cSpeedDef, m_pTool, m_nCalcThreadNum, m_cParam) ;
	BOOL bRet = cNcGen.Create5AxUWLinePath ( &DriveMdl, AllPath, AllSurf, *dNcMtx, m_cParam,  
		m_c5DCtrlDef, allGraph, m_cPrgDef, AllLoop, dCur, &m_nErrorType) ;
	// DriveMdl的数据在本函数拷贝，则在本函数删除
	if( bLocalCopy == TRUE ) 
	{
		DriveMdl.DeleteMultiCopy() ;
	}
	if( pChkTool ) delete pChkTool ;
	if( pDrvTool ) delete pDrvTool ;
    DriveMdl.UpdateCurrTool( m_pTool ) ;
    CheckMdl.UpdateCurrTool( m_pSafeTool ) ;
	if( !bRet ) return FALSE ;

	NewPath.m_nType = NC_PATHTYPE_ENGRAVE5AX ;
	if (Get3DRCompMask() && FALSE) // 五轴路径因连刀问题以及50系统暂不支持，所以暂时不启用三维磨损补偿功能 qqs 2013.11.05
	{
		AddRCompPathTo5AxPathLib(AllPath, TRUE);
		Add3DRCompVectorToPath(AllPath, NewPath, 2);
	}
	else
	{
		////////////////////连刀///////////////
		CPathCombine *PComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
		AllPath.AddToPathCombine ( *PComb ) ;
		NewPath.AddData ( 0., PComb ) ;
	}

	return TRUE ;
}

// 把首尾相连的路径合并
void CSmartFUWLineGen::CombinePath(CSmtCPathList &PathList)
{
	CSmtCPathList FinalList, SameLinePaths;
	while (!PathList.IsEmpty())
	{
		SameLinePaths.RemoveAll();
		// 移动同行号的路径
		MoveSameLinePath(PathList, SameLinePaths, PathList.GetHeadPosition());
		// 同行号路径个数小于2，则直接移到最终路径中
		if (SameLinePaths.GetCount() < 2)
		{
			FinalList.AddTail(&SameLinePaths);
			continue;
		}
            
		POSITION pos = SameLinePaths.GetTailPosition(), nextpos = NULL;
		CSmtCutPath *pPath = NULL, *pNext = NULL;
		// 匹配路径
		while (pos)
		{
			nextpos = pos;
			pPath = SameLinePaths.GetPrev(pos);
			SameLinePaths.GetNext(nextpos);
			if (NULL == nextpos)
				nextpos = SameLinePaths.GetHeadPosition();
			pNext = SameLinePaths.GetAt(nextpos);
			
			// 如果下一路径尾跟本路径头不相连，循环下一个
			if (fabs(pPath->m_pTail->m_fPoint[0] - pNext->m_pHead->m_fPoint[0]) > NC_ZERO ||
				fabs(pPath->m_pTail->m_fPoint[1] - pNext->m_pHead->m_fPoint[1]) > NC_ZERO ||
				fabs(pPath->m_pTail->m_fPoint[2] - pNext->m_pHead->m_fPoint[2]) > NC_ZERO)
			{
				continue;
			}

			// 连接这两个路径
			pPath->AppendCutPoint(pNext->m_pHead->next);
			pNext->m_pHead->next = pNext->m_pTail = NULL;
            SameLinePaths.RemoveAt(nextpos);
			delete pNext;
            if (SameLinePaths.GetCount() < 2)
				break;
		}		
		FinalList.AddTail(&SameLinePaths);
	}
	PathList.AddTail(&FinalList);
}

void CSmartFUWLineGen::AddAllPathLeadPath ( CSmtCheckMdl &DriveMdl, CSmtCPathLib &AllPath, BOOL bCheck, 
										    double dCur, BOOL *arrbSpiral/* = NULL*/, BOOL bCombSurfPath /*= FALSE*/ )
{
	if( AllPath.m_cAllPath.GetCount () < 1 ) return ;
	// 首先清除点数为1的路径
	DelOnePntPath(AllPath);

	int nNum = AllPath.GetNumPath () ;
	if( nNum < 1 ) nNum = 1 ;
	m_cPrgDef.m_dLimitAt  =  1.0			;   // 前进一次的计算量
	m_cPrgDef.m_dStepAt   =  0.0			;   // 当前计算进度
	m_cPrgDef.m_dIncStep  =  dCur / nNum	;   // 每次计算的前进量
	CSmtCPathLib TmpPathLib;
	while (AllPath.m_cAllPath.GetCount() > 0)
	{
		CSmtCPathLib LayerPath;
		int nLayerNo = MoveOneLayerPath(AllPath, LayerPath);// 移动一层路径
		if (arrbSpiral && arrbSpiral[nLayerNo] == FALSE)
		{// 非螺旋走刀，则对路径排序
			// 若加工次序为“区域优先”，则对路径作区域排序
			if (m_cSetupDef.m_cOrderDef.m_nLayerOrd == NCDEF_LAYERORD_REGION)
				SortPathByRegion(LayerPath.m_cAllPath, bCombSurfPath);
		}
		AddLayerPathLeadPath(DriveMdl, bCheck, LayerPath);
		TmpPathLib.AppendCPathLib(LayerPath);
	}
	AllPath.AppendCPathLib(TmpPathLib);
}

void CSmartFUWLineGen::AutoCheckAllPath ( CSmtCheckMdl& DriveMdl, CPathGroup& NewPath )
{
	// 自动干涉检查路径
//	MathCAM_SetNewStepEx( IDS_PATHGEN_REMOVE_OVERCUT ) ;
	CSmtCPathLib AllPath ;
	MathCAM_AddPathGroup( AllPath,  NewPath , m_cSetupDef.m_cTolDef, FALSE) ;
	if( ! AllPath.VerifyCutPath( DriveMdl, GetSurfaceTol(), m_cPrgDef ) )
	{
		return  ;
	}
	AllPath.DelPointOnLine() ; // 删除同一直线上的点
	CPathCombine * pPComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
	AllPath.AddToPathCombine( * pPComb   ) ;
	NewPath.ClearAll() ;
	NewPath.AddData( 0.0 , pPComb,TRUE ) ;
	AllPath.ClearAllPath() ;
}

void CSmartFUWLineGen::TransfSmtCutPath ( CSmtCutPath *pPath )
{
	CSmtCutPoint *pHead = pPath->m_pHead, *pNext = NULL ;
	CSmtCutPointEx *pInsert = NULL, *pTmp = NULL ;
	while( pHead )
	{
		pNext = pHead->next ;
		if( !pHead->IsPointEx () )
		{
			pPath->RemovePoint ( pHead ) ;
			pInsert = new CSmtCutPointEx() ;
			mathFCpyPnt( pHead->m_fPoint, pInsert->m_fPoint ) ;
			
			if( pNext && pNext->IsPointEx () )
			{
				pTmp = ( CSmtCutPointEx*)pNext ;
				mathFCpyPnt( pTmp->m_fSurfNor, pInsert->m_fSurfNor ) ;
				pPath->InsertBefore ( pInsert, pNext ) ;
			}
			else if( pHead->prev && pHead->prev->IsPointEx () )
			{
				pTmp = ( CSmtCutPointEx *)pHead->prev ;
				mathFCpyPnt( pTmp->m_fSurfNor, pInsert->m_fSurfNor ) ;
				pPath->InsertAfter ( pInsert, pHead->prev ) ;
			}
			delete pHead ;
		}
		pHead = pNext ;
	}
}

// 清除点数为1的路径
void CSmartFUWLineGen::DelOnePntPath(CSmtCPathLib &AllPath)
{
	double dLineTol = 0.0002 ;
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_NOMOSAIC ) 
		dLineTol = 1.0e-6 ;
	POSITION pos = AllPath.m_cAllPath.GetHeadPosition (), atpos ;
	CSmtCutPath *pPath = NULL;
	while( pos )
	{
		atpos = pos ;
		pPath = AllPath.m_cAllPath.GetNext ( pos ) ;
		pPath->DelPointOnLine( dLineTol );	// 删除同直线的点
		pPath->DefineBox () ;
		if( pPath->m_nNumPnt < 2 )
		{
			delete pPath ;
			AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		}
	}
}

// 移动一层路径
int CSmartFUWLineGen::MoveOneLayerPath(CSmtCPathLib &AllPath, CSmtCPathLib &LayerPath)
{
	if (AllPath.m_cAllPath.GetCount() <= 0)
		return -1;
	CSmtCutPath *pPath = AllPath.m_cAllPath.GetHead();
	int nLayerNo = pPath->m_nLayerNo;
	while (AllPath.m_cAllPath.GetCount() > 0)
	{
		pPath = AllPath.m_cAllPath.GetHead();
        if (pPath->m_nLayerNo != nLayerNo)
			break;
        LayerPath.AddToTail(pPath);
		AllPath.m_cAllPath.RemoveHead();
	}
	return nLayerNo;
}

// 把同一层号和行号的路径移动到目标路径组
void CSmartFUWLineGen::MoveSameLinePath(CSmtCPathList &PathFrom, CSmtCPathList &PathTo, POSITION pos)
{
	// 参数合理性判断
	if (PathFrom.IsEmpty() || NULL == pos)
		return;
	POSITION atpos = pos;
	CSmtCutPath *pPath = PathFrom.GetNext(pos);
	if (pPath == NULL)
		return;
	PathTo.AddTail(pPath);
	PathFrom.RemoveAt(atpos);
	int nLayerNo = pPath->m_nLayerNo, nLineNo = pPath->m_nLineNo;
	while(pos)
	{
		atpos = pos;
		pPath = PathFrom.GetNext(pos);
		if (pPath->m_nLayerNo == nLayerNo && pPath->m_nLineNo == nLineNo)
		{
			PathTo.AddTail(pPath);
			PathFrom.RemoveAt(atpos);
		}
		else
			break;
	}
}

// 找到与已知路径最接近的路径位置
POSITION CSmartFUWLineGen::GetClosestPathPos(const CSmtCPathList &PathFrom, CSmtCutPath *pPath, POSITION pos, BOOL bForward/* = TRUE*/, BOOL bCombSurfPath)
{
	if (NULL == pPath)
		return NULL;
	int nLayerNo = pPath->m_nLayerNo, nLineNo = pPath->m_nLineNo + 1;
	if (bForward == FALSE)
		nLineNo = pPath->m_nLineNo - 1;
	DOUBLE fMostClose = 0.f;
	POSITION fitpos = NULL;
	CSmtCutPath *pNext = NULL, *pFind = NULL;
	while (pos)
	{
		POSITION atpos = pos;
		if (bForward)
		{
			pNext = PathFrom.GetNext(pos);
			if (pNext->GetCutMode() != MINI_MILL_PATH)
			{
				continue;
			}
			if (pNext->m_nLayerNo != nLayerNo)
				continue;
			if (pNext->m_nLineNo > nLineNo)
				break;
			if (pNext->m_nLineNo < nLineNo)
				continue;
		}
		else
		{
			pNext = PathFrom.GetPrev(pos);
			if (pNext->GetCutMode() != MINI_MILL_PATH)
			{
				continue;
			}
			if (pNext->m_nLayerNo != nLayerNo)
				continue;
			if (pNext->m_nLineNo < nLineNo)
				break;
			if (pNext->m_nLineNo > nLineNo)
				continue;
		}
		pPath->DefineBox();
		pNext->DefineBox();
		// 两路径不在同一区域，不符合要求
		if (!PathInSameArea(pPath, pNext, bCombSurfPath))
			continue;
		DOUBLE fClose = TwoPathClose(pPath, pNext);
		// 两路径不接近，不符合要求
		if (fabs(fClose) < MIN_FLT)
			continue;
		// 第一次循环，直接替换
		if (NULL == fitpos)
		{
			fMostClose = fClose;					
			fitpos = atpos;
			pFind = pNext;
			continue;
		}
		// 上一符合的路径接近程度大于此路径2倍，不符合要求
		if (fMostClose > fClose * 2.f)
			continue;
		if (fMostClose < fClose * 0.5f)
		{// 上一符合的路径接近程度小于此路径一半，符合要求，替换之
			fMostClose = fClose;					
			fitpos = atpos;
			pFind = pNext;
			continue;
		}			
		// 检查两路径头与目标路径尾的距离，取较短的
		if (mathFDist(pPath->m_pTail->m_fPoint, pNext->m_pHead->m_fPoint) < 
			mathFDist(pPath->m_pTail->m_fPoint, pFind->m_pHead->m_fPoint))
		{
			fMostClose = fClose;
			fitpos = atpos;
		}
	}
	return fitpos;
}

// 定义两参数所构成的范围，bEx为真时，该范围被分为两部分，从0到较小值，从较大值到1，
// 为了给它变成一个范围，把较小值加1，范围变成较大值到较小值加1
void CSmartFUWLineGen::DefineRange(TFLOAT f1, TFLOAT f2, BOOL bEx, TFLOAT &fMin, TFLOAT &fMax)
{
	if (bEx == FALSE)
	{
		if (f1 > f2)
		{
			fMin = f2;
			fMax = f1;
		}
		else
		{
			fMin = f1;
			fMax = f2;
		}
	}
	else
	{
		if (f1 > f2)
		{
			fMin = f1;
			fMax = f2 + 1.f;
		}
		else
		{
			fMin = f2;
			fMax = f1 + 1.f;
		}
	}
}

// 定义路径的UW参数包围盒
void CSmartFUWLineGen::DefineUWBox(CSmtCutPath *pPath, FPNT2D UWBox[2], BOOL &bEx)
{
	// 参数合理性判断
	if (NULL == pPath)
		return;
	CSmtCutPointEx *pHead = (CSmtCutPointEx *)pPath->m_pHead, *pTail = (CSmtCutPointEx *)pPath->m_pTail;
	int nUWDir = 0;	// 参数方向，1为U向，-1为W向，0无意义
	if (fabs(pHead->m_fSurfPos[0] - pTail->m_fSurfPos[0]) < MIN_FLT)
		nUWDir = -1;
	if (fabs(pHead->m_fSurfPos[1] - pTail->m_fSurfPos[1]) < MIN_FLT)
		nUWDir = 1;
	if (nUWDir == 1)
	{
		if (pPath->m_nNumPnt > 2)
		{
			CSmtCutPointEx *pNext = (CSmtCutPointEx *)pHead->next;
			if ((pNext->m_fSurfPos[0] - pHead->m_fSurfPos[0]) * 
				(pNext->m_fSurfPos[0] - pTail->m_fSurfPos[0]) < 0)
				bEx = FALSE;
			else
				bEx = TRUE;			
		}
		else
			bEx = FALSE;
		DefineRange(pHead->m_fSurfPos[0], pTail->m_fSurfPos[0], bEx, UWBox[0][0], UWBox[1][0]);
		if (bEx == FALSE)
			UWBox[0][1] = UWBox[1][1] = pHead->m_fSurfPos[1];
		else
			UWBox[0][1] = UWBox[1][1] = pHead->m_fSurfPos[1] + 1.f;
	}
	else if (nUWDir == -1)
	{
		if (pPath->m_nNumPnt > 2)
		{
			CSmtCutPointEx *pNext = (CSmtCutPointEx *)pHead->next;
			if ((pNext->m_fSurfPos[1] - pHead->m_fSurfPos[1]) * 
				(pNext->m_fSurfPos[1] - pTail->m_fSurfPos[1]) < 0)
				bEx = FALSE;
			else
				bEx = TRUE;			
		}
		else
			bEx = FALSE;
		DefineRange(pHead->m_fSurfPos[1], pTail->m_fSurfPos[1], bEx, UWBox[0][1], UWBox[1][1]); 
		if (bEx == FALSE)
			UWBox[0][0] = UWBox[1][0] = pHead->m_fSurfPos[0];
		else
			UWBox[0][0] = UWBox[1][0] = pHead->m_fSurfPos[0] + 1.f;
	}
	else
	{
		if (pPath->m_nNumPnt > 2)
		{
			CSmtCutPointEx *pNext = (CSmtCutPointEx *)pHead->next;
			if ((pNext->m_fSurfPos[0] - pHead->m_fSurfPos[0]) * 
				(pNext->m_fSurfPos[0] - pTail->m_fSurfPos[0]) < 0)
				bEx = FALSE;
			else
				bEx = TRUE;			
		}
		else
			bEx = FALSE;
		DefineRange(pHead->m_fSurfPos[0], pTail->m_fSurfPos[0], bEx, UWBox[0][0], UWBox[1][0]); 
		DefineRange(pHead->m_fSurfPos[1], pTail->m_fSurfPos[1], bEx, UWBox[0][1], UWBox[1][1]); 
	}
}

// 两路径UW参数上的公共区域
TFLOAT CSmartFUWLineGen::TwoPathUWComArea(CSmtCutPath *pPath1, CSmtCutPath *pPath2)
{
	// 参数合理性判断
	if (NULL == pPath1 || NULL == pPath2)
		return 0.;
	TFLOAT fRet = 0.f;
	FPNT2D UWBox1[2], UWBox2[2];
	BOOL bEx1, bEx2;
	DefineUWBox(pPath1, UWBox1, bEx1);
	DefineUWBox(pPath2, UWBox2, bEx2);
	TFLOAT fMin, fMax, fDu, fDw;
	fMin = (UWBox1[0][0] > UWBox2[0][0]) ? (UWBox1[0][0]) : (UWBox2[0][0]);
	fMax = (UWBox1[1][0] < UWBox2[1][0]) ? (UWBox1[1][0]) : (UWBox2[1][0]);
	fDu = (fMin > fMax) ? (0.f) : (fMax - fMin);
	fMin = (UWBox1[0][1] > UWBox2[0][1]) ? (UWBox1[0][1]) : (UWBox2[0][1]);
	fMax = (UWBox1[1][1] < UWBox2[1][1]) ? (UWBox1[1][1]) : (UWBox2[1][1]);
	fDw = (fMin > fMax) ? (0.f) : (fMax - fMin);
	fRet += fDu + fDw;
	if (bEx1 != bEx2)	
	{
		if (bEx1)
		{
			UWBox2[0][0] += 1.f;UWBox2[0][1] += 1.f;
			UWBox2[1][0] += 1.f;UWBox2[1][1] += 1.f;
		}
		else
		{
			UWBox1[0][0] += 1.f;UWBox1[0][1] += 1.f;
			UWBox1[1][0] += 1.f;UWBox1[1][1] += 1.f;
		}
		fMin = (UWBox1[0][0] > UWBox2[0][0]) ? (UWBox1[0][0]) : (UWBox2[0][0]);
		fMax = (UWBox1[1][0] < UWBox2[1][0]) ? (UWBox1[1][0]) : (UWBox2[1][0]);
		fDu = (fMin > fMax) ? (0.f) : (fMax - fMin);
		fMin = (UWBox1[0][1] > UWBox2[0][1]) ? (UWBox1[0][1]) : (UWBox2[0][1]);
		fMax = (UWBox1[1][1] < UWBox2[1][1]) ? (UWBox1[1][1]) : (UWBox2[1][1]);
		fDw = (fMin > fMax) ? (0.f) : (fMax - fMin);
		fRet += fDu + fDw;
	}
	return fRet;
}

// 判断路径是否在同一区域内
BOOL CSmartFUWLineGen::PathInSameArea(CSmtCutPath *pPath1, CSmtCutPath *pPath2, BOOL bCombSurfPath)
{
	if (NULL == pPath1 || NULL == pPath2)
		return FALSE;

	// 如果两路径点uw参数无公共范围，则不符合要求
	if (!bCombSurfPath)
	{
		if (TwoPathUWComArea(pPath1, pPath2) < NC_ZERO)
			return FALSE;
	}
	
	DOUBLE dDist = min(3.0 * m_pTool->m_fRadius, 2.0 * m_cFeedDef.m_cStepDef.m_dOverStep);
	if (TwoPathMinDis(pPath1, pPath2) > dDist + MIN_DIS)
		return FALSE;
	DOUBLE fClose = TwoPathClose(pPath1, pPath2);
	// 两路径不接近，不符合要求
	if (fClose < MIN_FLT)
		return FALSE;
	return TRUE;
}

// 求两条路径的接近程度，返回值越大，越接近
TFLOAT CSmartFUWLineGen::TwoPathClose(CSmtCutPath *pPath1, CSmtCutPath *pPath2)
{
	if (NULL == pPath1 || NULL == pPath2)
		return 0.;
	FPNT3D &Box1min = pPath1->m_fBox[0], &Box1max = pPath1->m_fBox[1];
	FPNT3D &Box2min = pPath2->m_fBox[0], &Box2max = pPath2->m_fBox[1];
	TFLOAT fMin, fMax, fDx, fDy, fDz;
	fMin = (Box1min[0] > Box2min[0]) ? (Box1min[0]) : (Box2min[0]);
	fMax = (Box1max[0] < Box2max[0]) ? (Box1max[0]) : (Box2max[0]);
	fDx = (fMin > fMax) ? (0.f) : (fMax - fMin);
	fMin = (Box1min[1] > Box2min[1]) ? (Box1min[1]) : (Box2min[1]);
	fMax = (Box1max[1] < Box2max[1]) ? (Box1max[1]) : (Box2max[1]);
	fDy = (fMin > fMax) ? (0.f) : (fMax - fMin);
	fMin = (Box1min[2] > Box2min[2]) ? (Box1min[2]) : (Box2min[2]);
	fMax = (Box1max[2] < Box2max[2]) ? (Box1max[2]) : (Box2max[2]);
	fDz = (fMin > fMax) ? (0.f) : (fMax - fMin);
	return fDx + fDy +fDz;
}

// 得到与已知路径距离最短的路径位置
POSITION CSmartFUWLineGen::GetMinDistPathPos(const CSmtCPathList &PathFrom, CSmtCutPath *pPath)
{
	if (PathFrom.IsEmpty() || NULL == pPath)
		return NULL;
	double dMinDist = 1.0e10;
	POSITION pos = PathFrom.GetHeadPosition(), fitpos = NULL;
	CSmtCutPath *pNext = PathFrom.GetHead();
	int nLayerNo = pNext->m_nLayerNo, nLineNo = pNext->m_nLineNo;
	while (pos)
	{
		POSITION atpos = pos;
		pNext = PathFrom.GetNext(pos);
		if (pNext->GetCutMode() != MINI_MILL_PATH)
		{
			continue;
		}
		if (pNext->m_nLayerNo != nLayerNo)
			continue;
		if (pNext->m_nLineNo > nLineNo)
			break;
		if (pNext->m_nLineNo < nLineNo)
			continue;
		double dDist = TwoPathMinDis(pPath, pNext);
		if (dDist < dMinDist)
		{
			dMinDist = dDist;					
			fitpos = atpos;
		}			
	}
	return fitpos;
}

// 两路径之间的最短距离
DOUBLE CSmartFUWLineGen::TwoPathMinDis(CSmtCutPath *pPath1, CSmtCutPath *pPath2)
{
	if (NULL == pPath1 || NULL == pPath2)
		return 0.;	
	CSmtCutPath *pPathLong, *pPathShort;
	if (pPath1->GetLength() > pPath2->GetLength())
	{
		pPathLong = pPath1;
		pPathShort = pPath2;
	}
	else
	{
		pPathLong = pPath2;
		pPathShort = pPath1;
	}

	FPNT3D PathPt[5];
	TFLOAT fMinDis = 0.f;
	int nSize = SelectPathPoints(pPathShort, PathPt);
	for (int i=0; i<nSize; i++)
	{
		FPNT3D SnapAt;
		TFLOAT fDist, t;
		pPathLong->SnapPointOnPath(PathPt[i], SnapAt, fDist, t);
		fMinDis += fDist;
	}
	fMinDis /= nSize;
		
	return (DOUBLE)fMinDis;
}

// 从路径中均匀取nSize(nSize<=5)个点，返回个数
int CSmartFUWLineGen::SelectPathPoints(CSmtCutPath *pPath, FPNT3D SelPts[])
{
	if (NULL == pPath)
		return 0;
	int n = pPath->m_nNumPnt, nSize;
	if (n <= 5)
	{
		nSize = n;
		int nPos = 0;
		for( CSmtCutPoint *pHead = pPath->m_pHead; pHead ; pHead = pHead->next )
		{
			nc_VectorCopy(SelPts[nPos], pHead->m_fPoint, 3);
			nPos++;
		}
	}
	else
	{
		nSize = 5;
		int arrPos[5] = { 1, (n + 3) / 4, (n + 1) / 2, (3 * n + 1) / 4, n }, nCount = 0, nPos = 0;
		for( CSmtCutPoint *pHead = pPath->m_pHead; pHead ; pHead = pHead->next )
		{
			nCount++;
			if (nCount == arrPos[nPos])
			{
				nc_VectorCopy(SelPts[nPos], pHead->m_fPoint, 3);
				nPos++;
			}
		}
	}
	return nSize;
}

// 得到前一组路径
void CSmartFUWLineGen::GetPrevLinePaths(CSmtCPathList &PrevPaths, const CSmtCPathList &PathList, POSITION pos)
{
	// 参数合理性判断
	if (PathList.IsEmpty() || NULL == pos)
		return;
	CSmtCutPath *pPath = PathList.GetPrev(pos);
	if (pPath == NULL)
		return;
	int nLayerNo = pPath->m_nLayerNo, nLineNo = pPath->m_nLineNo - 1;
	while(pos)
	{
		pPath = PathList.GetPrev(pos);
		if (pPath->m_nLayerNo != nLayerNo)
			continue;
		if (pPath->m_nLineNo < nLineNo)
			break;
		if (pPath->m_nLineNo > nLineNo)
			continue;
		PrevPaths.AddTail(pPath);
	}
}

// 路径组中存在与输入路径相同区域的路径
CSmtCutPath *CSmartFUWLineGen::GetSameRegionPath(const CSmtCPathList &PathList,  CSmtCutPath *pPath, BOOL bCombSurfPath)
{
	// 参数合理性判断
	if (NULL == pPath || PathList.GetCount() == 0)
		return NULL;	
	POSITION pos = PathList.GetHeadPosition();
	CSmtCutPath *pTmpPath = NULL;
	while(pos)
	{
		pTmpPath = PathList.GetNext(pos);
        if (PathInSameArea(pTmpPath, pPath, bCombSurfPath))
			return pTmpPath;
	}
	return NULL;
}

// 沿相反方向，从输入路径开始，把同区域的路径移到目标路径组中
POSITION CSmartFUWLineGen::RegionPathBackward(CSmtCPathList &PathList, CSmtCutPath *pPathBegin, BOOL bCombSurfPath)
{
	// 参数合理性判断
	if (NULL == pPathBegin)
		return NULL;
	POSITION pos = PathList.Find(pPathBegin);
	if (NULL == pos)
		return NULL;
	POSITION atpos = NULL;
	CSmtCutPath *pPath = NULL;
	while(pos)
	{
		pPath = PathList.GetAt(pos);
		atpos = pos;
		// 找到与路径pPath最接近的上一路径位置
		pos = GetClosestPathPos(PathList, pPath, PathList.GetTailPosition(), FALSE, bCombSurfPath);
		// 如果得到的位置为空，返回
		if (NULL == pos)
			return atpos;
	}
	return NULL;
}

// 根据区域对路径排序
void CSmartFUWLineGen::SortPathByRegion(CSmtCPathList &PathList, BOOL bCombSurfPath)
{
	// 参数合理性判断
	if (PathList.IsEmpty())
		return;
	CSmtCPathList TmpList;
	// 增加头路径
	CSmtCutPath *pPath = PathList.RemoveHead();
	TmpList.AddTail(pPath);
	while(!PathList.IsEmpty())
	{
		pPath = TmpList.GetTail();
		// 找到与路径pPath最接近的路径位置
		POSITION fitpos = GetClosestPathPos(PathList, pPath, PathList.GetHeadPosition(),TRUE, bCombSurfPath);		
		// 如果得到的位置为空，则再根据距离最短的原则重新遍历
		if (NULL == fitpos)
		{
			fitpos = GetMinDistPathPos(PathList, pPath);
			if (NULL == fitpos)
			{
				TmpList.AddTail(&PathList);
				return;
			}
			TmpList.AddTail(PathList.GetAt(fitpos));
			PathList.RemoveAt(fitpos);
			continue;
		}
		
		// 得到前一组路径
		CSmtCutPath *pPathFit = PathList.GetAt(fitpos);
		CSmtCPathList PrevPaths;
		GetPrevLinePaths(PrevPaths, PathList, fitpos);		
		CSmtCutPath *pSameRegionPath = GetSameRegionPath(PrevPaths, pPathFit, bCombSurfPath);
		if (pSameRegionPath)
		{// 如果前一组路径有与此路径在同一区域的，则从该路径向前遍历，找到最前面的同一区域路径
			POSITION backpos = RegionPathBackward(PathList,  pSameRegionPath, bCombSurfPath);
			if (NULL == backpos)
			{
				TmpList.AddTail(&PathList);
				return;
			}
			TmpList.AddTail(PathList.GetAt(backpos));
			PathList.RemoveAt(backpos);
		}
		else
		{// 直接把该路径移到目标路径组
			TmpList.AddTail(pPathFit);
			PathList.RemoveAt(fitpos);
		}	
	}
	PathList.AddTail(&TmpList);
}

// 开启进退刀时，在进退刀路径上增加进退刀延长线方向的磨损补偿延长路径 qqs 2013.09.25
CSmtCutPath* CSmartFUWLineGen::CalRCompByLeadPath(CSmtCutPath*& pLead, BOOL bIsOut, BOOL bIs3D)
{
	CSmtCutPoint* ptStart = NULL;
	VEC3D dTan ;

	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;

	if (bIsOut) ptStart = pLead->m_pTail->CopyMyself();
	else ptStart = pLead->m_pHead->CopyMyself();

	if (bIsOut) mathFGetUnitVec( pLead->m_pTail->prev->m_fPoint, pLead->m_pTail->m_fPoint, dTan ) ;
	else        mathFGetUnitVec( pLead->m_pHead->next->m_fPoint, pLead->m_pHead->m_fPoint, dTan ) ;

    // 当路径起点法向为竖直时，无法生成2D补偿路径，生成3D补偿路径
	if (!bIs3D || !(abs(dTan[0]) < 0.01 && abs(dTan[1]) < 0.01))
	{
		dTan[2] = 0.;
	}	

	CSmtCutPath* RCompLeadPath = new CSmtCutPath(MINI_CONNECT_PATH);
	RCompLeadPath->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	CreateRCompPath(rcompLead, ptStart, NULL, RCompLeadPath, dTan, bIsOut);
	if (bIsOut)
	{
		// 将退刀路径标记出，用于最后添加关闭半径补偿标志
		RCompLeadPath->m_bFeedType = JDNC_FEEDTYPE_RCOMPCLOSE ; 
	}
	else
	{
		// 将进刀路径标记出，用于最后添加开启半径补偿标志
		RCompLeadPath->m_bFeedType = JDNC_FEEDTYPE_RCOMPOPEN ; 
	}
	return RCompLeadPath;
}

// 关闭进退刀，或进退刀未生成时，通过切削路径计算于加工曲面点法向垂直的磨损补偿延长路径 qqs 2013.09.25
CSmtCutPath* CSmartFUWLineGen::CalRCompByRoughPath(CSmtCutPath*& pRough, BOOL bIsOut, BOOL b3D, BOOL bIs5Ax)
{
	CSmtCutPoint* ptStart = NULL;
	VEC3D dTan = {0};

	JDNC_LEAD  rcompLead = m_cFeedDef.m_cLeadDef ;
	rcompLead.m_dLength = max( 0.05, m_cFeedDef.m_cLeadDef.m_dWearLine) ;

	if (bIsOut) ptStart = pRough->m_pTail->CopyMyself();
	else ptStart = pRough->m_pHead->CopyMyself();

	CSmtCutPointEx* tmp = (CSmtCutPointEx*) ptStart;

	FPNT3D surfNor;
	if (!bIs5Ax)
	{
		for (int i = 0; i < 3; i++)
		{
			surfNor[i] = tmp->m_fSurfNor[i];
		}
	}
	else
	{
		for (int i = 0; i < 3; i++)
		{
			surfNor[i] = tmp->m_fSurfPos[i];
		}
	}
	// 当路径起点法向为竖直时，无法生成2D补偿路径，生成3D补偿路径
	if (abs(surfNor[0]) < 1.0e-6 &&
		abs(surfNor[1]) < 1.0e-6)
	{
		b3D = TRUE;
	}
	int n = 2;
	if (b3D)
	{
		n = 3;
	}

	for (int i = 0; i < n; i++)
	{
		dTan[i] = surfNor[i]; 
	}		

	CSmtCutPath* RCompLeadPath = new CSmtCutPath(MINI_CONNECT_PATH);
	RCompLeadPath->m_bFeedType = JDNC_FEEDTYPE_LEAD ;
	CreateRCompPath(rcompLead, ptStart, NULL, RCompLeadPath, dTan, bIsOut);

 	if (bIsOut)
 	{
 		RCompLeadPath->m_bFeedType = JDNC_FEEDTYPE_RCOMPCLOSE  ;
 	}
 	else
 	{
 		RCompLeadPath->m_bFeedType = JDNC_FEEDTYPE_RCOMPOPEN  ;
 	}
	return RCompLeadPath;
}

BOOL CSmartFUWLineGen::CreateRCompPath( JDNC_LEAD&	   LeadDef ,
										CSmtCutPoint*  ptStart ,
										CSmartLoop*    Contour ,
										CSmtCutPath*&  RCompLeadPath,
										VEC3D          Normal,
										BOOL 		   IsOut )
{
	DOUBLE dLength  = LeadDef.m_dLength ;
	PNT2D sectSt, sectEnd;	

	CSmtCutPoint* ptEnd = new CSmtCutPoint;
	ptEnd->m_fPoint[0] = (float)(ptStart->m_fPoint[0] + Normal[0] * dLength );
	ptEnd->m_fPoint[1] = (float)(ptStart->m_fPoint[1] + Normal[1] * dLength );
	ptEnd->m_fPoint[2] = (float)(ptStart->m_fPoint[2] + Normal[2] * dLength );

	if( IsOut )
	{
 		sectSt[0] = ptStart->m_fPoint[0];
 		sectSt[1] = ptStart->m_fPoint[1];
 		sectEnd[0] = ptEnd->m_fPoint[0];
		sectEnd[1] = ptEnd->m_fPoint[1];
 		RCompLeadPath->AddTail(ptStart);
 		RCompLeadPath->AddTail(ptEnd);
	}
	else
	{
		sectSt[0] = ptEnd->m_fPoint[0];
		sectSt[1] = ptEnd->m_fPoint[1];
		sectEnd[0] = ptStart->m_fPoint[0];
		sectEnd[1] = ptStart->m_fPoint[1];

 		RCompLeadPath->AddTail(ptEnd);
 		RCompLeadPath->AddTail(ptStart);			
	}

	if( Contour )
	{
		PNT2D dSeedPnt[2] ;
		CSmartSect* pSect = new CSmartLine( sectSt , sectEnd ) ;
 		pSect->GetPoint( 0.5, dSeedPnt[0] ) ;
 
 		if( IsOut ) 	
 		{
 			pSect->GetPoint( 1.0, dSeedPnt[1] ) ;
 		}
 		else 
 		{
 			pSect->GetPoint( 0.0, dSeedPnt[1] ) ;
 		}
 		if( ! Contour->IsPtOnContour( dSeedPnt[0] ) || 
 			! Contour->IsPtOnContour( dSeedPnt[1] ) || 
 			Contour->IntSectContour( pSect )  ) 
 		{ 
 			delete pSect ;
 			delete RCompLeadPath;
			delete ptEnd;
 			pSect = NULL;
 			RCompLeadPath = NULL;
			ptEnd = NULL;
 			return FALSE  ;
 		} 
		delete pSect;
	}

	if (RCompLeadPath != NULL)
	{
		return TRUE;
	}
	return FALSE ; 
}
// 对每一层路径添加连刀
void CSmartFUWLineGen::AddLayerPathLeadPath(CSmtCheckMdl &DriveMdl, BOOL bCheck, CSmtCPathLib &LayerPath)
{
	POSITION pos, atpos ;
	CSmtCutPath *pPath = NULL, *pNext = NULL, *pLeadIn = NULL, *pLeadOut = NULL, *pConnect = NULL ;
	double dDist = 0, dDepth = 0. ;
	double dLen = m_cFeedDef.m_cConnectDef.m_dMaxDist ;
	FPNT3D start, end, fIntpt[100] ;
	int i = 0, nCnt = 0 ;
	BOOL bConnect = TRUE, bLead = TRUE, bLeadMidd = TRUE ;
	JDNC_TOL cTol = m_cSetupDef.m_cTolDef ;
	JDNC_CONNECT3D cConnect = m_cFeedDef.m_cConnectDef ;
	BOOL bGrndMdfy = (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY)? TRUE : FALSE;
	BOOL bRComp = Get3DRCompMask() ? TRUE:FALSE;
	if( m_cFeedDef.m_cConnectDef.m_nLead3DType == NCDEF_LEAD3D_CLOSE )
		bLead = FALSE ;
	if( m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_LEADEND )
		bLeadMidd = FALSE ;
	pos = LayerPath.m_cAllPath.GetHeadPosition () ;
	pPath = LayerPath.m_cAllPath.GetNext ( pos ) ;

	if (!pos)
	{
		if( bLead ) 
		{
			if (bGrndMdfy)
			{
				CalcCutPath2DLeadIn( DriveMdl, cTol, cConnect, pPath, pLeadIn, bCheck) ;
			}
			else
			{
				CalcCutPathLeadIn( DriveMdl, cTol, cConnect, pPath, pLeadIn, bCheck, surfncMethodSFinish ) ;
			}

			if( pLeadIn ) 
			{
				LayerPath.m_cAllPath.AddHead ( pLeadIn ) ;
				if (bRComp)
				{
					LayerPath.m_cAllPath.AddHead ( CalRCompByLeadPath(pLeadIn, FALSE) ) ;
				}						
			}
			else if(bRComp)
			{
				LayerPath.m_cAllPath.AddHead ( CalRCompByRoughPath(pPath, FALSE, FALSE)) ;
			}
		}
		else if(bRComp)
		{			
			LayerPath.m_cAllPath.AddHead ( CalRCompByRoughPath(pPath, FALSE, FALSE)) ;
		}

	}
	int m = 0;
	while( pos )
	{
		atpos = pos ;
		pNext = LayerPath.m_cAllPath.GetNext ( pos ) ;
		m_cPrgDef.m_dStepAt += m_cPrgDef.m_dIncStep ;
		while( m_cPrgDef.m_pPrgFunc && m_cPrgDef.m_dStepAt >= m_cPrgDef.m_dLimitAt )
		{
			m_cPrgDef.m_pPrgFunc(1) ;
			m_cPrgDef.m_dStepAt -= m_cPrgDef.m_dLimitAt  ;
		}

		//用于磨削补偿相邻路径之间的连刀 qqs 2013.05.24
		if (bGrndMdfy)
		{
			PNT3D pPathHead ,pPathTail ,pNextHead ,pNextTail;
			for (int j = 0; j < 3; j++)
			{
				pPathHead[j] = pPath->m_pHead->m_fPoint[j];
				pPathTail[j] = pPath->m_pTail->m_fPoint[j];
				pNextHead[j] = pNext->m_pHead->m_fPoint[j];
				pNextTail[j] = pNext->m_pTail->m_fPoint[j];
			}

			double disPhNh = mathDist(pPathHead, pNextHead);
			double disPhNt = mathDist(pPathHead, pNextTail);
			double disPtNh = mathDist(pPathTail, pNextHead);
			double disPtNt = mathDist(pPathTail, pNextTail);

			double minDis = disPtNh;
			if (minDis > disPhNh) minDis = disPhNh;
			if (minDis > disPhNt) minDis = disPhNt;
			if (minDis > disPtNt) minDis = disPtNt;

			if (minDis == disPhNh) pPath->ReverseDirect();
			if (minDis == disPtNt) pNext->ReverseDirect();
			if (minDis == disPhNt) pNext->ReverseDirect(), pPath->ReverseDirect();
		}		

		// 用于增加整个路径起始位置的进刀路径 qqs 2013.05.27
		if (m == 0)
		{
			if( bLead ) 
			{
				if (bGrndMdfy)
				{
					CalcCutPath2DLeadIn( DriveMdl, cTol, cConnect, pPath, pLeadIn, bCheck) ;
				}
				else
				{
					CalcCutPathLeadIn( DriveMdl, cTol, cConnect, pPath, pLeadIn, bCheck, surfncMethodSFinish ) ;
				}

				if( pLeadIn ) 
				{
					LayerPath.m_cAllPath.AddHead ( pLeadIn ) ;
					if (bRComp)
					{
						LayerPath.m_cAllPath.AddHead ( CalRCompByLeadPath(pLeadIn, FALSE) ) ;
					}		
				}
				else if(bRComp)
				{
					LayerPath.m_cAllPath.AddHead ( CalRCompByRoughPath(pPath, FALSE, FALSE) ) ;
				}
			}
			else if(bRComp)
			{
				LayerPath.m_cAllPath.AddHead ( CalRCompByRoughPath(pPath, FALSE, FALSE) ) ;
			}			
			m++;
		}

		dDist = mathFDist( pPath->m_pTail->m_fPoint, pNext->m_pHead->m_fPoint ) ;
		CSmtCutPath* pLeadOutRComp = NULL, *pLeadInRComp = NULL;
		if( dDist > 1.0e-5  )
		{
			pLeadIn = pLeadOut = NULL ;
			if( bLead && bLeadMidd ) 
			{
				if (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY)
				{
					CalcCutPath2DLeadOut( DriveMdl, cTol, cConnect, pPath, pLeadOut, bCheck) ;
					if( pLeadOut && bRComp)
					{
						pLeadOutRComp = CalRCompByLeadPath(pLeadOut, TRUE) ;
					}
					else if (bRComp)
					{
						pLeadOutRComp = CalRCompByRoughPath(pPath, TRUE, FALSE) ;
					}
				}
				else
				{
					CalcCutPathLeadOut( DriveMdl, cTol, cConnect, pPath, pLeadOut, bCheck, surfncMethodSFinish ) ;
				}
			}
			else if (bRComp)
			{
				pLeadOutRComp = CalRCompByRoughPath(pPath, TRUE, FALSE) ;
			}

			if( bLead && bLeadMidd ) 
			{
				if (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY)
				{
					CalcCutPath2DLeadIn( DriveMdl, cTol, cConnect, pNext, pLeadIn, bCheck) ;
					if( pLeadIn && bRComp)
					{
						pLeadInRComp = CalRCompByLeadPath(pLeadIn, FALSE) ;
					}
					else if (bRComp)
					{
						pLeadInRComp = CalRCompByRoughPath(pNext, FALSE, FALSE) ;
					}
				}
				else
				{
					CalcCutPathLeadIn ( DriveMdl, cTol, cConnect, pNext, pLeadIn , bCheck, surfncMethodSFinish ) ;
				}
			}
			else if (bRComp)
			{
				pLeadInRComp = CalRCompByRoughPath(pNext, FALSE, FALSE) ;
			}

			// 计算连刀路径起末点
			if (pLeadOutRComp)  mathFCpyPnt( pLeadOutRComp->m_pTail->m_fPoint, start ) ;
			else if(pLeadOut)   mathFCpyPnt( pLeadOut->m_pTail->m_fPoint, start ) ;
			else		        mathFCpyPnt( pPath->m_pTail->m_fPoint, start ) ;

			if (pLeadInRComp) mathFCpyPnt( pLeadInRComp->m_pHead->m_fPoint, end   ) ;
			else if(pLeadIn)  mathFCpyPnt( pLeadIn->m_pHead->m_fPoint, end   ) ;
			else		      mathFCpyPnt( pNext->m_pHead->m_fPoint, end ) ;

			pConnect = NULL ;
			if( dDist < dLen )
			{
				// 生成连刀线
				mathFCpyPnt( start, fIntpt[0] ) ;
				mathFCpyPnt( end  , fIntpt[1] ) ;
				dDepth = max( start[2], end[2] ) + 0.1 ;
				if( bCheck )
				{
					nCnt = DriveMdl.AdsorbConnectLine ( m_cSetupDef.m_cTolDef, fIntpt, 100, TRUE ) ;
				}
				else
				{
					nCnt = 2 ;
				}
				bConnect = TRUE ;
				for( i = 0 ; i < nCnt ; i++ )
				{
					if( fIntpt[i][2] > dDepth )
					{
						bConnect = FALSE ;
						break ;
					}
				}
				if( nCnt >= 2 && bConnect )
				{
					pConnect = new CSmtCutPath( MINI_CONNECT_PATH ) ;
					pConnect->m_bFeedType = JDNC_FEEDTYPE_CONNECT ;
					for( i = 0 ; i < nCnt ; i++ )
						pConnect->AddPoint ( fIntpt[i] ) ;
					pConnect->DefineBox () ;
				}
			}

			// 添加切入切出和连刀
			if( pLeadOut )
				LayerPath.m_cAllPath.InsertBefore ( atpos, pLeadOut ) ;
			if(pLeadOutRComp) 
				LayerPath.m_cAllPath.InsertBefore ( atpos, pLeadOutRComp ) ;
			if( pConnect )
				LayerPath.m_cAllPath.InsertBefore ( atpos, pConnect ) ;
			if(pLeadInRComp) 
				LayerPath.m_cAllPath.InsertBefore ( atpos, pLeadInRComp ) ;
			if( pLeadIn  ) 
				LayerPath.m_cAllPath.InsertBefore ( atpos, pLeadIn  ) ;
		}
		pPath = pNext ;
	}
	// 添加最后一根路径的切出路径
	if( pPath )
	{
		pLeadOut = NULL ;
		if( bLead ) 
		{
			if (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY)
			{
				CalcCutPath2DLeadOut( DriveMdl, cTol, cConnect, pPath, pLeadOut, bCheck) ;
			}
			else
			{
				CalcCutPathLeadOut( DriveMdl, cTol, cConnect, pPath, pLeadOut, bCheck, surfncMethodSFinish ) ;
			}
			if( pLeadOut ) 
			{
				LayerPath.m_cAllPath.AddTail ( pLeadOut ) ;

				if (bRComp)
				{
					LayerPath.m_cAllPath.AddTail ( CalRCompByLeadPath(pLeadOut, TRUE) ) ;
				}	
			}
			else if(bRComp)
			{
				LayerPath.m_cAllPath.AddTail ( CalRCompByRoughPath(pPath, TRUE, FALSE) ) ;
			}
		}
		else if (bRComp)
		{
			LayerPath.m_cAllPath.AddTail ( CalRCompByRoughPath(pPath, TRUE, FALSE) ) ;
		}		
	}
}

 int CSmartFUWLineGen::Get3DRCompMask ()
 {
	// 当未开启磨削调整时，半径磨损补偿标记为0；
	BOOL bGrndMdfy = (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY) ? TRUE : FALSE;
    if (!bGrndMdfy)
    {
		return 0;
	}

 	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_INC ) //正向磨损
 	{
 		return JDNC_FGCODE_RCOMPG42; 
 	}
 	else if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_WEARCOMP_DEC ) //反向磨损
 	{
 		return JDNC_FGCODE_RCOMPG41; 
 	}
 	else
 	{
 		return 0 ;
 	}
 }

 // 新边界构面生成路径，解决生成的路径混乱的问题 qqs 2013.11.15    
 BOOL CSmartFUWLineGen::CreateAllCombSurfPathNew( CSmartGraphic &Graph, CCombSurfArray &CombSurfArray, CSmtCheckMdl &DriveMdl,CSmtCheckMdl &DrMdl,
	                                              RFRAME *dNcMtx, CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg,double dCur[4])
 {	 
	 BOOL* arrbSpiral = new BOOL[CombSurfArray.GetCount()];
	 C5XGraph allGraph;
	 Graph.GetAll5XObject(m_cSetupDef, 0, &allGraph);
	 CNc5DAxisCtrl axisCtrl( m_c5DCtrlDef ) ; 
	 if( axisCtrl.UpdateAxisGraph( allGraph ) == FALSE )
	 {
		 m_nErrorType = JDERROR_GENPATH5AX_NOAXISCURVE ;
		 return FALSE ;
	 }

	 double dOffset1 = GetDriveSurfOffset() - m_cSetupDef.m_cTolDef.m_dArcTol * 2 ;
	 double dOffset2 = GetCheckSurfOffset() - m_cSetupDef.m_cTolDef.m_dArcTol * 2 ;
	 CSmartTool *pNewTool = CreateSmartTool( m_cToolDef, dOffset1, GetWedmPlanarGap() ) ;
	 if( pNewTool ) 
	 {
		 delete m_pTool ;
		 m_pTool = pNewTool ;
	 }
	 CSmartTool *pCheckTool = CreateSmartTool( m_cToolDef, dOffset2, GetWedmPlanarGap() ) ;
	 if( pCheckTool ) 
	 {
		 delete m_pSafeTool ;
		 m_pSafeTool = pCheckTool ;
	 }
 	 if( !TransTo5AxDriveAndCheckMdl( DriveMdl )  )
 	 {//转换成旋转模型
 		 m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
 		 return FALSE ;
 	 }
	
 	 BOOL bLocalCopy = FALSE;
 	 RFRAME ncMtx  ;
 	 mathInitRFrame( &ncMtx ) ;
 	 if( mathIsRFrameUnit(&DriveMdl.m_dNcMtx) == 0 )
 	 {
 		 ncMtx = DriveMdl.m_dNcMtx ;
 		 axisCtrl.m_dNcMtx = &ncMtx ;
 	 }
	
	 // 根据构造面的原始面，将DriveMdl进行重构，根据新的DriveMdl生成路径，
	 // 防止五轴投影路径投影到其他面上，造成路径混乱 qqs 2013.07.01
	 //////////////////////////////////////////////////////////////////////////
	 CSmtChkArray pAllCheck;  // 存储原始DriveMdl中的面片
	 CSmtChkArray pTempCheck; // 存储被removecheck掉的面片，用于最后释放内存

	 // 1.存储原始DriveMdl中的所有检查曲面
	 for (int j = 0; j < DriveMdl.m_cAllChk.GetCount(); j++)
	 {
		 pAllCheck.Add(DriveMdl.m_cAllChk.GetAt(j));
	 }
	 DriveMdl.m_cAllChk.RemoveAll();
	 BOOL bRet = FALSE;	
	 double eachPrg = (0.7 * dPrg) / CombSurfArray.GetSize();
	 for (int i=0; i < CombSurfArray.GetSize(); i++)
	 {
		 // 2.移除DriveMdl中所有检查曲面，并记录被移除的面片
		 for (int j = 0; j < DriveMdl.m_cAllChk.GetCount(); j++)
		 {
			 pTempCheck.Add(DriveMdl.m_cAllChk.GetAt(j));
		 }
		 while(DriveMdl.m_cAllChk.GetCount())
		 {
			 DriveMdl.RemoveCheck(DriveMdl.m_cAllChk.GetAt(0));
		 }

		 // 3.从存储的所有检查曲面中，找出与构造面的原始面对应的检查曲面，重新加入到DriveMdl中
		 for (int j = 0; j < pAllCheck.GetCount(); j++)
		 {
			 CSmartCheck* pCheck = pAllCheck.GetAt(j);
			 CGeoTrmSurf* pCheckSurf = (CGeoTrmSurf*) pCheck->m_pGeoSurf;
			 for (int m = 0; m < ((CombSurfArray.GetAt(i))->m_pSourceSurf).GetCount(); m++)
			 {
				 CGeoTrmSurf* pSurf = (CGeoTrmSurf*)((CombSurfArray.GetAt(i))->m_pSourceSurf).GetAt(m);
				 if (pSurf != pCheckSurf) continue;

				 //在构造面pTrimSurf对应的原始面中找到了pCheckSurf，将pCheck添加到DriveMdl中
				 DriveMdl.m_cAllChk.Add(pCheck);
 				 break;
			 }
		 }

		 // 4.因每次DriveMdl都会被改变，所以将多线程复制代码移至for循环内部，
		 // 每次DriveMdl改变时，都从新更新其他线程的DriveMdl
		 if( bLocalCopy == TRUE ) 
		 {
			 DriveMdl.DeleteMultiCopy() ;
			 bLocalCopy = FALSE;
		 }
		 if (IsNeedMultiCopy(DriveMdl))
		 {
			 bLocalCopy = TRUE ;
			 DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
		 }

		 // 5.生成构造曲面的流线路径
		 if (CreateCombineSurfPathNew(axisCtrl, (CGeoTrmSurf*)(CombSurfArray.GetAt(i)->m_pCombSurf), DriveMdl, dNcMtx, AllLoop, AllPath, eachPrg, i, arrbSpiral))
			 bRet = TRUE;
		
		 if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() ) break;
	 }

	 // 6.移除并记录最后一次需移除的面片
	 for (int j = 0; j < DriveMdl.m_cAllChk.GetCount(); j++)
	 {
		 pTempCheck.Add(DriveMdl.m_cAllChk.GetAt(j));
	 }
	 while(DriveMdl.m_cAllChk.GetCount())
	 {
		 DriveMdl.RemoveCheck(DriveMdl.m_cAllChk.GetAt(0));
	 }

	 // 7.将原始面片存入到DriveMdl中，用于DriveMdl正确释放内存
	 for (int m = 0; m < pTempCheck.GetCount(); m++)
	 {
		 CSmartCheck* pCheck = pTempCheck.GetAt(m);
		 DriveMdl.AddCheck(pCheck);
	 }
	 pTempCheck.RemoveAll();

	 // 8.释放构造面链表中的内存
	 for (int i=0; i < CombSurfArray.GetSize(); i++)
	 {
		 CCombSurf* pSurf = CombSurfArray.GetAt(i);
		 pSurf->ClearAllSurf();
		 delete pSurf;
		 pSurf = NULL;
	 }	

	 // 9.处理用户中断情况
	 if( m_cPrgDef.m_pBrkFunc && m_cPrgDef.m_pBrkFunc() )
	 {
		 if( bLocalCopy == TRUE ) 
			 DriveMdl.DeleteMultiCopy() ;
		 return FALSE;
	 }
	 if( bLocalCopy == TRUE ) 
		 DriveMdl.DeleteMultiCopy() ;
	 //////////////////////////////////////////////////////////////////////////

	 if (AllPath.m_cAllPath.IsEmpty())
	 {
		 m_nErrorType = JDERROR_GENPATH_NOTGENPATH;
		 return FALSE;
	 }
	 AllPath.SetCutMode(MINI_MILL_PATH);
	 CSmtCheckMdl ChMdl;
	 if( !BuildDriveAndCheckMdl(Graph,DrMdl,ChMdl))
	 {
		 return FALSE ; 
	 } 

	 CFlowlineGen	cNcGen( m_cSetupDef, m_cFeedDef, m_cStockDef, m_cToolDef, m_cSpeedDef, m_pTool, m_nCalcThreadNum, m_cParam) ;
	 JDNC_TOL Tol = GetSurfaceTol() ;	 
	 BOOL bLeadPathCheck = FALSE;
	 if (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_CHECK)
	 {
		 ProcessNewFunc( m_cPrgDef, IDS_PATHGEN_CHECK_PATH ) ;
		 if( !cNcGen.CheckAllPathByCheckMdl( DrMdl, AllPath, Tol, TRUE, TRUE,
			 m_cPrgDef, 100, &m_nErrorType) )
		 {
			 delete[] arrbSpiral;
			 return FALSE ;
		 }
		 bLeadPathCheck = TRUE;
	 }	 

	 AllPath.DeleteShortPath( m_cFeedDef.m_cConnectDef.m_dDelShort ) ;
	 AddAllPathLeadPath(DrMdl,AllPath,bLeadPathCheck,dCur[1],arrbSpiral,TRUE);
	 delete[] arrbSpiral;

	 return bRet;
 }

BOOL CSmartFUWLineGen::CreateAllCombSurfPath(CSmartGraphic &Graph, C3DSurfArray &CombSurfs, CSmtCheckMdl &DriveMdl,
											 RFRAME *dNcMtx, CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg)
{
	C5XGraph allGraph;
	Graph.GetAll5XObject(m_cSetupDef, 0, &allGraph);
	CNc5DAxisCtrl axisCtrl( m_c5DCtrlDef ) ;
	if( axisCtrl.UpdateAxisGraph( allGraph ) == FALSE )
	{
		m_nErrorType = JDERROR_GENPATH5AX_NOAXISCURVE ;
		return FALSE ;
	}

	double dOffset1 = GetDriveSurfOffset() - m_cSetupDef.m_cTolDef.m_dArcTol * 2 ;
	double dOffset2 = GetCheckSurfOffset() - m_cSetupDef.m_cTolDef.m_dArcTol * 2 ;
	CSmartTool *pNewTool = CreateSmartTool( m_cToolDef, dOffset1, GetWedmPlanarGap() ) ;
	if( pNewTool ) 
	{
		delete m_pTool ;
		m_pTool = pNewTool ;
	}
	CSmartTool *pCheckTool = CreateSmartTool( m_cToolDef, dOffset2, GetWedmPlanarGap() ) ;
	if( pCheckTool ) 
	{
		delete m_pSafeTool ;
		m_pSafeTool = pCheckTool ;
	}
	if( !TransTo5AxDriveAndCheckMdl( DriveMdl )  )
	{//转换成旋转模型
		m_nErrorType = JDERROR_GENPATH_NOSURFACE ;
		return FALSE ;
	}

	BOOL bLocalCopy = FALSE;
	// 需要多线程的拷贝，则拷贝一份DriveMdl
	if (IsNeedMultiCopy(DriveMdl))
	{
		bLocalCopy = TRUE ;
		DriveMdl.CreateMultiCopy(m_nCalcThreadNum - 1) ;
	}

	RFRAME ncMtx  ;
	mathInitRFrame( &ncMtx ) ;
	if( mathIsRFrameUnit(&DriveMdl.m_dNcMtx) == 0 )
	{
		ncMtx = DriveMdl.m_dNcMtx ;
		axisCtrl.m_dNcMtx = &ncMtx ;
	}

	BOOL bRet = FALSE;
	dPrg = dPrg / CombSurfs.GetSize();
	for (int i=0; i<CombSurfs.GetSize(); i++)
	{// 边界构面法生成曲面流线
		CGeoTrmSurf *pTrimSurf = (CGeoTrmSurf *)CombSurfs.GetAt(i);
		if (CreateCombineSurfPath(axisCtrl, pTrimSurf, DriveMdl,dNcMtx, AllLoop, AllPath, dPrg))
			bRet = TRUE;
		delete pTrimSurf;
	}

	if( bLocalCopy == TRUE ) 
		DriveMdl.DeleteMultiCopy() ;

	if (AllPath.m_cAllPath.IsEmpty())
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH;
		return FALSE;
	}
	AllPath.SetCutMode(MINI_MILL_PATH);

	return bRet;
}

// 边界构面生成路径
BOOL CSmartFUWLineGen::CreateCombineSurfPath(CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf *TrimSurf, CSmtCheckMdl& DriveMdl, RFRAME *dNcMtx,
											 CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg)
{
	// 调整合并后的曲面，如果法向不符合要求，反向
	ModifyCombineSurf(dNcMtx, TrimSurf);

	// STEP 1 : 计算参数域上的路径
	m_cPrgDef.m_dTotalMove = dPrg * 0.7;
	if( TrimSurf == NULL || TrimSurf->m_pSurface == NULL)
	{
		return FALSE ;
	}
	double uwStep[2] ;
	int uwCnt[2], nMoveDir = m_cParam.m_nCutDir;
	TrimSurf->GetDiscreteStep( 0.002, uwStep[0], uwStep[1], uwCnt[0], uwCnt[1] ) ;

	// 判断曲线是否闭合,如果闭合并且方向相同,则沿曲面加工
	BOOL bZigZag = FALSE;
	if( m_cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ) bZigZag = TRUE ;
	BOOL bSpiral = FALSE;
	BOOL bGrndMdfy = m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY ? TRUE : FALSE;
	BOOL bFlag = m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ? TRUE : FALSE;
	CFlowlineGen	cNcGen( m_cSetupDef, m_cFeedDef, m_cStockDef, m_cToolDef, m_cSpeedDef, m_pTool, m_nCalcThreadNum, m_cParam) ;
	
	// 判断曲面是否闭合
	if (!bGrndMdfy && bFlag)
	{
		CSmartPathGen  ncPathGen ;
		JDNC_TOL		cCurveTol = { 0.002, 10, 0, 0, 0, 0 } ;
		CSmartLoop* pLoop = ncPathGen.ExtractSurfLoop(TrimSurf, cCurveTol);
		if( !pLoop ) return 0 ;
		cNcGen.CalcSurfUWCut( TrimSurf, pLoop, m_cParam, uwStep, nMoveDir, bZigZag, bSpiral ) ;
		Mini_DeleteContours(pLoop);
	}
	
	CSmartCurve allSect ;
	CreateParamLine( TrimSurf, uwStep, allSect, nMoveDir, *dNcMtx, bSpiral) ;
	
	if( allSect.m_pHead == NULL ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	// STEP 2 : 将参数域上的路径影射到加工模型上
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_TOUCHMODE )
	{
		DriveMdl.SetTouchCalcMode( TRUE ) ;
	}
	// 投影方式设为曲面法向
	CSmtCPathLib TmpPath;
	CIsoSGuideTPath uwTPath( 5., NCDEF_PROJECTPDIR_NORMAL);
	uwTPath.m_bMaxStepMode = TRUE ;
	uwTPath.m_cPrgDef = m_cPrgDef ;
	uwTPath.m_pAxisCtrl = & axisCtrl ;
	uwTPath.m_dMaxDepth = 10 ;	// 投影深度默认为10
	uwTPath.SetupIsoGen( DriveMdl, TrimSurf->m_pSurface, 
		uwStep, nMoveDir, m_cSetupDef.m_cTolDef) ;
	if( ! uwTPath.GenSGuideTPath( DriveMdl, allSect, TmpPath ) )
	{
		m_nErrorType  = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	DriveMdl.SetTouchCalcMode( FALSE ) ;
	if(TmpPath.m_cAllPath.GetCount() <= 0)
	{
		m_nErrorType  = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE;
	}
	POSITION atTPos = TmpPath.m_cAllPath.GetHeadPosition() ;
	while( atTPos )
	{
		CSmtCutPath* pTBase = TmpPath.m_cAllPath.GetNext(atTPos) ;
		pTBase->SetMillDir( GetMillDir() ) ;
		pTBase->DefineBox();
	}
	
	// 将距离过近的相邻路径的起末点坐标重设为同一个点 qqs 2013.06.19
	ResetSameCutPnt(TmpPath, bSpiral);

	TrimPath(TmpPath, AllLoop);
	

	// STEP 3 : 路径连接
	m_cPrgDef.m_dTotalMove = dPrg * 0.3 ;
	JDNC_CONNECT3D tmpCnt3D = m_cFeedDef.m_cConnectDef ;
	if( bZigZag && !bGrndMdfy) //磨削调整时，往复走刀关闭
	{
		tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
	}
	else
	{
		tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
	}
	tmpCnt3D.m_dSafeDist = m_cFeedDef.m_cStepDef.m_dOverStep ;

	if (Get3DRCompMask())
		tmpCnt3D.m_dMaxDist = 0.;

	if( m_cSetupDef.m_cOrderDef.m_nLayerOrd == 1 && 
		tmpCnt3D.m_dSafeDist < m_pTool->m_fRadius * 2.0  )
	{ //高度优先
		tmpCnt3D.m_dSafeDist = m_pTool->m_fRadius * 2.0 ;
	}
	/*进退刀圆弧Y轴选择投影方向, CSmtCutPointEx::m_fSurfPos表示*/
	tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_PRJDIR ;
	TmpPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, m_cPrgDef);//根据行号连接多段刀具路径

	if (Get3DRCompMask())
	{
		AddRCompPathTo5AxPathLib(TmpPath);
	}	

	// 该部分用于去除起末点位置的五轴快速定位路径，防止与最后添加的三轴快速定位重复
	CSmtCutPath *pPath = NULL;
	POSITION headPos = TmpPath.m_cAllPath.GetHeadPosition();
	if (headPos)
	{
		pPath = TmpPath.m_cAllPath.GetAt(headPos);

		if (pPath->m_bFeedType == JDNC_FEEDTYPE_QUICK)
		{
			TmpPath.m_cAllPath.RemoveAt(headPos);
			delete pPath;
		}
	}	

	POSITION tailPos = TmpPath.m_cAllPath.GetTailPosition();	
	if (tailPos)
	{
		pPath = TmpPath.m_cAllPath.GetAt(tailPos);
		if (pPath->m_bFeedType == JDNC_FEEDTYPE_QUICK)
		{
			TmpPath.m_cAllPath.RemoveAt(tailPos);
			delete pPath;
		}
	}	

	// STEP 4 : 将加工路径转换成多轴加工路径
	if( TmpPath.m_cAllPath.GetCount() == 0 )
	{
		m_nErrorType  = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	AllPath.AppendCPathLib(TmpPath);
	return TRUE;	
}

// 新边界构面生成路径函数 qqs 2013.12.19
BOOL CSmartFUWLineGen::CreateCombineSurfPathNew(CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf *TrimSurf, CSmtCheckMdl& DriveMdl, RFRAME *dNcMtx,
											 CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg, int nLayerNo, BOOL* arrbSpiral)
{
	// 调整合并后的曲面，如果法向不符合要求，反向
	ModifyCombineSurf(dNcMtx, TrimSurf);

	// STEP 1 : 计算参数域上的路径
	m_cPrgDef.m_dTotalMove = dPrg * 0.7;
	if( TrimSurf == NULL || TrimSurf->m_pSurface == NULL)
	{
		return FALSE ;
	}
	double uwStep[2] ;
	int uwCnt[2], nMoveDir = m_cParam.m_nCutDir;
	TrimSurf->GetDiscreteStep( 0.002, uwStep[0], uwStep[1], uwCnt[0], uwCnt[1] ) ;

	// 判断曲线是否闭合,如果闭合并且方向相同,则沿曲面加工
	BOOL bZigZag = m_cParam.m_bUWLineFlag & NCDEF_UWLINE_ZIGZAG ? TRUE : FALSE;
	BOOL bGrndMdfy = m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY ? TRUE : FALSE;

	BOOL bSpiral = FALSE;	
	BOOL bFlag = m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ? TRUE : FALSE;

	CFlowlineGen	cNcGen( m_cSetupDef, m_cFeedDef, m_cStockDef, m_cToolDef, m_cSpeedDef, m_pTool, m_nCalcThreadNum, m_cParam) ;
	// 判断曲面是否闭合
	arrbSpiral[nLayerNo] = FALSE;
	if (!bGrndMdfy && bFlag)
	{
		CSmartPathGen  ncPathGen ;
		JDNC_TOL		cCurveTol = { 0.002, 10, 0, 0, 0, 0 } ;
		CSmartLoop* pLoop = ncPathGen.ExtractSurfLoop(TrimSurf, cCurveTol);
		if( !pLoop ) return 0 ;
		cNcGen.CalcSurfUWCut( TrimSurf, pLoop, m_cParam, uwStep, nMoveDir, bZigZag, bSpiral ) ;
		Mini_DeleteContours(pLoop);
		arrbSpiral[nLayerNo] = bSpiral;
	}
	
	CSmartCurve allSect ;
	CreateParamLine( TrimSurf, uwStep, allSect, nMoveDir, *dNcMtx, bSpiral) ;
	
	if( allSect.m_pHead == NULL ) 
	{
		m_nErrorType = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	// STEP 2 : 将参数域上的路径影射到加工模型上
	if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_TOUCHMODE )
	{
		DriveMdl.SetTouchCalcMode( TRUE ) ;
	}
	// 投影方式设为曲面法向
	CSmtCPathLib TmpPath;
	CIsoSGuideTPath uwTPath( 5., NCDEF_PROJECTPDIR_NORMAL);
	uwTPath.m_bMaxStepMode = TRUE ;
	uwTPath.m_cPrgDef = m_cPrgDef ;
	uwTPath.m_pAxisCtrl = & axisCtrl ;
	uwTPath.m_dMaxDepth = 10 ;	// 投影深度默认为10
	uwTPath.SetupIsoGen( DriveMdl, TrimSurf->m_pSurface, 
		uwStep, nMoveDir, m_cSetupDef.m_cTolDef) ;
	if( ! uwTPath.GenSGuideTPath( DriveMdl, allSect, TmpPath ) )
	{
		m_nErrorType  = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}
	DriveMdl.SetTouchCalcMode( FALSE ) ;
	if(TmpPath.m_cAllPath.GetCount() <= 0)
	{
		m_nErrorType  = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE;
	}
	POSITION atTPos = TmpPath.m_cAllPath.GetHeadPosition() ;
	while( atTPos )
	{
		CSmtCutPath* pTBase = TmpPath.m_cAllPath.GetNext(atTPos) ;
		pTBase->SetMillDir( GetMillDir() ) ;
		pTBase->DefineBox();
	}
	
	// 将距离过近的相邻路径的起末点坐标重设为同一个点 qqs 2013.06.19
	ResetSameCutPnt(TmpPath, bSpiral);
	TrimPath(TmpPath, AllLoop);

	if (!bSpiral)
	{
		cNcGen.ModifyPathDirection(TmpPath.m_cAllPath);
	}

	if (Get3DRCompMask())
	{
		AddRCompPathTo5AxPathLib(TmpPath);
	}	

	// 该部分用于去除起末点位置的五轴快速定位路径，防止与最后添加的三轴快速定位重复
	CSmtCutPath *pPath = NULL;
	POSITION headPos = TmpPath.m_cAllPath.GetHeadPosition();
	if (headPos)
	{
		pPath = TmpPath.m_cAllPath.GetAt(headPos);

		if (pPath->m_bFeedType == JDNC_FEEDTYPE_QUICK)
		{
			TmpPath.m_cAllPath.RemoveAt(headPos);
			delete pPath;
		}
	}	

	POSITION tailPos = TmpPath.m_cAllPath.GetTailPosition();	
	if (tailPos)
	{
		pPath = TmpPath.m_cAllPath.GetAt(tailPos);
		if (pPath->m_bFeedType == JDNC_FEEDTYPE_QUICK)
		{
			TmpPath.m_cAllPath.RemoveAt(tailPos);
			delete pPath;
		}
	}	

	if( TmpPath.m_cAllPath.GetCount() == 0 )
	{
		m_nErrorType  = JDERROR_GENPATH_NOTGENPATH ;
		return FALSE ;
	}	
	
	POSITION Pos = TmpPath.m_cAllPath.GetHeadPosition();
	while (Pos)
	{
		pPath = TmpPath.m_cAllPath.GetNext(Pos);
		pPath->m_nLayerNo = nLayerNo;
	}

	AllPath.AppendCPathLib(TmpPath);
	return TRUE;	
}

void CSmartFUWLineGen::ResetSameCutPnt(CSmtCPathLib& TmpPath, BOOL bSpiral, double minDis)
{

	POSITION atTPos = TmpPath.m_cAllPath.GetHeadPosition() ;
	while( atTPos )
	{
		CSmtCutPath* pFirst = TmpPath.m_cAllPath.GetNext(atTPos) ;

		PNT3D ptFirst, ptNext;
		CSmtCutPath* pNext = NULL;

		if (bSpiral && atTPos)     	 pNext = TmpPath.m_cAllPath.GetAt(atTPos) ;
		else if (bSpiral && !atTPos) break;
		else   			             pNext = pFirst;

		for (int i = 0; i < 3; i++)
		{
			ptFirst[i] = pFirst->m_pTail->m_fPoint[i];
			ptNext[i] = pNext->m_pHead->m_fPoint[i];
		}
		double dis = mathDist(ptFirst, ptNext) ;
		if (dis < minDis)
		{
			FPNT3D newPnt; // 新坐标取两个点连线的中点坐标
			for (int i = 0; i < 3; i++)
			{
				newPnt[i] = (pFirst->m_pTail->m_fPoint[i] + pNext->m_pHead->m_fPoint[i]) / 2.0f;
			}
			
			mathFCpyPnt(newPnt, pFirst->m_pTail->m_fPoint);
			mathFCpyPnt(newPnt, pNext->m_pHead->m_fPoint) ;
		}
	}
}


void CSmartFUWLineGen::AddRCompPathTo5AxPathLib(CSmtCPathLib& CutPathLib, BOOL bIs3D)
{
	//UNUSED_ALWAYS(bIs3D);
	POSITION pos = CutPathLib.m_cAllPath.GetHeadPosition() ;
	POSITION prepos = NULL;
	while( pos )
	{
		POSITION atPos = pos;
		
		CSmtCutPath* pCutPath = CutPathLib.m_cAllPath.GetNext(pos) ;
		if (pCutPath->m_bFeedType == JDNC_FEEDTYPE_LEAD)
		{
			if( prepos == NULL || CutPathLib.m_cAllPath.GetAt(prepos)->m_bFeedType == JDNC_FEEDTYPE_QUICK)
			{
				CutPathLib.m_cAllPath.InsertBefore(atPos, CalRCompByLeadPath(pCutPath,FALSE,bIs3D));
			}
			if ( pos == NULL || CutPathLib.m_cAllPath.GetAt(pos)->m_bFeedType == JDNC_FEEDTYPE_QUICK)
			{
				CutPathLib.m_cAllPath.InsertAfter(atPos,CalRCompByLeadPath(pCutPath,TRUE,bIs3D));
			}
		}
		if (pCutPath->m_bFeedType == JDNC_FEEDTYPE_ROUGH)
		{
			if( prepos == NULL || CutPathLib.m_cAllPath.GetAt(prepos)->m_bFeedType == JDNC_FEEDTYPE_QUICK)
			{
				CutPathLib.m_cAllPath.InsertBefore(atPos, CalRCompByRoughPath(pCutPath,FALSE, FALSE, TRUE));
			}

			if ( pos == NULL || CutPathLib.m_cAllPath.GetAt(pos)->m_bFeedType == JDNC_FEEDTYPE_QUICK)
			{
				CutPathLib.m_cAllPath.InsertAfter(atPos, CalRCompByRoughPath(pCutPath,TRUE,FALSE, TRUE));
			}
		}
		prepos = atPos;
	}

	pos = CutPathLib.m_cAllPath.GetHeadPosition() ;
	while (pos)
	{
		POSITION atpos = pos;
		CSmtCutPath* pCutPath = CutPathLib.m_cAllPath.GetNext(pos) ;

		if (pCutPath->m_bFeedType == JDNC_FEEDTYPE_QUICK)
		{
			CSmtCutPath* pPath = CutPathLib.m_cAllPath.GetAt(atpos);
			delete pPath;
			pPath = NULL;
			CutPathLib.m_cAllPath.RemoveAt(atpos);			
		}
	}
}

//把一组曲面变成若干张4边面
BOOL CSmartFUWLineGen::SurfsTo4BndSurf(C3DSurfArray &GeoSurfList, C3DSurfArray &SurfNew, C3DSurfArray &SurfLeft)
{
	INT_PTR nSize = GeoSurfList.GetSize();
	if (nSize == 0)
		return FALSE;
	if (nSize == 1)
	{// 单个面非旋转面时，直接调用常规曲面流线方法处理
		CGeoTrmSurf *pSurf = (CGeoTrmSurf *)GeoSurfList.GetAt(0);
		if (pSurf && pSurf->m_pSurface && pSurf->m_pSurface->GetType() != SURFACE_REVOLVED)
		{
			SurfLeft.Append(GeoSurfList);
			return FALSE;
		}
	}


	// 由几何面创建实体面
	CEntList EttSurfList;
	CEttSurface *pEttSurf;
	for (int i=0; i<GeoSurfList.GetSize(); i++)
	{
		if (NULL == GeoSurfList.GetAt(i))
			continue;
		pEttSurf = new CEttSurface(GeoSurfList.GetAt(i));
		EttSurfList.AddTail(pEttSurf);
	}

	// 调用面合并函数，得到合并的新面
	CEntList EttSurfNew, EttSurfLeft;
	CSurfEditor SfEdtr;
	BOOL bRet = SfEdtr.EntListTo4BndSurf(&EttSurfList, 3, EttSurfNew, EttSurfLeft);

	// 添加新面
	while (EttSurfNew.GetCount() > 0)
	{		
		pEttSurf = (CEttSurface *)EttSurfNew.RemoveHead();
		if (NULL == pEttSurf)
			continue;
		SurfNew.Add(pEttSurf->GetSurf());
		pEttSurf->SetSurf(NULL);
		delete pEttSurf;
	}	

	// 添加剩余面
	while (EttSurfLeft.GetCount() > 0)
	{		
		pEttSurf = (CEttSurface *)EttSurfLeft.RemoveHead();
		if (NULL == pEttSurf)
			continue;
		SurfLeft.Add(pEttSurf->GetSurf());
	}

	// 删除原始实体面
	while (EttSurfList.GetCount() > 0)
	{		
		pEttSurf = (CEttSurface *)EttSurfList.RemoveHead();
		if (NULL == pEttSurf)
			continue;
		pEttSurf->SetSurf(NULL);
		delete pEttSurf;
	}
	return bRet;
}

//把一组曲面变成若干张4边面，调用新的构造面函数 qqs 2013.11.15
BOOL CSmartFUWLineGen::SurfsTo4BndSurfNew(C3DSurfArray &GeoSurfList, CCombSurfArray &CombSurfArray, C3DSurfArray &SurfLeft)
{
	INT_PTR nSize = GeoSurfList.GetSize();
	if (nSize == 0)
		return FALSE;
	if (nSize == 1)
	{// 单个面非旋转面时，直接调用常规曲面流线方法处理
		CGeoTrmSurf *pSurf = (CGeoTrmSurf *)GeoSurfList.GetAt(0);
		if (pSurf && pSurf->m_pSurface && pSurf->m_pSurface->GetType() != SURFACE_REVOLVED)
		{
			SurfLeft.Append(GeoSurfList);
			return FALSE;
		}
	}

	// 由几何面创建实体面
	CEntList EttSurfList;
	CEttSurface *pEttSurf;
	for (int i=0; i<GeoSurfList.GetSize(); i++)
	{
		if (NULL == GeoSurfList.GetAt(i))
			continue;
		pEttSurf = new CEttSurface(GeoSurfList.GetAt(i));
		EttSurfList.AddTail(pEttSurf);
	}

	// 调用面合并函数，得到合并的新面
	typedef CTypedPtrArray<CPtrArray,CEntList*>	CEntListArray;
	C3DSurfArray SurfNew;
	CEntListArray EttSurfNewSource;
	CEntList EttSurfNew, EttSurfLeft;

	CSurfEditor SfEdtr;
	BOOL bRet = SfEdtr.EntListTo4BndSurfExt(&EttSurfList, 4, EttSurfNew, EttSurfLeft, EttSurfNewSource);
	// 添加新面
	while (EttSurfNew.GetCount() > 0)
	{		
		pEttSurf = (CEttSurface *)EttSurfNew.RemoveHead();
		if (NULL == pEttSurf)
			continue;

		SurfNew.Add(pEttSurf->GetSurf());
		pEttSurf->SetSurf(NULL);
		delete pEttSurf;
	}	

	for(int i = 0; i < EttSurfNewSource.GetSize() ; i++)
	{
		CEntList* pSurfSourceList = EttSurfNewSource.GetAt(i);
		CGeoSurf* pGeoSurf = SurfNew.GetAt(i);

		CCombSurf *pCombSurf =new CCombSurf(pGeoSurf, pSurfSourceList);
		CombSurfArray.Add(pCombSurf);
		delete pGeoSurf;
		pGeoSurf = NULL;
		delete pSurfSourceList;
	}

	// 添加剩余面
	while (EttSurfLeft.GetCount() > 0)
	{		
		pEttSurf = (CEttSurface *)EttSurfLeft.RemoveHead();
		if (NULL == pEttSurf)
			continue;
		SurfLeft.Add(pEttSurf->GetSurf());
	}

	// 删除原始实体面
	while (EttSurfList.GetCount() > 0)
	{		
		pEttSurf = (CEttSurface *)EttSurfList.RemoveHead();
		if (NULL == pEttSurf)
			continue;
		pEttSurf->SetSurf(NULL);
		delete pEttSurf;
	}
	return bRet;
}

// 调整合并后的曲面
void CSmartFUWLineGen::ModifyCombineSurf(RFRAME *dNcMtx, CGeoTrmSurf* pCombineSurf)
{
	int nOffsetDir = m_cParam.m_nOffsetDir;
	
	// 自动：根据它的方向是否向上来调整
	if (nOffsetDir == 0)
	{
		if(!pCombineSurf->m_pLoop)
		{		
			pCombineSurf->AddLoop(pCombineSurf->CreateBoxUVLoop());
		}
	}
	if (nOffsetDir == 1)
	{
		if(!pCombineSurf->m_pLoop)
		{		
			pCombineSurf->AddLoop(pCombineSurf->CreateBoxUVLoop());
		}
		pCombineSurf->ReverseDir();
	}
	else if (nOffsetDir == 2)
	{
		int nNormalDir = MathCAM_GetSurfNormalDir(pCombineSurf, *dNcMtx);
		if (nNormalDir == -1)
			pCombineSurf->ReverseDir();
	}
}

// 生成参数域直线段
int CSmartFUWLineGen::CreateParamLine(CGeoTrmSurf* TrimSurf, 
									  DOUBLE UWStep[2]     ,
									  CSmartCurve& AllLine ,
									  int& nMoveDir,
									  RFRAME& dNcMtx,
									  BOOL bSpiral)
{
	// STEP 1 : 计算曲面中线及其长度
	if( TrimSurf == NULL ) 
	{
		return 0 ; 
	}
	DOUBLE uwStep[2] = { UWStep[0], UWStep[1] } ;
	DOUBLE uwBox[2][2],uw[2] ;
	int uwCnt[2] ;
	CSmtCutPath  uwCurve[2] ;
	CSmtCutPoint* pTPnt     ;
	if( TrimSurf->m_pLoop )
	{
		memcpy( uwBox, TrimSurf->m_pLoop->m_dBox2d, sizeof( double[2][2] ) ) ;
	}
	else
	{
		uwBox[0][0] = uwBox[0][1] = 0.0 ;
		uwBox[1][0] = uwBox[1][1] = 1.0 ;
	}
	for( int k = 0 ; k < 2 ; k ++ )
	{
		if( uwStep[k] > 0.05 ) uwStep[k] = 0.05 ;
		else if( uwStep[k] < 0.002 ) uwStep[k] = 0.002 ;
		uwCnt[k] = max( 2, (int) ceil( (uwBox[1][k]-uwBox[0][k]) / uwStep[k]) );
		uwStep[k] = (uwBox[1][k]-uwBox[0][k]) / ( uwCnt[k] -1 ) ;
		for( int i = 0 ; i < uwCnt[k] ; i ++ )
		{
			PNT3D dPoint ;
			if( k == 0 ) 
			{
				uw[0] = uwBox[0][k] + i * uwStep[0]    ;
				uw[1] = 0.5 * (uwBox[0][1]+uwBox[1][1]) ;
			}
			else
			{
				uw[0] = 0.5 * (uwBox[1][0]+uwBox[0][0]) ;
				uw[1] = uwBox[0][1] + i * uwStep[1] ;
			}
			TrimSurf->GetPoint( uw[0], uw[1], dPoint ) ;
			pTPnt = new CSmtCutPoint( dPoint ) ;
			pTPnt->m_fPoint[3] = TFLOAT( i * uwStep[k]) ;
			uwCurve[k].AddTail( pTPnt ) ;
		}
		uwCurve[k].NormalizeLen() ;
	}

	BOOL bGrndMdfy = m_cParam.m_bUWLineFlag & NCDEF_UWLINE_GRNDMDFY ? TRUE : FALSE;
	PNT2D dMinPt, dMaxPt ;
	dMinPt[0] = dMinPt[1] = 0.0 ;
	dMaxPt[0] = uwCurve[0].GetLength() ;
	dMaxPt[1] = uwCurve[1].GetLength() ;
	// STEP 2 : 计算平铺参数域的上的路径分布
	CSmartLoop tmpLoop ;
	tmpLoop.CreateLoop( dMinPt, dMaxPt ) ;
	double dRotAng = 0.0 ;
	if (nMoveDir == 1)
		dRotAng = -90.0;
	dRotAng = ANGLE_TO_RADIAN( dRotAng ) ;
	tmpLoop.RotateContour( - dRotAng ) ;
	double yStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
	double yMin = tmpLoop.m_dBox[0][1] + 2.0e-6;
	double yMax = tmpLoop.m_dBox[1][1] - 2.0e-6;
	// 如果开启磨削调整，使路径条数为3条，取中间一条 qqs 2013.05.20
	int nLineCnt = 0;
	
	if (bGrndMdfy) 	nLineCnt = 2;
	else nLineCnt = (int)ceil((yMax - yMin ) / yStep) ;

	yStep = (yMax - yMin ) / nLineCnt  ;
	CSmartSect * sectBuff[20],*pSect ;
	PNT2D    dIntPt[20] ;
	double dYAt = yMin , dYInc = yStep ;
	for( int i = 0 ; i <= nLineCnt ; i ++, dYAt += dYInc ) 
	{
		if (bGrndMdfy && ( i != 1)) continue;

		UINT nIntCnt = tmpLoop.GetYLineInt( dYAt , dIntPt, sectBuff, 10 ) ;
		nIntCnt = tmpLoop.SortPointSect( dIntPt, sectBuff, nIntCnt ) ;
		if( nIntCnt != 2 ) continue ;
		if( nMoveDir == 1 ) 
		{// W向加工
			nc_VectorSwap( dIntPt[0], dIntPt[1], 2 ) ;
		}

		CSmartLine* pLine = new CSmartLine( dIntPt[0], dIntPt[1] ) ;		
		AllLine.AddSect( pLine ) ;
	}
	AllLine.RotateCurve( dRotAng ) ; 
	// STEP 4 : 影射到曲面参数域
	for( pSect = AllLine.GetHead() ; pSect ; pSect = pSect->next)
	{
		CSmartLine* pLine = (CSmartLine*) pSect ;
		for( k = 0 ; k < 2 ; k ++ ) 
		{
			pLine->m_aPoint[0][k] /= dMaxPt[k] ;
			pLine->m_aPoint[1][k] /= dMaxPt[k] ;
		}
	}
	for( k = 0 ; k < 2 ; k ++ ) 
	{
		i = 0 ;
		for( pTPnt = uwCurve[k].m_pHead ;pTPnt ; pTPnt = pTPnt->next )
		{
			pTPnt->m_fPoint[0] = TFLOAT( uwBox[0][k] + i * uwStep[k] ) ;
			i ++ ;
		}
	}
	CFlowlineGen	cNcGen( m_cSetupDef, m_cFeedDef, m_cStockDef, m_cToolDef, m_cSpeedDef, m_pTool, m_nCalcThreadNum, m_cParam) ;
	CSmtCutPath* pNew = new CSmtCutPath;
	for( pSect = AllLine.GetHead() ; pSect ; pSect = pSect->next)
	{
		FPNT3D fTPnt ;
		CSmartLine* pLine = (CSmartLine*) pSect ;
		for( k = 0 ; k < 2 ; k ++ ) 
		{
			uwCurve[k].GetPointAt( (TFLOAT) pLine->m_aPoint[0][k], fTPnt ) ;
			pLine->m_aPoint[0][k] = MathCAM_UWParam( fTPnt[0] ) ;
			uwCurve[k].GetPointAt( (TFLOAT) pLine->m_aPoint[1][k], fTPnt ) ;
			pLine->m_aPoint[1][k] = MathCAM_UWParam( fTPnt[0] ) ;
		}
		if (bGrndMdfy)
		{
			// 开启磨削调整 qqs 2013.05.20
			PNT4D			dPnt1={pLine->m_aPoint[0][0], pLine->m_aPoint[0][1], 0.0, 0.0};
			PNT4D			dPnt2={pLine->m_aPoint[1][0], pLine->m_aPoint[1][1], 0.0, 0.0};
			cNcGen.GrindParamPathGen(pNew, m_cParam, dNcMtx, TrimSurf, nMoveDir,m_cSetupDef.m_cTolDef.m_dArcTol, dPnt1, dPnt2);
		}
	}
	if (bGrndMdfy)
	{
		AllLine.ClearAll();
		CSmtCutPoint* ptSt = pNew->m_pHead;
		while(ptSt&&ptSt->next)
		{
			PNT2D st, end;
			st[0] = ptSt->m_fPoint[0];
			st[1] = ptSt->m_fPoint[1];
			end[0] = ptSt->next->m_fPoint[0];
			end[1] = ptSt->next->m_fPoint[1];
			CSmartLine* pLine = new CSmartLine( st, end ) ;		
			AllLine.AddSect( pLine ) ;
			ptSt = ptSt->next;
		}
	}	
	delete pNew;
	pNew = NULL;
 	
	// 开启封闭螺旋连刀
	if (bSpiral) 
		cNcGen.AdjustSpiralCurve(AllLine); 
	
	/* 1.由于此路径为构造面路径，构造出来的面与原始曲面可能流线方向、流线起点不同，所以调整出来的
	 *   路径方向与起点可能存在与设置不符的情况。
	 * 2.由于第二点所说的问题，当开启封闭螺旋连刀时，首先确保路径封闭，首尾相连，然后修改路径起点
	 *   可以修改路径的加工方向。 qqs 2013.12.19 */
	AdjustStartAt( AllLine, bSpiral, bGrndMdfy ) ;

	return 1 ;
}

// 调整路径的起点
void CSmartFUWLineGen::AdjustStartAt( CSmartCurve& AllLine, BOOL bSpindle, BOOL bGrndMdfy )
{
	int nStartAt = m_cParam.m_nStartAt;
	CSmartSect* pSect ;
	int nSectID = 1 ;
	if (bSpindle || bGrndMdfy)
	{
		if (nStartAt == 1 || nStartAt == 3)
		{
			AllLine.Reverse() ;
			for( pSect = AllLine.m_pHead ; pSect ; pSect = pSect->next)
			{
				pSect->m_nSectId = nSectID ;
				nSectID ++ ;
			}
		}		
		return;
	}
	int nCutDir = m_cParam.m_nCutDir;
	
	if( nStartAt == 0 )
	{
		for( pSect = AllLine.m_pHead ; pSect ; pSect = pSect->next)
		{
			pSect->m_nSectId = nSectID ;
			nSectID ++ ;
		}
	}
	else if( (nCutDir == 0 && nStartAt == 1) || (nCutDir == 1 && nStartAt == 3) )
	{// 反向每条路径
		for( pSect = AllLine.m_pHead ; pSect ; pSect = pSect->next)
		{
			pSect->Reverse();
			pSect->m_nSectId = nSectID ;
			nSectID ++ ;
		}
	}
	else if( nStartAt == 2 )
	{// 整体反向
		AllLine.Reverse() ;
		for( pSect = AllLine.m_pHead ; pSect ; pSect = pSect->next)
		{
			pSect->m_nSectId = nSectID ;
			nSectID ++ ;
		}
	}
	else if( (nCutDir == 0 && nStartAt == 3) || (nCutDir == 1 && nStartAt == 1) )
	{// 调换次序
		CSmartCurve tmpCv ;
		while( AllLine.m_pTail ) 
		{
			pSect = AllLine.m_pTail ;
			AllLine.RemoveSect( pSect ) ;
			pSect->m_nSectId = nSectID ;
			tmpCv.AddSect( pSect ) ;
			nSectID ++ ;			
		}
		AllLine.AppendCurve( tmpCv ) ;
	}
}

// 路径裁剪
void CSmartFUWLineGen::TrimPath(CSmtCPathLib &AllPath, CSmartLoop *AllLoop)
{
	// 轮廓裁减
	if( AllLoop )
	{
		CSmtCPathLib tmpLib ;
		CSmtCutPath *pPath = NULL ;
		AllPath.TrimCutPathByContours( tmpLib, AllLoop ) ;
		AllPath.ClearAllPath() ;
		while( tmpLib.m_cAllPath.GetCount() )
		{
			pPath = tmpLib.m_cAllPath.RemoveHead() ;
			TransfSmtCutPath( pPath ) ;
			pPath->DefineBox () ;
			AllPath.AddToTail( pPath) ;
		}
	}

	if (m_cParam.m_bUWLineFlag & NCDEF_UWLINE_TRIMBYHEIGHT)
	{
		// 高度裁减
		double dArcTol = m_cSetupDef.m_cTolDef.m_dArcTol ;
		double dTopHeight = GetTopHeight();
		double dBottomHeight = GetBottomHeight();

		AdjustTopAndBotPath(AllPath, dTopHeight, dBottomHeight, dArcTol);
		TrimAllPathByHeight( AllPath, dTopHeight + dArcTol, dBottomHeight - dArcTol) ;
	}
}

// 重置顶部底部路径
void CSmartFUWLineGen::AdjustTopAndBotPath(CSmtCPathLib &AllPath, double dTopHeight, double dBottomHeight, double dArcTol)
{
	// dTop、dBot用于判断路径是否没有超出裁剪范围或路径上下都超出裁剪范围
	// dTopHeight、dBottomHeight用于对路径进行精确裁剪重置
	double dTop = dTopHeight - dArcTol; 
	double dBot = dBottomHeight + dArcTol; 

	CSmtCutPath* pPath = NULL, *pPathNext = NULL;
	POSITION pos, atpos, tmpPos ;
	CSmtCutPoint *pHead = NULL;

	// 去除节点数为0和1的路径和超出加工深度范围的路径
	pos = AllPath.m_cAllPath.GetHeadPosition();
	while(pos)
	{
		atpos = pos;
		pPath = AllPath.m_cAllPath.GetNext(pos);
		if (pPath->m_fBox[0][2] > dTopHeight && pPath->m_fBox[1][2] > dTopHeight ||
			pPath->m_fBox[0][2] < dBottomHeight && pPath->m_fBox[1][2] < dBottomHeight || 
			pPath->m_nNumPnt < 2 )
		{
			delete pPath ;
			AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		}
	}

	// 调整首末两圈路径
	pos = AllPath.m_cAllPath.GetHeadPosition();
	while(pos)
	{
		atpos = pos;
		pPath = AllPath.m_cAllPath.GetNext(pos);

		//////////////////////////////////////////////////////////////////////////
		// nResult = 0：pPath不需裁剪，不作处理
		// nResult = 1：pPath需裁剪，pPathNext不需裁剪或pPathNext为空，调整pPath节点
		// nResult = 2：pPath、pPathNext需裁剪，且没有下一个不需裁剪的对象，调整pPath节点，删除pPath后所有路径
		// nResult = 3：pPath、pPathNext需裁剪，但后面仍有不需裁剪的对象，将pPath删除
		// 此过程中路径不需裁剪包括三种情况:(1)路径没有超出裁剪范围;(2)路径上下都超出裁剪范围（主要用于竖向的路径判断）
		// (3)路径Z向跨度过大(暂定过大的标准是大于三倍路径间距或0.5中较小的)，该路径直接删除或者调整可能会影响加工效果（主要用于竖向路径和曲面流线较不均匀的路径判断）
		// 第(2)(3)种不处理的情况，最后通过函数TrimAllPathByHeight进行路径裁剪
		//////////////////////////////////////////////////////////////////////////
		int nResult = 0; 
		if (pPath->m_fBox[0][2] < dBot && pPath->m_fBox[1][2] > dTop) nResult = 0; 
		else if ( fabs(pPath->m_fBox[0][2] - pPath->m_fBox[1][2]) > min(3.0 * m_cParam.m_dOverStep,0.5)) nResult = 0;
		else if (pPath->m_fBox[0][2] > dBot && pPath->m_fBox[1][2] < dTop) nResult = 0;
		else if ((pPath->m_fBox[0][2] < dTopHeight && pPath->m_fBox[1][2] > dTopHeight))
		{
			tmpPos = pos;
			if (tmpPos)
			{
				pPathNext = AllPath.m_cAllPath.GetNext(tmpPos);

				if (!(pPathNext->m_fBox[0][2] < dTopHeight && pPathNext->m_fBox[1][2] > dTopHeight)) nResult = 1;
				else
				{
					while(tmpPos)
					{
						pPathNext = AllPath.m_cAllPath.GetNext(tmpPos);
						if (!(pPathNext->m_fBox[0][2] < dTopHeight && pPathNext->m_fBox[1][2] > dTopHeight))
						{
							nResult = 3;
							break;
						}
						else nResult = 2;
					}
				}
			}
			else nResult = 1;
		}
		else if ((pPath->m_fBox[0][2] < dBottomHeight && pPath->m_fBox[1][2] > dBottomHeight))
		{
			tmpPos = pos;
			if(tmpPos)
			{
				pPathNext = AllPath.m_cAllPath.GetNext(tmpPos);
				
				if (!(pPathNext->m_fBox[0][2] < dBottomHeight && pPathNext->m_fBox[1][2] > dBottomHeight) &&
					(fabs(pPath->m_fBox[0][2] - pPath->m_fBox[1][2]) < 1)) nResult = 1;
				else
				{
					while(tmpPos)
					{
						pPathNext = AllPath.m_cAllPath.GetNext(tmpPos);
						if (!(pPathNext->m_fBox[0][2] < dBottomHeight && pPathNext->m_fBox[1][2] > dBottomHeight)&&
							(fabs(pPath->m_fBox[0][2] - pPath->m_fBox[1][2]) < 1))
						{
							nResult = 3;
							break;
						}
						else nResult = 2;
					}
				}
			}
			else nResult = 1;
		}

		if (nResult == 1)
		{
			pHead = pPath->m_pHead;
			while(pHead)
			{
				if (pHead->m_fPoint[2] > dTopHeight)
				{
					pHead->m_fPoint[2] = (float)dTopHeight;
				}
				if (pHead->m_fPoint[2] < dBottomHeight)
				{
					pHead->m_fPoint[2] = (float)dBottomHeight;
				}
				pHead = pHead->next;
			}
		}
		else if(nResult == 2)
		{
			pHead = pPath->m_pHead;
			while(pHead)
			{
				if (pHead->m_fPoint[2] > dTopHeight)
				{
					pHead->m_fPoint[2] = (float)dTopHeight;
				}
				if (pHead->m_fPoint[2] < dBottomHeight)
				{
					pHead->m_fPoint[2] = (float)dBottomHeight;
				}
				pHead = pHead->next;
			}
			while(pos)
			{
				POSITION tmpAtPos = pos;
				pPathNext = AllPath.m_cAllPath.GetNext(pos);
				delete pPathNext;
				AllPath.m_cAllPath.RemoveAt(tmpAtPos);
			}
		}
		else if ( nResult == 3 )
		{
			delete pPath ;
			AllPath.m_cAllPath.RemoveAt ( atpos ) ;
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
//CSmartFFollowGen : 环绕等距曲面精加工
//

CSmartFFollowGen::CSmartFFollowGen()
{
}

CSmartFFollowGen::~CSmartFFollowGen()
{
}

CSmartLoop* CSmartFFollowGen::ResortContour(CSmartLoop* AllLoop )  
{
	CSmartLoop LoopLib ;
	if( ! AllLoop ) return FALSE ;
    CSmartLoop *pLoop ;
	for( pLoop = AllLoop ; pLoop ; pLoop = pLoop->next )
	{
		if( pLoop->m_dArea < 0.0 ) pLoop->ReverseLoop() ;
	}
    AllLoop = LoopLib.BuildContour( AllLoop ) ;
	AllLoop = LoopLib.ResortContour( AllLoop, GetSortType(), TRUE ) ;
	return AllLoop ; 
}

BOOL CSmartFFollowGen::RemoveMosaic(CSmtCPathLib  &AllPath, CSmtCheckMdl & DriveMdl)
{
	double dSmoothTol = this->m_cSetupDef.m_cTolDef.m_dArcTol*3;
	//光顺误差不能太大，否则路径间距可能会明显不均匀
	if(dSmoothTol<= 0.002)
		dSmoothTol = 0.002;
	else if(dSmoothTol>= 0.02)
		dSmoothTol = 0.02;
	C3DStepNomosaic c3DStepNoMosac;
	if(!c3DStepNoMosac.RemoveMosaic(AllPath, DriveMdl, 
		m_cSetupDef.m_cTolDef, m_cPrgDef, dSmoothTol))
	{
		return FALSE;
	}
	return TRUE;
}
int CSmartFFollowGen::GeneratePathEx( CPathGroup&    NewPath  ,  /*雕刻路径*/
					                  CSmartGraphic& Graph     )  /*雕刻图形*/
{ 
	//STEP 0 :环绕等距消除马塞克计算速度慢，光顺插点前的中间路径没有必要算得过于精细，
	//若选中消除马赛克则先按照不选中计算、以提高计算速度
	int nOrigMdlFlag = m_cSetupDef.m_cModelTol.m_nMdlFlag;
	double dOrigMaxStep = m_cSetupDef.m_cTolDef.m_dMaxStep;
	if(nOrigMdlFlag & NCDEF_SETUP_NOMOSAIC )
	{
		m_cSetupDef.m_cModelTol.m_nMdlFlag &= ~NCDEF_SETUP_NOMOSAIC;
		m_cSetupDef.m_cTolDef.m_dMaxStep = m_dMaxStep ;
	}
    // STEP 1 : 构建加工模型
	if (!IsAxisCtrlValid(Graph))	// 刀轴控制模型判断
		return FALSE;

    CSmtCheckMdl DriveMdl, CheckMdl ;
    CSmartLoop* BoundCont = NULL    ; 
    if( ! BuildSFinishGen( NewPath, Graph, DriveMdl, CheckMdl, BoundCont ) )
    {
        return FALSE ;
    }
    if( m_cParam.m_cSlope.m_nAreaType == NCDEF_FSLOPE_SLOPE )
    {
        DriveMdl.SetDelFlatCalcMode( FALSE ) ;
    }
    // STEP 2 : 计算加工路径
	if (glbf_GetProductID() != PRODUCT_ID_DENTFORM)
	{
		SurfNC_SetNewStepEx( IDS_PATHGEN_STEP_ORGPATH ) ;
	}

    m_cPrgDef.m_dTotalMove = 100.0 ;
    CSmtCPathLib   guideCLib ;
    if( m_cParam.m_nFollowType == NCDEF_FOLLOW_GUIDE  && 
        (m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_3DSTEP ) )
    {/*导动线等距环切*/
        CreateGuideCurve( Graph, DriveMdl, BoundCont, guideCLib ) ;
    }

	if(m_cParam.m_nFollowType == NCDEF_FOLLOW_GUIDE && 
		guideCLib.m_cAllPath.GetCount()<1)
	{//导动线等距环切： 没有选择导动线
		m_nErrorType = JDERROR_FOLLOWPATH_NOGUIDECURVE;
		Mini_DeleteContours( BoundCont ) ;
		return FALSE ;
	}
	else if(m_cParam.m_nFollowType == NCDEF_FOLLOW_START && 
		m_cMethodDef.m_cSFinishCut.m_cStartPnt.m_nStartPntCnt<1)
	{//指定点等距环切： 没有选择指定点
		m_nErrorType = JDERROR_FOLLOWPATH_NOGUIDEPNT;
		Mini_DeleteContours( BoundCont ) ;
		return FALSE ;
	}

	// STEP 2 : 构建MeshMdl
	int dCur[4] = {19, 19, 60, 2};
	if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
	{
		dCur[0] = 7, dCur[1] = 7, dCur[2] = 32, dCur[3] = 4;
	}

    m_cPrgDef.m_dTotalMove = dCur[0] ;
    CSmtMeshMdl   MeshMdl ;
    if( BuildMeshMdl( MeshMdl, DriveMdl, BoundCont,TRUE ) == FALSE )
    {
        Mini_DeleteContours( BoundCont ) ;
        return FALSE ;
    }
    DriveMdl.CreateAccChkBox() ;

    CSmtCPathLib  AllPath , AutoBnd ;// 修边路经
    m_cPrgDef.m_dTotalMove = dCur[1] ;
    if( m_cParam.m_cSlope.m_nAreaType == NCDEF_FSLOPE_FLAT )
    {/*平坦面加工  */
        MeshMdl.ExtractFlatAreaBound( (TFLOAT)m_cParam.m_cSlope.m_dSlopeAngle, AutoBnd, m_cPrgDef ) ;
    }
    else if( m_cParam.m_cSlope.m_nAreaType == NCDEF_FSLOPE_SLOPE )
    {/*陡斜面加工  */
        MeshMdl.ExtractSteepAreaBound( (TFLOAT)m_cParam.m_cSlope.m_dSlopeAngle, AutoBnd, m_cPrgDef ) ;
    }
    else
    {/*所有曲面加工*/
        MeshMdl.GenMeshSelfBound( AutoBnd, m_cPrgDef ) ;
    }

    TFLOAT fZBtm = MeshMdl.GetValidMeshBottom() - TFLOAT(2.0*MeshMdl.m_fNetStep);
    TFLOAT fZOld = DriveMdl.SetBottom( max( fZBtm, DriveMdl.m_fBottom) ) ;
#ifdef _DEBUG
    if( MeshMdl.m_bDumpXYPath )
    {
        MeshMdl.DumpMeshToPathGroup( NewPath ) ;
        CPathCombine* pTComb = new CPathCombine( NC_WPROCESS_ROUGH ) ;
        AutoBnd.AddToPathCombine( *pTComb ) ;
        NewPath.AddData( 0.0, pTComb ); //最下面一层
        return TRUE ;
    }
#endif 
    if( AutoBnd.GetNumPath() == 0 ) 
    {
 	    m_nErrorType = JDERROR_GENPATH_NOAREA  ;
        return FALSE ;
    }
	// STEP 3 : 计算路径
    m_cPrgDef.m_dTotalMove = dCur[2] ;
    CSmtCutPath* pTPath;
    m_cParam.m_dOverStep = m_cFeedDef.m_cStepDef.m_dOverStep ;
    if( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_3DSTEP )
	{/* 空间路径间距 */
        MeshMdl.m_pGenMeshGuidePointBound = NcBound_GenMeshGuidePointBound ;
        MeshMdl.m_pGenMeshGuideCurveBound = NcBound_GenMeshGuideCurveBound ;
        MeshMdl.CreateFollow3DPath( m_cParam, 
                                    m_cMethodDef.m_cSFinishCut.m_cStartPnt, 
                                    guideCLib ,
                                    AutoBnd, AllPath,
                                    NewPath , m_cPrgDef ) ;
        if( SurfNC_IsAbort() )
        {
		    m_nErrorType = JDERROR_GENPATH_ABORT  ;
		    return FALSE ;
        }
    }
    else 
    {/* 平面路径间距 */
        DOUBLE dSideTol = min( 0.03, 0.5 * m_cParam.m_dOverStep ) ;
        CSmartLoop* AllLoop = AutoBnd.ConvertToContours( FALSE, GetCorDef(), dSideTol ) ; 
        if( AllLoop == NULL ) 
        {
            m_nErrorType  = JDERROR_GENPATH_NOAREA ;
            return FALSE ;
        }
		else
		{
			CSmartLoop cTmpLoop ;
			AllLoop =cTmpLoop.ResortContour( AllLoop, GetSortType(), FALSE);
		}
        CSmartCurveLib AllCurve ;
        CSmartCurve * pCurve ;
	    CSmartLoop *pContour ;
        CSmtCPathLib tmpTLib ;
        int nLayerNo = 0 ;
	    for( pContour = AllLoop ; pContour ; pContour = pContour->next )
	    {
            pContour->CreateFollowCurve( m_cParam, m_cSetupDef,
                                    m_cMethodDef.m_cSFinishCut.m_cStartPnt, AllCurve ) ;
            if( AllCurve.m_pTail == NULL ) continue ;
            nLayerNo ++ ;
            for( pCurve = AllCurve.m_pHead ; pCurve ; pCurve = pCurve->next )
            {
                pTPath = new CSmtCutPath() ;
                pTPath->AddCurve( *pCurve , m_cSetupDef.m_cTolDef ) ;
                pTPath->m_nLineNo = pCurve->m_nLevelNo ;
                pTPath->m_nLayerNo = nLayerNo ;
                AllPath.AddToTail( pTPath ) ;
            }
            AllCurve.DeleteAllCurves() ;
	    }
        Mini_DeleteContours( AllLoop ) ;
        // STEP 4 :干涉检查, 生成无干涉的刀具路径
        if( !AllPath.VerifyCutPath( DriveMdl, GetSurfaceTol(), m_cPrgDef ) )
	    {
		    m_nErrorType = JDERROR_GENPATH_ABORT  ;
		    return FALSE ;
	    }
        AllPath.DelPointOnLine() ;
    }
    MeshMdl.ClearAll() ;

	//STEP 4 :路径连接
    m_cPrgDef.m_dTotalMove = dCur[3] ;
	SmartNC_RegCreateConnect3DPath( MathCAM_GenNormalLeadPathEx );

	BOOL bZigzag = ( m_cParam.m_bFollowFlag & NCDEF_FOLLOW_ZIGZAG ) ? TRUE : FALSE ;
    JDNC_CONNECT3D  tmpCnt3D = m_cFeedDef.m_cConnectDef ;
    if( bZigzag ) 
    {
        tmpCnt3D.m_bConnect3DFlag |= NCDEF_FCONNECT3D_ZIGZAG ;
    }
    else
    {
        tmpCnt3D.m_bConnect3DFlag &= ~NCDEF_FCONNECT3D_ZIGZAG ;
    }
    tmpCnt3D.m_dSafeDist = m_cParam.m_dOverStep ;

	if ( m_cSetupDef.m_cModelTol.m_nMdlFlag & NCDEF_SETUP_SMOOTHCORNER )
	{/*光滑尖角*/
		AllPath.SmoothAllPath ( DriveMdl, m_cSetupDef,m_cPrgDef ) ;
	}

	//恢复马赛克设置
	m_cSetupDef.m_cModelTol.m_nMdlFlag = nOrigMdlFlag;
	m_cSetupDef.m_cTolDef.m_dMaxStep = dOrigMaxStep;
	BOOL bNoMosaic = nOrigMdlFlag & NCDEF_SETUP_NOMOSAIC;
	if(bNoMosaic )
	{
		DriveMdl.ResetCheckMdl() ;
		CheckMdl.ResetCheckMdl() ;
		Mini_DeleteContours( BoundCont ) ;
		if( ! BuildSFinishGen( NewPath, Graph, DriveMdl, CheckMdl, BoundCont ) )
		{//中断生成消除马塞克检查模型
			Mini_DeleteContours( BoundCont ) ;
			return FALSE ;
		}
	}
    Mini_DeleteContours( BoundCont ) ;
	if (glbf_GetProductID() == PRODUCT_ID_DENTFORM)
	{
		SurfNC_SetCurPos(70, 100);
	}		

	if (m_cFeedDef.m_cConnectDef.m_bConnect3DFlag & NCDEF_FCONNECT3D_SPIRAL ) 
	{/*螺旋连接*/
		CSmtCheckMdl * pNoMosaicMdl = NULL ;
		if(bNoMosaic) pNoMosaicMdl = &DriveMdl;
		NcBound_ConnectAllPath( DriveMdl, m_cSetupDef, m_cFeedDef,
							m_cParam.m_dOverStep,AllPath, 
							m_cPrgDef,tmpCnt3D.m_dMaxDist,TFLOAT(25.0*m_cParam.m_dOverStep),
							bZigzag, pNoMosaicMdl) ;

		if(bNoMosaic && !RemoveMosaic(AllPath, DriveMdl))
		{
			m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return FALSE ;
		}
	}
	else
	{
		if(bNoMosaic && !RemoveMosaic(AllPath, DriveMdl))
		{
			m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return FALSE ;
		}
		//先消除马赛克再连刀，否则连刀路径和切削路径之间会有空隙、断开
		//根据行号连接多段刀具路径
		AllPath.ConnectPathByLineNo3D( DriveMdl,m_cSetupDef.m_cTolDef, tmpCnt3D, m_cPrgDef );
	}

	SmartNC_RegCreateConnect3DPath( NULL );

	if( GetMillDir() == 1 )
	{ /*设置顺铣方向*/
		AutoBnd.ReverseDirect();
	}
	//沿指定点和导动线等距不加入边界
	if(m_cParam.m_nFollowType!= NCDEF_FOLLOW_START && m_cParam.m_nFollowType!= NCDEF_FOLLOW_GUIDE)
	{
		POSITION atPos = AutoBnd.m_cAllPath.GetHeadPosition() ;
		while( atPos )
		{
			POSITION atCur = atPos ;
			pTPath = AutoBnd.m_cAllPath.GetNext( atPos ) ;
			if( pTPath->GetCutMode() != MINI_MILL_PATH )
			{
				AutoBnd.m_cAllPath.RemoveAt( atCur ) ;
				delete pTPath ;
				continue ;
			}
		}
		if(bNoMosaic && !RemoveMosaic(AutoBnd, DriveMdl))
		{
			m_nErrorType = JDERROR_GENPATH_ABORT  ;
			return FALSE ;
		}
		MathCAM_AppendBoundPath( AllPath, AutoBnd ) ;
	}

	// 转化路径
	AddPlungeLeadPath( DriveMdl, m_cFeedDef.m_cConnectDef ,AllPath );	
    DriveMdl.DeleteAccChkBox() ;
    DriveMdl.SetBottom( fZOld ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType )
	{
		if(!TransAllPathFrom3AxisTo5Axis( Graph, DriveMdl,AllPath, NewPath ))
		{
			AllPath.ClearAllPath();
			return FALSE;
		}
	}
    if( SurfNC_IsAbort() )
    {
       m_nErrorType = JDERROR_GENPATH_ABORT  ;
       return FALSE ;
    }

    CPathCombine * pPComb = new CPathCombine(NC_WPROCESS_ROUGH) ;
	AllPath.AddToPathCombine( *pPComb ) ;
    NewPath.AddData( 0.0, pPComb  ) ;
	if( m_c5DCtrlDef.m_cToolAxis.m_nAxisType == NCDEF_AXIS_VERT )
    {
        AddPlungeAndRelativeH( &DriveMdl, &NewPath, TRUE ) ;
    }
	return TRUE ;
}

