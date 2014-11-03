#ifndef __SMART_PATHEX_H__
#define __SMART_PATHEX_H__

#include "SmartNC.h"
#include "SmartCheck.h"

////////////////////////////////////////////////////////////////////////////////////////
// ΪCSmtCutPath���������˵�
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

// Ϊ�����뾶ĥ�𲹳�������ĥ��·������ˮƽ���ϵĽ��˵� qqs
void    CalcCutPath2DLeadOut( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						  CSmtCutPath *pPath, CSmtCutPath *&LeadOut, BOOL bCheck );
void    CalcCutPath2DLeadIn( CSmtCheckMdl &CheckMdl, JDNC_TOL &cTol, JDNC_CONNECT3D &cConnect, 
						  CSmtCutPath *pPath, CSmtCutPath *&LeadOut, BOOL bCheck );
// ���ڴּӹ�����

class CRoughLink
{
public:
	JDNC_PRGDEF  m_cPrgDef    ; // �������
	double		 m_dDepth	  ; // �������
	CRoughLink( double dDepth ) ;
	~CRoughLink() ;
	void	InitProg() ;
public:
	// ����·������
	BOOL	AddLinkPath( CSmtCheckMdl* DriveMdl, CPathCombine *pTComb, JDNC_TOL& Tol, double dLastH ) ;
	// ���Combine·������
	BOOL	AddSameLayerLink( CSmtCheckMdl* DriveMdl, CPathCombine *TComb, JDNC_TOL& Tol, double dLastH ) ;
	// �������֮�������
	BOOL	AddNextLayerLink( CSmtCheckMdl* DriveMdl, CPathCombine *pHead, CPathCombine *pNext, JDNC_TOL& Tol, double dLastH ) ;
	// ΪCPathCombine���Ԫ��
	void	InsertBefore( CPathCombine* TComb, CPathEntity* pNew, CPathEntity* pNext ) ;
	void	InsertAfter ( CPathCombine* TComb, CPathEntity* pNew, CPathEntity* pPrev ) ;
	// ��һ��pHead�����Ԫ��,��������
	CPathEntity* AddPathEntity( CPathEntity* pHead, CPathEntity* pPath ) ;
	CSmartSect*  AddSmartSect( CSmartSect* pHead, CSmartSect* pSect ) ;
	// ����ĩ�˵������ĸ��ڵ��CPathPLine3D
	CPathEntity* AddSafeLink( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL &Tol, double dLastH ) ;
	CPathEntity* GetPLineEntity( PNT3D start, PNT3D end, double dH ) ;
	BOOL	AddPCombToHead( CPathCombine *pHead, CPathCombine *TComb ) ;
} ;

///////////////////////////////////////////////////////////////////////////////////
typedef CTypedPtrArray< CPtrArray, CSmartCurveLib* >CCurveLibArr ;
//�������ڲ��ϲ��ӹ�
class CSmtLoopPath
{
public:
	CSmartLoop		*m_pLoop   ;	// �ü���Ļ�,����ʱ����
	CSmtCPathLib     m_cPathLib;	// ��¼�ò��·��
	CSmartCurveLib   m_cCLib   ;	// �ڸ�������Ѱ���µ���
	CSmartLoop	    *m_pPlunge ;	// �������ϲ��������µ���ʹ�������
	JDNC_PRGDEF      m_cPrgDef ;	// ������
	CCurveLibArr     m_cLibArr ;	// �ü���õ�������
	JDNC_PLUNGE		 m_cPlunge ;	// �µ�����
	double           m_dStep   ;
	double			 m_dDepth  ;
	double			 m_dRadius ;
	JDNC_FEED		 m_cFeed   ;	// ���˵�����
	// ��������CPathCombineʹ�õ�����
	BOOL			 m_bRemain ;
	CSmtLoopPath *	 next	   ;	
	CSmtLoopPath *	 prev	   ;	

	BOOL			 m_bAddPath;	// ���·�����
	CSmartLoop	*	 m_pPtLoop ;	// �ȸ߻�
	BOOL			 m_bBoundTol	;
	JDNC_SPEED		 m_cSpeed		;
	TFLOAT			 m_fFeedRatio	;
	double			 m_dBoundTol	;
	BOOL			 m_bUsed		; // ����Ƿ�ʹ��
	BOOL             m_bClosedFlag  ; // ��Ǽӹ����Ƿ�"���"
	CSmtLoopPath( CSmartLoop* pLoop, JDNC_FEED &cFeed, double dDepth, 
					double dRadius, double dStep, BOOL bRemain ) ;
	~CSmtLoopPath() ;
	void RemoveLibArr() ;
public:
	// ���òв�����
	void SetBoundInfo( JDNC_SPEED &cSpeed, TFLOAT fRatio, BOOL bBoundTol, double dBoundTol ) ;
	// �����µ����ߺͻ�
	void SetPlungeInfo( JDNC_PLUNGE &cPlunge ) ;
	// �����߷���
	void ReverseAllCurve() ;
	// �ж������Ƿ����
	BOOL ConnectLine( CSmtCheckMdl *CheckMdl, PNT3D start, PNT3D end, double dDepth, JDNC_TOL& cTol ) ;
	
	// ʹ���������ȵķ�����������
	void ConnectAllCurve( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib& BndCurve, 
						  BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE & cPlunge, double dDepth ) ;
	// �������Ӻ���,���ӽ�ʡ����·��
	BOOL ConnectAllCurve( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib &BndCurve, 
						  BOOL bZigZag, BOOL bPrev, PNT3D prev, JDNC_SETUP &cSetup, double dDepth ) ;
	// �õ���һ������
	CSmartCurve *FindFstCurve( CSmartCurveLib &cCurveLib, PNT3D end, BOOL bZigZag ) ;
	// ��������ӵ�·������
	void AddCurveToPathLib( CSmartCurve *pCurve, CSmtCPathLib &cPathLib, BOOL bDel, int nLevel, JDNC_TOL &cTol ) ;
	// ������ת����·��
	CSmtCutPath * TransfCurveToPath(  CSmartCurve * pCurve , JDNC_TOL &cTol, BOOL IsDel ) ;
	// Ѱ�����������߽�������
	void FindNearCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, CSmartLoop *pPlunge, CSmartCurveLib* pLib, 
						int i, BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, double dDepth, BOOL &bAddClose ) ;
	// �ж��Ƿ������������
	BOOL IsNextCurve( CSmartCurve *pCurve, PNT3D p, int i, CSmartSect *&pNearSect, PNT2D nearpt, double dTol ) ;
	// ����㵽���ߵ���̾���
	double GetPntToCurveDist( PNT2D p, CSmartCurve *pCurve, CSmartSect *&pNearSect, PNT2D nearpt ) ;
	
	// �õ�·����Ķ˵�
	void GetEndPoint( CPathCombine *PComb, BOOL bFlag, PNT3D point ) ;
	void GetEndPoint( CPathEntity *pEntity, BOOL bFlag, PNT3D point ) ;
	CSmtCutPath *CreatePathLine( PNT3D start, PNT3D end, double dDepth ) ;
	// ��������
	CSmtCutPath *CreateAdsorbLine( CSmtCheckMdl *DriveMdl, PNT3D start, PNT3D end, JDNC_TOL &cTol ) ;
	// ����µ�·��,����������ӵ�·������
	void AddPlungeCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int nLevel, CSmartLoop *pPlunge, CSmartCurve *pCurve, 
						 JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, BOOL &bAddClose, BOOL bIsDel, BOOL bLine, BOOL bZigZag, BOOL bLeadOut = FALSE ) ;
	void AddPlungeCurveForCloseCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int nLevel, CSmartLoop *pPlunge, CSmartCurve *pCurve, 
									 JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, BOOL &bAddClose, BOOL bIsDel, BOOL bLine, BOOL bLeadOut ) ;
	// �õ������µ���
	BOOL FindAllPlungePnt( CPointList &allPnt, PNT2D pos, double dMaxDist ) ;
	// ����Բ���µ�·��
	BOOL AddArcForCloseCurve( CSmtCheckMdl *DriveMdl, CSmtCPathLib &cPathLib, CSmartCurve *pCurve, 
							  CPointList &allPnt, JDNC_SETUP &cSetup, BOOL bHead  ) ;
	BOOL IsLeadArcPath( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib ) ;
	CSmartCurve *CreateLeadArc( PNT3D pos, PNT3D tan, PNT3D nor, PNT3D bgn, double dAng, BOOL bHead ) ;

	void AddPlungeCurveForOpenCurve( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, int nLevel,  
								     CSmartCurve *pCurve, JDNC_SETUP &cSetup, BOOL bIsDel, BOOL bZigZag ) ;
	// �жϱպ����߷���
	void AdjustCloseCurveDir( CSmartCurve *pCurve, JDNC_SETUP &cSetup ) ;
	// �ж�ֱ���Ƿ���δ�ӹ��������ཻ
	BOOL LineIntAllCurve( PNT2D p1, PNT2D p2 ) ;
	// ���ֱ�������г�
	CSmtCutPath *GetPlungeLine( CSmtCheckMdl *CheckMdl, PNT2D start, PNT2D end, JDNC_TOL& cTol ) ;
	// ��������г�·��(Բ��״)
	CSmtCutPath *CreatePlungeArc( CSmtCheckMdl *DriveMdl, CSmartCurve *pCurve, PNT2D p, BOOL bHead, JDNC_SETUP &cSetup ) ; 
	// �õ�CSmtCPathLib����ĩ��
	BOOL GetSmtCPathLibEndPoint( CSmtCPathLib &tmpLib, PNT3D p, BOOL bHead ) ;
	// Ϊ�պ���������µ�·��
	BOOL AddPlungeForClosedCurve( CSmtCPathLib &cPathLib, CSmartCurve *pCurve, CSmartLoop *pCurr, JDNC_PLUNGE &cPlunge ) ;
	// ��·����ӵ�NewPath��
	void AddPathToGroup( CSmtCheckMdl *DriveMdl, CPathGroup &NewPath ) ;

	/*���¼�����������Բ������*/
	BOOL ConnectAllCurve_ArcLink( CSmtCheckMdl *CheckMdl, CSmartLoop *pPlunge, CSmartCurveLib &BndCurve, 
		                          BOOL bZigZag, BOOL bPrev, PNT3D prev, JDNC_SETUP &cSetup, double dDepth ) ;
	// Ѱ�����������߽�������
	CSmartCurve* FindNearCurve_ArcLink( CSmtCheckMdl *CheckMdl, CSmtCPathLib &cPathLib, CSmartLoop *pPlunge, CSmartCurveLib* pLib, CSmartCurve * &pPrev,
						                int i, BOOL bZigZag, JDNC_SETUP &cSetup, JDNC_PLUNGE &cPlunge, double dDepth, BOOL &bAddClose ) ;
	// �ж��Ƿ������������
	BOOL IsNextCurve_ArcLink( CSmartCurve *pCurve, CSmartCurve*pPrev, PNT3D p, int i, CSmartSect *&pNearSect, PNT2D nearpt, double dTol ) ;
	CSmtCutPath * CreateAdsobSpline( CSmtCheckMdl *Drivemdl, PNT3D IntPnt[2], PNT3D TanDir[2], JDNC_TOL &cTol ) ;
	CSmtCutPath * GetLeadoutSplinePath( CSmtCheckMdl *CheckMdl, JDNC_SETUP & cSetup, CSmartCurve * pCurve, CPointList &allPnt ) ;
	// ���¶�����˵�·���ĸ߶�
	BOOL       RedefineLeadPathHeight(CSmtCheckMdl *Drivemdl, CSmtCutPath * pPath, BOOL bLeadIn ) ;
	// �����һ�����·�����˵�·��̧��0.05
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
// ���º���Ϊ�ּӹ��Ͳ��ϲ��ӹ�����
// �õ���������߽绷
CSmartLoop *MathCam_CreateBigLoop( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, int nCount, double r ) ;  
CSmartLoop *MathCam_CreateBigLoopAndBox( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, BOX3D &dMaxBox, int nCount, double r ) ;
// �����ŵ�����
void		AddLoopToLoopArr( CSmtLoopArr& LoopArr, CSmartLoop* pHead, int Layer, double dMinArea, BOOL bDel = TRUE ) ;
// ��·��ת��
void		TransfPCombineToCPath( CPathCombine &PComb, CSmtCPathLib &cPathLib, double dFeedRate, double h, JDNC_TOL& Tol ) ;
void		TransfPCombineToSmartCurve( CPathCombine &PComb, CPtrList &SmtCurList, JDNC_TOL& Tol ) ;
void		TransfSmartCurveToCPath( CSmartCurve *pCurve, CSmtCPathLib &cPathLib, double h, JDNC_TOL &Tol ) ;
void		TransfAllLoopToCPath( CSmartLoop *AllBnd, CSmtCPathLib &AllPath, double h, JDNC_TOL &Tol ) ;
// ��ǰ�����ϰѵ���pLast���,Ȼ��ͱ߽绷AllLoop�󽻵õ���Ч���߶ηŵ�CSmartCurveLib��
BOOL		SubtractContour( CSmartLoop *pCurr, CSmartLoop *pLast, CSmartLoop *AllLoop, CSmartCurveLib& CurLib, double dMinLen, BOOL bBtw = FALSE ) ;
// ���������޼���һ������������������������߱�����
BOOL        TrimLoopByTwoContours( CSmartLoop *pLoop, CSmartLoop *pContour1, CSmartLoop *pContour2, CSmartCurveLib& CurLib ) ;
// ���ü��õ������߷ŵ�LPathArr��
BOOL		AddCurveToLPath( CSmtLPathArr& LPathArr, CSmartCurveLib& CurLib, int nLevel, BOOL bSetClosed = TRUE ) ;
//�жϼӹ�·���Ƿ�Ϊ���·��
BOOL		SetPathClosedFlag( CSmtLoopPath *pLPath, CSmartCurveLib *pLib ) ;
// ʹ�ñ߽�ConHead����CSmartCurve
int			IsPointOnContours( CSmartLoop* ConHead, PNT2D Point ) ;
// ��CutPath��ӵ�AllPath��
void		AddCutPathToAllPath( CPathGroup *NewPath, CSmtCPathLib &AllPath, CSmtCheckMdl *DriveMdl, 
								 double dMaxDist, CSmtCPathLib &cPathLib, JDNC_TOL &cTol, BOOL bAbsorb ) ;
// �õ�һ��·����ĩ��
BOOL		GetAllPathEndPoint( CSmtCPathLib &AllPath, PNT3D end ) ;
//////////////////////////////////////////////////////////////////////////
void		DeleteInvalidLoop( CSmartLoop *&pHead, JDNC_TOL &cTol, double dTol ) ;
// �ж�һ�����������Ƿ���Ч
BOOL		IsRemainLoopValid( CSmartLoop *pReLoop, JDNC_TOL &cTol, double dTol ) ;
// ��������볬��dTol����TRUE 
BOOL		BlankSectDistLoop( CSmartLoop *pReLoop, CSmartSect *pSect, JDNC_TOL &cTol, double dTol ) ;
BOOL		UnBlankSectDistLoop( CSmartLoop *pReloop, CSmartSect *pSect, JDNC_TOL &cTol, double dTol  ) ;
// �ú���Ϊ���ϲ��ӹ��ĺ���,Ϊ�в������������
void		AddToPathCombine( CSmtCPathLib &AllPath, CPathCombine *PComb, double dFeedRate ) ;
// ����Ϊ�߽绷Blank
void		SetContourBlank( CSmartLoop *Contour ) ;
BOOL		HasBlankSect( CSmartLoop *AllLoop ) ;
//////////////////////////////////////////////////////////////////////////////////////////
//ɾ������ä����
void		DeleteBlindLoop( CSmartLoop *&pHead, double dRadius ) ;
BOOL		IsBlindContour( CSmartLoop *Contour, double dRadius ) ;
///////////////////////////////////////////////////////////////////////////////////////////
// ��������������ھ�������������blank��
BOOL		SetContourHeadAtBlank(	CSmartLoop *AllLoop,	// <I> ���뻻
									double	 dist	,		// <I> ��̳���
									PNT3D InOutPnt ) ;		// <IO> ������
									
// �ж�CSmtCutPath�Ƿ�ͬLine�ཻ
BOOL		AllPathIntLine( CSmtCPathLib &AllPath	,	// <I> ����·�� 
							PNT3D start, PNT3D end	,	// <I> ����ֱ��
							BOOL bIncludeHead )		;	// <I> �Ƿ������ĩ��
// �ж�ֱ���Ƿ�ͬSmtLoopArr�е�loop�ཻ
BOOL		AllLoopIntLine( CSmartLoop *AllCont		,	// <I> ��������
							PNT3D start, PNT3D end );	// <I> ����ֱ��
// �ж�ֱ���Ƿ�ͬsmartcurve�ཻ
BOOL		AllCurveIntLine(	CPtrList &AllCurve		,	// <I> ������������
								PNT3D start, PNT3D end );	// <I> ����ֱ��

// �ж�ֱ���Ƿ�ͬCPathEntity�ཻ
// �����������������ж�����ӹ��м�������Ƿ���·���ཻ(��ĩ���ཻ����)
// ֻҪȷ���м䲿�ֲ���·���ཻ���ɣ��������е�dTol������Ϊ-0.0001
BOOL		AllPathEntIntLine(	CPathEntity *pEnt		,	// <I> ����·��
								PNT3D start, PNT3D end	,	// <I> ����ֱ��
								int nMethodType			,	// <I> ����ӹ�����
								BOOL bNext				) ; // <I> �������
BOOL		AllPathEntIntCurve( CPathEntity *pEnt		,	// <I> ����·��
								CSmartCurve &tmpCurve	,	// <I> ����ֱ��
								int nMethodType			,	// <I> ����ӹ�����
								BOOL bNext				) ; // <I> �������
// �жϵ���CSmartCurv�Ƿ���ֱ���ཻ
BOOL		SmartCurveIntLine(	CSmartCurve *pCurve			,	// <I> ��������
								PNT3D start, PNT3D end	)	;	// <I> ����ֱ��
// �������ߣ����ػ�
CSmartLoop *FormAllCurveToContour( CSmartCurveLib &AllLib ) ;	// <I> ��������
// ����һ���ṹ����ƽ̹��ӹ�
class CPlaneLoop
{
public:
	CSmartLoop *m_pPlaneLoop ; // ƽ�滷
	double	    m_dHeight	 ; // �߶�
	CSmartLoop *m_pOrgLoop	 ; // ԭʼ��
	CSmartLoop *m_pRoughLoop ; // ԭʼë����
	BOOL		m_bValid	 ; // ��Ч��
	CSmartLoop *m_pRemainLoop; // ԭʼ�в���
	
public:
    CSmtLoopArr m_PreLoopArr ; // ����������Ĵּӹ���
    BOOL m_bSearch3 ;          // �����β��ұ��

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
// ���ƽ�滷
void	ClearAllPlaneLoop( CPlaneList *pList ) ;
BOOL	GetAllPlaneHeight( CPlaneList *pList, double *&dLayer, int &nLayer ) ;
BOOL	GetAllBndContHeight( CSmartLoop *BndCont, double *&dLayer, int &nLayer ) ;
// ��nCnt1��nCnt2�ϲ�,�ŵ�dAlldZ��,bFlag��TRUEΪdZ1��,FALSEΪdZ2��
void   CombineAllHeight( double *dZ1, int nCnt1, double *dZ2, int nCnt2, double *&dAlldZ, int &nCnt, BOOL *&bFlag ) ;
// ����֮���������
double MinDistBetweenLoop( CSmartLoop *pLoop1, CSmartLoop *pLoop2 ) ;
double MinDistBetweenLoopAndSect( CSmartLoop *pLoop, CSmartSect *pSect ) ;
BOOL   IsBottomPlaneValid( CSmartLoop *pCutLoop, CSmartLoop *PlaneLoop, double dTol, double dRatio ) ;
double MinDistPntAndContour( CSmartLoop *Contour, PNT2D pt ) ;
double MinDistPntAndAllCurve( CPtrList &AllCurve, PNT2D pt ) ;

// ��·�����и߶Ȳü�
void	TrimAllPathByHeight( CSmtCPathLib &AllPath, double dTop, double dBot ) ;
BOOL	TrimPathByZValue( CSmtCutPath *pPath, double dZValue ) ;
BOOL	LineIntLine( CSmtCutPath *pPath, CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double dZValue ) ;
CSmtCutPointEx *GetMiddlePntEx( CSmtCutPointEx *pStart, CSmtCutPointEx *pEnd, double t ) ;
// ����һ����,�����ϲ���
typedef struct _LAYER_FLAG LayerFlag ;
struct _LAYER_FLAG
{
	double m_dCurZ ;
	BOOL   m_bFlag ;
} ;
///////////////////////////////////////////////////////////////////////////////////////////
/// ����ͶӰ�����·���ü�
class  CNcSmartPath : public CSmtCPathLib
{
public :
	CNcSmartPath() ;
	virtual ~CNcSmartPath() ;
public :
	// ���� Z �߶��޼�·��
    int			 TrimByZValue0 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& TopLib,     /*����·��*/
		                         DOUBLE  ZValue ) ;        /*�޼��߶�*/      
								    
	// ����һ��ķֲ�						 
	CSmtCutPath* TrimPath0     ( CSmtCPathLib& TempLib,
							     CSmtCutPath* pPath, 
							     DOUBLE ZValue ) ;

	// ���ڿ����ֲ�
	int          TrimByZValue1 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& LowLib,     /*�ײ�·��*/
		                         DOUBLE  ZValue,           /*�޼��߶�*/
								 DOUBLE  h) ; 			   /*·��̧��ĸ߶�*/
								               								 
							
	void         TrimPath1     ( CSmtCutPath* pPath, 
							     DOUBLE ZValue,
								 DOUBLE h,
								 CSmtCPathLib& LowLib ) ;



	int			 TrimByZValue2 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& TopLib,     /*����·��*/
		                         DOUBLE  ZValue,           /*�޼��߶�*/
								 int nCutMode ) ;        
								    
	// ����һ��ķֲ�						 
	CSmtCutPath* TrimPath2     ( CSmtCPathLib& TempLib,
							     CSmtCutPath* pPath, 
							     DOUBLE ZValue,
								 int nCutMode ) ;

	// ���ڿ����ֲ�
	int          TrimByZValue3 ( CSmtCPathLib& OldLib, 
							     CSmtCPathLib& LowLib,     /*��·��*/
		                         DOUBLE  ZValue,           /*�޼��߶�*/
								 DOUBLE  h) ;			   /*·��̧��ĸ߶�*/
								              
							
	void         TrimPath3     ( CSmtCutPath* pPath, 
							     DOUBLE ZValue,
								 DOUBLE h,
								 CSmtCPathLib& LowLib ) ;
	
	// ·����������̧��
	CSmtCutPath* CopyPath     ( CSmtCutPath* pPath, 
								DOUBLE h ) ;

	int			 RefineSmtPath( CSmtCutPath* pPath ) ;
};

/////////////////////////////////////////////////////////////
//���º���Ϊ�ּӹ��Ͳ��ϲ��ӹ�ʹ��
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
// ���ȸ�·�����Ƿ�ȫ
BOOL CheckCutLoop ( CSmtLoopArr *pTmpArr, int& nCount ) ;
// ���ͻ���
CSmartLoop* LoopLoopInt( CSmartLoop* pHead, CSmartLoop* pLoop ) ;
// �����ߵľ���
double  MinDistBetweenCurve( CSmartCurve * pCurve1, CSmartCurve * pCurve2 ) ;
// �������ߵ���С����
double  MinDistBetweenLoopAndCurve( CSmartLoop * pLoop, CSmartCurve * pCurve ) ;
// �õ���������߽绷
CSmartLoop *CreateBigLoop( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, int nCount, double r ) ;  
CSmartLoop *CreateBigLoopAndBox( CSmartLoop **pDriveLoop, CSmartLoop *AllLoop, 
								 BOX3D &dMaxBox, int nCount, double r ) ;

// �Եõ��ĵȸ߻����д���,�õ���ϵĵȸ߻���
CSmartLoop* ManageLoop ( CSmartLoop* pHead, CSmartTool* pTool, CSmartLoop* AllLoop, 
						 BOOL bIsRoughCast, BOOL bBtwLayer = FALSE ) ;
// �ж�CPathCombine�Ŀɿ���
BOOL IsErrorPComb( CPathCombine *PComb ) ;
// �ȸ�����
int     SortHeight( const void *arg1, const void *arg2 ) ;
// �ȸߵĲ��ϲ��ӹ��ķֲ�
void GetUsedLayerZ( double *dSurfZ, int nNum, double *dZ, int& nCount, 
				    double dMinStepZ, double dBot, double dTop ) ;
// �����鷴��
void ReverseAllContour( CSmartLoop *&pHead ) ;
// ����pPath�ĸ߶�
void SetCutPathZValue( CSmtCutPath *pPath, TFLOAT fZValue ) ;
// ɾ��pPath�б�fZValue���͵ĵ�,����pPath����
void TrimCutPathZValue( CSmtCutPath *&pPath, TFLOAT fZValue ) ;
// ���CutPath��pHead
CSmtCutPath *AddSmtCutPath( CSmtCutPath *PathHead, CSmtCutPath *NewPath ) ;
/////////////////////////////////////////////////////////////
//���º���Ϊ·���ռ�任ʹ��
/////////////////////////////////////////////////////////////
int  GetLocalFrame( CPathArc3D* pArc,  RFRAME& locMtx );
void RotatePComb( CPathCombine* PComb, PNT3D ptbase, VEC3D dir, double angle ) ;
void MirrorPComb( CPathCombine* PComb, PNT3D ptbase, VEC3D dir ) ;
void ScalePComb( CPathCombine* PComb, PNT3D ptbase, VEC3D scale ) ;
void TransfPath( CPathGroup& NewPath, JDNC_SPACETRAN cSpaceDef ) ;
// ·���任
void TransLocalGroup( CPathGroup &NewPath, RFRAME *Local ) ;
//���㻷��Ԫ�صĸ���
int GetLoopCount( CSmartLoop * pLoop ) ;

////����Ϊ���ӹ�ʹ��
class CBtwLPath
{// ������ּӹ�����Ϣ
public:
	CSmtLoopPath *m_pLPath		;	// ���ӹ���\����ṹ
	CSmtLoopArr m_UnderLoops	;	// ��ӹ���������ص��²�ּӹ���

public:
	int n_UnderDepth		;	// �²�ּӹ������ڵĲ��
	BOOL m_bSearch3				;	// �����β��ұ��

public:
	CBtwLPath( CSmtLoopPath *pLPath ) ;
	~CBtwLPath() ;

public:
	void AddUnderLoop(CSmartLoop * pLoop ) ;
};
typedef CTypedPtrArray< CPtrArray, CBtwLPath* > CBtwLPathArr ;

typedef CTypedPtrList< CPtrList, CSmartCurve * > CurveList ;
class CBtwCurve
{// �����侫�ӹ�������Ϣ
public:
    CurveList    m_CurveList ;    // ����ӹ�����
    CSmtLoopArr  m_UnderLoops ;   // ��ӹ�������صľ��ӹ���

public:
    int      m_nUnderDepth ;  // �²�ּӹ������ڵĲ��
    
public :
    CBtwCurve(){ m_nUnderDepth = -1 ; } ;
    ~CBtwCurve(){} ;
  
public:
    void AddUnderLoop(CSmartLoop * pLoop ) ;
};
typedef CTypedPtrArray< CPtrArray, CBtwCurve* > CBtwCurveArr ;
typedef CTypedPtrArray< CPtrArray, CSmartCurve*> CSmtCurveArr ;

//�ּӹ�ʹ��
//�жϰ�Χ���Ƿ��ཻ
int ChkBox3DInt(BOX3D* box1, BOX3D* box2, double tol) ;
//�ж��������Ƿ��ཻ
BOOL Is2ContoursInt( CSmartLoop * pLoop1, CSmartLoop * pLoop2, double dTol );
//�ж��������Ƿ���ͬһ������
BOOL IsInSameArea( CSmartLoop * pLoop1, CSmartLoop * pLoop2, double dTol ) ;
//��pNewLPath���߶�ֵ��������pLPathHead��
void InsertLPath( CSmtLoopPath * pNewLPath, CSmtLoopPath * &pLPathHead ) ;
//�жϻ�pLoop�͵ȸ߻�����pDriveLoop�Ƿ��ཻ
BOOL IsRelatedLoop( CSmartLoop *pLoop, CSmartLoop *pDriveLoop, double dTol ) ;
//����ֱ��Բ������������
CSmartCurve *  CreateConnectCurve( CSmartCurve * pCurve, CSmartCurve * pPrev, double dDist ) ;
//�����ߵ���㰴���߷���ƫ��dDist����
BOOL   ResetCurveStart( CSmartCurve * pCurve, double dDist ) ;
//�������߷���
void   AdjustCurveDir( CSmtCPathLib & cPathLib, CSmartCurve *pPrev, CSmartCurve *pCurve ) ;
//���ҵ�from������pCurve�������to
BOOL   FindNearestPnt( CSmartCurve *pCurve, PNT3D from, PNT3D to, double dTol, BOOL bFromHead ) ;
#endif // __NC_SMARTPATH_H__