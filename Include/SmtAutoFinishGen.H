#ifndef __SMART_AUTOFINISH_GEN_H__
#define __SMART_AUTOFINISH_GEN_H__

// ����ƽ���̼ӹ�·��
#include "SmtPathGen.H"
class CSmtCheckMdl ;
class CSmartAutoFinishGen;
#define MIN_FLT		1.0e-6
// ����ƽ�н��߾��ӹ�·�����߳�����
typedef struct CreateParallelPathThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	CSmtCPathLib *TmpPath ;
	LPVOID *	 AllPath ;
	CSmtCheckMdl *DriveMdl;
	JDNC_PRGDEF   PrgDef  ;
    BOOL         bRandMode;
	CSmartAutoFinishGen *pFinishGen;
}CPPT_DATA;

typedef struct TransFrom3AxisTo5AxisThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	CSmtCheckMdl *DriveMdl;
	CSmtCPathLib *OrgPath;
	LPVOID      *NewPath;
	JDNC_PRGDEF ProgDef;
	CNc5DAxisCtrl *pAxisCtrl;
	double dCur;
	CSmartPathGen *pPathGen;
}TF3X5XT_DATA;

// ƽ�н����̺߳���
static UINT MathCAM_CreateParallelPathSubProc(LPVOID lpParam);
// ��������̺߳���
static UINT MathCAM_CreateOrgEmitPathSubProc(LPVOID lpParam);
// ����ת�����̺߳���
static UINT MathCAM_TransFrom3AxisTo5AxisSubProc(LPVOID lpParam);
// �õ�����ķ�ʸ����
int  MathCAM_GetSurfNormalDir(CGeoTrmSurf* pSurf, RFRAME& cRFrame);
// �õ����ʵ�UW����
double MathCAM_UWParam( double uInit );
//////////////////////////////////////////////////////////////////////////

class DLL_EXPORT CSmartAutoFinishGen : public CSmartPathGen
{
public :
    CSmartAutoFinishGen() ;
    virtual ~CSmartAutoFinishGen() ;
public:
    CSmartGraphic *m_pSmtGraph  ; // �ӹ�ͼ�α߽� 
    BOOL   m_bTrimByZSize       ; // ʹ�ø߶Ȳü�
    BOOL BuildSFinishGen( CPathGroup&   NewPath  ,  /*���·��*/
                          CSmartGraphic& Graph   , 
                          CSmtCheckMdl& DriveMdl , 
                          CSmtCheckMdl& CheckMdl ,
                          CSmartLoop*&   BoundCont,
						  double dAngle = 0. );

    // ���ݼӹ��淶Χ���豣���淶Χ
    virtual void SetCheckFltBoxByDriveMdl( CSmtCheckMdl& DriveMdl ,
                                           CSmtCheckMdl& CheckMdl );
	friend UINT MathCAM_CreateParallelPathSubProc(LPVOID lpParam);
private :
    CPathGroup*   m_pDebugPath  ;

public :
    // ������������ļӹ�·��
    BOOL GenerateAllAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath ,
                                 JDNC_PRGDEF&  PrgDef );
    // ����ƽ̹����ļӹ�·��
    BOOL GenerateFlatAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath ,
                                 JDNC_PRGDEF&  PrgDef );
    // ���㶸������ļӹ�·��
    BOOL GenerateSteepAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath  ,
                                 JDNC_PRGDEF&  PrgDef );
    // ����ƽ������ļӹ�·��
    BOOL GeneratePlaneAreaCutPath( CSmtCheckMdl& DriveMdl, 
                                 CSmartLoop*   AllCont ,
                                 CSmtCPathLib& AllPath ,
                                 JDNC_PRGDEF&  PrgDef );
public :
    // ����ƽ�н���·��
    int  CreateParallelPath( CSmtCheckMdl& DriveMdl , 
                             DOUBLE  LineAngle      ,
                             CSmartLoop* AllCont    , 
                             CSmtCPathLib& AllPath  ,
                             JDNC_PRGDEF&  PrgDef );
    // ����3D·������ƽ�н���·��
    int  CreateParallel3DPath( CSmtCheckMdl& DriveMdl , 
                               DOUBLE  LineAngle      ,
                               CSmartLoop* AllCont    , 
                               CSmtCPathLib& AllPath  ,
                               JDNC_PRGDEF&  PrgDef   );
	// ���㵼����3D·������ƽ�н���·��
	int  CreateGuideParallel3DPath( CSmtCheckMdl& DriveMdl , 
                               DOUBLE  LineAngle      ,
                               CSmartLoop* AllCont    , 
                               CSmtCPathLib& AllPath  ,
                               JDNC_PRGDEF&  PrgDef   );

    // ����ƽ�滷�ƵȾ�·��
    int  CreateFollowPath ( CSmtCheckMdl& DriveMdl , 
                            JDNC_FOLLOW&  FollowDef,
                            CSmartLoop* AllCont    , 
                            CSmtCPathLib& AllPath  ,
                            JDNC_PRGDEF&  PrgDef   );
    // ���㾶�����ӹ�·��
    int  CreateSEmitPath ( CSmtCheckMdl& DriveMdl, 
                           CSmartLoop* AllCont   , 
                           CSmtCPathLib& AllPath ,
                           JDNC_PRGDEF&  PrgDef );
    // ����3D���ƵȾ�·��
    int  Create3DStepPath ( CSmtCheckMdl& DriveMdl ,
                            JDNC_FOLLOW&  FollowDef,
                            CSmartLoop* AllCont    ,
                            CSmtCPathLib& AllPath  ,
                            JDNC_PRGDEF&  PrgDef  );
    // ����ȸ߼ӹ�·��
    int  CreateHeightPath ( CSmtCheckMdl& DriveMdl , 
                            CSmartLoop* AllCont    , 
							int 		ValidMode  ,
                            BOOL        IsCutPlane , 
                            DOUBLE      SlopeAngle ,
                            CSmtCPathLib& AllPath  ,
                            JDNC_PRGDEF&  PrgDef  );
    // �����������߼ӹ�·��
    int  CreateUWLinePath ( CSmtCheckMdl& DriveMdl, 
                            CSmartLoop* AllCont, 
                            CSmtCPathLib& AllPath,
                            JDNC_PRGDEF&  PrgDef );
    // �������޶�ʾ��ӹ�·��
    int  CreateMPencilPath ( CSmtCheckMdl& DriveMdl, 
                             CSmartLoop* AllCont   , 
                             CSmtCPathLib& AllPath,
                             JDNC_PRGDEF&  PrgDef );
public :
    // ����ƽ̹�ӹ���߽�����
    BOOL CreateFlatAreas( CSmtCheckMdl& DriveMdl,     // �ӹ�ģ��
                          CSmartLoop*   AllCont ,     // ����Χ
                          DOUBLE SlopeAngle     ,     // �����Ƕ�, if SlopeAngle < 0.0 , Ϊ��������, if SlopeAngle == 0.0, Ϊƽ������, ����Ϊƽ̹����
                          CSmartLoop*& FlatAreaBnd  , // ƽ̹����߽�
                          DOUBLE     UsedBndTol ,     // �߽�ƫ���� 
                          JDNC_PRGDEF&  PrgDef,
						  double dStep,               // ·�����
						  BOOL bFollow,               // ƽ̹�����Ƿ���
						  CSmartLoop*& SteepAreaBnd , // ������߽� qqs��xiejunxian 2013.04.25
						  DOUBLE dExtLen = 0.0);      // �ص�������� qqs��xiejunxian 2013.04.25
    // ����ƽ�н��߷����ӹ��߽�����
    BOOL CreateXParallelAreas( CSmtCheckMdl& DriveMdl,// �ӹ�ģ��
                               CSmartLoop*   AllCont ,// ����Χ 
                               DOUBLE SlopeAngle     ,// �����Ƕ�, 5 ~ 85 ��
							   DOUBLE dOverLapDist   ,// �ص��������
                               DOUBLE LineAngle      ,// ���нǶ�
                               CSmartLoop* AreaBnd[2],// [0], ƽ̹����߽�,[1]��������߽�
                               DOUBLE  UsedBndTol    ,// �߽�ƫ���� 
                               JDNC_PRGDEF&  PrgDef );
    // ����Ƕȷ����ӹ���߽�����
    BOOL CreateMixtureAreas( CSmtCheckMdl& DriveMdl ,// �ӹ�ģ��
                             CSmartLoop*   AllCont  ,// ����Χ
                             DOUBLE SlopeAngle,      // �����Ƕ�, 5 ~ 85 ��
							 DOUBLE dOverLapDist,	 // �ص��������
							 BOOL	bFollow	,		 // ƽ̹�����Ƿ���
							 double	dStep		 ,	 // ·�����
                             CSmartLoop* AreaBnd[2], // [0], ƽ̹����߽磬[1]��������߽�
                             int    AreaType       , // ��������
                             JDNC_PRGDEF&  PrgDef );
	// ���ɼӹ���������
	BOOL CreateNcAreas ( CSmtCheckMdl	& DriveMdl,	// �ӹ�ģ��
						 CSmartLoop		*& AllCont );	// ����Χ

	void KeepSharpEdges(CSmtCPathLib & AllPath,CSmtCheckMdl & DriveMdl);

	//�ȸ�·�����죨����������������죩
	void ExtendFHeightPath(CSmtCPathLib & AllPath, CSmtCheckMdl& DriveMdl);

	//�������·��
	BOOL PathSplFit(CSmtCPathLib & AllPath);
	BOOL PathSplFit(CSmtCPathLib & AllPath, CPathCombine & PathComb);

protected :
    // ���ݱ߽����ù�������ķ�Χ
    void SetCheckFltBoxByContours( CSmartLoop* AllCont, CSmtCheckMdl& DriveMdl, CSmtCheckMdl& CheckMdl ) ;
    // ���ɵ����ӹ�����
    void CreateGuideCurve( CSmartGraphic& Graph, CSmtCheckMdl& DriveMdl, 
                           CSmartLoop*  AllCont, CSmtCPathLib& GuideLib ) ;
    int  CreateOneParallel3DPath( CSmtCheckMdl& DriveMdl, 
                                  CSmartLoop&   BndCont , 
                                  CSmtCPathLib& AllPath ,
                                  JDNC_PRGDEF&  PrgDef  );
    // ���������ޱ�·��
    BOOL CreateBoundPath( CSmtCheckMdl& DriveMdl, CSmartLoop* AllCont, CSmtCPathLib& BndPath ) ;
private:
	// ���ɳ�ʼ�Ľ���·��
	void GenOrginalPath(CSmartLoop *AreaCont, CSmtCPathLib &OrgPath);
	// ƽ�н����̺߳���
	BOOL CreateParallelPathSubProc(int nAtCore, int nCoreNum, CSmtCPathLib &TmpPath, LPVOID* AllPath, 
		                           CSmtCheckMdl &DriveMdl, BOOL RandMode , JDNC_PRGDEF &PrgDef);


	// ����ƽ�н��ߵ��̺߳Ͷ��̵߳ĺ���
	void SingleThreadCPPEx(CSmtCheckMdl& DriveMdl, CSmartLoop *AreaCont,
						   CSmtCPathLib &AllPath,double * aY,int nY,JDNC_PRGDEF &PrgDef);		// ���̼߳���ƽ�н���·��
protected:
	// ��������߽�����
	CSmartLoop* GenSurfBorder(CSmartGraphic *pSmtGraph, CSmtCheckMdl *pDriveMdl, double dAngle);
	// �����ر߽�������·��
	int CreatePathhByBound(CSmartGraphic &Graph, CSmtCheckMdl &DriveMdl, 
						   JDNC_LINEAR& FLinear, JDNC_SETUP& Setup, 
						   DOUBLE dAngle, CSmtCPathLib& AllPath);
};
///////////////////////////////////////////
// CSmartFLinearGen: ����ƽ�н��߾����
//
class DLL_EXPORT CSmartFLinearGen : public CSmartAutoFinishGen
{
public :
	CSmartFLinearGen() ;
	virtual ~CSmartFLinearGen() ;
public:
	JDNC_LINEAR  m_cParam    ;  // ��̲���
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*���·��*/
					            CSmartGraphic& Graph     ) ; /*���ͼ��*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
};

///////////////////////////////////////////
// CNCSFinishBPath: �ȸ��������澫���
//
class DLL_EXPORT CSmartFHeightGen : public CSmartAutoFinishGen
{
public :
	CSmartFHeightGen() ;
	virtual ~CSmartFHeightGen() ;
public :
	JDNC_HEIGHT		m_cParam ; 

public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*���·��*/
					            CSmartGraphic& Graph     ) ; /*���ͼ��*/
	virtual BOOL IsSupportProgress() { return TRUE ; }

};
///////////////////////////////////////////
// CNCSFinishAPath: ����ƽ�н��߾����
//
class DLL_EXPORT CSmartMixtureGen : public CSmartAutoFinishGen
{
public :
	CSmartMixtureGen() ;
	virtual ~CSmartMixtureGen() ;
public:
	JDNC_MIXTURE  m_cParam    ;  // ��̲���
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*���·��*/
					            CSmartGraphic& Graph     ) ; /*���ͼ��*/
	int  GenToolPath(CSmartGraphic& Graph, CMachineMdl& MachMdl, CSmartLoop* AllLoop, CPathGroup& NewPath) ;
	virtual BOOL IsSupportProgress() { return TRUE ; }

};

///////////////////////////////////////////
// CSmartFEmitGen: ����������澫���
//
class CSmartFEmiterGen;
// ���㾶�����ԭʼ�ӹ�·�����߳�����
typedef struct CreateOrgEmitPathThreadData  
{
	int nAtCore;
	int nCoreNum;
	double dRadius;
	BOOL DefNull;
	CSmtCheckMdl *pDriveMdl;
	JDNC_PRGDEF pTmpPrg;
	CSmartLoop* BndCont;
	LPVOID* AllPath;
	CSmartFEmiterGen *pFEmiterGen;
}COEPT_DATA;

class DLL_EXPORT CSmartFEmiterGen : public CSmartAutoFinishGen
{
public :
	CSmartFEmiterGen() ;
	virtual ~CSmartFEmiterGen() ;
public:
	JDNC_EMIT  m_cParam    ;  // ��̲���
    DOUBLE     m_dMaxRadius;  // ���뾶
public :
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*���·��*/
					            CSmartGraphic& Graph     ) ; /*���ͼ��*/
	virtual BOOL IsSupportProgress() { return TRUE ; }

	friend UINT MathCAM_TransFrom3AxisTo5AxisSubProc(LPVOID lpParam);
public :
	// ��������
	CSmartLoop* ResortContour(CSmartLoop* AllLoop );
	// ���ɵ���·��
protected :
	// ����ԭʼ�ӹ�·��
	int CreateOrgEmitPath( CSmtCheckMdl& DriveMdl, /*����ģ��*/ 
		                 FPNT3D MaxBox[2]     , /*��Χ��  */
                        CSmartLoop* BndCont   , /*�߽�ü�  */
                        BOOL   DefNull       , /*������Ч�� */
                        CSmtCPathLib&  AllPath); /*ԭʼ·�� */
	// ���㶵�ߵļӹ�·��
	int CreateEmitPathBnd( CSmtCheckMdl& DriveMdl, /*����ģ��*/ 
		                 FPNT3D MaxBox[2]     , /*��Χ��  */
                        CSmartLoop* BndCont   , /*�߽�ü�  */
                        CSmtCPathLib&  BndPath); /*����·�� */
	// ���ɵ���·�����Ӻ���
	BOOL GeneratePathExSubFunc(CSmtCheckMdl &DriveMdl, CSmartLoop *AllLoop, CSmtCPathLib &BndPath, 
		CSmtCPathLib &AllPath);
public:
	// ����ԭʼ�ӹ�·����������
	int CreateOrgEmitPathSubProc(int nAtCore, int nCoreNum, double dRadius, BOOL DefNull, 
		CSmtCheckMdl &DriveMdl, JDNC_PRGDEF &TmpPrg, 
		CSmartLoop* BndCont, LPVOID* AllPath);
};
///////////////////////////////////////////
// CSmartFUWLineGen: �����������澫���
// 
struct CCombSurf;
typedef CTypedPtrArray<CPtrArray, CCombSurf*>	CCombSurfArray;

class DLL_EXPORT CSmartFUWLineGen : public CSmartAutoFinishGen
{
public :
	CSmartFUWLineGen() ;
	virtual ~CSmartFUWLineGen() ;
public :
	JDNC_UWLINE m_cParam ;
public :
	// ���ɵ���·��
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*���·��*/
					            CSmartGraphic& Graph     ) ; /*���ͼ��*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
	
	BOOL Create3AxUWPath( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, CSmartLoop *AllLoop,
						  RFRAME *dNcMtx, BOOL bCheck, double dCur[4], CPathGroup &NewPath ) ;

	// �������㷨�����������˹����溯���͹���������·�����㷨 qqs 2013.12.17
	BOOL Create3AxUWPathNew( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, CSmartLoop *AllLoop,
		RFRAME *dNcMtx, BOOL bCheck, double dCur[4], CPathGroup &NewPath ) ;
	// Create3AxUWPath���Ӻ���
	BOOL Create3AxUWPathSubFunc(CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmtCPathLib &AllPath, C3DSurfArray &AllSurf, 
		RFRAME *dNcMtx, double dCur[4], CSmartLoop *AllLoop, BOOL bCheck);
	BOOL Create5AxUWPath( CSmtCheckMdl &DriveMdl, CSmtCheckMdl &CheckMdl, CSmartGraphic &Graph, 
						  CSmartLoop *AllLoop, RFRAME *dNcMtx, double dCur, CPathGroup &NewPath ) ;
	// �Զ�������
	void AutoCheckAllPath( CSmtCheckMdl& DriveMdl, CPathGroup&  NewPath ) ;
	// ����β������·���ϲ�
	void CombinePath(CSmtCPathList &PathList);
	// ���ӽ��˵�
	void AddAllPathLeadPath( CSmtCheckMdl &DriveMdl, CSmtCPathLib &AllPath, BOOL bCheck, 
		double dLeft, BOOL *arrbSpiral = NULL, BOOL bCombSurfPath = FALSE) ;
	// ����з�CSmtCutPointEx��,ת����
	void TransfSmtCutPath( CSmtCutPath *pPath ) ;
protected:
	// �������Ϊ1��·��
	void DelOnePntPath(CSmtCPathLib &AllPath);
	// �ƶ�һ��·�������ظò��
	int MoveOneLayerPath(CSmtCPathLib &AllPath, CSmtCPathLib &LayerPath);
	// ��ͬһ��ź��кŵ�·���ƶ���Ŀ��·����
	void MoveSameLinePath(CSmtCPathList &PathFrom, CSmtCPathList &PathTo, POSITION pos);
	// �ҵ�����֪·����ӽ���·��λ��
	POSITION GetClosestPathPos(const CSmtCPathList &PathFrom, CSmtCutPath *pPath, POSITION pos, BOOL bForward = TRUE, BOOL bCombSurfPath = FALSE);
	// ���������������ɵķ�Χ��bExΪ��ʱ���÷�Χ����Ϊ�����֣���0����Сֵ���ӽϴ�ֵ��1��
	// Ϊ�˸������һ����Χ���ѽ�Сֵ��1����Χ��ɽϴ�ֵ����Сֵ��1
	void DefineRange(TFLOAT f1, TFLOAT f2, BOOL bEx, TFLOAT &fMin, TFLOAT &fMax);
	// ����·����UW������Χ��
	void DefineUWBox(CSmtCutPath *pPath, FPNT2D UWBox[2], BOOL &bEx);
	// ��·��UW�����ϵĹ�������
	TFLOAT TwoPathUWComArea(CSmtCutPath *pPath1, CSmtCutPath *pPath2);
	// �ж�·���Ƿ���ͬһ������
	BOOL PathInSameArea(CSmtCutPath *pPath1, CSmtCutPath *pPath2, BOOL bCombSurfPath = FALSE);
	// ������·���Ľӽ��̶ȣ�����ֵԽ��Խ�ӽ�
	TFLOAT TwoPathClose(CSmtCutPath *pPath1, CSmtCutPath *pPath2);
	// �õ�����֪·��������̵�·��λ��
	POSITION GetMinDistPathPos(const CSmtCPathList &PathFrom, CSmtCutPath *pPath);
	// ��·��֮�����̾���
	DOUBLE TwoPathMinDis(CSmtCutPath *pPath1, CSmtCutPath *pPath2);
	// ��·���о���ȡnSize(nSize<=5)���㣬���ظ���
	int SelectPathPoints(CSmtCutPath *pPath, FPNT3D SelPts[]);
	// �õ�ǰһ��·��
	void GetPrevLinePaths(CSmtCPathList &PrevPaths, const CSmtCPathList &PathList, POSITION pos);
	// ·�����д���������·����ͬ�����·��
	CSmtCutPath *GetSameRegionPath(const CSmtCPathList &PathList,  CSmtCutPath *pPath, BOOL bCombSurfPath = FALSE);
	// ���෴���򣬴�����·����ʼ����ͬ�����·���Ƶ�Ŀ��·������
	POSITION RegionPathBackward(CSmtCPathList &PathList, CSmtCutPath *pPathBegin, BOOL bCombSurfPath = FALSE);
	// ���������·������
	void SortPathByRegion(CSmtCPathList &PathList, BOOL bCombSurfPath = FALSE);	
	// ��ÿһ��·����������
	void AddLayerPathLeadPath(CSmtCheckMdl &DriveMdl, BOOL bCheck, CSmtCPathLib &LayerPath);
	BOOL CreateAllCombSurfPath(CSmartGraphic &Graph, C3DSurfArray &CombSurfs, CSmtCheckMdl &DriveMdl, 
		                       RFRAME *dNcMtx, CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg);
	// �߽繹������·��
	BOOL CreateCombineSurfPath(CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf *TrimSurf, CSmtCheckMdl& DriveMdl,RFRAME *dNcMtx,
		CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg);
	// �¹�������·���������������ͶӰģ�ʹ�����������ɵ�·�����ҵ����� qqs 2013.11.15     
	BOOL CreateAllCombSurfPathNew( CSmartGraphic &Graph, CCombSurfArray &CombSurfArray, CSmtCheckMdl &DriveMdl,CSmtCheckMdl &DrMdl,
	                               RFRAME *dNcMtx, CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg,double dCur[4]);
	// �¹�����·�����㺯�� qqs 2013.12.17
	BOOL CreateCombineSurfPathNew(CNc5DAxisCtrl &axisCtrl, CGeoTrmSurf *TrimSurf, CSmtCheckMdl& DriveMdl, RFRAME *dNcMtx,
		                       CSmartLoop *AllLoop, CSmtCPathLib &AllPath, double dPrg, int nLayerNo, BOOL* arrbSpiral);
	//��һ��������������4����
	BOOL SurfsTo4BndSurf(C3DSurfArray &GeoSurfList, C3DSurfArray &SurfNew, C3DSurfArray &SurfLeft);
	//��һ��������������4���棬�����µĹ����溯�������Է���ÿ���������Ӧ��ԭʼ���� qqs 2013.11.15
	BOOL SurfsTo4BndSurfNew(C3DSurfArray &GeoSurfList, CCombSurfArray &CombSurfArray, C3DSurfArray &SurfLeft);
	// �����ϲ��������
	void ModifyCombineSurf(RFRAME *dNcMtx, CGeoTrmSurf* pCombineSurf);
	// ���ɲ�����ֱ�߶�
	int CreateParamLine(CGeoTrmSurf* TrimSurf, DOUBLE UWStep[2], CSmartCurve& AllLine, int& nMoveDir, RFRAME& dNcMtx, BOOL bSpiral = FALSE);
	// ����·�������
	void AdjustStartAt( CSmartCurve& AllLine, BOOL bSpindle, int nMoveDir );
	// ·���ü�
	void TrimPath(CSmtCPathLib &AllPath, CSmartLoop *AllLoop);
	
	// ���ö�����ײ�·���ڵ㣬������ӹ��������߲��������ɵĲ�����·�����ӹ���Ȳü���Ĺ���̧�� qqs 2013.06.21
	void AdjustTopAndBotPath(CSmtCPathLib &AllPath, double dTopHeight, double dBottomHeight, double dArcTol);
	// ������·������ĩ��������������������Ϊͬһ���㣬
	// ��Ҫ���ڹ���������·��ʱ������ıպ���պϱ߽羫�Ƚϵ�ʱ�����ɵ�·������������� qqs 2013.06.21
	void ResetSameCutPnt(CSmtCPathLib& TmpPath, BOOL bSpiral, double minDis = 0.01);

	/// ���º���Ϊ������������ĥ�������뾶ĥ�𲹳��ĺ��� qqs
	//////////////////////////////////////////////////////////////////////////
	// Ϊ·��������ά����ƫ�Ʒ�������
	void Add3DRCompVectorToPath(CSmtCPathLib &AllPath, CPathGroup &NewPath, int nPathType = 0);// 0Ϊ��ͨ�����ɵ�����·����1Ϊ�����水����ͶӰ�������ɵ�����·����2Ϊ����·��
	// �������˵�ʱ���ڽ��˵�·�������ӽ��˵��ӳ��߷����ĥ�𲹳��ӳ�·�� qqs 2013.09.25
	CSmtCutPath* CalRCompByLeadPath(CSmtCutPath*& pLead, BOOL bIsOut, BOOL bIs3D = TRUE);
	// �رս��˵�������˵�δ����ʱ��ͨ������·�������ڼӹ�����㷨��ֱ��ĥ�𲹳��ӳ�·�� qqs 2013.09.25
	CSmtCutPath* CalRCompByRoughPath(CSmtCutPath*& pRough, BOOL bIsOut, BOOL b3D, BOOL bIs5Ax = FALSE);
	// �����뾶ĥ�𲹳�������·��
	BOOL  CreateRCompPath( JDNC_LEAD&	  LeadDef ,
			 			   CSmtCutPoint*  ptStart ,
						   CSmartLoop*    Contour ,
						   CSmtCutPath*&  RCompLeadPath,
						   VEC3D          Normal,
						   BOOL 		  IsOut);
	// Ϊ���᷽�����ɵ�·������ĥ�𲹳�����·����
	void AddRCompPathTo5AxPathLib(CSmtCPathLib& CutPathLib, BOOL bIs3D = FALSE);
	// ��ȡ�뾶ĥ�𲹳����
	int	 Get3DRCompMask ();
	//////////////////////////////////////////////////////////////////////////
};
///////////////////////////////////////////
// CSmartFFollowGen: ���ƵȾྫ���
//
class DLL_EXPORT CSmartFFollowGen : public CSmartAutoFinishGen
{
public :
	CSmartFFollowGen() ;
	virtual ~CSmartFFollowGen() ;
public :
	JDNC_FFOLLOW m_cParam ; 
public :
	// ���ɵ���·��
	virtual int GeneratePathEx( CPathGroup&    NewPath    ,  /*���·��*/
					            CSmartGraphic& Graph     ) ; /*���ͼ��*/
	virtual BOOL IsSupportProgress() { return TRUE ; }
public :
	// ��������
	CSmartLoop* ResortContour(CSmartLoop* AllLoop );

	//���ƵȾ�����������
	BOOL RemoveMosaic(CSmtCPathLib  &AllPath, CSmtCheckMdl & DriveMdl);
};

#endif // __SMART_AUTOFINISH_GEN_H__