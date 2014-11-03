#ifndef PATH_SPLINE_FITTING_H
#define PATH_SPLINE_FITTING_H

class CPathSplFit;
class CPathSmooth;
// �����������·�����߳�����
typedef struct CreatePathSplFitThreadData 
{
	int			 nAtCore;
	int			 nCoreNum;
	double       dFitTol;
	VOID *	     AllPath ;
	VOID *       AllComb;
	JDNC_PRGDEF  PrgDef  ;
	CPathSplFit *pPathSplFitGen;
}PSPLFIT_DATA;



//�������·��
#define LINESEG_MAXLEN   2    //���ڸó��ȵĳ�ֱ�߶β��������
#define BRKSEG_ANG       160  //�ڽǶ�С�ڸ�ֵ�ĵط����·��
#define ONESPL_MAXLEN    6    //�ֶ���ϵ���󳤶�
#define ONESPL_MINLEN    0.1  //�ֶ���ϵ���С����
#define SPLINE_VRFYINC   0.05 //����У����ϼ��
#define SPLINE_PNTINC    0.15 //�����ɵ���
#define ONESPL_MAXPNT    50   //�ֶ���ϵ�������

#define PNTTYPE_SHARP   -200 //��ϴ���������
#define PNTTYPE_MAXLEN  -201 //��ϴ��㳤�ȳ���������
#define PNTTYPE_MAXPNT  -202 //��ϴ���������������
#define PNTTYPE_RCHANGE  -203 //���ʰ뾶�����

#define SPL_DISC_MAXPT  5000  
#define SPL_DISC_TOL    0.002 //������ɢ����
#define SPL_DISC_ANGTOL 5     

enum PathFitType
{
	BSplInterpolateFit,   //������ֵ���
	SectBezier,           //�ֶ�Bezier���(���淽����Ҫ�ﵽ��������Ҫ���ݹ�)
	SectBezierEx2,        //�ֶ�Bezier���Ex(�Ľ�һ������) 2��bezier
	SectBezierEx3,        //�ֶ�Bezier���Ex(�Ľ�һ������) 3��bezier
};

#define  NC_PATH_BEZIER3D     2000
class DLL_EXPORT CPathBezier3D : public CPathEntity 
{
public :
	int		m_n;			// degree and max index of control points
	PNT4D	m_Pw[MAX_ORD];	// control points, Pw[i], i=0,...,n
public :
	CPathBezier3D() ;
	virtual ~CPathBezier3D() ;
	BOOL CreateBezier( PNT4D aCtrlPt[], int Count ) ;
public :
	virtual int GetType() { return NC_PATH_BEZIER3D ; }
	virtual void RotateEntity( DOUBLE Cos, DOUBLE Sin )  ;
	virtual void MoveEntity( DOUBLE Dx, DOUBLE Dy, DOUBLE Dz ) ;
	virtual void SetZCoord( DOUBLE ZCoord ) ;
	virtual DOUBLE GetLength( ) ;
	virtual CPathEntity* CopySpire( int ) { return NULL ;}
	virtual BOOL GetEndPoint(BOOL End, TPNT3D TPoint )   ;
	virtual BOOL GetEndTangent(BOOL Flag, VEC3D Tangent) ;
	virtual BOOL GetEndNormal(BOOL Flag , VEC2D Normal ) ;
	virtual CPathEntity* CopyMyself() ;
	virtual BOOL ReverseDirect() ;
	virtual BOOL GetBoundBox( double min[3], double max[3] ) ;
	virtual BOOL IsValid()  ;
	virtual BOOL Serialize(REGEN_PARAM& Param ) ;
};

class DLL_EXPORT CPathSplFit
{
public:
	CPathSplFit();
	~CPathSplFit();

private:
	CPathSmooth*m_pPSmooth;
	PathFitType m_nGenType;
	double   m_dFitTol;
	PNT3D  * m_aSplDiscPt;
	PNT5D  * m_aBezierDiscPt;


//����Ϊ3��·���������
public:
	//�������·�������תΪֱ�߶ηŵ�SmtCutPath)
	BOOL SplFit(CSmtCPathLib& AllPath,      /*����·��*/
					double FitTol ,         /*����������*/
					JDNC_PRGDEF& PrgDef);

	//�������·��(תΪCPathNurbs3D(bezier)�ŵ�CPathCombine��)��Ϊ�����岹��׼��
	BOOL SplFit(CSmtCPathLib& AllPath,       /*����·��*/
					double FitTol ,          /*����������*/
					CPathCombine & PathComb, /*������Ͻ��*/
					JDNC_PRGDEF& PrgDef);
	//������ϵ㴮
	BOOL FitBySpline( CPathCombine& AllPath, JDNC_TOL& Tol, PNT3D Point[], int NumPnt );

	//����·���ڲ��
	BOOL GetWeakenCornerPath(CSmtCutPath & CurrPath);

//����Ϊ5��·���������
public:
	//�������·�������תΪֱ�߶ηŵ�SmtCutPath)
	BOOL SplFit5X(CSmtCPathLib& AllPath,      /*����·��*/
					double FitTol ,         /*����������*/
					JDNC_PRGDEF& PrgDef);

	//�������·��(תΪCPathNurbs3D(bezier)�ŵ�CPathCombine��)��Ϊ�����岹��׼��
	BOOL SplFit5X(CSmtCPathLib& AllPath,       /*����·��*/
					double FitTol ,          /*����������*/
					CPathCombine & PathComb, /*������Ͻ��*/
					JDNC_PRGDEF& PrgDef);

public:
	void SetFitTol(double FitTol) {	m_dFitTol = FitTol; }
	// �������·���̺߳���
	BOOL SplFitSubProc(int nAtCore, int nCoreNum, VOID* AllPath,
		double FitTol, JDNC_PRGDEF &PrgDef);
	BOOL SplFitSubProcEx(int nAtCore, int nCoreNum, VOID* AllPath,
		double FitTol, VOID* AllComb, JDNC_PRGDEF &PrgDef);

	BOOL GenSectBezierEx(CSmtCutPath &Path, CTypedPtrArray<CPtrArray,CBZCurve*> &cBezierArr);
protected:
	//3��
	void SplFittingOnePath(CSmtCutPath * pCurrPath);
	void SplFittingOnePath(CSmtCutPath * pCurrPath, CPathCombine & PathComb);
	void SectBezierFitOnePath(CSmtCutPath * pCurrPath);
	void SectBezierFitOnePath(CSmtCutPath &CurrPath, CPathCombine & PathComb);

	//5��
	void SectBezierFitOnePath5X(CSmtCutPath * pCurrPath);
	void SectBezierFitOnePath5X(CSmtCutPath &CurrPath, CPathCombine & PathComb);

	void MovePtAlongVec(double OrigPt[],double DestPt[], VEC3D Dir,double Dist);
	void ReverseVec(double Orig[],double Dest[]);

	//�����������Բ������
	double Cal3PtsArcCurvature(FPNT3D p0, FPNT3D p1, FPNT3D p2);
	BOOL GetPathHeadTan(CSmtCutPath *pPath,FPNT3D Tan);
	BOOL GetPathTailTan(CSmtCutPath *pPath,FPNT3D Tan);
private:
	//��·�����Ƕȡ������߶���󳤶Ⱥ͵���������󳤶ȴ�Ϸֶ�
	void BreakPathByAngAndLen(CSmtCutPath & Path, double Angle,double OneSegMaxLen,
				double OneSplMaxLen, CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr);

	//��·�����Ƕȡ������߶���󳤶Ⱥ͵���������������Ϸֶ�
	void BreakPathByAngAndMaxPt(CSmtCutPath & Path, double Angle, double OneSegMaxLen,
							   int OneSplMaxPt, CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr);

	double GetTriVertAng(CSmtCutPoint * pPrev, CSmtCutPoint * pPoint, CSmtCutPoint * pNext);
	void BreakPathByLen(CSmtCutPath & Path, double Len,
						CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr);
	void AddLinSegsToPath(CSmtCutPath * pCurrPath, CSmtCutPath * &pPath, int nPath,
						 FPNT3D vFirstTan, FPNT3D vLastTan);	
	void ClearPathArr(CTypedPtrArray<CPtrArray,CSmtCutPath*> & PathArr);
	BOOL IsPathSmooth(CSmtCutPath & CurrPath, double Ang);
	//����Ӧ�����ֵ��
	void AdaptiveAddInterpolatePt(CSmtCutPath  &Path);
	//��������ɢֱ�߶Σ���������ĩ�㣩
	int DiscLinSegByStepNoBE(FPNT3D Begin, FPNT3D End,double LinLen, double Step, PNT3D AllPt[]);
	void CalMidPnt(PNT3D dBegin,PNT3D dEnd,PNT3D dMid);
	void AddLinSegsToPath(CSmtCutPath * pCurrPath, CSmtCutPath * &pPath );
	BOOL GenSectBezier(CSmtCutPath &Path, CTypedPtrArray<CPtrArray,CBZCurve*> &cBezierArr);
	BOOL GenBezier(PNT4D dCtrlPnt[], double Tol, int & MaxDepth,
		CTypedPtrArray<CPtrArray,CBZCurve*> & cBezierArr);

	void AddPathPLine3D(CSmtCutPath *pPath, CPathCombine & PathComb);

	BOOL IsArcSeg(CSmtCutPath & CurrSeg);
	CPathArc3D * CreateArcPath(CGeoArc & arc);
	BOOL VerifyArc(CSmtCutPath & OrigCurv,CGeoArc & Arc);
	CGeoSpline * CreateCircle(CSmtCutPath * pCurrPath);
};

#endif