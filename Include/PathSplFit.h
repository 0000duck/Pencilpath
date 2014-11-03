#ifndef PATH_SPLINE_FITTING_H
#define PATH_SPLINE_FITTING_H

class CPathSplFit;
class CPathSmooth;
// 创建样条拟合路径的线程数据
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



//样条拟合路径
#define LINESEG_MAXLEN   2    //大于该长度的长直线段不样条拟合
#define BRKSEG_ANG       160  //在角度小于该值的地方打断路径
#define ONESPL_MAXLEN    6    //分段拟合的最大长度
#define ONESPL_MINLEN    0.1  //分段拟合的最小长度
#define SPLINE_VRFYINC   0.05 //样条校验拟合间距
#define SPLINE_PNTINC    0.15 //样条采点间距
#define ONESPL_MAXPNT    50   //分段拟合的最大点数

#define PNTTYPE_SHARP   -200 //打断处点尖点类型
#define PNTTYPE_MAXLEN  -201 //打断处点长度超过点类型
#define PNTTYPE_MAXPNT  -202 //打断处点数超过最大点数
#define PNTTYPE_RCHANGE  -203 //曲率半径跳变点

#define SPL_DISC_MAXPT  5000  
#define SPL_DISC_TOL    0.002 //样条离散精度
#define SPL_DISC_ANGTOL 5     

enum PathFitType
{
	BSplInterpolateFit,   //样条插值拟合
	SectBezier,           //分段Bezier拟合(常规方法，要达到拟合误差需要插点递归)
	SectBezierEx2,        //分段Bezier拟合Ex(改进一次生成) 2次bezier
	SectBezierEx3,        //分段Bezier拟合Ex(改进一次生成) 3次bezier
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


//以下为3轴路径样条拟合
public:
	//样条拟合路径（最后都转为直线段放到SmtCutPath)
	BOOL SplFit(CSmtCPathLib& AllPath,      /*所有路径*/
					double FitTol ,         /*样条拟合误差*/
					JDNC_PRGDEF& PrgDef);

	//样条拟合路径(转为CPathNurbs3D(bezier)放到CPathCombine中)、为样条插补做准备
	BOOL SplFit(CSmtCPathLib& AllPath,       /*所有路径*/
					double FitTol ,          /*样条拟合误差*/
					CPathCombine & PathComb, /*样条拟合结果*/
					JDNC_PRGDEF& PrgDef);
	//样条拟合点串
	BOOL FitBySpline( CPathCombine& AllPath, JDNC_TOL& Tol, PNT3D Point[], int NumPnt );

	//计算路径内插点
	BOOL GetWeakenCornerPath(CSmtCutPath & CurrPath);

//以下为5轴路径样条拟合
public:
	//样条拟合路径（最后都转为直线段放到SmtCutPath)
	BOOL SplFit5X(CSmtCPathLib& AllPath,      /*所有路径*/
					double FitTol ,         /*样条拟合误差*/
					JDNC_PRGDEF& PrgDef);

	//样条拟合路径(转为CPathNurbs3D(bezier)放到CPathCombine中)、为样条插补做准备
	BOOL SplFit5X(CSmtCPathLib& AllPath,       /*所有路径*/
					double FitTol ,          /*样条拟合误差*/
					CPathCombine & PathComb, /*样条拟合结果*/
					JDNC_PRGDEF& PrgDef);

public:
	void SetFitTol(double FitTol) {	m_dFitTol = FitTol; }
	// 样条拟合路径线程函数
	BOOL SplFitSubProc(int nAtCore, int nCoreNum, VOID* AllPath,
		double FitTol, JDNC_PRGDEF &PrgDef);
	BOOL SplFitSubProcEx(int nAtCore, int nCoreNum, VOID* AllPath,
		double FitTol, VOID* AllComb, JDNC_PRGDEF &PrgDef);

	BOOL GenSectBezierEx(CSmtCutPath &Path, CTypedPtrArray<CPtrArray,CBZCurve*> &cBezierArr);
protected:
	//3轴
	void SplFittingOnePath(CSmtCutPath * pCurrPath);
	void SplFittingOnePath(CSmtCutPath * pCurrPath, CPathCombine & PathComb);
	void SectBezierFitOnePath(CSmtCutPath * pCurrPath);
	void SectBezierFitOnePath(CSmtCutPath &CurrPath, CPathCombine & PathComb);

	//5轴
	void SectBezierFitOnePath5X(CSmtCutPath * pCurrPath);
	void SectBezierFitOnePath5X(CSmtCutPath &CurrPath, CPathCombine & PathComb);

	void MovePtAlongVec(double OrigPt[],double DestPt[], VEC3D Dir,double Dist);
	void ReverseVec(double Orig[],double Dest[]);

	//计算三点组成圆弧曲率
	double Cal3PtsArcCurvature(FPNT3D p0, FPNT3D p1, FPNT3D p2);
	BOOL GetPathHeadTan(CSmtCutPath *pPath,FPNT3D Tan);
	BOOL GetPathTailTan(CSmtCutPath *pPath,FPNT3D Tan);
private:
	//将路径按角度、单段线段最大长度和单段样条最大长度打断分段
	void BreakPathByAngAndLen(CSmtCutPath & Path, double Angle,double OneSegMaxLen,
				double OneSplMaxLen, CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr);

	//将路径按角度、单段线段最大长度和单段样条最大点数打断分段
	void BreakPathByAngAndMaxPt(CSmtCutPath & Path, double Angle, double OneSegMaxLen,
							   int OneSplMaxPt, CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr);

	double GetTriVertAng(CSmtCutPoint * pPrev, CSmtCutPoint * pPoint, CSmtCutPoint * pNext);
	void BreakPathByLen(CSmtCutPath & Path, double Len,
						CTypedPtrArray<CPtrArray,CSmtCutPath*> & BrkPathArr);
	void AddLinSegsToPath(CSmtCutPath * pCurrPath, CSmtCutPath * &pPath, int nPath,
						 FPNT3D vFirstTan, FPNT3D vLastTan);	
	void ClearPathArr(CTypedPtrArray<CPtrArray,CSmtCutPath*> & PathArr);
	BOOL IsPathSmooth(CSmtCutPath & CurrPath, double Ang);
	//自适应加入插值点
	void AdaptiveAddInterpolatePt(CSmtCutPath  &Path);
	//按长度离散直线段（不包括起末点）
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