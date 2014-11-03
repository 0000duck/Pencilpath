#ifndef FFOLLOW_NOMOSAIC_H
#define FFOLLOW_NOMOSAIC_H

//路径光顺要调用GeoEdit中的接口，所以放到mathcam中


struct CPNT3DBlock
{
	CTypedPtrArray<CPtrArray,double*> m_cAllPnt;
	FPNT3D m_fBox[2];
	void DefineBox();
};
typedef CTypedPtrArray<CPtrArray,CPNT3DBlock*> CPNT3DBlkArr;


struct CDirPoint
{
	PNT3D  m_fPoint;
	FPNT3D m_fDir;
	double m_dU; //在样条上的位置
};
struct CDirPointBlock
{
	CTypedPtrArray<CPtrArray,CDirPoint*> m_cAllPnt;
	FPNT3D m_fBox[2];
	void DefineBox();
};
typedef CTypedPtrArray<CPtrArray,CDirPointBlock*> CDirPntBlkArr;



//光顺路径
#define PSMOOTH_BUFFSIZE 20000
class DLL_EXPORT CPathSmooth
{
public:
	CPathSmooth();
	~CPathSmooth();

	int m_nMaxCheckNum;
	PNT3D  * m_aBuff;
public:
	CGeoSpline * Smooth(CSmtCutPath & Path, double SmoothTol);
	CGeoSpline * Smooth(CSmtCutPath & Path, double SmoothTol, FPNT3D tan0,
		FPNT3D tan1);
	CGeoSpline * Smooth(PNT3D AllPt[],int PtCnt, double SmoothTol);

public:
	int SmoothSpline( CGeoSpline* &Spline,double tolerance) ;
	CGeoSpline* CreateFairSpline( CSmtCutPath& CutPath);
};

//环绕等距消除马塞克
class DLL_EXPORT C3DStepNomosaic
{
public:
	C3DStepNomosaic();
	~C3DStepNomosaic();

public:
	JDNC_TOL m_cCalTol;
	int      m_nRecursive;
	CSmtCutPath * m_pCurrPathCpy;
public:
	BOOL RemoveMosaic(CSmtCPathLib& AllPath,
					  CSmtCheckMdl & CheckMdl,
					  JDNC_TOL & CalTol,
					  JDNC_PRGDEF& PrgDef,
					  double SmoothTol);

protected:
	PNT3D* GetSplinePntByStep(CGeoSpline & Spline,double Step, int & nPnt);
	PNT3D * GetSplinePntByPtCnt(CGeoSpline & Spline,int nPnt);
	PNT3D* GetAllPtNorm(PNT3D aSmoothPts[],int nPnt,
					  CSmtCheckMdl & CheckMdl);
	void GetPtNorm(float fCurrPt[3],float fNorm[3],CSmtCheckMdl & CheckMdl);
	void GetPtNorm(double fCurrPt[3],double Norm[3],CSmtCheckMdl & CheckMdl);
	void GetPathPtNorm(FPNT3D fCurrPt,FPNT3D fNorm,CSmtCheckMdl & CheckMdl);
	void GetNearestPathPt(double pt[3],CSmtCutPath * pPath,
						  FPNT3D & nearest_p/*,PNT3D Tangent[]*/);

	void MovePtAlongVec3D(double OrigPt[],double DestPt[],
						  VEC3D Dir,double Dist);
	void MovePtAlongVec3D(float OrigPt[],float DestPt[],
						  VEC3D Dir,double Dist);
	void MovePtAlongVec3D(double OrigPt[],float DestPt[],
						  float Dir[],double Dist);
	void MovePtAlongUnitVec3D(PNT3D OrigPt,PNT3D DestPt,VEC3D Dir);
	void DeletePntArr(PNT3D * & PtArr);
	void DeleteSpline(CGeoSpline * & Spline);
	void ClearPntBlockArr(CDirPntBlkArr & cPtBlock);
	void ClearPathArr(CTypedPtrArray<CPtrArray,CSmtCutPath*> & cBrkPathArr);
	CDirPoint * GetDirPntByStep(CGeoSpline & SmoothedPt,
								CGeoSpline & SmoothedVec,
								double Step, int & nPnt);

	void RecursiveAddMidPnt(CSmtCutPath * pPath,CSmtCutPoint * p,
						 CSmtCutPoint * pNext, CSmtCheckMdl & CheckMdl);
	void RecursiveAddMidPnt(CSmtCutPath * pPath,CSmtCutPoint * p,CSmtCutPoint * pNext,
						 CGeoSpline & SmoothedPt, CGeoSpline & SmoothedVec,
						 double U, double nextU, CSmtCheckMdl & CheckMdl);

	//点沿方向往检查模型投影
	BOOL PrjPntOnChkMdl(PNT3D Pnt,FPNT3D Dir,CSmtCheckMdl & CheckMdl, PNT3D PrjPt);
	void SmoothPrjSharpPath(CSmtCutPath *pPath, CSmtCutPath * pAtPath,
										 CSmtCheckMdl & CheckMdl);
	//点分块
	void SplitPntToBlocks(CDirPoint AllPnt[],int PntCnt,
						double dBlockLen, CDirPntBlkArr & PtBlock);
	void SplitPntToBlocks(PNT3D AllPnt[],int PntCnt,
						double dBlockLen, CPNT3DBlkArr & PtBlock);
	//路径分块
	//void SplitPathToBlocks(CSmtCutPath & Path,CSmtCutPtBlkArr & PtBlock);

	void OutputDebugInfo(CDirPntBlkArr & cPtBlock);

};

#endif