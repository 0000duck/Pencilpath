#ifndef FHEIGHT_KEEPSHARPEDGES_H
#define FHEIGHT_KEEPSHARPEDGES_H


class CStrpt;
typedef CTypedPtrArray<CPtrArray, CSmtCutPoint*> CSmtPtArr;

struct CEdgeChkMdl
{
	CSmtCheckMdl * m_pEdgeChk;
	PNT3D          m_dBegin;
	PNT3D          m_dEnd;
	BOOL           m_bIsSteep;
	CEdgeChkMdl()	{
		m_pEdgeChk = NULL;
		m_bIsSteep = FALSE;
	}
	~CEdgeChkMdl()	{
		if(m_pEdgeChk) {
			delete m_pEdgeChk;
			m_pEdgeChk = NULL;
		}
	}
};

typedef CTypedPtrArray<CPtrArray, CEdgeChkMdl*> CEdgeChkArr;


#define  SMART_CUTPNT_ONEDGER   -10
#define  SMART_CUTPNT_LEADINOUT -11
#define  SMART_CUTPNT_ONHOZEDGE  10000.0
//等高精加工保持锋利边界
class CFHeightKeepSharpEdge
{
public:
	CFHeightKeepSharpEdge();
	~CFHeightKeepSharpEdge();

public:
	//等高路径保持锋利边界
	void KeepSharpEdges(CSmtCPathLib& AllPath, 
						CPtrList & SharpEdges,  //锋利边界(CStrpt)
						JDNC_SETUP& SetupDef,   
						JDNC_STOCKEX & StockDef,
						CSmtCheckMdl &DriveMdl);

	BOOL KeepSharpEdge(CSmtCPathLib& AllPath,CSmtCutPath *pPath, JDNC_SETUP& SetupDef,
		CSmtCheckMdl &DriveMdl);
	//生成锋利边界检查模型
	BOOL GenSharpEdgeChkMdl(CPtrList & SharpEdges,CSmartTool * Tool);
	void ClearAll();
public:
	BOOL          m_bKeepHoz;        //保留水平锋利边
	double        m_dHozEdgeZShift;  //加工水平水平锋利边z向平移
private:
	//锋利边界检查模型数组
	CEdgeChkArr   m_cSharpEdgeChkMdl;


protected:
	CSmtCutPath * BreakPathByPntType(CSmtCutPath & Path, short Type);
	BOOL IsEdgeSteep(CStrpt & Strpt);
	//水平方向投影
	BOOL CanHorPrjPntOnChkMdl(PNT3D Pnt,PNT3D Dir,
						CSmtCheckMdl & CheckMdl, double dDist);
	BOOL IsPointOnEdge(CSmtCutPoint & Pnt,
					  CSmtCheckMdl & EdgeChkMdl);
	void ReconstructPathAtEdgeR(CSmtCPathLib& AllPath,CSmtCutPath *pPath,CEdgeChkMdl & EdgeMdl,
				CSmtCheckMdl &DriveMdl, CSmtPtArr & cEdegeRPnts);
	void ResetPathHeadTail(CSmtCutPath & Path);
	BOOL IsSmtCutPntOnSharpEdge(CSmtCutPoint *pPnt,CSmtCheckMdl * pEdgeChkMdl,
			double dEdgeMinZ,double dEdgeMaxZ, CEdgeChkMdl * pAtEdge);
	void MoveUpPathAtHozEdge(CSmtCutPoint * pArcBegin, CSmtCutPoint * pArcEnd);

	BOOL VerifyLocalPath(double Begin[], double End[],CSmtCheckMdl &DriveMdl);
	int GetSmtCutPathCount(CSmtCutPath* Path) ;
	CSmartCheck * GetSmtEdge(double Edge0[3],double Edge1[3]);
	BOOL GetEdgeIntPoint(double AtZ,CStrpt & Strpt,double IntPt[]);
	void AddOneEdgeMdl(PNT3D* aEdgePts, int EdgePtCnt,BOOL IsSteep,CSmartTool * Tool);
	void GetToolTouchDir2D(double Pnt[],CEdgeChkMdl & EdgeMdl,double Dir2D[]);
};
#endif