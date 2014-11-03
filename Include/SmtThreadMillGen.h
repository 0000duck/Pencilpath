#ifndef __SMART_THREADMILLGEN_H__
#define __SMART_THREADMILLGEN_H__

// 计算铣螺纹工路径

#include "SmtPathGen.H"
/////////////////////////////////////////////////////////////////////
// 路径发生器

struct THREAD_PARAM
{
	double m_dCutRad ;
	double m_dCutPitch ;
	double m_dCutDepth ;
	double m_dLeadRad ;
	double m_dLeadPitch ;
	double m_dBotLeadAng ;
	double m_dBotLeadDepth ;
	double m_dBotLeadOff ;
	double m_dBotLeadLen ;
	double m_dTopLeadAng ;
	double m_dTopLeadDepth ;
	double m_dTopLeadOff ;
	double m_dTopLeadLen ;
	double m_dCurZValue ;
} ;

class DLL_EXPORT CSmartThreadMillGen : public CSmartPathGen
{ /*铣螺纹加工路径*/
public :
	CSmartThreadMillGen ()          ; 
	virtual ~CSmartThreadMillGen () ; 
public :
	JDNC_THREADMILL  m_cParam ;
	double			 m_dModiNormDiam ;
	double			 m_dModiMinDiam ;
	// 标记是否开始加工新的点，若加工新的点，则与上个点之间的快速定位路径通过安全高度连接 
	// 该变量在切换加工域时置为TRUE，执行过一次连刀函数后置FALSE qqs 2014.03.21
	BOOL             m_bNewCutPoint; 
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup , /*加工路径*/ 
                                 CSmartGraphic& Graph); /*图形*/   
public:
	// 为方便多轴铣螺纹调用增加以下两个函数，2013.11.7 liuxin
	BOOL CreatInternalPath ( CSmartPoint & Point, CPathCombine & PComb) ;
	BOOL CreatExternalPath ( CSmartPoint & Point, CPathCombine & PComb) ;
protected :
	BOOL CreatInternalPath ( CPointList& AllPoint, CPathGroup& NewPath ) ;
	BOOL CreatExternalPath ( CPointList& AllPoint, CPathGroup& NewPath ) ;

	BOOL CreatIntPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	BOOL CreatIntRitClmPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	//BOOL CreatIntRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	BOOL CreatIntLftCvtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	//BOOL CreatIntLftClmPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;

	BOOL GenIntRitClmPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	//BOOL GenIntRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	BOOL GenIntLftCvtPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	//BOOL GenIntLftClmPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;

	BOOL CreatIntRitClmPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	//BOOL CreatIntRitCvtPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	BOOL CreatIntLftCvtPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	//BOOL CreatIntLftClmPath_Wear_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadAng, DOUBLE BottomZ ) ;
	BOOL AddLineLeadPath ( CSmartPoint *SmtPoint, CPathEntity *PathArr[3], CPathEntity *AllPathArr[5] ) ;

	BOOL CreatIntPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ ) ;
	BOOL CreatIntRitClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ ) ;
	//BOOL CreatIntRitCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ ) ;
	BOOL CreatIntLftCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ ) ;
	//BOOL CreatIntLftClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadOff, DOUBLE BottomZ ) ;

	BOOL GenIntRitClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	//BOOL GenIntRitCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	BOOL GenIntLftCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	//BOOL GenIntLftClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;

	BOOL CreatExtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadRad, DOUBLE LeadAng, DOUBLE BottomZ ) ;
	BOOL CreatExtRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadRad, DOUBLE LeadAng, DOUBLE BottomZ ) ;
	BOOL CreatExtLftClmPath_ArcLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadRad, DOUBLE LeadAng, DOUBLE BottomZ ) ;
	
	BOOL GenExtRitCvtPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	BOOL GenExtLftClmPath_ArcLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;

	BOOL CreatExtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadLen, DOUBLE BottomZ ) ;
	BOOL CreatExtRitCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadLen, DOUBLE BottomZ ) ;
	//BOOL CreatExtRitClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadLen, DOUBLE BottomZ ) ;
	BOOL CreatExtLftClmPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE LeadLen, DOUBLE BottomZ ) ;
	//BOOL CreatExtLftCvtPath_LineLead ( CSmartPoint *SmtPoint, CPathCombine *TComb, DOUBLE CutRad, DOUBLE BotLeadLen, DOUBLE BottomZ ) ;

	BOOL GenExtRitCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	//BOOL GenExtRitClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	BOOL GenExtLftClmPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;
	//BOOL GenExtLftCvtPath_LineLead ( CSmartPoint *SmtPoint, THREAD_PARAM &Param, CPathEntity *PathArr[3] ) ;

	CPathArc3D* CreatHelixPath ( PNT3D  Center,      //起点中心点
								 DOUBLE Radius,      //半径
								 DOUBLE Pitch,       //螺距
								 DOUBLE StAngle,     //起始角度
								 DOUBLE Depth,       //深度：>0：从下向上；<0：从上向下
								 BOOL   RotDir ) ;   //旋转方向：0：顺时针；1：逆时针
	DOUBLE CalHelixFeedScale ( DOUBLE PathRad, DOUBLE Pitch, BOOL CutSide ) ; //CutSide : 0: outside ; 1 : inside
	DOUBLE CalLeadArcAngle ( DOUBLE MinLeadRad, DOUBLE MaxLeadRad ) ; //计算加工内螺纹圆弧进刀角度
	DOUBLE CalLeadLineOff ( DOUBLE MinCutRad ) ; //计算加工内螺纹直线进刀在孔中间留下的长度
	DOUBLE CalExternalLeadArcAngle ( DOUBLE MinLeadRad ) ;
	DOUBLE CalExternalLeadLineLen () ;

	CPathPLine3D* CreateQuickPath ( PNT3D Start, PNT3D End, DOUBLE H ) ;
	int GetTeethNum () ;
	BOOL AddCutPathToComb ( CPathEntity* PathEnt[], int Num, CPathCombine *TComb ) ;

protected :
    int GetThreadMillRCompMask () ; // 计算半径补偿的号码
	BOOL SetRCompMask ( CPathEntity* PathArr[], int Num, int RCompMask ) ;
	int GetLeadType () ;
	BOOL CalCutDepthByLayer ( DOUBLE ToothH, BOOL IsoVol, int LayerCnt, DOUBLE *&CutDepth ) ;
	BOOL CalCutDepthByMaxDepth ( DOUBLE ToothH, BOOL IsoVol, DOUBLE MaxDepth, int &LayerCnt, DOUBLE *&CutDepth ) ;
	BOOL CalAllBottomZ ( int LayerCnt, DOUBLE *CutDepth, DOUBLE &BottomZ, BOOL SideInc, DOUBLE *BotZArr ) ;

	BOOL AddExtPathCompSeg ( CPathEntity* InPathArr[3], CPathEntity* OutPathArr[5] ) ;
};

/////////////////////////////////////////////////////////////////////
#endif // __SMART_THREADMILLGEN_H__