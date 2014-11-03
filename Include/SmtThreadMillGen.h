#ifndef __SMART_THREADMILLGEN_H__
#define __SMART_THREADMILLGEN_H__

// ����ϳ���ƹ�·��

#include "SmtPathGen.H"
/////////////////////////////////////////////////////////////////////
// ·��������

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
{ /*ϳ���Ƽӹ�·��*/
public :
	CSmartThreadMillGen ()          ; 
	virtual ~CSmartThreadMillGen () ; 
public :
	JDNC_THREADMILL  m_cParam ;
	double			 m_dModiNormDiam ;
	double			 m_dModiMinDiam ;
	// ����Ƿ�ʼ�ӹ��µĵ㣬���ӹ��µĵ㣬�����ϸ���֮��Ŀ��ٶ�λ·��ͨ����ȫ�߶����� 
	// �ñ������л��ӹ���ʱ��ΪTRUE��ִ�й�һ��������������FALSE qqs 2014.03.21
	BOOL             m_bNewCutPoint; 
public :
	virtual BOOL GeneratePathEx( CPathGroup& NewGroup , /*�ӹ�·��*/ 
                                 CSmartGraphic& Graph); /*ͼ��*/   
public:
	// Ϊ�������ϳ���Ƶ���������������������2013.11.7 liuxin
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

	CPathArc3D* CreatHelixPath ( PNT3D  Center,      //������ĵ�
								 DOUBLE Radius,      //�뾶
								 DOUBLE Pitch,       //�ݾ�
								 DOUBLE StAngle,     //��ʼ�Ƕ�
								 DOUBLE Depth,       //��ȣ�>0���������ϣ�<0����������
								 BOOL   RotDir ) ;   //��ת����0��˳ʱ�룻1����ʱ��
	DOUBLE CalHelixFeedScale ( DOUBLE PathRad, DOUBLE Pitch, BOOL CutSide ) ; //CutSide : 0: outside ; 1 : inside
	DOUBLE CalLeadArcAngle ( DOUBLE MinLeadRad, DOUBLE MaxLeadRad ) ; //����ӹ�������Բ�������Ƕ�
	DOUBLE CalLeadLineOff ( DOUBLE MinCutRad ) ; //����ӹ�������ֱ�߽����ڿ��м����µĳ���
	DOUBLE CalExternalLeadArcAngle ( DOUBLE MinLeadRad ) ;
	DOUBLE CalExternalLeadLineLen () ;

	CPathPLine3D* CreateQuickPath ( PNT3D Start, PNT3D End, DOUBLE H ) ;
	int GetTeethNum () ;
	BOOL AddCutPathToComb ( CPathEntity* PathEnt[], int Num, CPathCombine *TComb ) ;

protected :
    int GetThreadMillRCompMask () ; // ����뾶�����ĺ���
	BOOL SetRCompMask ( CPathEntity* PathArr[], int Num, int RCompMask ) ;
	int GetLeadType () ;
	BOOL CalCutDepthByLayer ( DOUBLE ToothH, BOOL IsoVol, int LayerCnt, DOUBLE *&CutDepth ) ;
	BOOL CalCutDepthByMaxDepth ( DOUBLE ToothH, BOOL IsoVol, DOUBLE MaxDepth, int &LayerCnt, DOUBLE *&CutDepth ) ;
	BOOL CalAllBottomZ ( int LayerCnt, DOUBLE *CutDepth, DOUBLE &BottomZ, BOOL SideInc, DOUBLE *BotZArr ) ;

	BOOL AddExtPathCompSeg ( CPathEntity* InPathArr[3], CPathEntity* OutPathArr[5] ) ;
};

/////////////////////////////////////////////////////////////////////
#endif // __SMART_THREADMILLGEN_H__