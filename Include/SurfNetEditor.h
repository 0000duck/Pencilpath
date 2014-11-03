///////////////////////////////////////////////////////////////
//		xiaochun liu
//		Mar. 2002
//      2005.8.  sz modified 
//////////////////////////////////////////////////////////////

#ifndef SURFNETEDITOR_H
#define SURFNETEDITOR_H

//structure for surfnet optimization
class SurfNetGridNode 
{
public:
	int		v[4];						//index to vet  array
	float	err;						//err already incurred by combination
	int 	nSize;	
	BYTE	type;					    //cell type
#define		CELLTYPE_ONETRIANGLE	3	//triangle defined by (0,1,2)
#define		CELLTYPE_RECTANGLE		4	//rectangle defined by (0,1,2,3)
#define		CELLTYPE_TWOTRIANGLE	5	//two triangles defined by (0,1,3) and (1,2,3)
//#define	CELLTYPE_STRIPRECTANGLE	6	//length != height rectangle

	//status
	BYTE    Status;
#define  DELETED        0x01     //is the whole cell was deleted when optimizing
#define  COMBINABLE     0x02
#define  BTRIDELETED	0x04     //for delete zero z polys;	if of one triangle (0,1,2), is it deleted (zero z poly)
#define  BQUADDELETED	0x08     //for delete zero z polys;	if of one rectangle (0,1,2,3), is it deleted (zero z poly)
#define  UP_COMBINABLE  0x10     //combination of four direction
#define  DN_COMBINABLE  0x20
#define  LT_COMBINABLE	0x40
#define  RT_COMBINABLE	0x80
	
	SurfNetGridNode()
	{
		type	=	CELLTYPE_RECTANGLE;
		nSize	=	1;
		err		=	0.f;
		Status = 0;
	};
};


class DLL_EXPORT CSurfNetEditor
{
public:
	PT_3D*	m_pVet;         //vertex
	int*	m_refNum;		//refNum[i] 0---4: how many facets refer the vertex
	int*	m_pos;			//new index after culling the non-referred vertex
	int		m_numVet;		//0....numVet-1
	int		m_numVetU;		//vet number in U dreiction
	int		m_numVetW;		//vet number in w  direction
	int		m_numGridU;		//grid number in U direction
	int		m_numGridW;		//grid number in W direction
	BOOL    m_bDisDistortRect; // 是否剖分扭曲的四边片
	BOOL    m_bNoMasaic      ; // 是否消除马赛克
	BOOL    m_bDelFlat       ; // 删除平面网格
    float   m_fMinZAt        ; // 平面网格的Z坐标
    float   m_fMaxTol        ; // 最大优化误差 
	SurfNetGridNode** m_Surf;
public:
	//to optimize a rectMesh  surfnet
	BOOL SurfBitMapOptimize(CGeoNetSurf*	OrgSurf,
							CGeoNetSurf*&	OutSurf);
    // 分割普通网格曲面 
    BOOL DepartNetSurf( CGeoNetSurf& OrgSurf, CPtrArray& SubSub ) ;
public :
	CSurfNetEditor();
	~CSurfNetEditor();

private:
	void FreeMemory();
	void OptimizeSurfGrid(float err,CGeoNetSurf*& surfOut);
	void ProcessFourGrids(int i,int j,int level,SurfNetGridNode*& g1,SurfNetGridNode*& g2,SurfNetGridNode*& g3,SurfNetGridNode*& g4,float err);
	BOOL ProcessFourGrids(int i,int j,int level,SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4);
	BOOL CombineFourGrids(int i,int j,int level,SurfNetGridNode*& g1,SurfNetGridNode*& g2,SurfNetGridNode*& g3,SurfNetGridNode*& g4, float err);

	BOOL CombineFourGridsOnPlane(int i,int j,int level,SurfNetGridNode* &g1,SurfNetGridNode* &g2,
		SurfNetGridNode* &g3,SurfNetGridNode* &g4,int OrigGridIndex[]);
	void SurfNetSetupData(CGeoNetSurf* nSurf);
	void ConvertGridSurfToSurfNet(CGeoNetSurf*& surfOut);
	BOOL IsAllRectCell(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4) ;
	BOOL IsLowNeighborCombinable(int level,int i,int j);
	BOOL IsRightNeighborCombinable(int level,int i,int j);
	BOOL IsUpNeighborCombinable(int level,int i,int j);
	BOOL IsLeftNeighborCombinable(int level,int i,int j);
	void MakeOneTriangleRight(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4);
	BOOL IsSameSize(int level,SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4);
	void MakeOneTriangleUp(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4);
	void MakeOneTriangleLeft(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4);
	void MakeOneTriangleLow(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4);
	float MaxCombineErr(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4,float err);
	float MaxCombineErrEx(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4,float err);
	float MaxPointPlaneDist(PT_3D p1,PT_3D p2,PT_3D p3,PT_3D* p,float Dis[]);
	void CreateOutlineVertex(int xNumVet,int yNumVet,int& numVetOutline,int*& vetOutline);
	int  CountOutlineVertex(int xNumVet,int yNumVet);

	float PointPlaneDist(PT_3D p1,PT_3D p2,PT_3D p3,PT_3D p);
	inline int Max4(int a,int b,int c,int d)	
	{ 
		int i1 = a > b ? a : b;
		int i2 = c > d ? c : d;
		return (i1>i2 ? i1 : i2);
	};
	inline double Max4(double a,double b,double c,double d) {return max(max(a,b),max(c,d));};
	inline float Max4(float a,float b,float c,float d) {return max(max(a,b),max(c,d));};
	inline float Max3(float a,float b,float c) { return max(max(a,b),c);};	
	//面片是否在一平面上
	BOOL IsSurfNetGridNodeOnOnePlane(SurfNetGridNode* pNode);
	BOOL IsFourSurfNetGridNodeOnOnePlane(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4);
	//四边片是否扭曲
	int IsRectNodeDistort(SurfNetGridNode* pNode);
	//剖分扭曲的四边片
	void DiscreteDistortRect(int numGridY,int numGridX);
};

#endif