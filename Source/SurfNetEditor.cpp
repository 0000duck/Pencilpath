///////////////////////////////////////////////////////////////
//		xiaochun liu
//		Mar. 2002
//      2005.8.  sz modified 
//////////////////////////////////////////////////////////////
#include "StdAfx.H"
#include "SurfGeo.h"
#include "PathDef.h"
#include <math.h>
#include "SurfNetEditor.h"
#include "SysPrompt.h"
#include "SmartMath.H"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif


CSurfNetEditor::CSurfNetEditor()
{
	m_Surf = NULL;
	m_pVet = NULL;
	m_pos = NULL;
	m_refNum = NULL;
	m_bDelFlat = FALSE;
	m_bDisDistortRect = FALSE;
	m_bNoMasaic = FALSE;
    m_fMinZAt = -1.0e6f ;
    m_fMaxTol = 0.01f   ;
}

CSurfNetEditor::~CSurfNetEditor()
{

}

//to optimize a rectMesh  surfnet
//////////////////////////////////////////////////////////////
BOOL CSurfNetEditor::SurfBitMapOptimize(CGeoNetSurf*	OrgSurf,
										CGeoNetSurf*&	OutSurf )
{
	OutSurf=NULL;
	if (!OrgSurf->IsTopoRectMesh() && !OrgSurf->IsRectMesh() && !OrgSurf->IsMetaRectMesh())
    {
		return FALSE;
    }

	//delete edges to simplify the situation
	//SurfNetEdgeDelete(nSurf);

	//allocate space for m_pVet
	m_numVet=OrgSurf->m_nNumVertX * OrgSurf->m_nNumVertY;
	m_pVet		= new PT_3D[m_numVet];
	m_refNum	= new int[m_numVet];
	m_pos		= new int[m_numVet];

	m_numVetU	= OrgSurf->m_nNumVertX;
	m_numVetW	= OrgSurf->m_nNumVertY;

	m_numGridU = m_numVetU-1;
	m_numGridW = m_numVetW-1;

	//allocate space for the grid surf
	m_Surf = new SurfNetGridNode* [m_numGridW];
	for (int j=0;j<m_numGridW;j++)
		m_Surf[j] = new SurfNetGridNode[m_numGridU];
	SurfNetSetupData(OrgSurf);

	OptimizeSurfGrid( m_fMaxTol,OutSurf) ;	
	//free the the grid surface
	FreeMemory();
	return TRUE;
}

void CSurfNetEditor::FreeMemory()
{
	if(m_Surf)
	{
		for (int j=0;j<m_numGridW;j++)
			delete[] m_Surf[j];
		delete[] m_Surf;
		m_Surf = NULL;
	}

	//delete m_pVet space
	if(m_pVet)
	{
		delete [] m_pVet;
		m_pVet = NULL;
	}
	if(m_refNum)
	{
		delete [] m_refNum;
		m_refNum = NULL;
	}
	if(m_pos)
	{
		delete [] m_pos;
		m_pos = NULL;
	}
}

//setting up data for rectMesh surf net
void CSurfNetEditor::SurfNetSetupData(CGeoNetSurf* NetSurf)
{
	int k = 0;
	//setting the m_pVet array
	if (NetSurf->IsRectMesh()) 
	{
		for (int i=0;i<m_numVetW;i++)
		for (int j=0;j<m_numVetU;j++) 
		{
            m_pVet[k].Set( NetSurf->m_aCoordX[j],
			               NetSurf->m_aCoordY[i],
			               NetSurf->m_aCoordZ[k]);
            m_refNum[k]=0;
            m_pos[k]=k;
            k ++ ;
		}
	}
	else if (NetSurf->IsMetaRectMesh()) 
    {
		for (int i=0;i<m_numVetW;i++)
		for (int j=0;j<m_numVetU;j++) 
        {
            m_pVet[k].Set( NetSurf->m_aCoordX[k],
						   NetSurf->m_aCoordY[k],
                           NetSurf->m_aCoordZ[k]);
			m_refNum[k]=0;
			m_pos[k]=k;
            k ++ ;
		}
	}
	else 
	{ //topo rect
		for (int i=0;i<m_numVetW;i++)
		for (int j=0;j<m_numVetU;j++)
		{
			m_pVet[k].Set( NetSurf->m_aVertex[k][0],
						   NetSurf->m_aVertex[k][1],
						   NetSurf->m_aVertex[k][2]);
			m_refNum[k]=0;
			m_pos[k]=k;
            k ++ ; 
		}
	}
	//to transfer the rectMesh SurfNet to grid surf representation
	for (int i=0;i<m_numGridW;i++)
	{
		for (int j=0;j<m_numGridU;j++)
		{
			//the four vetex in counterclockwise order
			k=i*m_numVetU+j;
			m_Surf[i][j].v[0]=k;	
			m_Surf[i][j].v[1]=k+1;		
			m_Surf[i][j].v[2]=k+1+m_numVetU;		
			m_Surf[i][j].v[3]=k+m_numVetU;		
		}	
	}
}

void CSurfNetEditor::OptimizeSurfGrid(float err,CGeoNetSurf*& surfOut)
{
	int			level;			//2,4,8,16
	SurfNetGridNode *g1,*g2,*g3,*g4;	//in counterclockwise order

	int MaxTimes = 10;
	int TimesCnter = 0;
	level = 2;
	int adjgrid = 1;
	while( TimesCnter< MaxTimes)
	{
		for (int i=0;i<m_numGridW/level*level;i += level)
		{
			for (int j=0;j<m_numGridU/level*level;j += level) 
			{
				g1=&m_Surf[i][j];
				g2=&m_Surf[i][j+adjgrid];
				g3=&m_Surf[i+adjgrid][j+adjgrid];
				g4=&m_Surf[i+adjgrid][j];
				ProcessFourGrids(i,j,level,g1,g2,g3,g4,err);
			}
		}
		for ( i=0;i<m_numGridW/level*level;i += level)
		{
			for (int j=0;j<m_numGridU/level*level;j += level) 
			{
				g1=&m_Surf[i][j];
				g2=&m_Surf[i][j+adjgrid];
				g3=&m_Surf[i+adjgrid][j+adjgrid];
				g4=&m_Surf[i+adjgrid][j];
				ProcessFourGrids(i,j,level,g1,g2,g3,g4);
			}
		}
		
		/*
		//combine grids that is on one plane
		for ( i=0;i<m_numGridW/level*level;i += level)
		{
			for (int j=0;j<m_numGridU/level*level;j += level) 
			{
				g1=&m_Surf[i][j];

				if(g1->type == CELLTYPE_RECTANGLE && g1->nSize == level*level)
					continue;
				g2 =&m_Surf[i][j+adjgrid];
				g3 =&m_Surf[i+adjgrid][j+adjgrid];
				g4 =&m_Surf[i+adjgrid][j];

				//if(g1->deleted == FALSE && g1->nSize == level*level)
				//	continue;
				k = i*m_numVetU + j;
				OrigGridIndex[0] = m_pos[k];
				OrigGridIndex[1] = m_pos[k+level];
				OrigGridIndex[2] = m_pos[k+level+m_numVetU*level];
				OrigGridIndex[3] = m_pos[k+m_numVetU*level];
				CombineFourGridsOnPlane(i,j,level,g1,g2,g3,g4,OrigGridIndex);
			}
		}
		*/
		
		for ( i=0;i<m_numGridW/level*level;i += level)
			for (int j=0;j<m_numGridU/level*level;j += level) 
				m_Surf[i][j].Status &= ~COMBINABLE;
		

		level *= 2;
		adjgrid *= 2;
		TimesCnter++;
	}

	//convert grid surf to SurfNet.
	ConvertGridSurfToSurfNet(surfOut);
}


void CSurfNetEditor::ProcessFourGrids(int i,int j,int level,SurfNetGridNode*& g1,SurfNetGridNode*& g2,SurfNetGridNode*& g3,SurfNetGridNode*& g4,float err)
{
	if (IsAllRectCell(g1,g2,g3,g4) &&
		IsSameSize(level,g1,g2,g3,g4) &&
		Max4(g1->err,g2->err,g3->err,g4->err)<err) 
	{
		//小于一定距离时限制不在一平面上的四个格子的优化，防止出现马赛克现象
		//if(!IsFourSurfNetGridNodeOnOnePlane(g1,g2,g3,g4) &&  (m_pVet[g2->v[1]].x-m_pVet[g1->v[0]].x)<0.15)
		if(m_bNoMasaic && !IsFourSurfNetGridNodeOnOnePlane(g1,g2,g3,g4) )
		{
            if( m_pVet[g1->v[1]].DistFrom( m_pVet[g1->v[0]]) > 0.085 ||
			     m_pVet[g1->v[3]].DistFrom( m_pVet[g1->v[0]]) > 0.085 )
			{
				return;
			}
		}
		CombineFourGrids(i,j,level,g1,g2,g3,g4, err);
	}
}
	
//logical size not phisycal size
BOOL CSurfNetEditor::IsSameSize(int level,SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4) 
{
	int size = level*level/4;
	if (size==g1->nSize && 
		size==g2->nSize && 
		size==g3->nSize &&
		size==g4->nSize)
		return TRUE;
	else
		return FALSE;
}

BOOL CSurfNetEditor::IsAllRectCell(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4) 
{
	if (g1->type == CELLTYPE_RECTANGLE &&
		g2->type == CELLTYPE_RECTANGLE &&
		g3->type == CELLTYPE_RECTANGLE &&
		g4->type == CELLTYPE_RECTANGLE )
		return TRUE;
	else
		return FALSE;
}

BOOL CSurfNetEditor::IsFourSurfNetGridNodeOnOnePlane(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4)
{
	if(IsSurfNetGridNodeOnOnePlane(g1) && IsSurfNetGridNodeOnOnePlane(g2) &&
		IsSurfNetGridNodeOnOnePlane(g3) && IsSurfNetGridNodeOnOnePlane(g4))
		return TRUE;
	else
		return FALSE;

}
BOOL CSurfNetEditor::IsSurfNetGridNodeOnOnePlane(SurfNetGridNode* pNode)
{
	if(!pNode)
		return FALSE;
	double tol = 1.0e-4;
	if(pNode->type == CELLTYPE_RECTANGLE )//|| pNode->type == CELLTYPE_STRIPRECTANGLE)
	{
		if( fabs(m_pVet[pNode->v[0]].z-m_pVet[pNode->v[1]].z)<tol && 
			fabs(m_pVet[pNode->v[1]].z-m_pVet[pNode->v[2]].z)<tol	&& 
			fabs(m_pVet[pNode->v[2]].z-m_pVet[pNode->v[3]].z)<tol &&
			fabs(m_pVet[pNode->v[3]].z-m_pVet[pNode->v[0]].z)<tol )
		{
			return TRUE;
		}
	}
	else if(pNode->type == CELLTYPE_ONETRIANGLE)
	{
		if( fabs(m_pVet[pNode->v[0]].z-m_pVet[pNode->v[1]].z)<tol && 
			fabs(m_pVet[pNode->v[1]].z-m_pVet[pNode->v[2]].z)<tol	&& 
			fabs(m_pVet[pNode->v[2]].z-m_pVet[pNode->v[0]].z)<tol )
		{
			return TRUE;
		}
	}
	return FALSE;

}
void CSurfNetEditor::DiscreteDistortRect(int numGridY,int numGridX)
{
	SurfNetGridNode * p = NULL;
	int nDistort = 0;
	int nTemp = 0;
	for (int i=0;i<numGridY;i ++)
	{
		for (int j=0;j<numGridX;j ++)
		{
			p=&m_Surf[i][j];
			if (p->Status & DELETED) 
				continue;

			if ((p->type == CELLTYPE_RECTANGLE ) 
				&& !(p->Status & DELETED) && !(p->Status & BQUADDELETED)) 
			{
				nDistort = IsRectNodeDistort(p);
				if(nDistort == 1)
				{
					p->type = CELLTYPE_TWOTRIANGLE;
					nTemp = p->v[2];
					p->v[2] = p->v[3];
					p->v[3] = nTemp;
					nTemp = p->v[1];
					p->v[1] = p->v[0];
					p->v[0] = nTemp;
				}
				else if(nDistort == 2)
				{
					p->type = CELLTYPE_TWOTRIANGLE;
				}
			}
		}
	}
}

int CSurfNetEditor::IsRectNodeDistort(SurfNetGridNode* pNode)
{
	if(!pNode || pNode->type!= CELLTYPE_RECTANGLE )
		return 0;

	double tol = 0.04;
	PT_3D MidPt1 = (m_pVet[pNode->v[0]] + m_pVet[pNode->v[2]])*0.5;
	PT_3D MidPt2 = (m_pVet[pNode->v[1]] + m_pVet[pNode->v[3]])*0.5;

	if((MidPt1.z -MidPt2.z ) > tol)
		return 1;

	if((MidPt2.z -MidPt1.z ) > tol)
		return 2;

	return 0;

}
BOOL CSurfNetEditor::CombineFourGrids(int /*i*/,int /*j*/,int level,SurfNetGridNode*& g1,SurfNetGridNode*& g2,SurfNetGridNode*& g3,SurfNetGridNode*& g4, float err)
{
	/*//for grid on one plane
	if( IsSurfNetGridNodeOnOnePlane(g1) && IsSurfNetGridNodeOnOnePlane(g2) && 
		IsSurfNetGridNodeOnOnePlane(g3) && IsSurfNetGridNodeOnOnePlane(g4))
	{
		g1->Status & COMBINABLE = TRUE;
		//g1->nSize *= 4;
		g1->nSize = level*level;
		g1->err = Max4(g1->err,g2->err,g3->err,g4->err);
		return TRUE;
	}*/

	float maxErr = Max4(g1->err,g2->err,g3->err,g4->err);
	float allowableErr = err; //-maxErr;
	float newErr = MaxCombineErrEx(g1,g2,g3,g4,err);

	if (newErr>allowableErr)
	{
		g1->Status &= ~COMBINABLE;
		return FALSE;
	}
	else 
	{
		g1->Status |= COMBINABLE;
		//g1->nSize *= 4;
		g1->nSize = level*level;
		g1->err = newErr+maxErr;
		return TRUE;
	}
}

BOOL CSurfNetEditor::IsLowNeighborCombinable(int /*level*/,int i,int j)
{
	if (i<=0) 
		return TRUE;
	if(m_Surf[i][j].Status & COMBINABLE)
		return TRUE;
	else
		return (m_Surf[i][j].Status & UP_COMBINABLE);
}

BOOL CSurfNetEditor::IsRightNeighborCombinable(int level,int i,int j)
{
	if ( j>= (m_numGridU-1) || j>=(m_numGridU/level)) 
		return TRUE;
	if(m_Surf[i][j].Status & COMBINABLE)
		return TRUE;
	else
		return (m_Surf[i][j].Status & LT_COMBINABLE);
}

BOOL CSurfNetEditor::IsUpNeighborCombinable(int level,int i,int j)
{
	if ( i>= (m_numGridW-1) || i>= (m_numGridW/level)) 
	{
		return TRUE;//FALSE;
	}
	if(m_Surf[i][j].Status & COMBINABLE)
		return TRUE;
	else
		return (m_Surf[i][j].Status & DN_COMBINABLE);

}

BOOL CSurfNetEditor::IsLeftNeighborCombinable(int /*level*/,int i,int j)
{
	if (j<=0) 
		return TRUE;
	if(m_Surf[i][j].Status & COMBINABLE)
		return TRUE;
	else
		return (m_Surf[i][j].Status & RT_COMBINABLE);
}


BOOL CSurfNetEditor::CombineFourGridsOnPlane(int /*i*/,int /*j*/,int level,SurfNetGridNode* &g1,SurfNetGridNode* &g2,
											 SurfNetGridNode* &g3,SurfNetGridNode* &g4,int OrigGridIndex[])
{
	SurfNetGridNode* pGrid[4] = {NULL,NULL,NULL,NULL};
	pGrid[0] = g1;
	pGrid[1] = g2;
	pGrid[2] = g3;
	pGrid[3] = g4;
	BOOL bOnPlane = TRUE;
	for(int i=0;i<4;i++)
	{
		if(pGrid[i]->Status & DELETED)
			continue;
		if(!IsSurfNetGridNodeOnOnePlane(pGrid[i]))
		{
			bOnPlane = FALSE;
			break;
		}
	}

	if(bOnPlane)
	{
		g1->type = CELLTYPE_RECTANGLE;
		g1->Status &= ~BQUADDELETED; 
		g1->Status &= ~DELETED;   
		g1->nSize = level*level;

		g1->v[0] = OrigGridIndex[0];
		g1->v[1] = OrigGridIndex[1];
		g1->v[2] = OrigGridIndex[2];
		g1->v[3] = OrigGridIndex[3];

		g2->Status |= DELETED;
		g3->Status |= DELETED;
		g4->Status |= DELETED;
		
	}
	return TRUE;
}

//according to g1->Status & COMBINABLE and the situation around the four grid,
//combine the four grids or split them into triangles
BOOL CSurfNetEditor::ProcessFourGrids(int i,int j,int level,SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4)
{
	if (!(g1->Status & COMBINABLE))
		return FALSE;

	BOOL combinable[4];
	combinable[0] = IsLowNeighborCombinable(level,i-level,j);
	combinable[1] = IsRightNeighborCombinable(level,i,j+level);
	combinable[2] = IsUpNeighborCombinable(level,i+level,j);
	combinable[3] = IsLeftNeighborCombinable(level,i,j-level);

	if (combinable[0] && combinable[1] && combinable[2] && combinable[3] ) {
		//all neighbors are combinable. combine
		g1->v[1]=g2->v[1];
		g1->v[2]=g3->v[2];
		g1->v[3]=g4->v[3];

		//setting delete tag for g2 g3 g4;
		g2->Status |= DELETED;
		g3->Status |= DELETED;
		g4->Status |= DELETED;

		return TRUE;
	}
	else if (!combinable[0] && combinable[1] && combinable[2] && combinable[3] ) {
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[2]=g4->v[3];

		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[2]=g3->v[2];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g2->v[0];
		g3->v[1]=g3->v[2];
		g3->v[2]=g4->v[3];
		g4->Status |= DELETED;
	}
	else if (combinable[0] && !combinable[1] && combinable[2] && combinable[3] ) 
	{
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[1]=g2->v[2];
		g1->v[2]=g4->v[3];


		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[0]=g1->v[0];
		g2->v[2]=g3->v[1];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g4->v[3];

		g4->Status |= DELETED;
	}
	else if (combinable[0] && combinable[1] && !combinable[2] && combinable[3] ) 
	{
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[1]=g3->v[3];
		g1->v[2]=g4->v[3];


		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[0]=g1->v[0];
		g2->v[2]=g3->v[3];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g2->v[1];
		g3->v[1]=g3->v[2];
		g3->v[2]=g3->v[3];
		g4->Status |= DELETED;
	}
	else if (combinable[0] && combinable[1] && combinable[2] && !combinable[3] ) 
	{
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[1]=g2->v[1];
		g1->v[2]=g4->v[0];

		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[0]=g4->v[0];
		g2->v[2]=g3->v[2];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g4->v[0];
		g3->v[1]=g3->v[2];
		g3->v[2]=g4->v[3];

		g4->Status |= DELETED;
	}
	else if (!combinable[0] && !combinable[1] && combinable[2] && combinable[3] ) 
	{
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[2]=g4->v[3];

		g2->type = CELLTYPE_ONETRIANGLE;

		g4->type = CELLTYPE_ONETRIANGLE;
		g4->v[0]=g3->v[1];
		g4->v[1]=g3->v[2];
		g4->v[2]=g4->v[3];
		
		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g2->v[0];
		g3->v[2]=g4->v[3];

	}
	else if (!combinable[0] && combinable[1] && !combinable[2] && combinable[3] ) 
	{
		g1->type = CELLTYPE_RECTANGLE;
		g1->v[2]=g4->v[2];
		g1->v[3]=g4->v[3];
		g1->nSize /= 2;

		g2->type = CELLTYPE_RECTANGLE;
		g2->v[2]=g3->v[2];
		g2->v[3]=g3->v[3];

		g3->Status |= DELETED;
		g4->Status |= DELETED;
	}
	else if (!combinable[0] && combinable[1] && combinable[2] && !combinable[3] )
	{
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[2]=g4->v[0];

		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[2]=g3->v[2];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g4->v[0];
		g3->v[1]=g1->v[1];

		g4->type = CELLTYPE_ONETRIANGLE;
		g4->v[1]=g3->v[2];
		g4->v[2]=g4->v[3];
	}
	else if (combinable[0] && !combinable[1] && !combinable[2] && combinable[3] )
	{
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[1]=g2->v[1];
		g1->v[2]=g2->v[2];

		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[0]=g1->v[0];
		g2->v[1]=g3->v[1];
		g2->v[2]=g3->v[3];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g1->v[2];
		g3->v[1]=g3->v[2];
		g3->v[2]=g4->v[2];

		g4->type = CELLTYPE_ONETRIANGLE;
		g4->v[0]=g1->v[0];
		g4->v[1]=g2->v[2];
		g4->v[2]=g4->v[3];
	}
	else if (combinable[0] && !combinable[1] && combinable[2] && !combinable[3] )
	{
		g1->type = CELLTYPE_RECTANGLE;
		g1->v[1]=g2->v[1];
		g1->v[2]=g2->v[2];
		g1->nSize /= 2;

		g2->type = CELLTYPE_RECTANGLE;
		g2->v[0]=g4->v[0];
		g2->v[1]=g3->v[1];
		g2->v[2]=g3->v[2];
		g2->v[3]=g4->v[3];

		g3->Status |= DELETED;
		g4->Status |= DELETED;
	}
	else if (combinable[0] && combinable[1] && !combinable[2] && !combinable[3] ) 
	{
		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[1]=g2->v[1];
		g1->v[2]=g4->v[0];

		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[0]=g4->v[0];
		g2->v[2]=g4->v[2];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g1->v[1];
		g3->v[1]=g3->v[2];
		g3->v[2]=g4->v[2];

		g4->type = CELLTYPE_ONETRIANGLE;
		g4->v[1]=g4->v[2];
		g4->v[2]=g4->v[3];
	}
	else if (!combinable[0] && !combinable[1] && !combinable[2] && combinable[3] ) 
	{
		g1->type = CELLTYPE_RECTANGLE;
		g1->v[2]=g4->v[2];
		g1->v[3]=g4->v[3];
		g1->nSize /= 2;

		g2->type = CELLTYPE_ONETRIANGLE;

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g2->v[2];
		g3->v[1]=g3->v[2];
		g3->v[2]=g4->v[2];

		g4->type = CELLTYPE_ONETRIANGLE;
		g4->v[0]=g2->v[0];
		g4->v[1]=g3->v[0];
		g4->v[2]=g3->v[2];
	}
	else if (!combinable[0] && !combinable[1] && combinable[2] && !combinable[3] ) 
	{
		g4->type = CELLTYPE_RECTANGLE;
		g4->v[1]=g3->v[1];
		g4->v[2]=g3->v[2];

		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[2]=g4->v[0];

		g2->type = CELLTYPE_ONETRIANGLE;

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g2->v[0];
		g3->v[1]=g2->v[2];
		g3->v[2]=g4->v[0];
	}
	else if (!combinable[0] && combinable[1] && !combinable[2] && !combinable[3] ) 
	{
		g2->type = CELLTYPE_RECTANGLE;
		g2->v[2]=g3->v[2];
		g2->v[3]=g3->v[3];

		g1->type = CELLTYPE_ONETRIANGLE;
		g1->v[2]=g4->v[0];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g2->v[0];
		g3->v[1]=g2->v[3];
		g3->v[2]=g4->v[0];

		g4->type = CELLTYPE_ONETRIANGLE;
		g4->v[1]=g2->v[3];
		g4->v[2]=g4->v[3];
	}
	else if (combinable[0] && !combinable[1] && !combinable[2] && !combinable[3] )
	{
		g1->type = CELLTYPE_RECTANGLE;
		g1->v[1]=g2->v[1];
		g1->v[2]=g2->v[2];
		g1->nSize /= 2;

		g2->type = CELLTYPE_ONETRIANGLE;
		g2->v[0]=g4->v[0];
		g2->v[1]=g3->v[1];
		g2->v[2]=g4->v[2];

		g3->type = CELLTYPE_ONETRIANGLE;
		g3->v[0]=g2->v[1];
		g3->v[1]=g3->v[2];
		g3->v[2]=g4->v[2];

		g4->type = CELLTYPE_ONETRIANGLE;
		g4->v[1]=g2->v[2];
		g4->v[2]=g4->v[3];
	}
	else if (!combinable[0] && !combinable[1] && !combinable[2] && !combinable[3] )
	{
		//do nothing
	}	
	return TRUE;
}

void CSurfNetEditor::MakeOneTriangleLow(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* /*g3*/,SurfNetGridNode* /*g4*/)
{
	g1->type = CELLTYPE_ONETRIANGLE;
	g1->v[0] = g1->v[0];
	g1->v[1] = g2->v[1];
	g1->v[2] = g2->v[3];
}

////////////////////////
void CSurfNetEditor::MakeOneTriangleRight(SurfNetGridNode* /*g1*/,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* /*g4*/)
{
	g2->type = CELLTYPE_ONETRIANGLE;
	g2->v[0] = g2->v[1];
	g2->v[1] = g3->v[2];
	g2->v[2] = g3->v[0];
}

////////////////////////
void CSurfNetEditor::MakeOneTriangleUp(SurfNetGridNode* /*g1*/,SurfNetGridNode* /*g2*/,SurfNetGridNode* g3,SurfNetGridNode* g4)
{
	g3->type = CELLTYPE_ONETRIANGLE;
	g3->v[0] = g3->v[2];
	g3->v[1] = g4->v[3];
	g3->v[2] = g4->v[1];
}

////////////////////////

//g1,g2,g3,g4 in counterclockwise order
float CSurfNetEditor::MaxCombineErr(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4,float /*err*/)
{
	PT_3D p1,p2,p3;		//the three points to construct a plane
	PT_3D p;			//the three points to be tested
	float maxErr1;
	float maxErr2;

	float maxErr3;
	float maxErr4;

	//***********first half triangle
	//3 pts for the plane
	p1=m_pVet[g1->v[0]];
	p2=m_pVet[g2->v[1]];
	p3=m_pVet[g3->v[0]];
	//3 pts to be tested
	p=m_pVet[g2->v[0]];
	maxErr1=PointPlaneDist(p1,p2,p3,p);

	//***********second half triangle
	p1=m_pVet[g2->v[1]];
	p2=m_pVet[g3->v[0]];
	p3=m_pVet[g3->v[2]];
	//3 pts to be tested
	p=m_pVet[g3->v[1]];
	maxErr2=PointPlaneDist(p1,p2,p3,p);

	//
	p1=m_pVet[g3->v[0]];
	p2=m_pVet[g3->v[2]];
	p3=m_pVet[g4->v[3]];
	//3 pts to be tested
	p=m_pVet[g3->v[3]];
	maxErr3=PointPlaneDist(p1,p2,p3,p);

	//
	p1=m_pVet[g3->v[0]];
	p2=m_pVet[g1->v[0]];
	p3=m_pVet[g4->v[3]];
	//3 pts to be tested
	p=m_pVet[g1->v[3]];
	maxErr4=PointPlaneDist(p1,p2,p3,p);

	return Max4(maxErr1,maxErr2,maxErr3,maxErr4);
}

float CSurfNetEditor::MaxCombineErrEx(SurfNetGridNode* g1,SurfNetGridNode* g2,SurfNetGridNode* g3,SurfNetGridNode* g4,float /*err*/)
{
	PT_3D p1,p2,p3;		//the three points to construct a plane
	PT_3D p[3];			//the three points to be tested
	float maxErr1;
	float maxErr2;

	float maxErr3;
	float maxErr4;
	float Dis[3];

	//***********first half triangle
	//3 pts for the plane
	p1=m_pVet[g1->v[0]];
	p2=m_pVet[g2->v[1]];
	p3=m_pVet[g3->v[2]];
	//3 pts to be tested
	p[0]=m_pVet[g2->v[0]];
	p[1]=m_pVet[g2->v[2]];
	p[2]=m_pVet[g2->v[3]];
	maxErr1=MaxPointPlaneDist(p1,p2,p3,p,Dis);
	float maxDnErr1 = Dis[0];
	float maxRtErr1 = Dis[1];
	//float maxCenterErr1 = Dis[2];

	//***********second half triangle
	p1=m_pVet[g1->v[0]];
	p2=m_pVet[g4->v[3]];
	p3=m_pVet[g3->v[2]];
	//3 pts to be tested
	p[0]=m_pVet[g4->v[0]];
	p[1]=m_pVet[g4->v[1]];
	p[2]=m_pVet[g4->v[2]];
	maxErr2=MaxPointPlaneDist(p1,p2,p3,p,Dis);
	float maxLtErr1 = Dis[0];
	float maxUpErr1 = Dis[2];
	//float maxCenterErr2 = Dis[1];

	//
	p1=m_pVet[g2->v[1]];
	p2=m_pVet[g4->v[3]];
	p3=m_pVet[g1->v[0]];
	//3 pts to be tested
	p[0]=m_pVet[g1->v[1]];
	p[1]=m_pVet[g1->v[2]];
	p[2]=m_pVet[g1->v[3]];
	maxErr3=MaxPointPlaneDist(p1,p2,p3,p,Dis);
	float maxDnErr2 = Dis[0];
	float maxLtErr2 = Dis[2];
	//float maxCenterErr3 = Dis[1];

	//
	p1=m_pVet[g2->v[1]];
	p2=m_pVet[g3->v[2]];
	p3=m_pVet[g4->v[3]];
	//3 pts to be tested
	p[0]=m_pVet[g3->v[0]];
	p[1]=m_pVet[g3->v[1]];
	p[2]=m_pVet[g3->v[3]];
	maxErr4=MaxPointPlaneDist(p1,p2,p3,p,Dis);
	float maxRtErr2 = Dis[1];
	float maxUpErr2 = Dis[2];
	//float maxCenterErr4 = Dis[0];
	/*
	float maxUpErr = max(maxUpErr1,maxUpErr2);
	float maxDnErr = max(maxDnErr1,maxDnErr2);
	float maxLtErr = max(maxLtErr1,maxLtErr2);
	float maxRtErr = max(maxRtErr1,maxRtErr2);
	//float maxCenterErr = Max4(maxCenterErr1,maxCenterErr2,maxCenterErr3,maxCenterErr4);

	if(maxUpErr<err && max(maxCenterErr2,maxCenterErr4)<err)
		g1->UpCombinable = TRUE;
	if(maxDnErr<err && max(maxCenterErr1,maxCenterErr3)<err)
		g1->DnCombinable = TRUE;
	if(maxLtErr<err && max(maxCenterErr2,maxCenterErr3)<err)
		g1->LtCombinable = TRUE;
	if(maxRtErr<err && max(maxCenterErr1,maxCenterErr4)<err)
		g1->RtCombinable = TRUE;
	*/

	PT_3D Pt;
	p1=m_pVet[g4->v[3]];
	p2=m_pVet[g3->v[2]];
	p3=m_pVet[g3->v[0]];
	Pt=m_pVet[g3->v[3]];
	float maxUpErr = Max3(maxUpErr1,maxUpErr2,PointPlaneDist(p1,p2,p3,Pt));

	p1=m_pVet[g1->v[0]];
	p2=m_pVet[g2->v[1]];
	p3=m_pVet[g3->v[0]];
	Pt=m_pVet[g2->v[0]];
	float maxDnErr = Max3(maxDnErr1,maxDnErr2,PointPlaneDist(p1,p2,p3,Pt));

	p1=m_pVet[g1->v[0]];
	p2=m_pVet[g2->v[3]];
	p3=m_pVet[g4->v[3]];
	Pt=m_pVet[g1->v[3]];
	float maxLtErr = Max3(maxLtErr1,maxLtErr2,PointPlaneDist(p1,p2,p3,Pt));

	p1=m_pVet[g1->v[2]];
	p2=m_pVet[g2->v[1]];
	p3=m_pVet[g3->v[2]];
	Pt=m_pVet[g2->v[2]];
	float maxRtErr = Max3(maxRtErr1,maxRtErr2,PointPlaneDist(p1,p2,p3,Pt));
	//float maxCenterErr = Max4(maxCenterErr1,maxCenterErr2,maxCenterErr3,maxCenterErr4);

	if(maxUpErr<0.002 )
		g1->Status |= UP_COMBINABLE;
	if(maxDnErr<0.002 )
		g1->Status |= DN_COMBINABLE;
	if(maxLtErr<0.002 )
		g1->Status |= LT_COMBINABLE;
	if(maxRtErr<0.002 )
		g1->Status |= RT_COMBINABLE;

	//float maxDnErr = max(PointPlaneDist(p1,p2,p3,p),);
	return Max4(maxErr1,maxErr2,maxErr3,maxErr4);
}


//assumed 3 pts in array p[]
float CSurfNetEditor::MaxPointPlaneDist(PT_3D p1,PT_3D p2,PT_3D p3,PT_3D* p,float Dis[])
{
	float A,B,C,D;      //the plane equation---Ax+By+Cz+D=0
	PT_3D v1,v2;
	PT_3D norm;			//norm of the plane
	float err[3];

	v1=p2-p1;
	v2=p3-p2;

	norm=v1.Xproduct(v2);
	A=norm.x;
	B=norm.y;
	C=norm.z;
	D= -A*p1.x-B*p1.y-C*p1.z;

	if (fabs(A)<=MATHZERO && 
		fabs(B)<=MATHZERO &&
		fabs(C)<=MATHZERO)
		return 1000.f;  

	for (int i=0;i<3;i++) 
		Dis[i] = err[i]= (float) (fabs(A*p[i].x+B*p[i].y+C*p[i].z+D)/sqrt(A*A+B*B+C*C));
	return Max3(err[0],err[1],err[2]);
}

float CSurfNetEditor::PointPlaneDist(PT_3D p1,PT_3D p2,PT_3D p3,PT_3D p)
{
	float A,B,C,D;      //the plane equation---Ax+By+Cz+D=0
	PT_3D v1,v2;
	PT_3D norm;			//norm of the plane
	float err;

	v1=p2-p1;
	v2=p3-p2;

	norm=v1.Xproduct(v2);
	A=norm.x;
	B=norm.y;
	C=norm.z;
	D= -A*p1.x-B*p1.y-C*p1.z;

	if (fabs(A)<=MATHZERO && 
		fabs(B)<=MATHZERO &&
		fabs(C)<=MATHZERO)
		return 1000.f;  

	err = (float) (fabs(A*p.x+B*p.y+C*p.z+D)/sqrt(A*A+B*B+C*C));
	return err;
}

void ConvertPT3DToPNT3D( float * desPt, PT_3D & srcPt)
{
	desPt[0] = srcPt.x;
	desPt[1] = srcPt.y;
	desPt[2] = srcPt.z;
}

void CSurfNetEditor::ConvertGridSurfToSurfNet(CGeoNetSurf*& surfOut)
{
	CGeoNetSurf* nSurf = new CGeoNetSurf;
	SurfNetGridNode* p = NULL;
	int numGridX = m_numGridU;
	int numGridY = m_numGridW;

	for (int i=0;i<numGridY;i ++)
	{
		for (int j=0;j<numGridX;j ++) 
		{
			p=&m_Surf[i][j];
			p->Status &= ~BQUADDELETED ;
			p->Status &= ~BTRIDELETED;
		}
	}

	//delete zero plane polys if required
	int i0,i1,i2,i3;
	if (m_bDelFlat)
	{
		for (int i=0;i<numGridY;i ++)
		{	for (int j=0;j<numGridX;j ++)
			{
				p=&m_Surf[i][j];
				if ((p->Status & DELETED)) 
					continue;

				if (p->type == CELLTYPE_RECTANGLE )
				{
					i0  = p->v[0];
					i1  = p->v[1];
					i2  = p->v[2];
					i3  = p->v[3];
					if( m_pVet[i0].z <= m_fMinZAt &&
						m_pVet[i1].z <= m_fMinZAt &&
						m_pVet[i2].z <= m_fMinZAt &&
						m_pVet[i3].z <= m_fMinZAt)
                    {
						p->Status |= BQUADDELETED ;
                    }
				}
				else if (p->type == CELLTYPE_ONETRIANGLE) 
				{
					i0  = p->v[0];
					i1  = p->v[1];
					i2  = p->v[2];
					if( m_pVet[i0].z <= m_fMinZAt &&
						m_pVet[i1].z <= m_fMinZAt &&
						m_pVet[i2].z <= m_fMinZAt  )
                    {
						p->Status |= BTRIDELETED;
                    }
				}
			}
		}
	}
	//剖分扭曲的四边片
	if(m_bDisDistortRect)
    {
		DiscreteDistortRect(numGridY,numGridX);
    }
	//count vertex ref number and facet numbers
	int quadNum=0;
	int triNum =0;
	for (int i=0;i<numGridY;i ++)
	{
		for (int j=0;j<numGridX;j ++)
		{
			p=&m_Surf[i][j];
			if ((p->Status & DELETED)) 
				continue;

			if ((p->type == CELLTYPE_RECTANGLE )
				&& !(p->Status & BQUADDELETED)) 
			{
				m_refNum[p->v[0]]++;
				m_refNum[p->v[1]]++;
				m_refNum[p->v[2]]++;
				m_refNum[p->v[3]]++;
				quadNum++;
			}
			else if (p->type == CELLTYPE_ONETRIANGLE && !(p->Status & BTRIDELETED)) 
			{
				m_refNum[p->v[0]]++;
				m_refNum[p->v[1]]++;
				m_refNum[p->v[2]]++;
				triNum++;
			}
			else if (p->type == CELLTYPE_TWOTRIANGLE) 
			{
				m_refNum[p->v[0]]++;
				m_refNum[p->v[1]]++;
				m_refNum[p->v[3]]++;
				triNum++;

				m_refNum[p->v[1]]++;
				m_refNum[p->v[2]]++;
				m_refNum[p->v[3]]++;
				triNum++;

			}
			
		}
	}

	//create new index for the m_pVet array
	int numVetRefered=0;
	for (i=0;i<m_numVet;i++)  
	{
		if (m_refNum[i]>0 ) 
		{
			m_pos[i]=numVetRefered;
			numVetRefered++;
		}
	}

	//update vertex index of the facets
	for ( i=0;i<numGridY;i ++)
	{
		for (int j=0;j<numGridX;j ++) 
		{
			p=&m_Surf[i][j];
			if ((p->Status & DELETED)) 
				continue;

			if ((p->type == CELLTYPE_RECTANGLE )
				&& !(p->Status & BQUADDELETED))
			{
				p->v[0]=m_pos[p->v[0]];
				p->v[1]=m_pos[p->v[1]];
				p->v[2]=m_pos[p->v[2]];
				p->v[3]=m_pos[p->v[3]];
			}
			else if (p->type == CELLTYPE_ONETRIANGLE && !(p->Status&BTRIDELETED)) 
			{
				p->v[0]=m_pos[p->v[0]];
				p->v[1]=m_pos[p->v[1]];
				p->v[2]=m_pos[p->v[2]];
			}
			else if (p->type == CELLTYPE_TWOTRIANGLE)
			{
				p->v[0] = m_pos[p->v[0]];
				p->v[1] = m_pos[p->v[1]];
				p->v[2] = m_pos[p->v[2]];
				p->v[3] = m_pos[p->v[3]];
			}
		}
	}

	//to create the outline edge
#if 0 
	int* vetOutline=NULL;
	int numVetOutline=0;
	if (!m_bDeleteZeroZPoly)
	{
		numVetOutline=CountOutlineVertex(m_numVetU,m_numVetW);
		vetOutline=new int[numVetOutline];	
		CreateOutlineVertex(m_numVetU,m_numVetW,numVetOutline,vetOutline);
	}
#endif

	//create  surfNet
	//vertex
    if( numVetRefered < 3 )
    {
        delete nSurf ;
        return ;
    }
	nSurf->m_nNumVert = numVetRefered;
	nSurf->m_aVertex = new PT_3D[numVetRefered];

	int count=0;
	for (i=0;i<m_numVet;i++) 
	{
		if (m_refNum[i]>0 )
		{
			nSurf->m_aVertex[count][0]=m_pVet[i].x;
			nSurf->m_aVertex[count][1]=m_pVet[i].y;
			nSurf->m_aVertex[count][2]=m_pVet[i].z;
			count++;
		}
	}

	//facets
	nSurf->m_nNumQuad= quadNum; 
	if (quadNum>0)
	{
		nSurf->m_aQuad=new int[quadNum][4];
		quadNum=0;
 		for ( i=0;i<numGridY;i ++)
		{
			for (int j=0;j<numGridX;j ++) 
			{
				p=&m_Surf[i][j];
				if ((p->Status & DELETED) || (p->Status & BQUADDELETED)
					|| p->type!=CELLTYPE_RECTANGLE )
					continue;

				nSurf->m_aQuad[quadNum][0]=p->v[0];
				nSurf->m_aQuad[quadNum][1]=p->v[1];
				nSurf->m_aQuad[quadNum][2]=p->v[2];
				nSurf->m_aQuad[quadNum][3]=p->v[3];
				quadNum++;
			}
		}
	}

	nSurf->m_nNumTri= triNum; 
	float Pt1[3],Pt2[3],Pt3[3];
	float TriNormal[3];
	if (triNum>0) 
	{
		nSurf->m_aTri=new int[triNum][3];
		triNum=0;
 		for ( i=0;i<numGridY;i ++)
		{
			for (int j=0;j<numGridX;j ++)
			{
				p=&m_Surf[i][j];
				if ((p->Status & DELETED) ) 
					continue;
				if (p->type == CELLTYPE_ONETRIANGLE && !(p->Status&BTRIDELETED))
				{
					ConvertPT3DToPNT3D(Pt1,nSurf->m_aVertex[p->v[0]]);
					ConvertPT3DToPNT3D(Pt2,nSurf->m_aVertex[p->v[1]]);
					ConvertPT3DToPNT3D(Pt3,nSurf->m_aVertex[p->v[2]]);
					nc_CalcutNormal(Pt1,Pt2,Pt3,TriNormal);
					if(TriNormal[2]>0.0)
					{
						nSurf->m_aTri[triNum][0]=p->v[0];
						nSurf->m_aTri[triNum][1]=p->v[1];
						nSurf->m_aTri[triNum][2]=p->v[2];
					}
					else
					{
						nSurf->m_aTri[triNum][0]=p->v[1];
						nSurf->m_aTri[triNum][1]=p->v[0];
						nSurf->m_aTri[triNum][2]=p->v[2];
					}
					triNum++;
				}
				else if (p->type ==CELLTYPE_TWOTRIANGLE) 
				{
					ConvertPT3DToPNT3D(Pt1,nSurf->m_aVertex[p->v[0]]);
					ConvertPT3DToPNT3D(Pt2,nSurf->m_aVertex[p->v[1]]);
					ConvertPT3DToPNT3D(Pt3,nSurf->m_aVertex[p->v[3]]);
					nc_CalcutNormal(Pt1,Pt2,Pt3,TriNormal);
					if(TriNormal[2]>0.0)
					{
						nSurf->m_aTri[triNum][0]=p->v[0];
						nSurf->m_aTri[triNum][1]=p->v[1];
						nSurf->m_aTri[triNum][2]=p->v[3];
					}
					else
					{
						nSurf->m_aTri[triNum][0]=p->v[1];
						nSurf->m_aTri[triNum][1]=p->v[0];
						nSurf->m_aTri[triNum][2]=p->v[3];
					}
					triNum++;

					ConvertPT3DToPNT3D(Pt1,nSurf->m_aVertex[p->v[1]]);
					ConvertPT3DToPNT3D(Pt2,nSurf->m_aVertex[p->v[2]]);
					ConvertPT3DToPNT3D(Pt3,nSurf->m_aVertex[p->v[3]]);
					nc_CalcutNormal(Pt1,Pt2,Pt3,TriNormal);
					if(TriNormal[2]>0.0)
					{
						nSurf->m_aTri[triNum][0]=p->v[1];
						nSurf->m_aTri[triNum][1]=p->v[2];
						nSurf->m_aTri[triNum][2]=p->v[3];
					}
					else
					{
						nSurf->m_aTri[triNum][0]=p->v[2];
						nSurf->m_aTri[triNum][1]=p->v[1];
						nSurf->m_aTri[triNum][2]=p->v[3];
					}
					triNum++;
				}
			}
		}
	}

	//outline
#if 0 
	if (!m_bDelFlat)
	{
		nSurf->m_nNumEdge=1;
		nSurf->m_aNumVertEdge = new int[1];
		nSurf->m_aNumVertEdge[0] = numVetOutline;
		nSurf->m_aEdge  = new int*[1];
		nSurf->m_aEdge[0] = new int[numVetOutline];
		for (i=0;i<numVetOutline;i++)
			nSurf->m_aEdge[0][i]=vetOutline[i];
		delete[] vetOutline;
		vetOutline = NULL;
	}
#endif
	surfOut=nSurf;
}

void CSurfNetEditor::CreateOutlineVertex(int xNumVet,int yNumVet,int& numVetOutline,int*& vetOutline)
{
	int i,j;
	int k;

	numVetOutline=0;

	//lower edge
	i=0;
	for (j=0;j<xNumVet;j++)
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0) 
		{
			vetOutline[numVetOutline]=m_pos[k];
			numVetOutline++;
		}
	}

	//right edge
	j=xNumVet-1;
	for ( i=0+1;i<yNumVet-1;i++) 
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0)
		{
			vetOutline[numVetOutline]=m_pos[k];
			numVetOutline++;
		}
	}


	//top edge
	i=yNumVet-1;
	for (j=xNumVet-1;j>=0;j--) 
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0)
		{
			vetOutline[numVetOutline]=m_pos[k];
			numVetOutline++;
		}
	}

	//left edge, i>=0 "=" to close the loop
	j=0;
	for ( i=yNumVet-1-1;i>=0;i--) 
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0) 
		{
			vetOutline[numVetOutline]=m_pos[k];
			numVetOutline++;
		}
	}
} 


int CSurfNetEditor::CountOutlineVertex(int xNumVet,int yNumVet)
{
	int i,j;
	int numVetOutline=0;
	int k;

	//lower edge
	i=0;
	for (j=0;j<xNumVet;j++)
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0)
			numVetOutline++;
	}

	//right edge
	j=xNumVet-1;
	for ( i=0+1;i<yNumVet-1;i++)
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0)
			numVetOutline++;
	}


	//top edge
	i=yNumVet-1;
	for (j=xNumVet-1;j>=0;j--)
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0)
			numVetOutline++;
	}

	//left edge
	j=0;
	for ( i=yNumVet-1-1;i>=0;i--) 
	{
		k=i*xNumVet+j;
		if (m_refNum[k]>0)
			numVetOutline++;
	}

	return numVetOutline;
} 
#include "Section.H"
BOOL millface_FindSubGroup( Box3D& Box, 
                            DOUBLE Base[2]  , 
                            float Step[2]  , 
                            int Size[2]    , 
                            int& atBox )
{
    int nIDx = int( (Box.a.x - Base[0] ) / Step[0]) ;
	int nIDy = int( (Box.a.y - Base[1] ) / Step[1]) ;
	if( nIDx < 0 ) nIDx = 0  ;
	else if( nIDx >= Size[0] ) nIDx = Size[0]-1 ;
	if( nIDy < 0 ) nIDy = 0 ;
	else if( nIDy >= Size[1] ) nIDy = Size[1]-1 ;
	atBox = nIDy * Size[0]+ nIDx ;
	return TRUE ;
}
void millface_UpdateSubVertex(  CGeoNetSurf& BaseSurf, CGeoNetSurf& SubSurf, int VtFlag[] )
{
    memset( VtFlag, -1, sizeof( int) * BaseSurf.m_nNumVert ) ;
    for( int i = 0 ; i < SubSurf.m_nNumTri ; i ++ ) 
    {
       for( int k = 0 ; k < 3 ; k ++ ) 
       {
           VtFlag[ SubSurf.m_aTri[i][k]] = 1 ;
       }
    }
    for( i = 0 ; i < SubSurf.m_nNumQuad ; i ++ ) 
    {
       for( int k = 0 ; k < 4 ; k ++ ) 
       {
           VtFlag[ SubSurf.m_aQuad[i][k]] = 1 ;
       }
    }
    int nNumVt = 0 ;
    for( i = 0 ; i < BaseSurf.m_nNumVert ; i ++ ) 
    {
        if( VtFlag[i] != -1 )
        {
            VtFlag[i] = nNumVt ;
            nNumVt ++ ;
        }
    }
    SubSurf.m_aVertex = new PT_3D[ nNumVt ] ;
    SubSurf.m_nNumVert = nNumVt ;
    nNumVt = 0 ;
    for( i = 0 ; i < BaseSurf.m_nNumVert ; i ++ ) 
    {
        if( VtFlag[i] != -1 )
        {
            SubSurf.m_aVertex[ nNumVt] = BaseSurf.m_aVertex[i] ;
            nNumVt ++ ;
        }
    }
    for( i = 0 ; i < SubSurf.m_nNumTri ; i ++ ) 
    {
       for( int k = 0 ; k < 3 ; k ++ ) 
       {
           SubSurf.m_aTri[i][k] = VtFlag[ SubSurf.m_aTri[i][k]]  ;
       }
    }
    for( i = 0 ; i < SubSurf.m_nNumQuad ; i ++ ) 
    {
       for( int k = 0 ; k < 4 ; k ++ ) 
       {
           SubSurf.m_aQuad[i][k] = VtFlag[ SubSurf.m_aQuad[i][k]] ;
       }
    }
}

BOOL CSurfNetEditor::DepartNetSurf( CGeoNetSurf& OrgSurf, CPtrArray& SubSurf )
{
    int nTotal = OrgSurf.m_nNumVert / 100000 + 1 ;
    if( nTotal <= 1 ) return 0 ;
    if( OrgSurf.m_nNumQuad + OrgSurf.m_nNumTri < 100000 )
    {
        return 0 ; 
    }
    if( nTotal > 1000 ) nTotal = 1000 ;
    BOX3D surfBox ;
    OrgSurf.UpdateBox( & surfBox ) ;
    double fLen = (surfBox.max[0] - surfBox.min[0] ) * (surfBox.max[1] - surfBox.min[1] ) ;
	float fDist = (float)sqrt( fLen / float(nTotal) ); 
	int  nSize[2] ;
	nSize[0] = (int)ceil( (surfBox.max[0] - surfBox.min[0] )  / fDist ) ;
	nSize[1] = (int)ceil( (surfBox.max[1] - surfBox.min[1] )  / fDist ) ;
	if( nSize[0] < 1 ) nSize[0] = 1 ;
	if( nSize[1] < 1 ) nSize[1] = 1 ;
	float fStep[2] ;
	fStep[0] = (float)(surfBox.max[0] - surfBox.min[0] ) / float( nSize[0] ) ;
	fStep[1] = (float)(surfBox.max[1] - surfBox.min[1] ) / float( nSize[1] ) ;
	if( fStep[0] < 0.5f ) fStep[0] = 0.5f ;
	if( fStep[1] < 0.5f ) fStep[1] = 0.5f ;
	int nNumBox =  nSize[0] * nSize[1] ; 
    CGeoNetSurf** allSurf   = new CGeoNetSurf* [ nNumBox ] ;
    for( int k = 0 ; k < nNumBox ; k ++ ) 
    {
        allSurf[k] = new CGeoNetSurf() ;
    }
    int *nAtBox = new int[ OrgSurf.m_nNumQuad + OrgSurf.m_nNumTri ] ;
	// STEP 2 : 统计个数
	int nCell = 0 ;
    Box3D tmpBox ;
    for( int i = 0 ; i < OrgSurf.m_nNumTri ; i ++ )
	{
        for( int k = 0 ;   k < 3 ; k ++ ) 
        {
            PT_3D& pt = OrgSurf.m_aVertex[ OrgSurf.m_aTri[i][k] ] ;
            if( k == 0 )
            {
                tmpBox.a = tmpBox.b = pt ;
            }
            else
            {
                tmpBox.AddPt( pt ) ;
            }
        }
        millface_FindSubGroup( tmpBox, surfBox.min, fStep, nSize, nAtBox[nCell] ) ;
        allSurf[ nAtBox[nCell]]->m_nNumTri ++ ;
		nCell ++ ;
	}
    for( i = 0 ; i < OrgSurf.m_nNumQuad ; i ++ )
	{
        for( int k = 0 ;   k < 4 ; k ++ ) 
        {
            PT_3D& pt = OrgSurf.m_aVertex[ OrgSurf.m_aQuad[i][k] ] ;
            if( k == 0 )
            {
                tmpBox.a = tmpBox.b = pt ;
            }
            else
            {
                tmpBox.AddPt( pt ) ;
            }
        }
        millface_FindSubGroup( tmpBox, surfBox.min, fStep, nSize, nAtBox[nCell] ) ;
        allSurf[ nAtBox[nCell]]->m_nNumQuad ++ ;
		nCell ++ ;
	}
	// STEP 3 : 申请内存
	for( i = 0 ; i < nNumBox ; i ++ ) 
	{
        if( allSurf[i]->m_nNumQuad+allSurf[i]->m_nNumTri > 0 ) 
		{
            allSurf[i]->m_aTri = new int[allSurf[i]->m_nNumTri][3] ;
            allSurf[i]->m_aQuad = new int[allSurf[i]->m_nNumQuad][4] ;
            allSurf[i]->m_nNumQuad  = 0 ;
            allSurf[i]->m_nNumTri   = 0 ;
		}
	}
	// STEP 4 : 添加面片
	nCell = 0 ;
    for( i = 0 ; i < OrgSurf.m_nNumTri ; i ++ )
	{
        CGeoNetSurf* pSub = allSurf[ nAtBox[nCell]] ;
        memcpy( pSub->m_aTri[pSub->m_nNumTri], OrgSurf.m_aTri[i], sizeof( int[3])) ;
        pSub->m_nNumTri ++ ;
		nCell ++ ;
	}
    for( i = 0 ; i < OrgSurf.m_nNumQuad ; i ++ )
	{
        CGeoNetSurf* pSub = allSurf[ nAtBox[nCell]] ;
        memcpy( pSub->m_aQuad[pSub->m_nNumQuad], OrgSurf.m_aQuad[i], sizeof( int[4])) ;
        pSub->m_nNumQuad ++ ;
		nCell ++ ;
	}
	delete[] nAtBox ;
    // STEP 5 : 合并包围盒
    int  * allVt = new int[ OrgSurf.m_nNumVert ] ;
	for( i  = 0 ; i < nNumBox ; i ++ )
	{
        if( allSurf[i]->m_nNumQuad+allSurf[i]->m_nNumTri > 0 ) 
        {
            millface_UpdateSubVertex( OrgSurf, *allSurf[i],allVt) ;
            SubSurf.Add( allSurf[i]) ;
        }
        else
        {
            delete  allSurf[i] ;
        }
    }
    delete[] allVt ;
    delete[] allSurf ;
    return 1 ;
}
