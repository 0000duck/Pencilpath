#ifndef PATH_POINTS_DISTRIBUTE_H
#define PATH_POINTS_DISTRIBUTE_H

#include "SmartNC.H"
#include "Entity.H"
#include "SurfEtt.H"
#include "Convert.h"//guomin
#include "Nc5DPathTreat.h"

// RedistributeFlag
#define REDISTRIBUTE_ROUTH		0x0001		// ���·ֲ�����·��
#define REDISTRIBUTE_CONNECT	0x0002		// ���·ֲ�����·��
#define REDISTRIBUTE_PLUNGE		0x0004		// ���·ֲ��µ�·��
#define REDISTRIBUTE_LEAD		0x0008		// ���·ֲ����˵�·��
#define REDISTRIBUTE_SLOT		0x0010		// ���·ֲ�����·��

//���Բ��
void MathCAM_ReplaceArc_PathGroup(CPathGroup	*NewPath,
								  int			ReDistFlag,
								  DOUBLE		ArcTol,
								  DOUBLE		AngTol,
								  BOOL			bMaxDist,								  
								  double		dMaxDist) ;
//���ƽڵ㲽��
void MathCAM_MaxPointDistance_PathGroup(CPathGroup	*NewPath,
										int			ReDistFlag,
										DOUBLE		ArcTol,
										DOUBLE		AngTol,
										DOUBLE		MaxStep ) ;
#endif