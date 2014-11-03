#ifndef PATH_POINTS_DISTRIBUTE_H
#define PATH_POINTS_DISTRIBUTE_H

#include "SmartNC.H"
#include "Entity.H"
#include "SurfEtt.H"
#include "Convert.h"//guomin
#include "Nc5DPathTreat.h"

// RedistributeFlag
#define REDISTRIBUTE_ROUTH		0x0001		// 重新分布切削路径
#define REDISTRIBUTE_CONNECT	0x0002		// 重新分布连刀路径
#define REDISTRIBUTE_PLUNGE		0x0004		// 重新分布下刀路径
#define REDISTRIBUTE_LEAD		0x0008		// 重新分布进退刀路径
#define REDISTRIBUTE_SLOT		0x0010		// 重新分布开槽路径

//替代圆弧
void MathCAM_ReplaceArc_PathGroup(CPathGroup	*NewPath,
								  int			ReDistFlag,
								  DOUBLE		ArcTol,
								  DOUBLE		AngTol,
								  BOOL			bMaxDist,								  
								  double		dMaxDist) ;
//控制节点步长
void MathCAM_MaxPointDistance_PathGroup(CPathGroup	*NewPath,
										int			ReDistFlag,
										DOUBLE		ArcTol,
										DOUBLE		AngTol,
										DOUBLE		MaxStep ) ;
#endif