/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_SIMPLEX_SOLVER_INTERFACE_H
#define BT_SIMPLEX_SOLVER_INTERFACE_H

#include "LinearMath/btVector3.h"

#define NO_VIRTUAL_INTERFACE 1
#ifdef NO_VIRTUAL_INTERFACE
#include "btVoronoiSimplexSolver.h"
#define btlSimplexSolverInterface btlVoronoiSimplexSolver
#else

/// btSimplexSolverInterface can incrementally calculate distance between origin and up to 4 vertices
/// Used by GJK or Linear Casting. Can be implemented by the Johnson-algorithm or alternative approaches based on
/// voronoi regions or barycentric coordinates
class btlSimplexSolverInterface
{
	public:
		virtual ~btlSimplexSolverInterface() {};

	virtual void reset() = 0;

	virtual void addVertex(const btlVector3& w, const btlVector3& p, const btlVector3& q) = 0;
	
	virtual bool closest(btlVector3& v) = 0;

	virtual btlScalar maxVertex() = 0;

	virtual bool fullSimplex() const = 0;

	virtual int getSimplex(btlVector3 *pBuf, btlVector3 *qBuf, btlVector3 *yBuf) const = 0;

	virtual bool inSimplex(const btlVector3& w) = 0;
	
	virtual void backup_closest(btlVector3& v) = 0;

	virtual bool emptySimplex() const = 0;

	virtual void compute_points(btlVector3& p1, btlVector3& p2) = 0;

	virtual int numVertices() const =0;


};
#endif
#endif //BT_SIMPLEX_SOLVER_INTERFACE_H

