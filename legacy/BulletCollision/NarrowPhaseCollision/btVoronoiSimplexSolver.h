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



#ifndef BT_VORONOI_SIMPLEX_SOLVER_H
#define BT_VORONOI_SIMPLEX_SOLVER_H

#include "btSimplexSolverInterface.h"



#define VORONOI_SIMPLEX_MAX_VERTS 5

///disable next define, or use defaultCollisionConfiguration->getSimplexSolver()->setEqualVertexThreshold(0.f) to disable/configure
#define BT_USE_EQUAL_VERTEX_THRESHOLD
#define VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD 0.0001f


struct btlUsageBitfield{
	btlUsageBitfield()
	{
		reset();
	}

	void reset()
	{
		usedVertexA = false;
		usedVertexB = false;
		usedVertexC = false;
		usedVertexD = false;
	}
	unsigned short usedVertexA	: 1;
	unsigned short usedVertexB	: 1;
	unsigned short usedVertexC	: 1;
	unsigned short usedVertexD	: 1;
	unsigned short unused1		: 1;
	unsigned short unused2		: 1;
	unsigned short unused3		: 1;
	unsigned short unused4		: 1;
};


struct	btlSubSimplexClosestResult
{
	btlVector3	m_closestPointOnSimplex;
	//MASK for m_usedVertices
	//stores the simplex vertex-usage, using the MASK, 
	// if m_usedVertices & MASK then the related vertex is used
	btlUsageBitfield	m_usedVertices;
	btlScalar	m_barycentricCoords[4];
	bool m_degenerate;

	void	reset()
	{
		m_degenerate = false;
		setBarycentricCoordinates();
		m_usedVertices.reset();
	}
	bool	isValid()
	{
		bool valid = (m_barycentricCoords[0] >= btlScalar(0.)) &&
			(m_barycentricCoords[1] >= btlScalar(0.)) &&
			(m_barycentricCoords[2] >= btlScalar(0.)) &&
			(m_barycentricCoords[3] >= btlScalar(0.));


		return valid;
	}
	void	setBarycentricCoordinates(btlScalar a=btlScalar(0.),btlScalar b=btlScalar(0.),btlScalar c=btlScalar(0.),btlScalar d=btlScalar(0.))
	{
		m_barycentricCoords[0] = a;
		m_barycentricCoords[1] = b;
		m_barycentricCoords[2] = c;
		m_barycentricCoords[3] = d;
	}

};

/// btVoronoiSimplexSolver is an implementation of the closest point distance algorithm from a 1-4 points simplex to the origin.
/// Can be used with GJK, as an alternative to Johnson distance algorithm.
#ifdef NO_VIRTUAL_INTERFACE
ATTRIBUTE_ALIGNED16(class) btlVoronoiSimplexSolver
#else
ATTRIBUTE_ALIGNED16(class) btlVoronoiSimplexSolver : public btlSimplexSolverInterface
#endif
{
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	int	m_numVertices;

	btlVector3	m_simplexVectorW[VORONOI_SIMPLEX_MAX_VERTS];
	btlVector3	m_simplexPointsP[VORONOI_SIMPLEX_MAX_VERTS];
	btlVector3	m_simplexPointsQ[VORONOI_SIMPLEX_MAX_VERTS];

	

	btlVector3	m_cachedP1;
	btlVector3	m_cachedP2;
	btlVector3	m_cachedV;
	btlVector3	m_lastW;
	
	btlScalar	m_equalVertexThreshold;
	bool		m_cachedValidClosest;


	btlSubSimplexClosestResult m_cachedBC;

	bool	m_needsUpdate;
	
	void	removeVertex(int index);
	void	reduceVertices (const btlUsageBitfield& usedVerts);
	bool	updateClosestVectorAndPoints();

	bool	closestPtPointTetrahedron(const btlVector3& p, const btlVector3& a, const btlVector3& b, const btlVector3& c, const btlVector3& d, btlSubSimplexClosestResult& finalResult);
	int		pointOutsideOfPlane(const btlVector3& p, const btlVector3& a, const btlVector3& b, const btlVector3& c, const btlVector3& d);
	bool	closestPtPointTriangle(const btlVector3& p, const btlVector3& a, const btlVector3& b, const btlVector3& c,btlSubSimplexClosestResult& result);

public:

	btlVoronoiSimplexSolver()
		:  m_equalVertexThreshold(VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD)
	{
	}
	 void reset();

	 void addVertex(const btlVector3& w, const btlVector3& p, const btlVector3& q);

	 void	setEqualVertexThreshold(btlScalar threshold)
	 {
		 m_equalVertexThreshold = threshold;
	 }

	 btlScalar	getEqualVertexThreshold() const
	 {
		 return m_equalVertexThreshold;
	 }

	 bool closest(btlVector3& v);

	 btlScalar maxVertex();

	 bool fullSimplex() const
	 {
		 return (m_numVertices == 4);
	 }

	 int getSimplex(btlVector3 *pBuf, btlVector3 *qBuf, btlVector3 *yBuf) const;

	 bool inSimplex(const btlVector3& w);
	
	 void backup_closest(btlVector3& v) ;

	 bool emptySimplex() const ;

	 void compute_points(btlVector3& p1, btlVector3& p2) ;

	 int numVertices() const 
	 {
		 return m_numVertices;
	 }


};

#endif //BT_VORONOI_SIMPLEX_SOLVER_H

