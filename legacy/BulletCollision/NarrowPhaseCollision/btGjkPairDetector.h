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




#ifndef BT_GJK_PAIR_DETECTOR_H
#define BT_GJK_PAIR_DETECTOR_H

#include "btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

class btlConvexShape;
#include "btSimplexSolverInterface.h"
class btlConvexPenetrationDepthSolver;

/// btGjkPairDetector uses GJK to implement the btDiscreteCollisionDetectorInterface
class btlGjkPairDetector : public btlDiscreteCollisionDetectorInterface
{
	

	btlVector3	m_cachedSeparatingAxis;
	btlConvexPenetrationDepthSolver*	m_penetrationDepthSolver;
	btlSimplexSolverInterface* m_simplexSolver;
	const btlConvexShape* m_minkowskiA;
	const btlConvexShape* m_minkowskiB;
	int	m_shapeTypeA;
	int m_shapeTypeB;
	btlScalar	m_marginA;
	btlScalar	m_marginB;

	bool		m_ignoreMargin;
	btlScalar	m_cachedSeparatingDistance;
	

public:

	//some debugging to fix degeneracy problems
	int			m_lastUsedMethod;
	int			m_curIter;
	int			m_degenerateSimplex;
	int			m_catchDegeneracies;
	int			m_fixContactNormalDirection;

	btlGjkPairDetector(const btlConvexShape* objectA,const btlConvexShape* objectB,btlSimplexSolverInterface* simplexSolver,btlConvexPenetrationDepthSolver*	penetrationDepthSolver);
	btlGjkPairDetector(const btlConvexShape* objectA,const btlConvexShape* objectB,int shapeTypeA,int shapeTypeB,btlScalar marginA, btlScalar marginB, btlSimplexSolverInterface* simplexSolver,btlConvexPenetrationDepthSolver*	penetrationDepthSolver);
	virtual ~btlGjkPairDetector() {};

	virtual void	getClosestPoints(const ClosestPointInput& input,Result& output,class btlIDebugDraw* debugDraw,bool swapResults=false);

	void	getClosestPointsNonVirtual(const ClosestPointInput& input,Result& output,class btlIDebugDraw* debugDraw);
	

	void setMinkowskiA(const btlConvexShape* minkA)
	{
		m_minkowskiA = minkA;
	}

	void setMinkowskiB(const btlConvexShape* minkB)
	{
		m_minkowskiB = minkB;
	}
	void setCachedSeperatingAxis(const btlVector3& seperatingAxis)
	{
		m_cachedSeparatingAxis = seperatingAxis;
	}

	const btlVector3& getCachedSeparatingAxis() const
	{
		return m_cachedSeparatingAxis;
	}
	btlScalar	getCachedSeparatingDistance() const
	{
		return m_cachedSeparatingDistance;
	}

	void	setPenetrationDepthSolver(btlConvexPenetrationDepthSolver*	penetrationDepthSolver)
	{
		m_penetrationDepthSolver = penetrationDepthSolver;
	}

	///don't use setIgnoreMargin, it's for Bullet's internal use
	void	setIgnoreMargin(bool ignoreMargin)
	{
		m_ignoreMargin = ignoreMargin;
	}


};

#endif //BT_GJK_PAIR_DETECTOR_H
