/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btConvexInternalShape.h"



btlConvexInternalShape::btlConvexInternalShape()
: m_localScaling(btlScalar(1.),btlScalar(1.),btlScalar(1.)),
m_collisionMargin(CONVEX_DISTANCE_MARGIN)
{
}


void	btlConvexInternalShape::setLocalScaling(const btlVector3& scaling)
{
	m_localScaling = scaling.absolute();
}



void	btlConvexInternalShape::getAabbSlow(const btlTransform& trans,btlVector3&minAabb,btlVector3&maxAabb) const
{
#ifndef __SPU__
	//use localGetSupportingVertexWithoutMargin?
	btlScalar margin = getMargin();
	for (int i=0;i<3;i++)
	{
		btlVector3 vec(btlScalar(0.),btlScalar(0.),btlScalar(0.));
		vec[i] = btlScalar(1.);

		btlVector3 sv = localGetSupportingVertex(vec*trans.getBasis());

		btlVector3 tmp = trans(sv);
		maxAabb[i] = tmp[i]+margin;
		vec[i] = btlScalar(-1.);
		tmp = trans(localGetSupportingVertex(vec*trans.getBasis()));
		minAabb[i] = tmp[i]-margin;
	}
#endif
}



btlVector3	btlConvexInternalShape::localGetSupportingVertex(const btlVector3& vec)const
{
#ifndef __SPU__

	 btlVector3	supVertex = localGetSupportingVertexWithoutMargin(vec);

	if ( getMargin()!=btlScalar(0.) )
	{
		btlVector3 vecnorm = vec;
		if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
		{
			vecnorm.setValue(btlScalar(-1.),btlScalar(-1.),btlScalar(-1.));
		} 
		vecnorm.normalize();
		supVertex+= getMargin() * vecnorm;
	}
	return supVertex;

#else
	btlAssert(0);
	return btlVector3(0,0,0);
#endif //__SPU__

 }


btlConvexInternalAabbCachingShape::btlConvexInternalAabbCachingShape()
	:	btlConvexInternalShape(),
m_localAabbMin(1,1,1),
m_localAabbMax(-1,-1,-1),
m_isLocalAabbValid(false)
{
}


void btlConvexInternalAabbCachingShape::getAabb(const btlTransform& trans,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	getNonvirtualAabb(trans,aabbMin,aabbMax,getMargin());
}

void	btlConvexInternalAabbCachingShape::setLocalScaling(const btlVector3& scaling)
{
	btlConvexInternalShape::setLocalScaling(scaling);
	recalcLocalAabb();
}


void	btlConvexInternalAabbCachingShape::recalcLocalAabb()
{
	m_isLocalAabbValid = true;
	
	#if 1
	static const btlVector3 _directions[] =
	{
		btlVector3( 1.,  0.,  0.),
		btlVector3( 0.,  1.,  0.),
		btlVector3( 0.,  0.,  1.),
		btlVector3( -1., 0.,  0.),
		btlVector3( 0., -1.,  0.),
		btlVector3( 0.,  0., -1.)
	};
	
	btlVector3 _supporting[] =
	{
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.)
	};
	
	batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
	
	for ( int i = 0; i < 3; ++i )
	{
		m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
		m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
	}
	
	#else

	for (int i=0;i<3;i++)
	{
		btlVector3 vec(btlScalar(0.),btlScalar(0.),btlScalar(0.));
		vec[i] = btlScalar(1.);
		btlVector3 tmp = localGetSupportingVertex(vec);
		m_localAabbMax[i] = tmp[i]+m_collisionMargin;
		vec[i] = btlScalar(-1.);
		tmp = localGetSupportingVertex(vec);
		m_localAabbMin[i] = tmp[i]-m_collisionMargin;
	}
	#endif
}
