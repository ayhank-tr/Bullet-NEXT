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

#include "btUniformScalingShape.h"

btlUniformScalingShape::btlUniformScalingShape(	btlConvexShape* convexChildShape,btlScalar uniformScalingFactor):
btlConvexShape (), m_childConvexShape(convexChildShape),
m_uniformScalingFactor(uniformScalingFactor)
{
	m_shapeType = UNIFORM_SCALING_SHAPE_PROXYTYPE;
}
	
btlUniformScalingShape::~btlUniformScalingShape()
{
}
	

btlVector3	btlUniformScalingShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
{
	btlVector3 tmpVertex;
	tmpVertex = m_childConvexShape->localGetSupportingVertexWithoutMargin(vec);
	return tmpVertex*m_uniformScalingFactor;
}

void	btlUniformScalingShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	m_childConvexShape->batchedUnitVectorGetSupportingVertexWithoutMargin(vectors,supportVerticesOut,numVectors);
	int i;
	for (i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = supportVerticesOut[i] * m_uniformScalingFactor;
	}
}


btlVector3	btlUniformScalingShape::localGetSupportingVertex(const btlVector3& vec)const
{
	btlVector3 tmpVertex;
	tmpVertex = m_childConvexShape->localGetSupportingVertex(vec);
	return tmpVertex*m_uniformScalingFactor;
}


void	btlUniformScalingShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{

	///this linear upscaling is not realistic, but we don't deal with large mass ratios...
	btlVector3 tmpInertia;
	m_childConvexShape->calculateLocalInertia(mass,tmpInertia);
	inertia = tmpInertia * m_uniformScalingFactor;
}


	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
void btlUniformScalingShape::getAabb(const btlTransform& trans,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	getAabbSlow(trans,aabbMin,aabbMax);

}

void btlUniformScalingShape::getAabbSlow(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const
{
#if 1
	btlVector3 _directions[] =
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

	for (int i=0;i<6;i++)
	{
		_directions[i] = _directions[i]*t.getBasis();
	}
	
	batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
	
	btlVector3 aabbMin1(0,0,0),aabbMax1(0,0,0);

	for ( int i = 0; i < 3; ++i )
	{
		aabbMax1[i] = t(_supporting[i])[i];
		aabbMin1[i] = t(_supporting[i + 3])[i];
	}
	btlVector3 marginVec(getMargin(),getMargin(),getMargin());
	aabbMin = aabbMin1-marginVec;
	aabbMax = aabbMax1+marginVec;
	
#else

	btlScalar margin = getMargin();
	for (int i=0;i<3;i++)
	{
		btlVector3 vec(btlScalar(0.),btlScalar(0.),btlScalar(0.));
		vec[i] = btlScalar(1.);
		btlVector3 sv = localGetSupportingVertex(vec*t.getBasis());
		btlVector3 tmp = t(sv);
		aabbMax[i] = tmp[i]+margin;
		vec[i] = btlScalar(-1.);
		sv = localGetSupportingVertex(vec*t.getBasis());
		tmp = t(sv);
		aabbMin[i] = tmp[i]-margin;
	}

#endif
}

void	btlUniformScalingShape::setLocalScaling(const btlVector3& scaling) 
{
	m_childConvexShape->setLocalScaling(scaling);
}

const btlVector3& btlUniformScalingShape::getLocalScaling() const
{
	return m_childConvexShape->getLocalScaling();
}

void	btlUniformScalingShape::setMargin(btlScalar margin)
{
	m_childConvexShape->setMargin(margin);
}
btlScalar	btlUniformScalingShape::getMargin() const
{
	return m_childConvexShape->getMargin() * m_uniformScalingFactor;
}

int		btlUniformScalingShape::getNumPreferredPenetrationDirections() const
{
	return m_childConvexShape->getNumPreferredPenetrationDirections();
}
	
void	btlUniformScalingShape::getPreferredPenetrationDirection(int index, btlVector3& penetrationVector) const
{
	m_childConvexShape->getPreferredPenetrationDirection(index,penetrationVector);
}
