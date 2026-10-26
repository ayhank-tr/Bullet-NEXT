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

#include "btStaticPlaneShape.h"

#include "LinearMath/btTransformUtil.h"


btlStaticPlaneShape::btlStaticPlaneShape(const btlVector3& planeNormal,btlScalar planeConstant)
: btlConcaveShape (), m_planeNormal(planeNormal.normalized()),
m_planeConstant(planeConstant),
m_localScaling(btlScalar(1.),btlScalar(1.),btlScalar(1.))
{
	m_shapeType = STATIC_PLANE_PROXYTYPE;
	//	btAssert( btFuzzyZero(m_planeNormal.length() - btScalar(1.)) );
}


btlStaticPlaneShape::~btlStaticPlaneShape()
{
}



void btlStaticPlaneShape::getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	(void)t;
	/*
	btVector3 infvec (btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT),btScalar(BT_LARGE_FLOAT));

	btVector3 center = m_planeNormal*m_planeConstant;
	aabbMin = center + infvec*m_planeNormal;
	aabbMax = aabbMin;
	aabbMin.setMin(center - infvec*m_planeNormal);
	aabbMax.setMax(center - infvec*m_planeNormal); 
	*/

	aabbMin.setValue(btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT));
	aabbMax.setValue(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));

}




void	btlStaticPlaneShape::processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const
{

	btlVector3 halfExtents = (aabbMax - aabbMin) * btlScalar(0.5);
	btlScalar radius = halfExtents.length();
	btlVector3 center = (aabbMax + aabbMin) * btlScalar(0.5);
	
	//this is where the triangles are generated, given AABB and plane equation (normal/constant)

	btlVector3 tangentDir0,tangentDir1;

	//tangentDir0/tangentDir1 can be precalculated
	btlPlaneSpace1(m_planeNormal,tangentDir0,tangentDir1);

	btlVector3 supVertex0,supVertex1;

	btlVector3 projectedCenter = center - (m_planeNormal.dot(center) - m_planeConstant)*m_planeNormal;
	
	btlVector3 triangle[3];
	triangle[0] = projectedCenter + tangentDir0*radius + tangentDir1*radius;
	triangle[1] = projectedCenter + tangentDir0*radius - tangentDir1*radius;
	triangle[2] = projectedCenter - tangentDir0*radius - tangentDir1*radius;

	callback->processTriangle(triangle,0,0);

	triangle[0] = projectedCenter - tangentDir0*radius - tangentDir1*radius;
	triangle[1] = projectedCenter - tangentDir0*radius + tangentDir1*radius;
	triangle[2] = projectedCenter + tangentDir0*radius + tangentDir1*radius;

	callback->processTriangle(triangle,0,1);

}

void	btlStaticPlaneShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	(void)mass;

	//moving concave objects not supported
	
	inertia.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
}

void	btlStaticPlaneShape::setLocalScaling(const btlVector3& scaling)
{
	m_localScaling = scaling;
}
const btlVector3& btlStaticPlaneShape::getLocalScaling() const
{
	return m_localScaling;
}
