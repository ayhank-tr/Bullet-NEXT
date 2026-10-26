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

#include "btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"

btlVector3	btlSphereShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
{
	(void)vec;
	return btlVector3(btlScalar(0.),btlScalar(0.),btlScalar(0.));
}

void	btlSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	(void)vectors;

	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i].setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	}
}


btlVector3	btlSphereShape::localGetSupportingVertex(const btlVector3& vec)const
{
	btlVector3 supVertex;
	supVertex = localGetSupportingVertexWithoutMargin(vec);

	btlVector3 vecnorm = vec;
	if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
	{
		vecnorm.setValue(btlScalar(-1.),btlScalar(-1.),btlScalar(-1.));
	} 
	vecnorm.normalize();
	supVertex+= getMargin() * vecnorm;
	return supVertex;
}


//broken due to scaling
void btlSphereShape::getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	const btlVector3& center = t.getOrigin();
	btlVector3 extent(getMargin(),getMargin(),getMargin());
	aabbMin = center - extent;
	aabbMax = center + extent;
}



void	btlSphereShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	btlScalar elem = btlScalar(0.4) * mass * getMargin()*getMargin();
	inertia.setValue(elem,elem,elem);

}

