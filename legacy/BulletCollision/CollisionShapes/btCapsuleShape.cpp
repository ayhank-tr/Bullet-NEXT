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


#include "btCapsuleShape.h"

#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"

btlCapsuleShape::btlCapsuleShape(btlScalar radius, btlScalar height) : btlConvexInternalShape ()
{
	m_shapeType = CAPSULE_SHAPE_PROXYTYPE;
	m_upAxis = 1;
	m_implicitShapeDimensions.setValue(radius,0.5f*height,radius);
}

 
 btlVector3	btlCapsuleShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec0)const
{

	btlVector3 supVec(0,0,0);

	btlScalar maxDot(btlScalar(-BT_LARGE_FLOAT));

	btlVector3 vec = vec0;
	btlScalar lenSqr = vec.length2();
	if (lenSqr < btlScalar(0.0001))
	{
		vec.setValue(1,0,0);
	} else
	{
		btlScalar rlen = btlScalar(1.) / btlSqrt(lenSqr );
		vec *= rlen;
	}

	btlVector3 vtx;
	btlScalar newDot;
	
	btlScalar radius = getRadius();


	{
		btlVector3 pos(0,0,0);
		pos[getUpAxis()] = getHalfHeight();

		vtx = pos +vec*(radius) - vec * getMargin();
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}
	{
		btlVector3 pos(0,0,0);
		pos[getUpAxis()] = -getHalfHeight();

		vtx = pos +vec*(radius) - vec * getMargin();
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}

	return supVec;

}

 void	btlCapsuleShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{

	
	btlScalar radius = getRadius();

	for (int j=0;j<numVectors;j++)
	{
		btlScalar maxDot(btlScalar(-BT_LARGE_FLOAT));
		const btlVector3& vec = vectors[j];

		btlVector3 vtx;
		btlScalar newDot;
		{
			btlVector3 pos(0,0,0);
			pos[getUpAxis()] = getHalfHeight();
			vtx = pos +vec*(radius) - vec * getMargin();
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
		{
			btlVector3 pos(0,0,0);
			pos[getUpAxis()] = -getHalfHeight();
			vtx = pos +vec*(radius) - vec * getMargin();
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
		
	}
}


void	btlCapsuleShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	//as an approximation, take the inertia of the box that bounds the spheres

	btlTransform ident;
	ident.setIdentity();

	
	btlScalar radius = getRadius();

	btlVector3 halfExtents(radius,radius,radius);
	halfExtents[getUpAxis()]+=getHalfHeight();

	btlScalar margin = CONVEX_DISTANCE_MARGIN;

	btlScalar lx=btlScalar(2.)*(halfExtents[0]+margin);
	btlScalar ly=btlScalar(2.)*(halfExtents[1]+margin);
	btlScalar lz=btlScalar(2.)*(halfExtents[2]+margin);
	const btlScalar x2 = lx*lx;
	const btlScalar y2 = ly*ly;
	const btlScalar z2 = lz*lz;
	const btlScalar scaledmass = mass * btlScalar(.08333333);

	inertia[0] = scaledmass * (y2+z2);
	inertia[1] = scaledmass * (x2+z2);
	inertia[2] = scaledmass * (x2+y2);

}

btlCapsuleShapeX::btlCapsuleShapeX(btlScalar radius,btlScalar height)
{
	m_upAxis = 0;
	m_implicitShapeDimensions.setValue(0.5f*height, radius,radius);
}






btlCapsuleShapeZ::btlCapsuleShapeZ(btlScalar radius,btlScalar height)
{
	m_upAxis = 2;
	m_implicitShapeDimensions.setValue(radius,radius,0.5f*height);
}




