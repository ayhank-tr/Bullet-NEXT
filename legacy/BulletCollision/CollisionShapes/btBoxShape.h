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

#ifndef BT_OBB_BOX_MINKOWSKI_H
#define BT_OBB_BOX_MINKOWSKI_H

#include "btPolyhedralConvexShape.h"
#include "btCollisionMargin.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMinMax.h"

///The btBoxShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.
ATTRIBUTE_ALIGNED16(class) btlBoxShape: public btlPolyhedralConvexShape
{

	//btVector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead


public:

BT_DECLARE_ALIGNED_ALLOCATOR();

	btlVector3 getHalfExtentsWithMargin() const
	{
		btlVector3 halfExtents = getHalfExtentsWithoutMargin();
		btlVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		return halfExtents;
	}
	
	const btlVector3& getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;//scaling is included, margin is not
	}
	

	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec) const
	{
		btlVector3 halfExtents = getHalfExtentsWithoutMargin();
		btlVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		
		return btlVector3(btlFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
			btlFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
			btlFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	SIMD_FORCE_INLINE  btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
	{
		const btlVector3& halfExtents = getHalfExtentsWithoutMargin();
		
		return btlVector3(btlFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
			btlFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
			btlFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
	{
		const btlVector3& halfExtents = getHalfExtentsWithoutMargin();
	
		for (int i=0;i<numVectors;i++)
		{
			const btlVector3& vec = vectors[i];
			supportVerticesOut[i].setValue(btlFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
				btlFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
				btlFsels(vec.z(), halfExtents.z(), -halfExtents.z())); 
		}

	}


	btlBoxShape( const btlVector3& boxHalfExtents);

	virtual void setMargin(btlScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		btlVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btlVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		
		btlConvexInternalShape::setMargin(collisionMargin);
		btlVector3 newMargin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

	}
	virtual void	setLocalScaling(const btlVector3& scaling)
	{
		btlVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btlVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		btlVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		btlConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

	}

	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	

	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

	virtual void getPlane(btlVector3& planeNormal,btlVector3& planeSupport,int i ) const
	{
		//this plane might not be aligned...
		btlVector4 plane ;
		getPlaneEquation(plane,i);
		planeNormal = btlVector3(plane.getX(),plane.getY(),plane.getZ());
		planeSupport = localGetSupportingVertex(-planeNormal);
	}

	
	virtual int getNumPlanes() const
	{
		return 6;
	}	
	
	virtual int	getNumVertices() const 
	{
		return 8;
	}

	virtual int getNumEdges() const
	{
		return 12;
	}


	virtual void getVertex(int i,btlVector3& vtx) const
	{
		btlVector3 halfExtents = getHalfExtentsWithMargin();

		vtx = btlVector3(
				halfExtents.x() * (1-(i&1)) - halfExtents.x() * (i&1),
				halfExtents.y() * (1-((i&2)>>1)) - halfExtents.y() * ((i&2)>>1),
				halfExtents.z() * (1-((i&4)>>2)) - halfExtents.z() * ((i&4)>>2));
	}
	

	virtual void	getPlaneEquation(btlVector4& plane,int i) const
	{
		btlVector3 halfExtents = getHalfExtentsWithoutMargin();

		switch (i)
		{
		case 0:
			plane.setValue(btlScalar(1.),btlScalar(0.),btlScalar(0.),-halfExtents.x());
			break;
		case 1:
			plane.setValue(btlScalar(-1.),btlScalar(0.),btlScalar(0.),-halfExtents.x());
			break;
		case 2:
			plane.setValue(btlScalar(0.),btlScalar(1.),btlScalar(0.),-halfExtents.y());
			break;
		case 3:
			plane.setValue(btlScalar(0.),btlScalar(-1.),btlScalar(0.),-halfExtents.y());
			break;
		case 4:
			plane.setValue(btlScalar(0.),btlScalar(0.),btlScalar(1.),-halfExtents.z());
			break;
		case 5:
			plane.setValue(btlScalar(0.),btlScalar(0.),btlScalar(-1.),-halfExtents.z());
			break;
		default:
			btlAssert(0);
		}
	}

	
	virtual void getEdge(int i,btlVector3& pa,btlVector3& pb) const
	//virtual void getEdge(int i,Edge& edge) const
	{
		int edgeVert0 = 0;
		int edgeVert1 = 0;

		switch (i)
		{
		case 0:
				edgeVert0 = 0;
				edgeVert1 = 1;
			break;
		case 1:
				edgeVert0 = 0;
				edgeVert1 = 2;
			break;
		case 2:
			edgeVert0 = 1;
			edgeVert1 = 3;

			break;
		case 3:
			edgeVert0 = 2;
			edgeVert1 = 3;
			break;
		case 4:
			edgeVert0 = 0;
			edgeVert1 = 4;
			break;
		case 5:
			edgeVert0 = 1;
			edgeVert1 = 5;

			break;
		case 6:
			edgeVert0 = 2;
			edgeVert1 = 6;
			break;
		case 7:
			edgeVert0 = 3;
			edgeVert1 = 7;
			break;
		case 8:
			edgeVert0 = 4;
			edgeVert1 = 5;
			break;
		case 9:
			edgeVert0 = 4;
			edgeVert1 = 6;
			break;
		case 10:
			edgeVert0 = 5;
			edgeVert1 = 7;
			break;
		case 11:
			edgeVert0 = 6;
			edgeVert1 = 7;
			break;
		default:
			btlAssert(0);

		}

		getVertex(edgeVert0,pa );
		getVertex(edgeVert1,pb );
	}




	
	virtual	bool isInside(const btlVector3& pt,btlScalar tolerance) const
	{
		btlVector3 halfExtents = getHalfExtentsWithoutMargin();

		//btScalar minDist = 2*tolerance;
		
		bool result =	(pt.x() <= (halfExtents.x()+tolerance)) &&
						(pt.x() >= (-halfExtents.x()-tolerance)) &&
						(pt.y() <= (halfExtents.y()+tolerance)) &&
						(pt.y() >= (-halfExtents.y()-tolerance)) &&
						(pt.z() <= (halfExtents.z()+tolerance)) &&
						(pt.z() >= (-halfExtents.z()-tolerance));
		
		return result;
	}


	//debugging
	virtual const char*	getName()const
	{
		return "Box";
	}

	virtual int		getNumPreferredPenetrationDirections() const
	{
		return 6;
	}
	
	virtual void	getPreferredPenetrationDirection(int index, btlVector3& penetrationVector) const
	{
		switch (index)
		{
		case 0:
			penetrationVector.setValue(btlScalar(1.),btlScalar(0.),btlScalar(0.));
			break;
		case 1:
			penetrationVector.setValue(btlScalar(-1.),btlScalar(0.),btlScalar(0.));
			break;
		case 2:
			penetrationVector.setValue(btlScalar(0.),btlScalar(1.),btlScalar(0.));
			break;
		case 3:
			penetrationVector.setValue(btlScalar(0.),btlScalar(-1.),btlScalar(0.));
			break;
		case 4:
			penetrationVector.setValue(btlScalar(0.),btlScalar(0.),btlScalar(1.));
			break;
		case 5:
			penetrationVector.setValue(btlScalar(0.),btlScalar(0.),btlScalar(-1.));
			break;
		default:
			btlAssert(0);
		}
	}

};


#endif //BT_OBB_BOX_MINKOWSKI_H


