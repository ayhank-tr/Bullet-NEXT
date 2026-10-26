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

#include "btTriangleMeshShape.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "btStridingMeshInterface.h"
#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"


btlTriangleMeshShape::btlTriangleMeshShape(btlStridingMeshInterface* meshInterface)
: btlConcaveShape (), m_meshInterface(meshInterface)
{
	m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
	if(meshInterface->hasPremadeAabb())
	{
		meshInterface->getPremadeAabb(&m_localAabbMin, &m_localAabbMax);
	}
	else
	{
		recalcLocalAabb();
	}
}


btlTriangleMeshShape::~btlTriangleMeshShape()
{
		
}




void btlTriangleMeshShape::getAabb(const btlTransform& trans,btlVector3& aabbMin,btlVector3& aabbMax) const
{

	btlVector3 localHalfExtents = btlScalar(0.5)*(m_localAabbMax-m_localAabbMin);
	localHalfExtents += btlVector3(getMargin(),getMargin(),getMargin());
	btlVector3 localCenter = btlScalar(0.5)*(m_localAabbMax+m_localAabbMin);
	
	btlMatrix3x3 abs_b = trans.getBasis().absolute();  

	btlVector3 center = trans(localCenter);

    btlVector3 extent = localHalfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
	aabbMin = center - extent;
	aabbMax = center + extent;
}

void	btlTriangleMeshShape::recalcLocalAabb()
{
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
}



class btl_SupportVertexCallback : public btlTriangleCallback
{

	btlVector3 m_supportVertexLocal;
public:

	btlTransform	m_worldTrans;
	btlScalar m_maxDot;
	btlVector3 m_supportVecLocal;

	btl_SupportVertexCallback(const btlVector3& supportVecWorld,const btlTransform& trans)
		: m_supportVertexLocal(btlScalar(0.),btlScalar(0.),btlScalar(0.)), m_worldTrans(trans) ,m_maxDot(btlScalar(-BT_LARGE_FLOAT))
		
	{
		m_supportVecLocal = supportVecWorld * m_worldTrans.getBasis();
	}

	virtual void processTriangle( btlVector3* triangle,int partId, int triangleIndex)
	{
		(void)partId;
		(void)triangleIndex;
		for (int i=0;i<3;i++)
		{
			btlScalar dot = m_supportVecLocal.dot(triangle[i]);
			if (dot > m_maxDot)
			{
				m_maxDot = dot;
				m_supportVertexLocal = triangle[i];
			}
		}
	}

	btlVector3 GetSupportVertexWorldSpace()
	{
		return m_worldTrans(m_supportVertexLocal);
	}

	btlVector3	GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}

};

	
void btlTriangleMeshShape::setLocalScaling(const btlVector3& scaling)
{
	m_meshInterface->setScaling(scaling);
	recalcLocalAabb();
}

const btlVector3& btlTriangleMeshShape::getLocalScaling() const
{
	return m_meshInterface->getScaling();
}






//#define DEBUG_TRIANGLE_MESH



void	btlTriangleMeshShape::processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const
{
		struct FilteredCallback : public btlInternalTriangleIndexCallback
	{
		btlTriangleCallback* m_callback;
		btlVector3 m_aabbMin;
		btlVector3 m_aabbMax;

		FilteredCallback(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax)
			:m_callback(callback),
			m_aabbMin(aabbMin),
			m_aabbMax(aabbMax)
		{
		}

		virtual void internalProcessTriangleIndex(btlVector3* triangle,int partId,int triangleIndex)
		{
			if (TestTriangleAgainstAabb2(&triangle[0],m_aabbMin,m_aabbMax))
			{
				//check aabb in triangle-space, before doing this
				m_callback->processTriangle(triangle,partId,triangleIndex);
			}
			
		}

	};

	FilteredCallback filterCallback(callback,aabbMin,aabbMax);

	m_meshInterface->InternalProcessAllTriangles(&filterCallback,aabbMin,aabbMax);
}





void	btlTriangleMeshShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
	(void)mass;
	//moving concave objects not supported
	btlAssert(0);
	inertia.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
}


btlVector3 btlTriangleMeshShape::localGetSupportingVertex(const btlVector3& vec) const
{
	btlVector3 supportVertex;

	btlTransform ident;
	ident.setIdentity();

	btl_SupportVertexCallback supportCallback(vec,ident);

	btlVector3 aabbMax(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
	
	processAllTriangles(&supportCallback,-aabbMax,aabbMax);
		
	supportVertex = supportCallback.GetSupportVertexLocal();

	return supportVertex;
}


