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

#include "btSphereBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
//#include <stdio.h>

btlSphereBoxCollisionAlgorithm::btlSphereBoxCollisionAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* col0Wrap,const btlCollisionObjectWrapper* col1Wrap, bool isSwapped)
: btlActivatingCollisionAlgorithm(ci,col0Wrap,col1Wrap),
m_ownManifold(false),
m_manifoldPtr(mf),
m_isSwapped(isSwapped)
{
	const btlCollisionObjectWrapper* sphereObjWrap = m_isSwapped? col1Wrap : col0Wrap;
	const btlCollisionObjectWrapper* boxObjWrap = m_isSwapped? col0Wrap : col1Wrap;
	
	if (!m_manifoldPtr && m_dispatcher->needsCollision(sphereObjWrap->getCollisionObject(),boxObjWrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(sphereObjWrap->getCollisionObject(),boxObjWrap->getCollisionObject());
		m_ownManifold = true;
	}
}


btlSphereBoxCollisionAlgorithm::~btlSphereBoxCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}



void btlSphereBoxCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* body0Wrap, const btlCollisionObjectWrapper* body1Wrap, const btlDispatcherInfo& dispatchInfo, btlManifoldResult* resultOut)
{
	(void)dispatchInfo;
	(void)resultOut;
	if (!m_manifoldPtr)
		return;

	const btlCollisionObjectWrapper* sphereObjWrap = m_isSwapped? body1Wrap : body0Wrap;
	const btlCollisionObjectWrapper* boxObjWrap = m_isSwapped? body0Wrap : body1Wrap;

	btlVector3 pOnBox;

	btlVector3 normalOnSurfaceB;
	btlScalar penetrationDepth;
	btlVector3 sphereCenter = sphereObjWrap->getWorldTransform().getOrigin();
	const btlSphereShape* sphere0 = (const btlSphereShape*)sphereObjWrap->getCollisionShape();
	btlScalar radius = sphere0->getRadius();
	btlScalar maxContactDistance = m_manifoldPtr->getContactBreakingThreshold();

	resultOut->setPersistentManifold(m_manifoldPtr);

	if (getSphereDistance(boxObjWrap, pOnBox, normalOnSurfaceB, penetrationDepth, sphereCenter, radius, maxContactDistance))
	{
		/// report a contact. internally this will be kept persistent, and contact reduction is done
		resultOut->addContactPoint(normalOnSurfaceB, pOnBox, penetrationDepth);
	}

	if (m_ownManifold)
	{
		if (m_manifoldPtr->getNumContacts())
		{
			resultOut->refreshContactPoints();
		}
	}

}

btlScalar btlSphereBoxCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* col0,btlCollisionObject* col1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return btlScalar(1.);
}


bool btlSphereBoxCollisionAlgorithm::getSphereDistance(const btlCollisionObjectWrapper* boxObjWrap, btlVector3& pointOnBox, btlVector3& normal, btlScalar& penetrationDepth, const btlVector3& sphereCenter, btlScalar fRadius, btlScalar maxContactDistance ) 
{
	const btlBoxShape* boxShape= (const btlBoxShape*)boxObjWrap->getCollisionShape();
	btlVector3 const &boxHalfExtent = boxShape->getHalfExtentsWithoutMargin();
	btlScalar boxMargin = boxShape->getMargin();
	penetrationDepth = 1.0f;

	// convert the sphere position to the box's local space
	btlTransform const &m44T = boxObjWrap->getWorldTransform();
	btlVector3 sphereRelPos = m44T.invXform(sphereCenter);

	// Determine the closest point to the sphere center in the box
	btlVector3 closestPoint = sphereRelPos;
	closestPoint.setX( btlMin(boxHalfExtent.getX(), closestPoint.getX()) );
	closestPoint.setX( btlMax(-boxHalfExtent.getX(), closestPoint.getX()) );
	closestPoint.setY( btlMin(boxHalfExtent.getY(), closestPoint.getY()) );
	closestPoint.setY( btlMax(-boxHalfExtent.getY(), closestPoint.getY()) );
	closestPoint.setZ( btlMin(boxHalfExtent.getZ(), closestPoint.getZ()) );
	closestPoint.setZ( btlMax(-boxHalfExtent.getZ(), closestPoint.getZ()) );
	
	btlScalar intersectionDist = fRadius + boxMargin;
	btlScalar contactDist = intersectionDist + maxContactDistance;
	normal = sphereRelPos - closestPoint;

	//if there is no penetration, we are done
	btlScalar dist2 = normal.length2();
	if (dist2 > contactDist * contactDist)
	{
		return false;
	}

	btlScalar distance;

	//special case if the sphere center is inside the box
	if (dist2 <= SIMD_EPSILON)
	{
		distance = -getSpherePenetration(boxHalfExtent, sphereRelPos, closestPoint, normal);
	}
	else //compute the penetration details
	{
		distance = normal.length();
		normal /= distance;
	}

	pointOnBox = closestPoint + normal * boxMargin;
//	v3PointOnSphere = sphereRelPos - (normal * fRadius);	
	penetrationDepth = distance - intersectionDist;

	// transform back in world space
	btlVector3 tmp = m44T(pointOnBox);
	pointOnBox = tmp;
//	tmp = m44T(v3PointOnSphere);
//	v3PointOnSphere = tmp;
	tmp = m44T.getBasis() * normal;
	normal = tmp;

	return true;
}

btlScalar btlSphereBoxCollisionAlgorithm::getSpherePenetration( btlVector3 const &boxHalfExtent, btlVector3 const &sphereRelPos, btlVector3 &closestPoint, btlVector3& normal ) 
{
	//project the center of the sphere on the closest face of the box
	btlScalar faceDist = boxHalfExtent.getX() - sphereRelPos.getX();
	btlScalar minDist = faceDist;
	closestPoint.setX( boxHalfExtent.getX() );
	normal.setValue(btlScalar(1.0f),  btlScalar(0.0f),  btlScalar(0.0f));

	faceDist = boxHalfExtent.getX() + sphereRelPos.getX();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setX( -boxHalfExtent.getX() );
		normal.setValue(btlScalar(-1.0f),  btlScalar(0.0f),  btlScalar(0.0f));
	}

	faceDist = boxHalfExtent.getY() - sphereRelPos.getY();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setY( boxHalfExtent.getY() );
		normal.setValue(btlScalar(0.0f),  btlScalar(1.0f),  btlScalar(0.0f));
	}

	faceDist = boxHalfExtent.getY() + sphereRelPos.getY();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setY( -boxHalfExtent.getY() );
		normal.setValue(btlScalar(0.0f),  btlScalar(-1.0f),  btlScalar(0.0f));
	}

	faceDist = boxHalfExtent.getZ() - sphereRelPos.getZ();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setZ( boxHalfExtent.getZ() );
		normal.setValue(btlScalar(0.0f),  btlScalar(0.0f),  btlScalar(1.0f));
	}

	faceDist = boxHalfExtent.getZ() + sphereRelPos.getZ();
	if (faceDist < minDist)
	{
		minDist = faceDist;
		closestPoint = sphereRelPos;
		closestPoint.setZ( -boxHalfExtent.getZ() );
		normal.setValue(btlScalar(0.0f),  btlScalar(0.0f),  btlScalar(-1.0f));
	}

	return minDist;
}
