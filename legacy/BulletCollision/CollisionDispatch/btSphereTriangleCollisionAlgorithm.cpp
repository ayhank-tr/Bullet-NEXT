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


#include "btSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

btlSphereTriangleCollisionAlgorithm::btlSphereTriangleCollisionAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool swapped)
: btlActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_ownManifold(false),
m_manifoldPtr(mf),
m_swapped(swapped)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

btlSphereTriangleCollisionAlgorithm::~btlSphereTriangleCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void btlSphereTriangleCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* col0Wrap,const btlCollisionObjectWrapper* col1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	if (!m_manifoldPtr)
		return;

	const btlCollisionObjectWrapper* sphereObjWrap = m_swapped? col1Wrap : col0Wrap;
	const btlCollisionObjectWrapper* triObjWrap = m_swapped? col0Wrap : col1Wrap;

	btlSphereShape* sphere = (btlSphereShape*)sphereObjWrap->getCollisionShape();
	btlTriangleShape* triangle = (btlTriangleShape*)triObjWrap->getCollisionShape();
	
	/// report a contact. internally this will be kept persistent, and contact reduction is done
	resultOut->setPersistentManifold(m_manifoldPtr);
	btl_SphereTriangleDetector detector(sphere,triangle, m_manifoldPtr->getContactBreakingThreshold());
	
	btlDiscreteCollisionDetectorInterface::ClosestPointInput input;
	input.m_maximumDistanceSquared = btlScalar(BT_LARGE_FLOAT);///@todo: tighter bounds
	input.m_transformA = sphereObjWrap->getWorldTransform();
	input.m_transformB = triObjWrap->getWorldTransform();

	bool swapResults = m_swapped;

	detector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw,swapResults);

	if (m_ownManifold)
		resultOut->refreshContactPoints();
	
}

btlScalar btlSphereTriangleCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* col0,btlCollisionObject* col1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return btlScalar(1.);
}
