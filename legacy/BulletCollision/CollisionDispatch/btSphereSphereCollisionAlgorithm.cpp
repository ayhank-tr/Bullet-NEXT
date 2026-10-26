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

#include "btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

btlSphereSphereCollisionAlgorithm::btlSphereSphereCollisionAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* col0Wrap,const btlCollisionObjectWrapper* col1Wrap)
: btlActivatingCollisionAlgorithm(ci,col0Wrap,col1Wrap),
m_ownManifold(false),
m_manifoldPtr(mf)
{
	if (!m_manifoldPtr)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(col0Wrap->getCollisionObject(),col1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
}

btlSphereSphereCollisionAlgorithm::~btlSphereSphereCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void btlSphereSphereCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* col0Wrap,const btlCollisionObjectWrapper* col1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)dispatchInfo;

	if (!m_manifoldPtr)
		return;

	resultOut->setPersistentManifold(m_manifoldPtr);

	btlSphereShape* sphere0 = (btlSphereShape*)col0Wrap->getCollisionShape();
	btlSphereShape* sphere1 = (btlSphereShape*)col1Wrap->getCollisionShape();

	btlVector3 diff = col0Wrap->getWorldTransform().getOrigin()-  col1Wrap->getWorldTransform().getOrigin();
	btlScalar len = diff.length();
	btlScalar radius0 = sphere0->getRadius();
	btlScalar radius1 = sphere1->getRadius();

#ifdef CLEAR_MANIFOLD
	m_manifoldPtr->clearManifold(); //don't do this, it disables warmstarting
#endif

	///iff distance positive, don't generate a new contact
	if ( len > (radius0+radius1))
	{
#ifndef CLEAR_MANIFOLD
		resultOut->refreshContactPoints();
#endif //CLEAR_MANIFOLD
		return;
	}
	///distance (negative means penetration)
	btlScalar dist = len - (radius0+radius1);

	btlVector3 normalOnSurfaceB(1,0,0);
	if (len > SIMD_EPSILON)
	{
		normalOnSurfaceB = diff / len;
	}

	///point on A (worldspace)
	///btVector3 pos0 = col0->getWorldTransform().getOrigin() - radius0 * normalOnSurfaceB;
	///point on B (worldspace)
	btlVector3 pos1 = col1Wrap->getWorldTransform().getOrigin() + radius1* normalOnSurfaceB;

	/// report a contact. internally this will be kept persistent, and contact reduction is done
	
	
	resultOut->addContactPoint(normalOnSurfaceB,pos1,dist);

#ifndef CLEAR_MANIFOLD
	resultOut->refreshContactPoints();
#endif //CLEAR_MANIFOLD

}

btlScalar btlSphereSphereCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* col0,btlCollisionObject* col1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)col0;
	(void)col1;
	(void)dispatchInfo;
	(void)resultOut;

	//not yet
	return btlScalar(1.);
}
