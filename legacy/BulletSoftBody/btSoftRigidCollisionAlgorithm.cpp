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

#include "btSoftRigidCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btSoftBody.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

///TODO: include all the shapes that the softbody can collide with
///alternatively, implement special case collision algorithms (just like for rigid collision shapes)

//#include <stdio.h>

btlSoftRigidCollisionAlgorithm::btlSoftRigidCollisionAlgorithm(btlPersistentManifold* /*mf*/,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* ,const btlCollisionObjectWrapper* , bool isSwapped)
: btlCollisionAlgorithm(ci),
//m_ownManifold(false),
//m_manifoldPtr(mf),
m_isSwapped(isSwapped)
{
}


btlSoftRigidCollisionAlgorithm::~btlSoftRigidCollisionAlgorithm()
{

	//m_softBody->m_overlappingRigidBodies.remove(m_rigidCollisionObject);

	/*if (m_ownManifold)
	{
	if (m_manifoldPtr)
	m_dispatcher->releaseManifold(m_manifoldPtr);
	}
	*/

}


#include <stdio.h>

void btlSoftRigidCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)dispatchInfo;
	(void)resultOut;
	//printf("btSoftRigidCollisionAlgorithm\n");
//	const btCollisionObjectWrapper* softWrap = m_isSwapped?body1Wrap:body0Wrap;
//	const btCollisionObjectWrapper* rigidWrap = m_isSwapped?body0Wrap:body1Wrap;
	btlSoftBody* softBody =  m_isSwapped? (btlSoftBody*)body1Wrap->getCollisionObject() : (btlSoftBody*)body0Wrap->getCollisionObject();
	const btlCollisionObjectWrapper* rigidCollisionObjectWrap = m_isSwapped? body0Wrap : body1Wrap;
	
	if (softBody->m_collisionDisabledObjects.findLinearSearch(rigidCollisionObjectWrap->getCollisionObject())==softBody->m_collisionDisabledObjects.size())
	{
		softBody->getSoftBodySolver()->processCollision(softBody, rigidCollisionObjectWrap);
	}


}

btlScalar btlSoftRigidCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* col0,btlCollisionObject* col1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return btlScalar(1.);
}



