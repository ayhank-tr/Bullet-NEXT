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

#include "btConvexPlaneCollisionAlgorithm.h"

#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

//#include <stdio.h>

btlConvexPlaneCollisionAlgorithm::btlConvexPlaneCollisionAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* col0Wrap,const btlCollisionObjectWrapper* col1Wrap, bool isSwapped, int numPerturbationIterations,int minimumPointsPerturbationThreshold)
: btlCollisionAlgorithm(ci),
m_ownManifold(false),
m_manifoldPtr(mf),
m_isSwapped(isSwapped),
m_numPerturbationIterations(numPerturbationIterations),
m_minimumPointsPerturbationThreshold(minimumPointsPerturbationThreshold)
{
	const btlCollisionObjectWrapper* convexObjWrap = m_isSwapped? col1Wrap : col0Wrap;
	const btlCollisionObjectWrapper* planeObjWrap = m_isSwapped? col0Wrap : col1Wrap;

	if (!m_manifoldPtr && m_dispatcher->needsCollision(convexObjWrap->getCollisionObject(),planeObjWrap->getCollisionObject()))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(convexObjWrap->getCollisionObject(),planeObjWrap->getCollisionObject());
		m_ownManifold = true;
	}
}


btlConvexPlaneCollisionAlgorithm::~btlConvexPlaneCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void btlConvexPlaneCollisionAlgorithm::collideSingleContact (const btlQuaternion& perturbeRot, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
    const btlCollisionObjectWrapper* convexObjWrap = m_isSwapped? body1Wrap : body0Wrap;
	const btlCollisionObjectWrapper* planeObjWrap = m_isSwapped? body0Wrap: body1Wrap;

	btlConvexShape* convexShape = (btlConvexShape*) convexObjWrap->getCollisionShape();
	btlStaticPlaneShape* planeShape = (btlStaticPlaneShape*) planeObjWrap->getCollisionShape();

    bool hasCollision = false;
	const btlVector3& planeNormal = planeShape->getPlaneNormal();
	const btlScalar& planeConstant = planeShape->getPlaneConstant();
	
	btlTransform convexWorldTransform = convexObjWrap->getWorldTransform();
	btlTransform convexInPlaneTrans;
	convexInPlaneTrans= planeObjWrap->getWorldTransform().inverse() * convexWorldTransform;
	//now perturbe the convex-world transform
	convexWorldTransform.getBasis()*=btlMatrix3x3(perturbeRot);
	btlTransform planeInConvex;
	planeInConvex= convexWorldTransform.inverse() * planeObjWrap->getWorldTransform();
	
	btlVector3 vtx = convexShape->localGetSupportingVertex(planeInConvex.getBasis()*-planeNormal);

	btlVector3 vtxInPlane = convexInPlaneTrans(vtx);
	btlScalar distance = (planeNormal.dot(vtxInPlane) - planeConstant);

	btlVector3 vtxInPlaneProjected = vtxInPlane - distance*planeNormal;
	btlVector3 vtxInPlaneWorld = planeObjWrap->getWorldTransform() * vtxInPlaneProjected;

	hasCollision = distance < m_manifoldPtr->getContactBreakingThreshold();
	resultOut->setPersistentManifold(m_manifoldPtr);
	if (hasCollision)
	{
		/// report a contact. internally this will be kept persistent, and contact reduction is done
		btlVector3 normalOnSurfaceB = planeObjWrap->getWorldTransform().getBasis() * planeNormal;
		btlVector3 pOnB = vtxInPlaneWorld;
		resultOut->addContactPoint(normalOnSurfaceB,pOnB,distance);
	}
}


void btlConvexPlaneCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)dispatchInfo;
	if (!m_manifoldPtr)
		return;

	const btlCollisionObjectWrapper* convexObjWrap = m_isSwapped? body1Wrap : body0Wrap;
	const btlCollisionObjectWrapper* planeObjWrap = m_isSwapped? body0Wrap: body1Wrap;

	btlConvexShape* convexShape = (btlConvexShape*) convexObjWrap->getCollisionShape();
	btlStaticPlaneShape* planeShape = (btlStaticPlaneShape*) planeObjWrap->getCollisionShape();

	bool hasCollision = false;
	const btlVector3& planeNormal = planeShape->getPlaneNormal();
	const btlScalar& planeConstant = planeShape->getPlaneConstant();
	btlTransform planeInConvex;
	planeInConvex= convexObjWrap->getWorldTransform().inverse() * planeObjWrap->getWorldTransform();
	btlTransform convexInPlaneTrans;
	convexInPlaneTrans= planeObjWrap->getWorldTransform().inverse() * convexObjWrap->getWorldTransform();

	btlVector3 vtx = convexShape->localGetSupportingVertex(planeInConvex.getBasis()*-planeNormal);
	btlVector3 vtxInPlane = convexInPlaneTrans(vtx);
	btlScalar distance = (planeNormal.dot(vtxInPlane) - planeConstant);

	btlVector3 vtxInPlaneProjected = vtxInPlane - distance*planeNormal;
	btlVector3 vtxInPlaneWorld = planeObjWrap->getWorldTransform() * vtxInPlaneProjected;

	hasCollision = distance < m_manifoldPtr->getContactBreakingThreshold();
	resultOut->setPersistentManifold(m_manifoldPtr);
	if (hasCollision)
	{
		/// report a contact. internally this will be kept persistent, and contact reduction is done
		btlVector3 normalOnSurfaceB = planeObjWrap->getWorldTransform().getBasis() * planeNormal;
		btlVector3 pOnB = vtxInPlaneWorld;
		resultOut->addContactPoint(normalOnSurfaceB,pOnB,distance);
	}

	//the perturbation algorithm doesn't work well with implicit surfaces such as spheres, cylinder and cones:
	//they keep on rolling forever because of the additional off-center contact points
	//so only enable the feature for polyhedral shapes (btBoxShape, btConvexHullShape etc)
	if (convexShape->isPolyhedral() && resultOut->getPersistentManifold()->getNumContacts()<m_minimumPointsPerturbationThreshold)
	{
		btlVector3 v0,v1;
		btlPlaneSpace1(planeNormal,v0,v1);
		//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects

		const btlScalar angleLimit = 0.125f * SIMD_PI;
		btlScalar perturbeAngle;
		btlScalar radius = convexShape->getAngularMotionDisc();
		perturbeAngle = btl_gContactBreakingThreshold / radius;
		if ( perturbeAngle > angleLimit ) 
				perturbeAngle = angleLimit;

		btlQuaternion perturbeRot(v0,perturbeAngle);
		for (int i=0;i<m_numPerturbationIterations;i++)
		{
			btlScalar iterationAngle = i*(SIMD_2_PI/btlScalar(m_numPerturbationIterations));
			btlQuaternion rotq(planeNormal,iterationAngle);
			collideSingleContact(rotq.inverse()*perturbeRot*rotq,body0Wrap,body1Wrap,dispatchInfo,resultOut);
		}
	}

	if (m_ownManifold)
	{
		if (m_manifoldPtr->getNumContacts())
		{
			resultOut->refreshContactPoints();
		}
	}
}

btlScalar btlConvexPlaneCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* col0,btlCollisionObject* col1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return btlScalar(1.);
}
