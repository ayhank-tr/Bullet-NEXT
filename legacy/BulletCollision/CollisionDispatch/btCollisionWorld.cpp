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

#include "btCollisionWorld.h"
#include "btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h" //for raycasting
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h" //for raycasting
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btAabbUtil2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btSerializer.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

//#define DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION


//#define USE_BRUTEFORCE_RAYBROADPHASE 1
//RECALCULATE_AABB is slower, but benefit is that you don't need to call 'stepSimulation'  or 'updateAabbs' before using a rayTest
//#define RECALCULATE_AABB_RAYCAST 1

//When the user doesn't provide dispatcher or broadphase, create basic versions (and delete them in destructor)
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionDispatch/btCollisionConfiguration.h"


///for debug drawing

//for debug rendering
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"



btlCollisionWorld::btlCollisionWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* pairCache, btlCollisionConfiguration* collisionConfiguration)
:m_dispatcher1(dispatcher),
m_broadphasePairCache(pairCache),
m_debugDrawer(0),
m_forceUpdateAllAabbs(true)
{
}


btlCollisionWorld::~btlCollisionWorld()
{

	//clean up remaining objects
	int i;
	for (i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* collisionObject= m_collisionObjects[i];

		btlBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
		if (bp)
		{
			//
			// only clear the cached algorithms
			//
			getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
			getBroadphase()->destroyProxy(bp,m_dispatcher1);
			collisionObject->setBroadphaseHandle(0);
		}
	}


}










void	btlCollisionWorld::addCollisionObject(btlCollisionObject* collisionObject,short int collisionFilterGroup,short int collisionFilterMask)
{

	btlAssert(collisionObject);

	//check that the object isn't already added
	btlAssert( m_collisionObjects.findLinearSearch(collisionObject)  == m_collisionObjects.size());

	m_collisionObjects.push_back(collisionObject);

	//calculate new AABB
	btlTransform trans = collisionObject->getWorldTransform();

	btlVector3	minAabb;
	btlVector3	maxAabb;
	collisionObject->getCollisionShape()->getAabb(trans,minAabb,maxAabb);

	int type = collisionObject->getCollisionShape()->getShapeType();
	collisionObject->setBroadphaseHandle( getBroadphase()->createProxy(
		minAabb,
		maxAabb,
		type,
		collisionObject,
		collisionFilterGroup,
		collisionFilterMask,
		m_dispatcher1,0
		))	;





}



void	btlCollisionWorld::updateSingleAabb(btlCollisionObject* colObj)
{
	btlVector3 minAabb,maxAabb;
	colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb,maxAabb);
	//need to increase the aabb for contact thresholds
	btlVector3 contactThreshold(btl_gContactBreakingThreshold,btl_gContactBreakingThreshold,btl_gContactBreakingThreshold);
	minAabb -= contactThreshold;
	maxAabb += contactThreshold;

	if(getDispatchInfo().m_useContinuous && colObj->getInternalType()==btlCollisionObject::CO_RIGID_BODY && !colObj->isStaticOrKinematicObject())
	{
		btlVector3 minAabb2,maxAabb2;
		colObj->getCollisionShape()->getAabb(colObj->getInterpolationWorldTransform(),minAabb2,maxAabb2);
		minAabb2 -= contactThreshold;
		maxAabb2 += contactThreshold;
		minAabb.setMin(minAabb2);
		maxAabb.setMax(maxAabb2);
	}

	btlBroadphaseInterface* bp = (btlBroadphaseInterface*)m_broadphasePairCache;

	//moving objects should be moderately sized, probably something wrong if not
	if ( colObj->isStaticObject() || ((maxAabb-minAabb).length2() < btlScalar(1e12)))
	{
		bp->setAabb(colObj->getBroadphaseHandle(),minAabb,maxAabb, m_dispatcher1);
	} else
	{
		//something went wrong, investigate
		//this assert is unwanted in 3D modelers (danger of loosing work)
		colObj->setActivationState(DISABLE_SIMULATION);

		static bool reportMe = true;
		if (reportMe && m_debugDrawer)
		{
			reportMe = false;
			m_debugDrawer->reportErrorWarning("Overflow in AABB, object removed from simulation");
			m_debugDrawer->reportErrorWarning("If you can reproduce this, please email bugs@continuousphysics.com\n");
			m_debugDrawer->reportErrorWarning("Please include above information, your Platform, version of OS.\n");
			m_debugDrawer->reportErrorWarning("Thanks.\n");
		}
	}
}

void	btlCollisionWorld::updateAabbs()
{
	BT_PROFILE("updateAabbs");

	btlTransform predictedTrans;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];

		//only update aabb of active objects
		if (m_forceUpdateAllAabbs || colObj->isActive())
		{
			updateSingleAabb(colObj);
		}
	}
}


void	btlCollisionWorld::computeOverlappingPairs()
{
	BT_PROFILE("calculateOverlappingPairs");
	m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);
}

void	btlCollisionWorld::performDiscreteCollisionDetection()
{
	BT_PROFILE("performDiscreteCollisionDetection");

	btlDispatcherInfo& dispatchInfo = getDispatchInfo();

	updateAabbs();

	computeOverlappingPairs();

	btlDispatcher* dispatcher = getDispatcher();
	{
		BT_PROFILE("dispatchAllCollisionPairs");
		if (dispatcher)
			dispatcher->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(),dispatchInfo,m_dispatcher1);
	}

}



void	btlCollisionWorld::removeCollisionObject(btlCollisionObject* collisionObject)
{


	//bool removeFromBroadphase = false;

	{

		btlBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
		if (bp)
		{
			//
			// only clear the cached algorithms
			//
			getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
			getBroadphase()->destroyProxy(bp,m_dispatcher1);
			collisionObject->setBroadphaseHandle(0);
		}
	}


	//swapremove
	m_collisionObjects.remove(collisionObject);

}


void	btlCollisionWorld::rayTestSingle(const btlTransform& rayFromTrans,const btlTransform& rayToTrans,
										btlCollisionObject* collisionObject,
										const btlCollisionShape* collisionShape,
										const btlTransform& colObjWorldTransform,
										RayResultCallback& resultCallback)
{
	btlCollisionObjectWrapper colObWrap(0,collisionShape,collisionObject,colObjWorldTransform,-1,-1);
	btlCollisionWorld::rayTestSingleInternal(rayFromTrans,rayToTrans,&colObWrap,resultCallback);
}

void	btlCollisionWorld::rayTestSingleInternal(const btlTransform& rayFromTrans,const btlTransform& rayToTrans,
										const btlCollisionObjectWrapper* collisionObjectWrap,
										RayResultCallback& resultCallback)
{
	btlSphereShape pointShape(btlScalar(0.0));
	pointShape.setMargin(0.f);
	const btlConvexShape* castShape = &pointShape;
	const btlCollisionShape* collisionShape = collisionObjectWrap->getCollisionShape();
	const btlTransform& colObjWorldTransform = collisionObjectWrap->getWorldTransform();

	if (collisionShape->isConvex())
	{
		//		BT_PROFILE("rayTestConvex");
		btlConvexCast::CastResult castResult;
		castResult.m_fraction = resultCallback.m_closestHitFraction;

		btlConvexShape* convexShape = (btlConvexShape*) collisionShape;
		btlVoronoiSimplexSolver	simplexSolver;
		btlSubsimplexConvexCast subSimplexConvexCaster(castShape,convexShape,&simplexSolver);
		
		btlGjkConvexCast	gjkConvexCaster(castShape,convexShape,&simplexSolver);
		
		//btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);

		btlConvexCast* convexCasterPtr = 0;
		//use kF_UseSubSimplexConvexCastRaytest by default
		if (resultCallback.m_flags & btlTriangleRaycastCallback::kF_UseGjkConvexCastRaytest)
			convexCasterPtr = &gjkConvexCaster;
		else
			convexCasterPtr = &subSimplexConvexCaster;
		
		btlConvexCast& convexCaster = *convexCasterPtr;

		if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,colObjWorldTransform,colObjWorldTransform,castResult))
		{
			//add hit
			if (castResult.m_normal.length2() > btlScalar(0.0001))
			{
				if (castResult.m_fraction < resultCallback.m_closestHitFraction)
				{
					//todo: figure out what this is about. When is rayFromTest.getBasis() not identity?
#ifdef USE_SUBSIMPLEX_CONVEX_CAST
					//rotate normal into worldspace
					castResult.m_normal = rayFromTrans.getBasis() * castResult.m_normal;
#endif //USE_SUBSIMPLEX_CONVEX_CAST

					castResult.m_normal.normalize();
					btlCollisionWorld::LocalRayResult localRayResult
						(
						collisionObjectWrap->getCollisionObject(),
						0,
						castResult.m_normal,
						castResult.m_fraction
						);

					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(localRayResult, normalInWorldSpace);

				}
			}
		}
	} else {
		if (collisionShape->isConcave())
		{

			//ConvexCast::CastResult
				struct BridgeTriangleRaycastCallback : public btlTriangleRaycastCallback
				{
					btlCollisionWorld::RayResultCallback* m_resultCallback;
					const btlCollisionObject*	m_collisionObject;
					const btlConcaveShape*	m_triangleMesh;

					btlTransform m_colObjWorldTransform;

					BridgeTriangleRaycastCallback( const btlVector3& from,const btlVector3& to,
					btlCollisionWorld::RayResultCallback* resultCallback, const btlCollisionObject* collisionObject,const btlConcaveShape*	triangleMesh,const btlTransform& colObjWorldTransform):
						//@BP Mod
						btlTriangleRaycastCallback(from,to, resultCallback->m_flags),
							m_resultCallback(resultCallback),
							m_collisionObject(collisionObject),
							m_triangleMesh(triangleMesh),
							m_colObjWorldTransform(colObjWorldTransform)
						{
						}


					virtual btlScalar reportHit(const btlVector3& hitNormalLocal, btlScalar hitFraction, int partId, int triangleIndex )
					{
						btlCollisionWorld::LocalShapeInfo	shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;

						btlVector3 hitNormalWorld = m_colObjWorldTransform.getBasis() * hitNormalLocal;

						btlCollisionWorld::LocalRayResult rayResult
							(m_collisionObject,
							&shapeInfo,
							hitNormalWorld,
							hitFraction);

						bool	normalInWorldSpace = true;
						return m_resultCallback->addSingleResult(rayResult,normalInWorldSpace);
					}

				};

			btlTransform worldTocollisionObject = colObjWorldTransform.inverse();
			btlVector3 rayFromLocal = worldTocollisionObject * rayFromTrans.getOrigin();
			btlVector3 rayToLocal = worldTocollisionObject * rayToTrans.getOrigin();

			//			BT_PROFILE("rayTestConcave");
			if (collisionShape->getShapeType()==TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				///optimized version for btBvhTriangleMeshShape
				btlBvhTriangleMeshShape* triangleMesh = (btlBvhTriangleMeshShape*)collisionShape;
				
				BridgeTriangleRaycastCallback rcb(rayFromLocal,rayToLocal,&resultCallback,collisionObjectWrap->getCollisionObject(),triangleMesh,colObjWorldTransform);
				rcb.m_hitFraction = resultCallback.m_closestHitFraction;
				triangleMesh->performRaycast(&rcb,rayFromLocal,rayToLocal);
			}
			else
			{
				//generic (slower) case
				btlConcaveShape* concaveShape = (btlConcaveShape*)collisionShape;

				btlTransform worldTocollisionObject = colObjWorldTransform.inverse();

				btlVector3 rayFromLocal = worldTocollisionObject * rayFromTrans.getOrigin();
				btlVector3 rayToLocal = worldTocollisionObject * rayToTrans.getOrigin();

				//ConvexCast::CastResult

				struct BridgeTriangleRaycastCallback : public btlTriangleRaycastCallback
				{
					btlCollisionWorld::RayResultCallback* m_resultCallback;
					const btlCollisionObject*	m_collisionObject;
					btlConcaveShape*	m_triangleMesh;

					btlTransform m_colObjWorldTransform;

					BridgeTriangleRaycastCallback( const btlVector3& from,const btlVector3& to,
						btlCollisionWorld::RayResultCallback* resultCallback, const btlCollisionObject* collisionObject,btlConcaveShape*	triangleMesh, const btlTransform& colObjWorldTransform):
					//@BP Mod
					btlTriangleRaycastCallback(from,to, resultCallback->m_flags),
						m_resultCallback(resultCallback),
						m_collisionObject(collisionObject),
						m_triangleMesh(triangleMesh),
						m_colObjWorldTransform(colObjWorldTransform)
					{
					}


					virtual btlScalar reportHit(const btlVector3& hitNormalLocal, btlScalar hitFraction, int partId, int triangleIndex )
					{
						btlCollisionWorld::LocalShapeInfo	shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;

						btlVector3 hitNormalWorld = m_colObjWorldTransform.getBasis() * hitNormalLocal;

						btlCollisionWorld::LocalRayResult rayResult
							(m_collisionObject,
							&shapeInfo,
							hitNormalWorld,
							hitFraction);

						bool	normalInWorldSpace = true;
						return m_resultCallback->addSingleResult(rayResult,normalInWorldSpace);
					}

				};


				BridgeTriangleRaycastCallback	rcb(rayFromLocal,rayToLocal,&resultCallback,collisionObjectWrap->getCollisionObject(),concaveShape, colObjWorldTransform);
				rcb.m_hitFraction = resultCallback.m_closestHitFraction;

				btlVector3 rayAabbMinLocal = rayFromLocal;
				rayAabbMinLocal.setMin(rayToLocal);
				btlVector3 rayAabbMaxLocal = rayFromLocal;
				rayAabbMaxLocal.setMax(rayToLocal);

				concaveShape->processAllTriangles(&rcb,rayAabbMinLocal,rayAabbMaxLocal);
			}
		} else {
			//			BT_PROFILE("rayTestCompound");
			if (collisionShape->isCompound())
			{
				struct LocalInfoAdder2 : public RayResultCallback
				{
					RayResultCallback* m_userCallback;
					int m_i;
					
					LocalInfoAdder2 (int i, RayResultCallback *user)
						: m_userCallback(user), m_i(i)
					{ 
						m_closestHitFraction = m_userCallback->m_closestHitFraction;
						m_flags = m_userCallback->m_flags;
					}
					virtual bool needsCollision(btlBroadphaseProxy* p) const
					{
						return m_userCallback->needsCollision(p);
					}

					virtual btlScalar addSingleResult (btlCollisionWorld::LocalRayResult &r, bool b)
					{
						btlCollisionWorld::LocalShapeInfo shapeInfo;
						shapeInfo.m_shapePart = -1;
						shapeInfo.m_triangleIndex = m_i;
						if (r.m_localShapeInfo == NULL)
							r.m_localShapeInfo = &shapeInfo;

						const btlScalar result = m_userCallback->addSingleResult(r, b);
						m_closestHitFraction = m_userCallback->m_closestHitFraction;
						return result;
					}
				};
				
				struct RayTester : btlDbvt::ICollide
				{
					const btlCollisionObject* m_collisionObject;
					const btlCompoundShape* m_compoundShape;
					const btlTransform& m_colObjWorldTransform;
					const btlTransform& m_rayFromTrans;
					const btlTransform& m_rayToTrans;
					RayResultCallback& m_resultCallback;
					
					RayTester(const btlCollisionObject* collisionObject,
							const btlCompoundShape* compoundShape,
							const btlTransform& colObjWorldTransform,
							const btlTransform& rayFromTrans,
							const btlTransform& rayToTrans,
							RayResultCallback& resultCallback):
						m_collisionObject(collisionObject),
						m_compoundShape(compoundShape),
						m_colObjWorldTransform(colObjWorldTransform),
						m_rayFromTrans(rayFromTrans),
						m_rayToTrans(rayToTrans),
						m_resultCallback(resultCallback)
					{
						
					}
					
					void ProcessLeaf(int i)
					{
						const btlCollisionShape* childCollisionShape = m_compoundShape->getChildShape(i);
						const btlTransform& childTrans = m_compoundShape->getChildTransform(i);
						btlTransform childWorldTrans = m_colObjWorldTransform * childTrans;
						
						btlCollisionObjectWrapper tmpOb(0,childCollisionShape,m_collisionObject,childWorldTrans,-1,i);
						// replace collision shape so that callback can determine the triangle

						

						LocalInfoAdder2 my_cb(i, &m_resultCallback);

						rayTestSingleInternal(
							m_rayFromTrans,
							m_rayToTrans,
							&tmpOb,
							my_cb);
						
					}
				
					void Process(const btlDbvtNode* leaf)
					{
						ProcessLeaf(leaf->dataAsInt);
					}
				};
				
				const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(collisionShape);
				const btlDbvt* dbvt = compoundShape->getDynamicAabbTree();


				RayTester rayCB(
					collisionObjectWrap->getCollisionObject(),
					compoundShape,
					colObjWorldTransform,
					rayFromTrans,
					rayToTrans,
					resultCallback);
#ifndef	DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
				if (dbvt)
				{
					btlVector3 localRayFrom = colObjWorldTransform.inverseTimes(rayFromTrans).getOrigin();
					btlVector3 localRayTo = colObjWorldTransform.inverseTimes(rayToTrans).getOrigin();
					btlDbvt::rayTest(dbvt->m_root, localRayFrom , localRayTo, rayCB);
				}
				else
#endif //DISABLE_DBVT_COMPOUNDSHAPE_RAYCAST_ACCELERATION
				{
					for (int i = 0, n = compoundShape->getNumChildShapes(); i < n; ++i)
					{
						rayCB.ProcessLeaf(i);
					}	
				}
			}
		}
	}
}

void	btlCollisionWorld::objectQuerySingle(const btlConvexShape* castShape,const btlTransform& convexFromTrans,const btlTransform& convexToTrans,
											btlCollisionObject* collisionObject,
											const btlCollisionShape* collisionShape,
											const btlTransform& colObjWorldTransform,
											ConvexResultCallback& resultCallback, btlScalar allowedPenetration)
{
	btlCollisionObjectWrapper tmpOb(0,collisionShape,collisionObject,colObjWorldTransform,-1,-1);
	btlCollisionWorld::objectQuerySingleInternal(castShape,convexFromTrans,convexToTrans,&tmpOb,resultCallback,allowedPenetration);
}

void	btlCollisionWorld::objectQuerySingleInternal(const btlConvexShape* castShape,const btlTransform& convexFromTrans,const btlTransform& convexToTrans,
											const btlCollisionObjectWrapper* colObjWrap,
											ConvexResultCallback& resultCallback, btlScalar allowedPenetration)
{
	const btlCollisionShape* collisionShape = colObjWrap->getCollisionShape();
	const btlTransform& colObjWorldTransform = colObjWrap->getWorldTransform();

	if (collisionShape->isConvex())
	{
		//BT_PROFILE("convexSweepConvex");
		btlConvexCast::CastResult castResult;
		castResult.m_allowedPenetration = allowedPenetration;
		castResult.m_fraction = resultCallback.m_closestHitFraction;//btScalar(1.);//??

		btlConvexShape* convexShape = (btlConvexShape*) collisionShape;
		btlVoronoiSimplexSolver	simplexSolver;
		btlGjkEpaPenetrationDepthSolver	gjkEpaPenetrationSolver;

		btlContinuousConvexCollision convexCaster1(castShape,convexShape,&simplexSolver,&gjkEpaPenetrationSolver);
		//btGjkConvexCast convexCaster2(castShape,convexShape,&simplexSolver);
		//btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

		btlConvexCast* castPtr = &convexCaster1;



		if (castPtr->calcTimeOfImpact(convexFromTrans,convexToTrans,colObjWorldTransform,colObjWorldTransform,castResult))
		{
			//add hit
			if (castResult.m_normal.length2() > btlScalar(0.0001))
			{
				if (castResult.m_fraction < resultCallback.m_closestHitFraction)
				{
					castResult.m_normal.normalize();
					btlCollisionWorld::LocalConvexResult localConvexResult
						(
						colObjWrap->getCollisionObject(),
						0,
						castResult.m_normal,
						castResult.m_hitPoint,
						castResult.m_fraction
						);

					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);

				}
			}
		}
	} else {
		if (collisionShape->isConcave())
		{
			if (collisionShape->getShapeType()==TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				//BT_PROFILE("convexSweepbtBvhTriangleMesh");
				btlBvhTriangleMeshShape* triangleMesh = (btlBvhTriangleMeshShape*)collisionShape;
				btlTransform worldTocollisionObject = colObjWorldTransform.inverse();
				btlVector3 convexFromLocal = worldTocollisionObject * convexFromTrans.getOrigin();
				btlVector3 convexToLocal = worldTocollisionObject * convexToTrans.getOrigin();
				// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
				btlTransform rotationXform = btlTransform(worldTocollisionObject.getBasis() * convexToTrans.getBasis());

				//ConvexCast::CastResult
				struct BridgeTriangleConvexcastCallback : public btlTriangleConvexcastCallback
				{
					btlCollisionWorld::ConvexResultCallback* m_resultCallback;
					const btlCollisionObject*	m_collisionObject;
					btlTriangleMeshShape*	m_triangleMesh;

					BridgeTriangleConvexcastCallback(const btlConvexShape* castShape, const btlTransform& from,const btlTransform& to,
						btlCollisionWorld::ConvexResultCallback* resultCallback, const btlCollisionObject* collisionObject,btlTriangleMeshShape*	triangleMesh, const btlTransform& triangleToWorld):
					btlTriangleConvexcastCallback(castShape, from,to, triangleToWorld, triangleMesh->getMargin()),
						m_resultCallback(resultCallback),
						m_collisionObject(collisionObject),
						m_triangleMesh(triangleMesh)
					{
					}


					virtual btlScalar reportHit(const btlVector3& hitNormalLocal, const btlVector3& hitPointLocal, btlScalar hitFraction, int partId, int triangleIndex )
					{
						btlCollisionWorld::LocalShapeInfo	shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;
						if (hitFraction <= m_resultCallback->m_closestHitFraction)
						{

							btlCollisionWorld::LocalConvexResult convexResult
								(m_collisionObject,
								&shapeInfo,
								hitNormalLocal,
								hitPointLocal,
								hitFraction);

							bool	normalInWorldSpace = true;


							return m_resultCallback->addSingleResult(convexResult,normalInWorldSpace);
						}
						return hitFraction;
					}

				};

				BridgeTriangleConvexcastCallback tccb(castShape, convexFromTrans,convexToTrans,&resultCallback,colObjWrap->getCollisionObject(),triangleMesh, colObjWorldTransform);
				tccb.m_hitFraction = resultCallback.m_closestHitFraction;
				tccb.m_allowedPenetration = allowedPenetration;
				btlVector3 boxMinLocal, boxMaxLocal;
				castShape->getAabb(rotationXform, boxMinLocal, boxMaxLocal);
				triangleMesh->performConvexcast(&tccb,convexFromLocal,convexToLocal,boxMinLocal, boxMaxLocal);
			} else
			{
				if (collisionShape->getShapeType()==STATIC_PLANE_PROXYTYPE)
				{
					btlConvexCast::CastResult castResult;
					castResult.m_allowedPenetration = allowedPenetration;
					castResult.m_fraction = resultCallback.m_closestHitFraction;
					btlStaticPlaneShape* planeShape = (btlStaticPlaneShape*) collisionShape;
					btlContinuousConvexCollision convexCaster1(castShape,planeShape);
					btlConvexCast* castPtr = &convexCaster1;

					if (castPtr->calcTimeOfImpact(convexFromTrans,convexToTrans,colObjWorldTransform,colObjWorldTransform,castResult))
					{
						//add hit
						if (castResult.m_normal.length2() > btlScalar(0.0001))
						{
							if (castResult.m_fraction < resultCallback.m_closestHitFraction)
							{
								castResult.m_normal.normalize();
								btlCollisionWorld::LocalConvexResult localConvexResult
									(
									colObjWrap->getCollisionObject(),
									0,
									castResult.m_normal,
									castResult.m_hitPoint,
									castResult.m_fraction
									);

								bool normalInWorldSpace = true;
								resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
							}
						}
					}

				} else
				{
					//BT_PROFILE("convexSweepConcave");
					btlConcaveShape* concaveShape = (btlConcaveShape*)collisionShape;
					btlTransform worldTocollisionObject = colObjWorldTransform.inverse();
					btlVector3 convexFromLocal = worldTocollisionObject * convexFromTrans.getOrigin();
					btlVector3 convexToLocal = worldTocollisionObject * convexToTrans.getOrigin();
					// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
					btlTransform rotationXform = btlTransform(worldTocollisionObject.getBasis() * convexToTrans.getBasis());

					//ConvexCast::CastResult
					struct BridgeTriangleConvexcastCallback : public btlTriangleConvexcastCallback
					{
						btlCollisionWorld::ConvexResultCallback* m_resultCallback;
						const btlCollisionObject*	m_collisionObject;
						btlConcaveShape*	m_triangleMesh;

						BridgeTriangleConvexcastCallback(const btlConvexShape* castShape, const btlTransform& from,const btlTransform& to,
							btlCollisionWorld::ConvexResultCallback* resultCallback, const btlCollisionObject* collisionObject,btlConcaveShape*	triangleMesh, const btlTransform& triangleToWorld):
						btlTriangleConvexcastCallback(castShape, from,to, triangleToWorld, triangleMesh->getMargin()),
							m_resultCallback(resultCallback),
							m_collisionObject(collisionObject),
							m_triangleMesh(triangleMesh)
						{
						}


						virtual btlScalar reportHit(const btlVector3& hitNormalLocal, const btlVector3& hitPointLocal, btlScalar hitFraction, int partId, int triangleIndex )
						{
							btlCollisionWorld::LocalShapeInfo	shapeInfo;
							shapeInfo.m_shapePart = partId;
							shapeInfo.m_triangleIndex = triangleIndex;
							if (hitFraction <= m_resultCallback->m_closestHitFraction)
							{

								btlCollisionWorld::LocalConvexResult convexResult
									(m_collisionObject,
									&shapeInfo,
									hitNormalLocal,
									hitPointLocal,
									hitFraction);

								bool	normalInWorldSpace = true;

								return m_resultCallback->addSingleResult(convexResult,normalInWorldSpace);
							}
							return hitFraction;
						}

					};

					BridgeTriangleConvexcastCallback tccb(castShape, convexFromTrans,convexToTrans,&resultCallback,colObjWrap->getCollisionObject(),concaveShape, colObjWorldTransform);
					tccb.m_hitFraction = resultCallback.m_closestHitFraction;
					tccb.m_allowedPenetration = allowedPenetration;
					btlVector3 boxMinLocal, boxMaxLocal;
					castShape->getAabb(rotationXform, boxMinLocal, boxMaxLocal);

					btlVector3 rayAabbMinLocal = convexFromLocal;
					rayAabbMinLocal.setMin(convexToLocal);
					btlVector3 rayAabbMaxLocal = convexFromLocal;
					rayAabbMaxLocal.setMax(convexToLocal);
					rayAabbMinLocal += boxMinLocal;
					rayAabbMaxLocal += boxMaxLocal;
					concaveShape->processAllTriangles(&tccb,rayAabbMinLocal,rayAabbMaxLocal);
				}
			}
		} else {
			///@todo : use AABB tree or other BVH acceleration structure!
			if (collisionShape->isCompound())
			{
				BT_PROFILE("convexSweepCompound");
				const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(collisionShape);
				int i=0;
				for (i=0;i<compoundShape->getNumChildShapes();i++)
				{
					btlTransform childTrans = compoundShape->getChildTransform(i);
					const btlCollisionShape* childCollisionShape = compoundShape->getChildShape(i);
					btlTransform childWorldTrans = colObjWorldTransform * childTrans;
					
                    struct	LocalInfoAdder : public ConvexResultCallback {
                            ConvexResultCallback* m_userCallback;
							int m_i;

                            LocalInfoAdder (int i, ConvexResultCallback *user)
								: m_userCallback(user), m_i(i)
							{
								m_closestHitFraction = m_userCallback->m_closestHitFraction;
							}
							virtual bool needsCollision(btlBroadphaseProxy* p) const
							{
								return m_userCallback->needsCollision(p);
							}
                            virtual btlScalar addSingleResult (btlCollisionWorld::LocalConvexResult&	r,	bool b)
                            {
                                    btlCollisionWorld::LocalShapeInfo	shapeInfo;
                                    shapeInfo.m_shapePart = -1;
                                    shapeInfo.m_triangleIndex = m_i;
                                    if (r.m_localShapeInfo == NULL)
                                        r.m_localShapeInfo = &shapeInfo;
									const btlScalar result = m_userCallback->addSingleResult(r, b);
									m_closestHitFraction = m_userCallback->m_closestHitFraction;
									return result;
                                    
                            }
                    };

                    LocalInfoAdder my_cb(i, &resultCallback);
					
					btlCollisionObjectWrapper tmpObj(colObjWrap,childCollisionShape,colObjWrap->getCollisionObject(),childWorldTrans,-1,i);

					objectQuerySingleInternal(castShape, convexFromTrans,convexToTrans,
						&tmpObj,my_cb, allowedPenetration);
					
				}
			}
		}
	}
}


struct btlSingleRayCallback : public btlBroadphaseRayCallback
{

	btlVector3	m_rayFromWorld;
	btlVector3	m_rayToWorld;
	btlTransform	m_rayFromTrans;
	btlTransform	m_rayToTrans;
	btlVector3	m_hitNormal;

	const btlCollisionWorld*	m_world;
	btlCollisionWorld::RayResultCallback&	m_resultCallback;

	btlSingleRayCallback(const btlVector3& rayFromWorld,const btlVector3& rayToWorld,const btlCollisionWorld* world,btlCollisionWorld::RayResultCallback& resultCallback)
		:m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld),
		m_world(world),
		m_resultCallback(resultCallback)
	{
		m_rayFromTrans.setIdentity();
		m_rayFromTrans.setOrigin(m_rayFromWorld);
		m_rayToTrans.setIdentity();
		m_rayToTrans.setOrigin(m_rayToWorld);

		btlVector3 rayDir = (rayToWorld-rayFromWorld);

		rayDir.normalize ();
		///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(m_rayToWorld-m_rayFromWorld);

	}



	virtual bool	process(const btlBroadphaseProxy* proxy)
	{
		///terminate further ray tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == btlScalar(0.f))
			return false;

		btlCollisionObject*	collisionObject = (btlCollisionObject*)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if(m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) 
		{
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			//btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
#if 0
#ifdef RECALCULATE_AABB
			btlVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
#else
			//getBroadphase()->getAabb(collisionObject->getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
			const btlVector3& collisionObjectAabbMin = collisionObject->getBroadphaseHandle()->m_aabbMin;
			const btlVector3& collisionObjectAabbMax = collisionObject->getBroadphaseHandle()->m_aabbMax;
#endif
#endif
			//btScalar hitLambda = m_resultCallback.m_closestHitFraction;
			//culling already done by broadphase
			//if (btRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
			{
				m_world->rayTestSingle(m_rayFromTrans,m_rayToTrans,
					collisionObject,
					collisionObject->getCollisionShape(),
					collisionObject->getWorldTransform(),
					m_resultCallback);
			}
		}
		return true;
	}
};

void	btlCollisionWorld::rayTest(const btlVector3& rayFromWorld, const btlVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	//BT_PROFILE("rayTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	btlSingleRayCallback rayCB(rayFromWorld,rayToWorld,this,resultCallback);

#ifndef USE_BRUTEFORCE_RAYBROADPHASE
	m_broadphasePairCache->rayTest(rayFromWorld,rayToWorld,rayCB);
#else
	for (int i=0;i<this->getNumCollisionObjects();i++)
	{
		rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
	}	
#endif //USE_BRUTEFORCE_RAYBROADPHASE

}


struct btlSingleSweepCallback : public btlBroadphaseRayCallback
{

	btlTransform	m_convexFromTrans;
	btlTransform	m_convexToTrans;
	btlVector3	m_hitNormal;
	const btlCollisionWorld*	m_world;
	btlCollisionWorld::ConvexResultCallback&	m_resultCallback;
	btlScalar	m_allowedCcdPenetration;
	const btlConvexShape* m_castShape;


	btlSingleSweepCallback(const btlConvexShape* castShape, const btlTransform& convexFromTrans,const btlTransform& convexToTrans,const btlCollisionWorld* world,btlCollisionWorld::ConvexResultCallback& resultCallback,btlScalar allowedPenetration)
		:m_convexFromTrans(convexFromTrans),
		m_convexToTrans(convexToTrans),
		m_world(world),
		m_resultCallback(resultCallback),
		m_allowedCcdPenetration(allowedPenetration),
		m_castShape(castShape)
	{
		btlVector3 unnormalizedRayDir = (m_convexToTrans.getOrigin()-m_convexFromTrans.getOrigin());
		btlVector3 rayDir = unnormalizedRayDir.normalized();
		///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(unnormalizedRayDir);

	}

	virtual bool	process(const btlBroadphaseProxy* proxy)
	{
		///terminate further convex sweep tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == btlScalar(0.f))
			return false;

		btlCollisionObject*	collisionObject = (btlCollisionObject*)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if(m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			m_world->objectQuerySingle(m_castShape, m_convexFromTrans,m_convexToTrans,
				collisionObject,
				collisionObject->getCollisionShape(),
				collisionObject->getWorldTransform(),
				m_resultCallback,
				m_allowedCcdPenetration);
		}

		return true;
	}
};



void	btlCollisionWorld::convexSweepTest(const btlConvexShape* castShape, const btlTransform& convexFromWorld, const btlTransform& convexToWorld, ConvexResultCallback& resultCallback, btlScalar allowedCcdPenetration) const
{

	BT_PROFILE("convexSweepTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	/// unfortunately the implementation for rayTest and convexSweepTest duplicated, albeit practically identical



	btlTransform	convexFromTrans,convexToTrans;
	convexFromTrans = convexFromWorld;
	convexToTrans = convexToWorld;
	btlVector3 castShapeAabbMin, castShapeAabbMax;
	/* Compute AABB that encompasses angular movement */
	{
		btlVector3 linVel, angVel;
		btlTransformUtil::calculateVelocity (convexFromTrans, convexToTrans, 1.0f, linVel, angVel);
		btlVector3 zeroLinVel;
		zeroLinVel.setValue(0,0,0);
		btlTransform R;
		R.setIdentity ();
		R.setRotation (convexFromTrans.getRotation());
		castShape->calculateTemporalAabb (R, zeroLinVel, angVel, 1.0f, castShapeAabbMin, castShapeAabbMax);
	}

#ifndef USE_BRUTEFORCE_RAYBROADPHASE

	btlSingleSweepCallback	convexCB(castShape,convexFromWorld,convexToWorld,this,resultCallback,allowedCcdPenetration);

	m_broadphasePairCache->rayTest(convexFromTrans.getOrigin(),convexToTrans.getOrigin(),convexCB,castShapeAabbMin,castShapeAabbMax);

#else
	/// go over all objects, and if the ray intersects their aabb + cast shape aabb,
	// do a ray-shape query using convexCaster (CCD)
	int i;
	for (i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject*	collisionObject= m_collisionObjects[i];
		//only perform raycast if filterMask matches
		if(resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			btlVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
			AabbExpand (collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
			btlScalar hitLambda = btlScalar(1.); //could use resultCallback.m_closestHitFraction, but needs testing
			btlVector3 hitNormal;
			if (btlRayAabb(convexFromWorld.getOrigin(),convexToWorld.getOrigin(),collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,hitNormal))
			{
				objectQuerySingle(castShape, convexFromTrans,convexToTrans,
					collisionObject,
					collisionObject->getCollisionShape(),
					collisionObject->getWorldTransform(),
					resultCallback,
					allowedCcdPenetration);
			}
		}
	}
#endif //USE_BRUTEFORCE_RAYBROADPHASE
}



struct btlBridgedManifoldResult : public btlManifoldResult
{

	btlCollisionWorld::ContactResultCallback&	m_resultCallback;

	btlBridgedManifoldResult( const btlCollisionObjectWrapper* obj0Wrap,const btlCollisionObjectWrapper* obj1Wrap,btlCollisionWorld::ContactResultCallback& resultCallback )
		:btlManifoldResult(obj0Wrap,obj1Wrap),
		m_resultCallback(resultCallback)
	{
	}

	virtual void addContactPoint(const btlVector3& normalOnBInWorld,const btlVector3& pointInWorld,btlScalar depth)
	{
		bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();
		btlVector3 pointA = pointInWorld + normalOnBInWorld * depth;
		btlVector3 localA;
		btlVector3 localB;
		if (isSwapped)
		{
			localA = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointA );
			localB = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
		} else
		{
			localA = m_body0Wrap->getCollisionObject()->getWorldTransform().invXform(pointA );
			localB = m_body1Wrap->getCollisionObject()->getWorldTransform().invXform(pointInWorld);
		}
		
		btlManifoldPoint newPt(localA,localB,normalOnBInWorld,depth);
		newPt.m_positionWorldOnA = pointA;
		newPt.m_positionWorldOnB = pointInWorld;
		
	   //BP mod, store contact triangles.
		if (isSwapped)
		{
			newPt.m_partId0 = m_partId1;
			newPt.m_partId1 = m_partId0;
			newPt.m_index0  = m_index1;
			newPt.m_index1  = m_index0;
		} else
		{
			newPt.m_partId0 = m_partId0;
			newPt.m_partId1 = m_partId1;
			newPt.m_index0  = m_index0;
			newPt.m_index1  = m_index1;
		}

		//experimental feature info, for per-triangle material etc.
		const btlCollisionObjectWrapper* obj0Wrap = isSwapped? m_body1Wrap : m_body0Wrap;
		const btlCollisionObjectWrapper* obj1Wrap = isSwapped? m_body0Wrap : m_body1Wrap;
		m_resultCallback.addSingleResult(newPt,obj0Wrap,newPt.m_partId0,newPt.m_index0,obj1Wrap,newPt.m_partId1,newPt.m_index1);

	}
	
};



struct btlSingleContactCallback : public btlBroadphaseAabbCallback
{

	btlCollisionObject* m_collisionObject;
	btlCollisionWorld*	m_world;
	btlCollisionWorld::ContactResultCallback&	m_resultCallback;
	
	
	btlSingleContactCallback(btlCollisionObject* collisionObject, btlCollisionWorld* world,btlCollisionWorld::ContactResultCallback& resultCallback)
		:m_collisionObject(collisionObject),
		m_world(world),
		m_resultCallback(resultCallback)
	{
	}

	virtual bool	process(const btlBroadphaseProxy* proxy)
	{
		btlCollisionObject*	collisionObject = (btlCollisionObject*)proxy->m_clientObject;
		if (collisionObject == m_collisionObject)
			return true;

		//only perform raycast if filterMask matches
		if(m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) 
		{
			btlCollisionObjectWrapper ob0(0,m_collisionObject->getCollisionShape(),m_collisionObject,m_collisionObject->getWorldTransform(),-1,-1);
			btlCollisionObjectWrapper ob1(0,collisionObject->getCollisionShape(),collisionObject,collisionObject->getWorldTransform(),-1,-1);

			btlCollisionAlgorithm* algorithm = m_world->getDispatcher()->findAlgorithm(&ob0,&ob1);
			if (algorithm)
			{
				btlBridgedManifoldResult contactPointResult(&ob0,&ob1, m_resultCallback);
				//discrete collision detection query
				
				algorithm->processCollision(&ob0,&ob1, m_world->getDispatchInfo(),&contactPointResult);

				algorithm->~btlCollisionAlgorithm();
				m_world->getDispatcher()->freeCollisionAlgorithm(algorithm);
			}
		}
		return true;
	}
};


///contactTest performs a discrete collision test against all objects in the btCollisionWorld, and calls the resultCallback.
///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
void	btlCollisionWorld::contactTest( btlCollisionObject* colObj, ContactResultCallback& resultCallback)
{
	btlVector3 aabbMin,aabbMax;
	colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(),aabbMin,aabbMax);
	btlSingleContactCallback	contactCB(colObj,this,resultCallback);
	
	m_broadphasePairCache->aabbTest(aabbMin,aabbMax,contactCB);
}


///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
///it reports one or more contact points (including the one with deepest penetration)
void	btlCollisionWorld::contactPairTest(btlCollisionObject* colObjA, btlCollisionObject* colObjB, ContactResultCallback& resultCallback)
{
	btlCollisionObjectWrapper obA(0,colObjA->getCollisionShape(),colObjA,colObjA->getWorldTransform(),-1,-1);
	btlCollisionObjectWrapper obB(0,colObjB->getCollisionShape(),colObjB,colObjB->getWorldTransform(),-1,-1);

	btlCollisionAlgorithm* algorithm = getDispatcher()->findAlgorithm(&obA,&obB);
	if (algorithm)
	{
		btlBridgedManifoldResult contactPointResult(&obA,&obB, resultCallback);
		//discrete collision detection query
		algorithm->processCollision(&obA,&obB, getDispatchInfo(),&contactPointResult);

		algorithm->~btlCollisionAlgorithm();
		getDispatcher()->freeCollisionAlgorithm(algorithm);
	}

}




class btl_DebugDrawcallback : public btlTriangleCallback, public btlInternalTriangleIndexCallback
{
	btlIDebugDraw*	m_debugDrawer;
	btlVector3	m_color;
	btlTransform	m_worldTrans;

public:

	btl_DebugDrawcallback(btlIDebugDraw*	debugDrawer,const btlTransform& worldTrans,const btlVector3& color) :
	  m_debugDrawer(debugDrawer),
		  m_color(color),
		  m_worldTrans(worldTrans)
	  {
	  }

	  virtual void internalProcessTriangleIndex(btlVector3* triangle,int partId,int  triangleIndex)
	  {
		  processTriangle(triangle,partId,triangleIndex);
	  }

	  virtual void processTriangle(btlVector3* triangle,int partId, int triangleIndex)
	  {
		  (void)partId;
		  (void)triangleIndex;

		  btlVector3 wv0,wv1,wv2;
		  wv0 = m_worldTrans*triangle[0];
		  wv1 = m_worldTrans*triangle[1];
		  wv2 = m_worldTrans*triangle[2];
		  btlVector3 center = (wv0+wv1+wv2)*btlScalar(1./3.);
          
          if (m_debugDrawer->getDebugMode() & btlIDebugDraw::DBG_DrawNormals )
          {
		    btlVector3 normal = (wv1-wv0).cross(wv2-wv0);
		    normal.normalize();
		    btlVector3 normalColor(1,1,0);
		    m_debugDrawer->drawLine(center,center+normal,normalColor);
          }
		  m_debugDrawer->drawLine(wv0,wv1,m_color);
		  m_debugDrawer->drawLine(wv1,wv2,m_color);
		  m_debugDrawer->drawLine(wv2,wv0,m_color);
	  }
};


void btlCollisionWorld::debugDrawObject(const btlTransform& worldTransform, const btlCollisionShape* shape, const btlVector3& color)
{
	// Draw a small simplex at the center of the object
	if (getDebugDrawer() && getDebugDrawer()->getDebugMode() & btlIDebugDraw::DBG_DrawFrames)
	{
		getDebugDrawer()->drawTransform(worldTransform,1);
	}

	if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(shape);
		for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
		{
			btlTransform childTrans = compoundShape->getChildTransform(i);
			const btlCollisionShape* colShape = compoundShape->getChildShape(i);
			debugDrawObject(worldTransform*childTrans,colShape,color);
		}

	} else
	{

        switch (shape->getShapeType())
        {

        case BOX_SHAPE_PROXYTYPE:
            {
                const btlBoxShape* boxShape = static_cast<const btlBoxShape*>(shape);
                btlVector3 halfExtents = boxShape->getHalfExtentsWithMargin();
                getDebugDrawer()->drawBox(-halfExtents,halfExtents,worldTransform,color);
                break;
            }

        case SPHERE_SHAPE_PROXYTYPE:
            {
                const btlSphereShape* sphereShape = static_cast<const btlSphereShape*>(shape);
                btlScalar radius = sphereShape->getMargin();//radius doesn't include the margin, so draw with margin

                getDebugDrawer()->drawSphere(radius, worldTransform, color);
                break;
            }
        case MULTI_SPHERE_SHAPE_PROXYTYPE:
            {
                const btlMultiSphereShape* multiSphereShape = static_cast<const btlMultiSphereShape*>(shape);

                btlTransform childTransform;
                childTransform.setIdentity();

                for (int i = multiSphereShape->getSphereCount()-1; i>=0;i--)
                {
                    childTransform.setOrigin(multiSphereShape->getSpherePosition(i));
                    getDebugDrawer()->drawSphere(multiSphereShape->getSphereRadius(i), worldTransform*childTransform, color);
                }

                break;
            }
        case CAPSULE_SHAPE_PROXYTYPE:
            {
                const btlCapsuleShape* capsuleShape = static_cast<const btlCapsuleShape*>(shape);

                btlScalar radius = capsuleShape->getRadius();
                btlScalar halfHeight = capsuleShape->getHalfHeight();

                int upAxis = capsuleShape->getUpAxis();
                getDebugDrawer()->drawCapsule(radius, halfHeight, upAxis, worldTransform, color);
                break;
            }
        case CONE_SHAPE_PROXYTYPE:
            {
                const btlConeShape* coneShape = static_cast<const btlConeShape*>(shape);
                btlScalar radius = coneShape->getRadius();//+coneShape->getMargin();
                btlScalar height = coneShape->getHeight();//+coneShape->getMargin();

                int upAxis= coneShape->getConeUpIndex();
                getDebugDrawer()->drawCone(radius, height, upAxis, worldTransform, color);
                break;

            }
        case CYLINDER_SHAPE_PROXYTYPE:
            {
                const btlCylinderShape* cylinder = static_cast<const btlCylinderShape*>(shape);
                int upAxis = cylinder->getUpAxis();
                btlScalar radius = cylinder->getRadius();
                btlScalar halfHeight = cylinder->getHalfExtentsWithMargin()[upAxis];
                getDebugDrawer()->drawCylinder(radius, halfHeight, upAxis, worldTransform, color);
                break;
            }

        case STATIC_PLANE_PROXYTYPE:
            {
                const btlStaticPlaneShape* staticPlaneShape = static_cast<const btlStaticPlaneShape*>(shape);
                btlScalar planeConst = staticPlaneShape->getPlaneConstant();
                const btlVector3& planeNormal = staticPlaneShape->getPlaneNormal();
                getDebugDrawer()->drawPlane(planeNormal, planeConst,worldTransform, color);
                break;

            }
        default:
            {

                /// for polyhedral shapes
                if (shape->isPolyhedral())
                {
                    btlPolyhedralConvexShape* polyshape = (btlPolyhedralConvexShape*) shape;
                    
                    int i;
                    if (polyshape->getConvexPolyhedron())
                    {
                        const btlConvexPolyhedron* poly = polyshape->getConvexPolyhedron();
                        for (i=0;i<poly->m_faces.size();i++)
                        {
                            btlVector3 centroid(0,0,0);
                            int numVerts = poly->m_faces[i].m_indices.size();
                            if (numVerts)
                            {
                                int lastV = poly->m_faces[i].m_indices[numVerts-1];
                                for (int v=0;v<poly->m_faces[i].m_indices.size();v++)
                                {
                                    int curVert = poly->m_faces[i].m_indices[v];
                                    centroid+=poly->m_vertices[curVert];
                                    getDebugDrawer()->drawLine(worldTransform*poly->m_vertices[lastV],worldTransform*poly->m_vertices[curVert],color);
                                    lastV = curVert;
                                }
                            }
                            centroid*= btlScalar(1.f)/btlScalar(numVerts);
                            if (getDebugDrawer()->getDebugMode() & btlIDebugDraw::DBG_DrawNormals)
                            {
                                btlVector3 normalColor(1,1,0);
                                btlVector3 faceNormal(poly->m_faces[i].m_plane[0],poly->m_faces[i].m_plane[1],poly->m_faces[i].m_plane[2]);
                                getDebugDrawer()->drawLine(worldTransform*centroid,worldTransform*(centroid+faceNormal),normalColor);
                            }
                            
                        }
                        
                        
                    } else
                    {
                        for (i=0;i<polyshape->getNumEdges();i++)
                        {
                            btlVector3 a,b;
                            polyshape->getEdge(i,a,b);
                            btlVector3 wa = worldTransform * a;
                            btlVector3 wb = worldTransform * b;
                            getDebugDrawer()->drawLine(wa,wb,color);
                        }
                    }
                    
                    
                }
                    
                if (shape->isConcave())
                {
                    btlConcaveShape* concaveMesh = (btlConcaveShape*) shape;

                    ///@todo pass camera, for some culling? no -> we are not a graphics lib
                    btlVector3 aabbMax(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
                    btlVector3 aabbMin(btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT));

                    btl_DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
                    concaveMesh->processAllTriangles(&drawCallback,aabbMin,aabbMax);

                }

                if (shape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE)
                {
                    btlConvexTriangleMeshShape* convexMesh = (btlConvexTriangleMeshShape*) shape;
                    //todo: pass camera for some culling			
                    btlVector3 aabbMax(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
                    btlVector3 aabbMin(btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT));
                    //DebugDrawcallback drawCallback;
                    btl_DebugDrawcallback drawCallback(getDebugDrawer(),worldTransform,color);
                    convexMesh->getMeshInterface()->InternalProcessAllTriangles(&drawCallback,aabbMin,aabbMax);
                }


                
            }
       
		}
	}
}


void	btlCollisionWorld::debugDrawWorld()
{
	if (getDebugDrawer())
	{
		btlIDebugDraw::DefaultColors defaultColors = getDebugDrawer()->getDefaultColors();

		if ( getDebugDrawer()->getDebugMode() & btlIDebugDraw::DBG_DrawContactPoints)
		{
		

			if (getDispatcher())
			{
				int numManifolds = getDispatcher()->getNumManifolds();
			
				for (int i=0;i<numManifolds;i++)
				{
					btlPersistentManifold* contactManifold = getDispatcher()->getManifoldByIndexInternal(i);
					//btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
					//btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

					int numContacts = contactManifold->getNumContacts();
					for (int j=0;j<numContacts;j++)
					{
						btlManifoldPoint& cp = contactManifold->getContactPoint(j);
						getDebugDrawer()->drawContactPoint(cp.m_positionWorldOnB,cp.m_normalWorldOnB,cp.getDistance(),cp.getLifeTime(),defaultColors.m_contactPoint);
					}
				}
			}
		}

		if ((getDebugDrawer()->getDebugMode() & (btlIDebugDraw::DBG_DrawWireframe | btlIDebugDraw::DBG_DrawAabb)))
		{
			int i;

			for (  i=0;i<m_collisionObjects.size();i++)
			{
				btlCollisionObject* colObj = m_collisionObjects[i];
				if ((colObj->getCollisionFlags() & btlCollisionObject::CF_DISABLE_VISUALIZE_OBJECT)==0)
				{
					if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & btlIDebugDraw::DBG_DrawWireframe))
					{
						btlVector3 color(btlScalar(0.4),btlScalar(0.4),btlScalar(0.4));

						switch(colObj->getActivationState())
						{
						case  ACTIVE_TAG:
							color = defaultColors.m_activeObject; break;
						case ISLAND_SLEEPING:
							color =  defaultColors.m_deactivatedObject;break;
						case WANTS_DEACTIVATION:
							color = defaultColors.m_wantsDeactivationObject;break;
						case DISABLE_DEACTIVATION:
							color = defaultColors.m_disabledDeactivationObject;break;
						case DISABLE_SIMULATION:
							color = defaultColors.m_disabledSimulationObject;break;
						default:
							{
								color = btlVector3(btlScalar(.3),btlScalar(0.3),btlScalar(0.3));
							}
						};

						debugDrawObject(colObj->getWorldTransform(),colObj->getCollisionShape(),color);
					}
					if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btlIDebugDraw::DBG_DrawAabb))
					{
						btlVector3 minAabb,maxAabb;
						btlVector3 colorvec = defaultColors.m_aabb;
						colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb,maxAabb);
						btlVector3 contactThreshold(btl_gContactBreakingThreshold,btl_gContactBreakingThreshold,btl_gContactBreakingThreshold);
						minAabb -= contactThreshold;
						maxAabb += contactThreshold;

						btlVector3 minAabb2,maxAabb2;

						if(getDispatchInfo().m_useContinuous && colObj->getInternalType()==btlCollisionObject::CO_RIGID_BODY && !colObj->isStaticOrKinematicObject())
						{
							colObj->getCollisionShape()->getAabb(colObj->getInterpolationWorldTransform(),minAabb2,maxAabb2);
							minAabb2 -= contactThreshold;
							maxAabb2 += contactThreshold;
							minAabb.setMin(minAabb2);
							maxAabb.setMax(maxAabb2);
						}

						m_debugDrawer->drawAabb(minAabb,maxAabb,colorvec);
					}
				}
			}
		}
	}
}


void	btlCollisionWorld::serializeCollisionObjects(btlSerializer* serializer)
{
	int i;

	///keep track of shapes already serialized
	btlHashMap<btlHashPtr,btlCollisionShape*>	serializedShapes;

	for (i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		btlCollisionShape* shape = colObj->getCollisionShape();

		if (!serializedShapes.find(shape))
		{
			serializedShapes.insert(shape,shape);
			shape->serializeSingleShape(serializer);
		}
	}

	//serialize all collision objects
	for (i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		if ((colObj->getInternalType() == btlCollisionObject::CO_COLLISION_OBJECT) || (colObj->getInternalType() == btlCollisionObject::CO_FEATHERSTONE_LINK))
		{
			colObj->serializeSingleObject(serializer);
		}
	}
}


void	btlCollisionWorld::serialize(btlSerializer* serializer)
{

	serializer->startSerialization();
	
	serializeCollisionObjects(serializer);
	
	serializer->finishSerialization();
}

