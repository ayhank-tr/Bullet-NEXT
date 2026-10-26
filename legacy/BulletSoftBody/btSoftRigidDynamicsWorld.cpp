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


#include "btSoftRigidDynamicsWorld.h"
#include "LinearMath/btQuickprof.h"

//softbody & helpers
#include "btSoftBody.h"
#include "btSoftBodyHelpers.h"
#include "btSoftBodySolvers.h"
#include "btDefaultSoftBodySolver.h"
#include "LinearMath/btSerializer.h"


btlSoftRigidDynamicsWorld::btlSoftRigidDynamicsWorld(
	btlDispatcher* dispatcher,
	btlBroadphaseInterface* pairCache,
	btlConstraintSolver* constraintSolver,
	btlCollisionConfiguration* collisionConfiguration,
	btlSoftBodySolver *softBodySolver ) : 
		btlDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration),
		m_softBodySolver( softBodySolver ),
		m_ownsSolver(false)
{
	if( !m_softBodySolver )
	{
		void* ptr = btlAlignedAlloc(sizeof(btlDefaultSoftBodySolver),16);
		m_softBodySolver = new(ptr) btlDefaultSoftBodySolver();
		m_ownsSolver = true;
	}

	m_drawFlags			=	fDrawFlags::Std;
	m_drawNodeTree		=	true;
	m_drawFaceTree		=	false;
	m_drawClusterTree	=	false;
	m_sbi.m_broadphase = pairCache;
	m_sbi.m_dispatcher = dispatcher;
	m_sbi.m_sparsesdf.Initialize();
	m_sbi.m_sparsesdf.Reset();

	m_sbi.air_density		=	(btlScalar)1.2;
	m_sbi.water_density	=	0;
	m_sbi.water_offset		=	0;
	m_sbi.water_normal		=	btlVector3(0,0,0);
	m_sbi.m_gravity.setValue(0,-10,0);

	m_sbi.m_sparsesdf.Initialize();


}

btlSoftRigidDynamicsWorld::~btlSoftRigidDynamicsWorld()
{
	if (m_ownsSolver)
	{
		m_softBodySolver->~btlSoftBodySolver();
		btlAlignedFree(m_softBodySolver);
	}
}

void	btlSoftRigidDynamicsWorld::predictUnconstraintMotion(btlScalar timeStep)
{
	btlDiscreteDynamicsWorld::predictUnconstraintMotion( timeStep );
	{
		BT_PROFILE("predictUnconstraintMotionSoftBody");
		m_softBodySolver->predictMotion( float(timeStep) );
	}
}

void	btlSoftRigidDynamicsWorld::internalSingleStepSimulation( btlScalar timeStep )
{

	// Let the solver grab the soft bodies and if necessary optimize for it
	m_softBodySolver->optimize( getSoftBodyArray() );

	if( !m_softBodySolver->checkInitialized() )
	{
		btlAssert( "Solver initialization failed\n" );
	}

	btlDiscreteDynamicsWorld::internalSingleStepSimulation( timeStep );

	///solve soft bodies constraints
	solveSoftBodiesConstraints( timeStep );

	//self collisions
	for ( int i=0;i<m_softBodies.size();i++)
	{
		btlSoftBody*	psb=(btlSoftBody*)m_softBodies[i];
		psb->defaultCollisionHandler(psb);
	}

	///update soft bodies
	m_softBodySolver->updateSoftBodies( );
	
	// End solver-wise simulation step
	// ///////////////////////////////

}

void	btlSoftRigidDynamicsWorld::solveSoftBodiesConstraints( btlScalar timeStep )
{
	BT_PROFILE("solveSoftConstraints");

	if(m_softBodies.size())
	{
		btlSoftBody::solveClusters(m_softBodies);
	}

	// Solve constraints solver-wise
	m_softBodySolver->solveConstraints( timeStep * m_softBodySolver->getTimeScale() );

}

void	btlSoftRigidDynamicsWorld::addSoftBody(btlSoftBody* body,short int collisionFilterGroup,short int collisionFilterMask)
{
	m_softBodies.push_back(body);

	// Set the soft body solver that will deal with this body
	// to be the world's solver
	body->setSoftBodySolver( m_softBodySolver );

	btlCollisionWorld::addCollisionObject(body,
		collisionFilterGroup,
		collisionFilterMask);

}

void	btlSoftRigidDynamicsWorld::removeSoftBody(btlSoftBody* body)
{
	m_softBodies.remove(body);

	btlCollisionWorld::removeCollisionObject(body);
}

void	btlSoftRigidDynamicsWorld::removeCollisionObject(btlCollisionObject* collisionObject)
{
	btlSoftBody* body = btlSoftBody::upcast(collisionObject);
	if (body)
		removeSoftBody(body);
	else
		btlDiscreteDynamicsWorld::removeCollisionObject(collisionObject);
}

void	btlSoftRigidDynamicsWorld::debugDrawWorld()
{
	btlDiscreteDynamicsWorld::debugDrawWorld();

	if (getDebugDrawer())
	{
		int i;
		for (  i=0;i<this->m_softBodies.size();i++)
		{
			btlSoftBody*	psb=(btlSoftBody*)this->m_softBodies[i];
			if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btlIDebugDraw::DBG_DrawWireframe)))
			{
				btlSoftBodyHelpers::DrawFrame(psb,m_debugDrawer);
				btlSoftBodyHelpers::Draw(psb,m_debugDrawer,m_drawFlags);
			}
			
			if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btlIDebugDraw::DBG_DrawAabb))
			{
				if(m_drawNodeTree)		btlSoftBodyHelpers::DrawNodeTree(psb,m_debugDrawer);
				if(m_drawFaceTree)		btlSoftBodyHelpers::DrawFaceTree(psb,m_debugDrawer);
				if(m_drawClusterTree)	btlSoftBodyHelpers::DrawClusterTree(psb,m_debugDrawer);
			}
		}		
	}	
}




struct btlSoftSingleRayCallback : public btlBroadphaseRayCallback
{
	btlVector3	m_rayFromWorld;
	btlVector3	m_rayToWorld;
	btlTransform	m_rayFromTrans;
	btlTransform	m_rayToTrans;
	btlVector3	m_hitNormal;

	const btlSoftRigidDynamicsWorld*	m_world;
	btlCollisionWorld::RayResultCallback&	m_resultCallback;

	btlSoftSingleRayCallback(const btlVector3& rayFromWorld,const btlVector3& rayToWorld,const btlSoftRigidDynamicsWorld* world,btlCollisionWorld::RayResultCallback& resultCallback)
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
		///what about division by zero? --> just set rayDirection[i] to INF/1e30
		m_rayDirectionInverse[0] = rayDir[0] == btlScalar(0.0) ? btlScalar(1e30) : btlScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btlScalar(0.0) ? btlScalar(1e30) : btlScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btlScalar(0.0) ? btlScalar(1e30) : btlScalar(1.0) / rayDir[2];
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

void	btlSoftRigidDynamicsWorld::rayTest(const btlVector3& rayFromWorld, const btlVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	BT_PROFILE("rayTest");
	/// use the broadphase to accelerate the search for objects, based on their aabb
	/// and for each object with ray-aabb overlap, perform an exact ray test
	btlSoftSingleRayCallback rayCB(rayFromWorld,rayToWorld,this,resultCallback);

#ifndef USE_BRUTEFORCE_RAYBROADPHASE
	m_broadphasePairCache->rayTest(rayFromWorld,rayToWorld,rayCB);
#else
	for (int i=0;i<this->getNumCollisionObjects();i++)
	{
		rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
	}	
#endif //USE_BRUTEFORCE_RAYBROADPHASE

}


void	btlSoftRigidDynamicsWorld::rayTestSingle(const btlTransform& rayFromTrans,const btlTransform& rayToTrans,
					  btlCollisionObject* collisionObject,
					  const btlCollisionShape* collisionShape,
					  const btlTransform& colObjWorldTransform,
					  RayResultCallback& resultCallback)
{
	if (collisionShape->isSoftBody()) {
		btlSoftBody* softBody = btlSoftBody::upcast(collisionObject);
		if (softBody) {
			btlSoftBody::sRayCast softResult;
			if (softBody->rayTest(rayFromTrans.getOrigin(), rayToTrans.getOrigin(), softResult)) 
			{
				
				if (softResult.fraction<= resultCallback.m_closestHitFraction)
				{

					btlCollisionWorld::LocalShapeInfo shapeInfo;
					shapeInfo.m_shapePart = 0;
					shapeInfo.m_triangleIndex = softResult.index;
					// get the normal
					btlVector3 rayDir = rayToTrans.getOrigin() - rayFromTrans.getOrigin();
					btlVector3 normal=-rayDir;
					normal.normalize();

					if (softResult.feature == btlSoftBody::eFeature::Face)
					{
						normal = softBody->m_faces[softResult.index].m_normal;
						if (normal.dot(rayDir) > 0) {
							// normal always point toward origin of the ray
							normal = -normal;
						}
					}
	
					btlCollisionWorld::LocalRayResult rayResult
						(collisionObject,
						 &shapeInfo,
						 normal,
						 softResult.fraction);
					bool	normalInWorldSpace = true;
					resultCallback.addSingleResult(rayResult,normalInWorldSpace);
				}
			}
		}
	} 
	else {
		btlCollisionWorld::rayTestSingle(rayFromTrans,rayToTrans,collisionObject,collisionShape,colObjWorldTransform,resultCallback);
	}
}


void	btlSoftRigidDynamicsWorld::serializeSoftBodies(btlSerializer* serializer)
{
	int i;
	//serialize all collision objects
	for (i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->getInternalType() & btlCollisionObject::CO_SOFT_BODY)
		{
			int len = colObj->calculateSerializeBufferSize();
			btlChunk* chunk = serializer->allocate(len,1);
			const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk,structType,BT_SOFTBODY_CODE,colObj);
		}
	}

}

void	btlSoftRigidDynamicsWorld::serialize(btlSerializer* serializer)
{

	serializer->startSerialization();

	serializeDynamicsWorldInfo( serializer);

	serializeSoftBodies(serializer);

	serializeRigidBodies(serializer);

	serializeCollisionObjects(serializer);

	serializer->finishSerialization();
}


