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


#include "btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

btlConvexConcaveCollisionAlgorithm::btlConvexConcaveCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped)
: btlActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_isSwapped(isSwapped),
m_btConvexTriangleCallback(ci.m_dispatcher1,body0Wrap,body1Wrap,isSwapped)
{
}

btlConvexConcaveCollisionAlgorithm::~btlConvexConcaveCollisionAlgorithm()
{
}

void	btlConvexConcaveCollisionAlgorithm::getAllContactManifolds(btlManifoldArray&	manifoldArray)
{
	if (m_btConvexTriangleCallback.m_manifoldPtr)
	{
		manifoldArray.push_back(m_btConvexTriangleCallback.m_manifoldPtr);
	}
}


btlConvexTriangleCallback::btlConvexTriangleCallback(btlDispatcher*  dispatcher,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped):
	  m_dispatcher(dispatcher),
	m_dispatchInfoPtr(0)
{
	m_convexBodyWrap = isSwapped? body1Wrap:body0Wrap;
	m_triBodyWrap = isSwapped? body0Wrap:body1Wrap;
	
	  //
	  // create the manifold from the dispatcher 'manifold pool'
	  //
	  m_manifoldPtr = m_dispatcher->getNewManifold(m_convexBodyWrap->getCollisionObject(),m_triBodyWrap->getCollisionObject());

  	  clearCache();
}

btlConvexTriangleCallback::~btlConvexTriangleCallback()
{
	clearCache();
	m_dispatcher->releaseManifold( m_manifoldPtr );
  
}
  

void	btlConvexTriangleCallback::clearCache()
{
	m_dispatcher->clearManifold(m_manifoldPtr);
}


void btlConvexTriangleCallback::processTriangle(btlVector3* triangle,int
partId, int triangleIndex)
{

	if (!TestTriangleAgainstAabb2(triangle, m_aabbMin, m_aabbMax))
	{
		return;
	}

        //just for debugging purposes
        //printf("triangle %d",m_triangleCount++);



	btlCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher1 = m_dispatcher;



#if 0	
	
	///debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr->m_debugDraw && (m_dispatchInfoPtr->m_debugDraw->getDebugMode() &btlIDebugDraw::DBG_DrawWireframe ))
	{
		const btlCollisionObject* ob = const_cast<btlCollisionObject*>(m_triBodyWrap->getCollisionObject());
		btlVector3 color(1,1,0);
		btlTransform& tr = ob->getWorldTransform();
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(triangle[1]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(triangle[0]),color);
	}
#endif
	
	if (m_convexBodyWrap->getCollisionShape()->isConvex())
	{
		btlTriangleShape tm(triangle[0],triangle[1],triangle[2]);	
		tm.setMargin(m_collisionMarginTriangle);
		
		
		btlCollisionObjectWrapper triObWrap(m_triBodyWrap,&tm,m_triBodyWrap->getCollisionObject(),m_triBodyWrap->getWorldTransform(),partId,triangleIndex);//correct transform?
		btlCollisionAlgorithm* colAlgo = ci.m_dispatcher1->findAlgorithm(m_convexBodyWrap,&triObWrap,m_manifoldPtr);

		const btlCollisionObjectWrapper* tmpWrap = 0;

		if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject())
		{
			tmpWrap = m_resultOut->getBody0Wrap();
			m_resultOut->setBody0Wrap(&triObWrap);
			m_resultOut->setShapeIdentifiersA(partId,triangleIndex);
		}
		else
		{
			tmpWrap = m_resultOut->getBody1Wrap();
			m_resultOut->setBody1Wrap(&triObWrap);
			m_resultOut->setShapeIdentifiersB(partId,triangleIndex);
		}
	
		colAlgo->processCollision(m_convexBodyWrap,&triObWrap,*m_dispatchInfoPtr,m_resultOut);

		if (m_resultOut->getBody0Internal() == m_triBodyWrap->getCollisionObject())
		{
			m_resultOut->setBody0Wrap(tmpWrap);
		} else
		{
			m_resultOut->setBody1Wrap(tmpWrap);
		}
		


		colAlgo->~btlCollisionAlgorithm();
		ci.m_dispatcher1->freeCollisionAlgorithm(colAlgo);
	}

}



void	btlConvexTriangleCallback::setTimeStepAndCounters(btlScalar collisionMarginTriangle,const btlDispatcherInfo& dispatchInfo,const btlCollisionObjectWrapper* convexBodyWrap, const btlCollisionObjectWrapper* triBodyWrap, btlManifoldResult* resultOut)
{
	m_convexBodyWrap = convexBodyWrap;
	m_triBodyWrap = triBodyWrap;

	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle;
	m_resultOut = resultOut;

	//recalc aabbs
	btlTransform convexInTriangleSpace;
	convexInTriangleSpace = m_triBodyWrap->getWorldTransform().inverse() * m_convexBodyWrap->getWorldTransform();
	const btlCollisionShape* convexShape = static_cast<const btlCollisionShape*>(m_convexBodyWrap->getCollisionShape());
	//CollisionShape* triangleShape = static_cast<btCollisionShape*>(triBody->m_collisionShape);
	convexShape->getAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);
	btlScalar extraMargin = collisionMarginTriangle;
	btlVector3 extra(extraMargin,extraMargin,extraMargin);

	m_aabbMax += extra;
	m_aabbMin -= extra;
	
}

void btlConvexConcaveCollisionAlgorithm::clearCache()
{
	m_btConvexTriangleCallback.clearCache();

}

void btlConvexConcaveCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	
	
	const btlCollisionObjectWrapper* convexBodyWrap = m_isSwapped ? body1Wrap : body0Wrap;
	const btlCollisionObjectWrapper* triBodyWrap = m_isSwapped ? body0Wrap : body1Wrap;

	if (triBodyWrap->getCollisionShape()->isConcave())
	{


		
		const btlConcaveShape* concaveShape = static_cast<const btlConcaveShape*>( triBodyWrap->getCollisionShape());
		
		if (convexBodyWrap->getCollisionShape()->isConvex())
		{
			btlScalar collisionMarginTriangle = concaveShape->getMargin();
					
			resultOut->setPersistentManifold(m_btConvexTriangleCallback.m_manifoldPtr);
			m_btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle,dispatchInfo,convexBodyWrap,triBodyWrap,resultOut);

			m_btConvexTriangleCallback.m_manifoldPtr->setBodies(convexBodyWrap->getCollisionObject(),triBodyWrap->getCollisionObject());

			concaveShape->processAllTriangles( &m_btConvexTriangleCallback,m_btConvexTriangleCallback.getAabbMin(),m_btConvexTriangleCallback.getAabbMax());
			
			resultOut->refreshContactPoints();

			m_btConvexTriangleCallback.clearWrapperData();
	
		}
	
	}

}


btlScalar btlConvexConcaveCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	btlCollisionObject* convexbody = m_isSwapped ? body1 : body0;
	btlCollisionObject* triBody = m_isSwapped ? body0 : body1;


	//quick approximation using raycast, todo: hook up to the continuous collision detection (one of the btConvexCast)

	//only perform CCD above a certain threshold, this prevents blocking on the long run
	//because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
	btlScalar squareMot0 = (convexbody->getInterpolationWorldTransform().getOrigin() - convexbody->getWorldTransform().getOrigin()).length2();
	if (squareMot0 < convexbody->getCcdSquareMotionThreshold())
	{
		return btlScalar(1.);
	}

	//const btVector3& from = convexbody->m_worldTransform.getOrigin();
	//btVector3 to = convexbody->m_interpolationWorldTransform.getOrigin();
	//todo: only do if the motion exceeds the 'radius'

	btlTransform triInv = triBody->getWorldTransform().inverse();
	btlTransform convexFromLocal = triInv * convexbody->getWorldTransform();
	btlTransform convexToLocal = triInv * convexbody->getInterpolationWorldTransform();

	struct LocalTriangleSphereCastCallback	: public btlTriangleCallback
	{
		btlTransform m_ccdSphereFromTrans;
		btlTransform m_ccdSphereToTrans;
		btlTransform	m_meshTransform;

		btlScalar	m_ccdSphereRadius;
		btlScalar	m_hitFraction;
	

		LocalTriangleSphereCastCallback(const btlTransform& from,const btlTransform& to,btlScalar ccdSphereRadius,btlScalar hitFraction)
			:m_ccdSphereFromTrans(from),
			m_ccdSphereToTrans(to),
			m_ccdSphereRadius(ccdSphereRadius),
			m_hitFraction(hitFraction)
		{			
		}
		
		
		virtual void processTriangle(btlVector3* triangle, int partId, int triangleIndex)
		{
			(void)partId;
			(void)triangleIndex;
			//do a swept sphere for now
			btlTransform ident;
			ident.setIdentity();
			btlConvexCast::CastResult castResult;
			castResult.m_fraction = m_hitFraction;
			btlSphereShape	pointShape(m_ccdSphereRadius);
			btlTriangleShape	triShape(triangle[0],triangle[1],triangle[2]);
			btlVoronoiSimplexSolver	simplexSolver;
			btlSubsimplexConvexCast convexCaster(&pointShape,&triShape,&simplexSolver);
			//GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
			//ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
			//local space?

			if (convexCaster.calcTimeOfImpact(m_ccdSphereFromTrans,m_ccdSphereToTrans,
				ident,ident,castResult))
			{
				if (m_hitFraction > castResult.m_fraction)
					m_hitFraction = castResult.m_fraction;
			}

		}

	};


	

	
	if (triBody->getCollisionShape()->isConcave())
	{
		btlVector3 rayAabbMin = convexFromLocal.getOrigin();
		rayAabbMin.setMin(convexToLocal.getOrigin());
		btlVector3 rayAabbMax = convexFromLocal.getOrigin();
		rayAabbMax.setMax(convexToLocal.getOrigin());
		btlScalar ccdRadius0 = convexbody->getCcdSweptSphereRadius();
		rayAabbMin -= btlVector3(ccdRadius0,ccdRadius0,ccdRadius0);
		rayAabbMax += btlVector3(ccdRadius0,ccdRadius0,ccdRadius0);

		btlScalar curHitFraction = btlScalar(1.); //is this available?
		LocalTriangleSphereCastCallback raycastCallback(convexFromLocal,convexToLocal,
			convexbody->getCcdSweptSphereRadius(),curHitFraction);

		raycastCallback.m_hitFraction = convexbody->getHitFraction();

		btlCollisionObject* concavebody = triBody;

		btlConcaveShape* triangleMesh = (btlConcaveShape*) concavebody->getCollisionShape();
		
		if (triangleMesh)
		{
			triangleMesh->processAllTriangles(&raycastCallback,rayAabbMin,rayAabbMax);
		}
	


		if (raycastCallback.m_hitFraction < convexbody->getHitFraction())
		{
			convexbody->setHitFraction( raycastCallback.m_hitFraction);
			return raycastCallback.m_hitFraction;
		}
	}

	return btlScalar(1.);

}
