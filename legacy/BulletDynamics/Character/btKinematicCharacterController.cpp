/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <stdio.h>
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btDefaultMotionState.h"
#include "btKinematicCharacterController.h"


// static helper method
static btlVector3
getNormalizedVector(const btlVector3& v)
{
	btlVector3 n(0, 0, 0);

	if (v.length() > SIMD_EPSILON) {
		n = v.normalized();
	}
	return n;
}


///@todo Interact with dynamic objects,
///Ride kinematicly animated platforms properly
///More realistic (or maybe just a config option) falling
/// -> Should integrate falling velocity manually and use that in stepDown()
///Support jumping
///Support ducking
class btlKinematicClosestNotMeRayResultCallback : public btlCollisionWorld::ClosestRayResultCallback
{
public:
	btlKinematicClosestNotMeRayResultCallback (btlCollisionObject* me) : btlCollisionWorld::ClosestRayResultCallback(btlVector3(0.0, 0.0, 0.0), btlVector3(0.0, 0.0, 0.0))
	{
		m_me = me;
	}

	virtual btlScalar addSingleResult(btlCollisionWorld::LocalRayResult& rayResult,bool normalInWorldSpace)
	{
		if (rayResult.m_collisionObject == m_me)
			return 1.0;

		return ClosestRayResultCallback::addSingleResult (rayResult, normalInWorldSpace);
	}
protected:
	btlCollisionObject* m_me;
};

class btlKinematicClosestNotMeConvexResultCallback : public btlCollisionWorld::ClosestConvexResultCallback
{
public:
	btlKinematicClosestNotMeConvexResultCallback (btlCollisionObject* me, const btlVector3& up, btlScalar minSlopeDot)
	: btlCollisionWorld::ClosestConvexResultCallback(btlVector3(0.0, 0.0, 0.0), btlVector3(0.0, 0.0, 0.0))
	, m_me(me)
	, m_up(up)
	, m_minSlopeDot(minSlopeDot)
	{
	}

	virtual btlScalar addSingleResult(btlCollisionWorld::LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return btlScalar(1.0);

		if (!convexResult.m_hitCollisionObject->hasContactResponse())
			return btlScalar(1.0);

		btlVector3 hitNormalWorld;
		if (normalInWorldSpace)
		{
			hitNormalWorld = convexResult.m_hitNormalLocal;
		} else
		{
			///need to transform normal into worldspace
			hitNormalWorld = convexResult.m_hitCollisionObject->getWorldTransform().getBasis()*convexResult.m_hitNormalLocal;
		}

		btlScalar dotUp = m_up.dot(hitNormalWorld);
		if (dotUp < m_minSlopeDot) {
			return btlScalar(1.0);
		}

		return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	}
protected:
	btlCollisionObject* m_me;
	const btlVector3 m_up;
	btlScalar m_minSlopeDot;
};

/*
 * Returns the reflection direction of a ray going 'direction' hitting a surface with normal 'normal'
 *
 * from: http://www-cs-students.stanford.edu/~adityagp/final/node3.html
 */
btlVector3 btlKinematicCharacterController::computeReflectionDirection (const btlVector3& direction, const btlVector3& normal)
{
	return direction - (btlScalar(2.0) * direction.dot(normal)) * normal;
}

/*
 * Returns the portion of 'direction' that is parallel to 'normal'
 */
btlVector3 btlKinematicCharacterController::parallelComponent (const btlVector3& direction, const btlVector3& normal)
{
	btlScalar magnitude = direction.dot(normal);
	return normal * magnitude;
}

/*
 * Returns the portion of 'direction' that is perpindicular to 'normal'
 */
btlVector3 btlKinematicCharacterController::perpindicularComponent (const btlVector3& direction, const btlVector3& normal)
{
	return direction - parallelComponent(direction, normal);
}

btlKinematicCharacterController::btlKinematicCharacterController (btlPairCachingGhostObject* ghostObject,btlConvexShape* convexShape,btlScalar stepHeight, int upAxis)
{
	m_upAxis = upAxis;
	m_addedMargin = 0.02;
	m_walkDirection.setValue(0,0,0);
	m_useGhostObjectSweepTest = true;
	m_ghostObject = ghostObject;
	m_stepHeight = stepHeight;
	m_turnAngle = btlScalar(0.0);
	m_convexShape=convexShape;	
	m_useWalkDirection = true;	// use walk direction by default, legacy behavior
	m_velocityTimeInterval = 0.0;
	m_verticalVelocity = 0.0;
	m_verticalOffset = 0.0;
	m_gravity = 9.8 * 3 ; // 3G acceleration.
	m_fallSpeed = 55.0; // Terminal velocity of a sky diver in m/s.
	m_jumpSpeed = 10.0; // ?
	m_wasOnGround = false;
	m_wasJumping = false;
	m_interpolateUp = true;
	setMaxSlope(btlRadians(45.0));
	m_currentStepOffset = 0;
	full_drop = false;
	bounce_fix = false;
}

btlKinematicCharacterController::~btlKinematicCharacterController ()
{
}

btlPairCachingGhostObject* btlKinematicCharacterController::getGhostObject()
{
	return m_ghostObject;
}

bool btlKinematicCharacterController::recoverFromPenetration ( btlCollisionWorld* collisionWorld)
{
	// Here we must refresh the overlapping paircache as the penetrating movement itself or the
	// previous recovery iteration might have used setWorldTransform and pushed us into an object
	// that is not in the previous cache contents from the last timestep, as will happen if we
	// are pushed into a new AABB overlap. Unhandled this means the next convex sweep gets stuck.
	//
	// Do this by calling the broadphase's setAabb with the moved AABB, this will update the broadphase
	// paircache and the ghostobject's internal paircache at the same time.    /BW

	btlVector3 minAabb, maxAabb;
	m_convexShape->getAabb(m_ghostObject->getWorldTransform(), minAabb,maxAabb);
	collisionWorld->getBroadphase()->setAabb(m_ghostObject->getBroadphaseHandle(), 
						 minAabb, 
						 maxAabb, 
						 collisionWorld->getDispatcher());
						 
	bool penetration = false;

	collisionWorld->getDispatcher()->dispatchAllCollisionPairs(m_ghostObject->getOverlappingPairCache(), collisionWorld->getDispatchInfo(), collisionWorld->getDispatcher());

	m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();
	
	btlScalar maxPen = btlScalar(0.0);
	for (int i = 0; i < m_ghostObject->getOverlappingPairCache()->getNumOverlappingPairs(); i++)
	{
		m_manifoldArray.resize(0);

		btlBroadphasePair* collisionPair = &m_ghostObject->getOverlappingPairCache()->getOverlappingPairArray()[i];

		btlCollisionObject* obj0 = static_cast<btlCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
                btlCollisionObject* obj1 = static_cast<btlCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

		if ((obj0 && !obj0->hasContactResponse()) || (obj1 && !obj1->hasContactResponse()))
			continue;
		
		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(m_manifoldArray);

		
		for (int j=0;j<m_manifoldArray.size();j++)
		{
			btlPersistentManifold* manifold = m_manifoldArray[j];
			btlScalar directionSign = manifold->getBody0() == m_ghostObject ? btlScalar(-1.0) : btlScalar(1.0);
			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btlManifoldPoint&pt = manifold->getContactPoint(p);

				btlScalar dist = pt.getDistance();

				if (dist < 0.0)
				{
					if (dist < maxPen)
					{
						maxPen = dist;
						m_touchingNormal = pt.m_normalWorldOnB * directionSign;//??

					}
					m_currentPosition += pt.m_normalWorldOnB * directionSign * dist * btlScalar(0.2);
					penetration = true;
				} else {
					//printf("touching %f\n", dist);
				}
			}
			
			//manifold->clearManifold();
		}
	}
	btlTransform newTrans = m_ghostObject->getWorldTransform();
	newTrans.setOrigin(m_currentPosition);
	m_ghostObject->setWorldTransform(newTrans);
//	printf("m_touchingNormal = %f,%f,%f\n",m_touchingNormal[0],m_touchingNormal[1],m_touchingNormal[2]);
	return penetration;
}

void btlKinematicCharacterController::stepUp ( btlCollisionWorld* world)
{
	// phase 1: up
	btlTransform start, end;
	m_targetPosition = m_currentPosition + getUpAxisDirections()[m_upAxis] * (m_stepHeight + (m_verticalOffset > 0.f?m_verticalOffset:0.f));

	start.setIdentity ();
	end.setIdentity ();

	/* FIXME: Handle penetration properly */
	start.setOrigin (m_currentPosition + getUpAxisDirections()[m_upAxis] * (m_convexShape->getMargin() + m_addedMargin));
	end.setOrigin (m_targetPosition);

	btlKinematicClosestNotMeConvexResultCallback callback (m_ghostObject, -getUpAxisDirections()[m_upAxis], btlScalar(0.7071));
	callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
	callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;
	
	if (m_useGhostObjectSweepTest)
	{
		m_ghostObject->convexSweepTest (m_convexShape, start, end, callback, world->getDispatchInfo().m_allowedCcdPenetration);
	}
	else
	{
		world->convexSweepTest (m_convexShape, start, end, callback);
	}
	
	if (callback.hasHit())
	{
		// Only modify the position if the hit was a slope and not a wall or ceiling.
		if(callback.m_hitNormalWorld.dot(getUpAxisDirections()[m_upAxis]) > 0.0)
		{
			// we moved up only a fraction of the step height
			m_currentStepOffset = m_stepHeight * callback.m_closestHitFraction;
			if (m_interpolateUp == true)
				m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
			else
				m_currentPosition = m_targetPosition;
		}
		m_verticalVelocity = 0.0;
		m_verticalOffset = 0.0;
	} else {
		m_currentStepOffset = m_stepHeight;
		m_currentPosition = m_targetPosition;
	}
}

void btlKinematicCharacterController::updateTargetPositionBasedOnCollision (const btlVector3& hitNormal, btlScalar tangentMag, btlScalar normalMag)
{
	btlVector3 movementDirection = m_targetPosition - m_currentPosition;
	btlScalar movementLength = movementDirection.length();
	if (movementLength>SIMD_EPSILON)
	{
		movementDirection.normalize();

		btlVector3 reflectDir = computeReflectionDirection (movementDirection, hitNormal);
		reflectDir.normalize();

		btlVector3 parallelDir, perpindicularDir;

		parallelDir = parallelComponent (reflectDir, hitNormal);
		perpindicularDir = perpindicularComponent (reflectDir, hitNormal);

		m_targetPosition = m_currentPosition;
		if (0)//tangentMag != 0.0)
		{
			btlVector3 parComponent = parallelDir * btlScalar (tangentMag*movementLength);
//			printf("parComponent=%f,%f,%f\n",parComponent[0],parComponent[1],parComponent[2]);
			m_targetPosition +=  parComponent;
		}

		if (normalMag != 0.0)
		{
			btlVector3 perpComponent = perpindicularDir * btlScalar (normalMag*movementLength);
//			printf("perpComponent=%f,%f,%f\n",perpComponent[0],perpComponent[1],perpComponent[2]);
			m_targetPosition += perpComponent;
		}
	} else
	{
//		printf("movementLength don't normalize a zero vector\n");
	}
}

void btlKinematicCharacterController::stepForwardAndStrafe ( btlCollisionWorld* collisionWorld, const btlVector3& walkMove)
{
	// printf("m_normalizedDirection=%f,%f,%f\n",
	// 	m_normalizedDirection[0],m_normalizedDirection[1],m_normalizedDirection[2]);
	// phase 2: forward and strafe
	btlTransform start, end;
	m_targetPosition = m_currentPosition + walkMove;

	start.setIdentity ();
	end.setIdentity ();
	
	btlScalar fraction = 1.0;
	btlScalar distance2 = (m_currentPosition-m_targetPosition).length2();
//	printf("distance2=%f\n",distance2);

	if (m_touchingContact)
	{
		if (m_normalizedDirection.dot(m_touchingNormal) > btlScalar(0.0))
		{
			//interferes with step movement
			//updateTargetPositionBasedOnCollision (m_touchingNormal);
		}
	}

	int maxIter = 10;

	while (fraction > btlScalar(0.01) && maxIter-- > 0)
	{
		start.setOrigin (m_currentPosition);
		end.setOrigin (m_targetPosition);
		btlVector3 sweepDirNegative(m_currentPosition - m_targetPosition);

		btlKinematicClosestNotMeConvexResultCallback callback (m_ghostObject, sweepDirNegative, btlScalar(0.0));
		callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
		callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;


		btlScalar margin = m_convexShape->getMargin();
		m_convexShape->setMargin(margin + m_addedMargin);


		if (m_useGhostObjectSweepTest)
		{
			m_ghostObject->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
		} else
		{
			collisionWorld->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
		}
		
		m_convexShape->setMargin(margin);

		
		fraction -= callback.m_closestHitFraction;

		if (callback.hasHit())
		{	
			// we moved only a fraction
			//btScalar hitDistance;
			//hitDistance = (callback.m_hitPointWorld - m_currentPosition).length();

//			m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);

			updateTargetPositionBasedOnCollision (callback.m_hitNormalWorld);
			btlVector3 currentDir = m_targetPosition - m_currentPosition;
			distance2 = currentDir.length2();
			if (distance2 > SIMD_EPSILON)
			{
				currentDir.normalize();
				/* See Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners." */
				if (currentDir.dot(m_normalizedDirection) <= btlScalar(0.0))
				{
					break;
				}
			} else
			{
//				printf("currentDir: don't normalize a zero vector\n");
				break;
			}

		} else {
			// we moved whole way
			m_currentPosition = m_targetPosition;
		}

	//	if (callback.m_closestHitFraction == 0.f)
	//		break;

	}
}

void btlKinematicCharacterController::stepDown ( btlCollisionWorld* collisionWorld, btlScalar dt)
{
	btlTransform start, end, end_double;
	bool runonce = false;

	// phase 3: down
	/*btScalar additionalDownStep = (m_wasOnGround && !onGround()) ? m_stepHeight : 0.0;
	btVector3 step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + additionalDownStep);
	btScalar downVelocity = (additionalDownStep == 0.0 && m_verticalVelocity<0.0?-m_verticalVelocity:0.0) * dt;
	btVector3 gravity_drop = getUpAxisDirections()[m_upAxis] * downVelocity; 
	m_targetPosition -= (step_drop + gravity_drop);*/

	btlVector3 orig_position = m_targetPosition;
	
	btlScalar downVelocity = (m_verticalVelocity<0.f?-m_verticalVelocity:0.f) * dt;

	if(downVelocity > 0.0 && downVelocity > m_fallSpeed
		&& (m_wasOnGround || !m_wasJumping))
		downVelocity = m_fallSpeed;

	btlVector3 step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + downVelocity);
	m_targetPosition -= step_drop;

	btlKinematicClosestNotMeConvexResultCallback callback (m_ghostObject, getUpAxisDirections()[m_upAxis], m_maxSlopeCosine);
        callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
        callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;

        btlKinematicClosestNotMeConvexResultCallback callback2 (m_ghostObject, getUpAxisDirections()[m_upAxis], m_maxSlopeCosine);
        callback2.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
        callback2.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;

	while (1)
	{
		start.setIdentity ();
		end.setIdentity ();

		end_double.setIdentity ();

		start.setOrigin (m_currentPosition);
		end.setOrigin (m_targetPosition);

		//set double test for 2x the step drop, to check for a large drop vs small drop
		end_double.setOrigin (m_targetPosition - step_drop);

		if (m_useGhostObjectSweepTest)
		{
			m_ghostObject->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);

			if (!callback.hasHit())
			{
				//test a double fall height, to see if the character should interpolate it's fall (full) or not (partial)
				m_ghostObject->convexSweepTest (m_convexShape, start, end_double, callback2, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
			}
		} else
		{
			collisionWorld->convexSweepTest (m_convexShape, start, end, callback, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);

			if (!callback.hasHit())
					{
							//test a double fall height, to see if the character should interpolate it's fall (large) or not (small)
							collisionWorld->convexSweepTest (m_convexShape, start, end_double, callback2, collisionWorld->getDispatchInfo().m_allowedCcdPenetration);
					}
		}
	
		btlScalar downVelocity2 = (m_verticalVelocity<0.f?-m_verticalVelocity:0.f) * dt;
		bool has_hit = false;
		if (bounce_fix == true)
			has_hit = callback.hasHit() || callback2.hasHit();
		else
			has_hit = callback2.hasHit();

		if(downVelocity2 > 0.0 && downVelocity2 < m_stepHeight && has_hit == true && runonce == false
					&& (m_wasOnGround || !m_wasJumping))
		{
			//redo the velocity calculation when falling a small amount, for fast stairs motion
			//for larger falls, use the smoother/slower interpolated movement by not touching the target position

			m_targetPosition = orig_position;
					downVelocity = m_stepHeight;

				btlVector3 step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + downVelocity);
			m_targetPosition -= step_drop;
			runonce = true;
			continue; //re-run previous tests
		}
		break;
	}

	if (callback.hasHit() || runonce == true)
	{
		// we dropped a fraction of the height -> hit floor

		btlScalar fraction = (m_currentPosition.getY() - callback.m_hitPointWorld.getY()) / 2;

		//printf("hitpoint: %g - pos %g\n", callback.m_hitPointWorld.getY(), m_currentPosition.getY());

		if (bounce_fix == true)
		{
			if (full_drop == true)
                                m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);
                        else
                                //due to errors in the closestHitFraction variable when used with large polygons, calculate the hit fraction manually
                                m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, fraction);
		}
		else
			m_currentPosition.setInterpolate3 (m_currentPosition, m_targetPosition, callback.m_closestHitFraction);

		full_drop = false;

		m_verticalVelocity = 0.0;
		m_verticalOffset = 0.0;
		m_wasJumping = false;
	} else {
		// we dropped the full height
		
		full_drop = true;

		if (bounce_fix == true)
		{
			downVelocity = (m_verticalVelocity<0.f?-m_verticalVelocity:0.f) * dt;
			if (downVelocity > m_fallSpeed && (m_wasOnGround || !m_wasJumping))
			{
				m_targetPosition += step_drop; //undo previous target change
				downVelocity = m_fallSpeed;
				step_drop = getUpAxisDirections()[m_upAxis] * (m_currentStepOffset + downVelocity);
				m_targetPosition -= step_drop;
			}
		}
		//printf("full drop - %g, %g\n", m_currentPosition.getY(), m_targetPosition.getY());

		m_currentPosition = m_targetPosition;
	}
}



void btlKinematicCharacterController::setWalkDirection
(
const btlVector3& walkDirection
)
{
	m_useWalkDirection = true;
	m_walkDirection = walkDirection;
	m_normalizedDirection = getNormalizedVector(m_walkDirection);
}



void btlKinematicCharacterController::setVelocityForTimeInterval
(
const btlVector3& velocity,
btlScalar timeInterval
)
{
//	printf("setVelocity!\n");
//	printf("  interval: %f\n", timeInterval);
//	printf("  velocity: (%f, %f, %f)\n",
//		 velocity.x(), velocity.y(), velocity.z());

	m_useWalkDirection = false;
	m_walkDirection = velocity;
	m_normalizedDirection = getNormalizedVector(m_walkDirection);
	m_velocityTimeInterval += timeInterval;
}

void btlKinematicCharacterController::reset ( btlCollisionWorld* collisionWorld )
{
        m_verticalVelocity = 0.0;
        m_verticalOffset = 0.0;
        m_wasOnGround = false;
        m_wasJumping = false;
        m_walkDirection.setValue(0,0,0);
        m_velocityTimeInterval = 0.0;

        //clear pair cache
        btlHashedOverlappingPairCache *cache = m_ghostObject->getOverlappingPairCache();
        while (cache->getOverlappingPairArray().size() > 0)
        {
                cache->removeOverlappingPair(cache->getOverlappingPairArray()[0].m_pProxy0, cache->getOverlappingPairArray()[0].m_pProxy1, collisionWorld->getDispatcher());
        }
}

void btlKinematicCharacterController::warp (const btlVector3& origin)
{
	btlTransform xform;
	xform.setIdentity();
	xform.setOrigin (origin);
	m_ghostObject->setWorldTransform (xform);
}


void btlKinematicCharacterController::preStep (  btlCollisionWorld* collisionWorld)
{
	
	int numPenetrationLoops = 0;
	m_touchingContact = false;
	while (recoverFromPenetration (collisionWorld))
	{
		numPenetrationLoops++;
		m_touchingContact = true;
		if (numPenetrationLoops > 4)
		{
			//printf("character could not recover from penetration = %d\n", numPenetrationLoops);
			break;
		}
	}

	m_currentPosition = m_ghostObject->getWorldTransform().getOrigin();
	m_targetPosition = m_currentPosition;
//	printf("m_targetPosition=%f,%f,%f\n",m_targetPosition[0],m_targetPosition[1],m_targetPosition[2]);

	
}

#include <stdio.h>

void btlKinematicCharacterController::playerStep (  btlCollisionWorld* collisionWorld, btlScalar dt)
{
//	printf("playerStep(): ");
//	printf("  dt = %f", dt);

	// quick check...
	if (!m_useWalkDirection && (m_velocityTimeInterval <= 0.0 || m_walkDirection.fuzzyZero())) {
//		printf("\n");
		return;		// no motion
	}

	m_wasOnGround = onGround();

	// Update fall velocity.
	m_verticalVelocity -= m_gravity * dt;
	if(m_verticalVelocity > 0.0 && m_verticalVelocity > m_jumpSpeed)
	{
		m_verticalVelocity = m_jumpSpeed;
	}
	if(m_verticalVelocity < 0.0 && btlFabs(m_verticalVelocity) > btlFabs(m_fallSpeed))
	{
		m_verticalVelocity = -btlFabs(m_fallSpeed);
	}
	m_verticalOffset = m_verticalVelocity * dt;


	btlTransform xform;
	xform = m_ghostObject->getWorldTransform ();

//	printf("walkDirection(%f,%f,%f)\n",walkDirection[0],walkDirection[1],walkDirection[2]);
//	printf("walkSpeed=%f\n",walkSpeed);

	stepUp (collisionWorld);
	if (m_useWalkDirection) {
		stepForwardAndStrafe (collisionWorld, m_walkDirection);
	} else {
		//printf("  time: %f", m_velocityTimeInterval);
		// still have some time left for moving!
		btlScalar dtMoving =
			(dt < m_velocityTimeInterval) ? dt : m_velocityTimeInterval;
		m_velocityTimeInterval -= dt;

		// how far will we move while we are moving?
		btlVector3 move = m_walkDirection * dtMoving;

		//printf("  dtMoving: %f", dtMoving);

		// okay, step
		stepForwardAndStrafe(collisionWorld, move);
	}
	stepDown (collisionWorld, dt);

	// printf("\n");

	xform.setOrigin (m_currentPosition);
	m_ghostObject->setWorldTransform (xform);
}

void btlKinematicCharacterController::setFallSpeed (btlScalar fallSpeed)
{
	m_fallSpeed = fallSpeed;
}

void btlKinematicCharacterController::setJumpSpeed (btlScalar jumpSpeed)
{
	m_jumpSpeed = jumpSpeed;
}

void btlKinematicCharacterController::setMaxJumpHeight (btlScalar maxJumpHeight)
{
	m_maxJumpHeight = maxJumpHeight;
}

bool btlKinematicCharacterController::canJump () const
{
	return onGround();
}

void btlKinematicCharacterController::jump ()
{
	if (!canJump())
		return;

	m_verticalVelocity = m_jumpSpeed;
	m_wasJumping = true;

#if 0
	currently no jumping.
	btlTransform xform;
	m_rigidBody->getMotionState()->getWorldTransform (xform);
	btlVector3 up = xform.getBasis()[1];
	up.normalize ();
	btlScalar magnitude = (btlScalar(1.0)/m_rigidBody->getInvMass()) * btlScalar(8.0);
	m_rigidBody->applyCentralImpulse (up * magnitude);
#endif
}

void btlKinematicCharacterController::setGravity(btlScalar gravity)
{
	m_gravity = gravity;
}

btlScalar btlKinematicCharacterController::getGravity() const
{
	return m_gravity;
}

void btlKinematicCharacterController::setMaxSlope(btlScalar slopeRadians)
{
	m_maxSlopeRadians = slopeRadians;
	m_maxSlopeCosine = btlCos(slopeRadians);
}

btlScalar btlKinematicCharacterController::getMaxSlope() const
{
	return m_maxSlopeRadians;
}

bool btlKinematicCharacterController::onGround () const
{
	return m_verticalVelocity == 0.0 && m_verticalOffset == 0.0;
}


btlVector3* btlKinematicCharacterController::getUpAxisDirections()
{
	static btlVector3 sUpAxisDirection[3] = { btlVector3(1.0f, 0.0f, 0.0f), btlVector3(0.0f, 1.0f, 0.0f), btlVector3(0.0f, 0.0f, 1.0f) };
	
	return sUpAxisDirection;
}

void btlKinematicCharacterController::debugDraw(btlIDebugDraw* debugDrawer)
{
}

void btlKinematicCharacterController::setUpInterpolate(bool value)
{
	m_interpolateUp = value;
}
