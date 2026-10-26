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


#ifndef BT_KINEMATIC_CHARACTER_CONTROLLER_H
#define BT_KINEMATIC_CHARACTER_CONTROLLER_H

#include "LinearMath/btVector3.h"

#include "btCharacterControllerInterface.h"

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"


class btlCollisionShape;
class btlConvexShape;
class btlRigidBody;
class btlCollisionWorld;
class btlCollisionDispatcher;
class btlPairCachingGhostObject;

///btKinematicCharacterController is an object that supports a sliding motion in a world.
///It uses a ghost object and convex sweep test to test for upcoming collisions. This is combined with discrete collision detection to recover from penetrations.
///Interaction between btKinematicCharacterController and dynamic rigid bodies needs to be explicity implemented by the user.
ATTRIBUTE_ALIGNED16(class) btlKinematicCharacterController : public btlCharacterControllerInterface
{
protected:

	btlScalar m_halfHeight;
	
	btlPairCachingGhostObject* m_ghostObject;
	btlConvexShape*	m_convexShape;//is also in m_ghostObject, but it needs to be convex, so we store it here to avoid upcast
	
	btlScalar m_verticalVelocity;
	btlScalar m_verticalOffset;
	btlScalar m_fallSpeed;
	btlScalar m_jumpSpeed;
	btlScalar m_maxJumpHeight;
	btlScalar m_maxSlopeRadians; // Slope angle that is set (used for returning the exact value)
	btlScalar m_maxSlopeCosine;  // Cosine equivalent of m_maxSlopeRadians (calculated once when set, for optimization)
	btlScalar m_gravity;

	btlScalar m_turnAngle;
	
	btlScalar m_stepHeight;

	btlScalar	m_addedMargin;//@todo: remove this and fix the code

	///this is the desired walk direction, set by the user
	btlVector3	m_walkDirection;
	btlVector3	m_normalizedDirection;

	//some internal variables
	btlVector3 m_currentPosition;
	btlScalar  m_currentStepOffset;
	btlVector3 m_targetPosition;

	///keep track of the contact manifolds
	btlManifoldArray	m_manifoldArray;

	bool m_touchingContact;
	btlVector3 m_touchingNormal;

	bool  m_wasOnGround;
	bool  m_wasJumping;
	bool	m_useGhostObjectSweepTest;
	bool	m_useWalkDirection;
	btlScalar	m_velocityTimeInterval;
	int m_upAxis;

	static btlVector3* getUpAxisDirections();
	bool  m_interpolateUp;
	bool  full_drop;
	bool  bounce_fix;

	btlVector3 computeReflectionDirection (const btlVector3& direction, const btlVector3& normal);
	btlVector3 parallelComponent (const btlVector3& direction, const btlVector3& normal);
	btlVector3 perpindicularComponent (const btlVector3& direction, const btlVector3& normal);

	bool recoverFromPenetration ( btlCollisionWorld* collisionWorld);
	void stepUp (btlCollisionWorld* collisionWorld);
	void updateTargetPositionBasedOnCollision (const btlVector3& hit_normal, btlScalar tangentMag = btlScalar(0.0), btlScalar normalMag = btlScalar(1.0));
	void stepForwardAndStrafe (btlCollisionWorld* collisionWorld, const btlVector3& walkMove);
	void stepDown (btlCollisionWorld* collisionWorld, btlScalar dt);
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlKinematicCharacterController (btlPairCachingGhostObject* ghostObject,btlConvexShape* convexShape,btlScalar stepHeight, int upAxis = 1);
	~btlKinematicCharacterController ();
	

	///btActionInterface interface
	virtual void updateAction( btlCollisionWorld* collisionWorld,btlScalar deltaTime)
	{
		preStep ( collisionWorld);
		playerStep (collisionWorld, deltaTime);
	}
	
	///btActionInterface interface
	void	debugDraw(btlIDebugDraw* debugDrawer);

	void setUpAxis (int axis)
	{
		if (axis < 0)
			axis = 0;
		if (axis > 2)
			axis = 2;
		m_upAxis = axis;
	}

	/// This should probably be called setPositionIncrementPerSimulatorStep.
	/// This is neither a direction nor a velocity, but the amount to
	///	increment the position each simulation iteration, regardless
	///	of dt.
	/// This call will reset any velocity set by setVelocityForTimeInterval().
	virtual void	setWalkDirection(const btlVector3& walkDirection);

	/// Caller provides a velocity with which the character should move for
	///	the given time period.  After the time period, velocity is reset
	///	to zero.
	/// This call will reset any walk direction set by setWalkDirection().
	/// Negative time intervals will result in no motion.
	virtual void setVelocityForTimeInterval(const btlVector3& velocity,
				btlScalar timeInterval);

	void reset ( btlCollisionWorld* collisionWorld );
	void warp (const btlVector3& origin);

	void preStep (  btlCollisionWorld* collisionWorld);
	void playerStep ( btlCollisionWorld* collisionWorld, btlScalar dt);

	void setFallSpeed (btlScalar fallSpeed);
	void setJumpSpeed (btlScalar jumpSpeed);
	void setMaxJumpHeight (btlScalar maxJumpHeight);
	bool canJump () const;

	void jump ();

	void setGravity(btlScalar gravity);
	btlScalar getGravity() const;

	/// The max slope determines the maximum angle that the controller can walk up.
	/// The slope angle is measured in radians.
	void setMaxSlope(btlScalar slopeRadians);
	btlScalar getMaxSlope() const;

	btlPairCachingGhostObject* getGhostObject();
	void	setUseGhostSweepTest(bool useGhostObjectSweepTest)
	{
		m_useGhostObjectSweepTest = useGhostObjectSweepTest;
	}

	bool onGround () const;
	void setUpInterpolate (bool value);
};

#endif // BT_KINEMATIC_CHARACTER_CONTROLLER_H
