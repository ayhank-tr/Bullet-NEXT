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

#ifndef BT_CHARACTER_CONTROLLER_INTERFACE_H
#define BT_CHARACTER_CONTROLLER_INTERFACE_H

#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class btlCollisionShape;
class btlRigidBody;
class btlCollisionWorld;

class btlCharacterControllerInterface : public btlActionInterface
{
public:
	btlCharacterControllerInterface () {};
	virtual ~btlCharacterControllerInterface () {};
	
	virtual void	setWalkDirection(const btlVector3& walkDirection) = 0;
	virtual void	setVelocityForTimeInterval(const btlVector3& velocity, btlScalar timeInterval) = 0;
	virtual void	reset ( btlCollisionWorld* collisionWorld ) = 0;
	virtual void	warp (const btlVector3& origin) = 0;

	virtual void	preStep ( btlCollisionWorld* collisionWorld) = 0;
	virtual void	playerStep (btlCollisionWorld* collisionWorld, btlScalar dt) = 0;
	virtual bool	canJump () const = 0;
	virtual void	jump () = 0;

	virtual bool	onGround () const = 0;
	virtual void	setUpInterpolate (bool value) = 0;
};

#endif //BT_CHARACTER_CONTROLLER_INTERFACE_H

