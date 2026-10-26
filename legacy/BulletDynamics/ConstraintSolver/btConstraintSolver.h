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

#ifndef BT_CONSTRAINT_SOLVER_H
#define BT_CONSTRAINT_SOLVER_H

#include "LinearMath/btScalar.h"

class btlPersistentManifold;
class btlRigidBody;
class btlCollisionObject;
class btlTypedConstraint;
struct btlContactSolverInfo;
struct btlBroadphaseProxy;
class btlIDebugDraw;
class btlStackAlloc;
class	btlDispatcher;
/// btConstraintSolver provides solver interface


enum btlConstraintSolverType
{
	BT_SEQUENTIAL_IMPULSE_SOLVER=1,
	BT_MLCP_SOLVER=2,
	BT_NNCG_SOLVER=4
};

class btlConstraintSolver
{

public:

	virtual ~btlConstraintSolver() {}
	
	virtual void prepareSolve (int /* numBodies */, int /* numManifolds */) {;}

	///solve a group of constraints
	virtual btlScalar solveGroup(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifold,int numManifolds,btlTypedConstraint** constraints,int numConstraints, const btlContactSolverInfo& info,class btlIDebugDraw* debugDrawer,btlDispatcher* dispatcher) = 0;

	virtual void allSolved (const btlContactSolverInfo& /* info */,class btlIDebugDraw* /* debugDrawer */) {;}

	///clear internal cached data and reset random seed
	virtual	void	reset() = 0;

	virtual btlConstraintSolverType	getSolverType() const=0;


};




#endif //BT_CONSTRAINT_SOLVER_H
