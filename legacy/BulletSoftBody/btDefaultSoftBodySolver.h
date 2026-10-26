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

#ifndef BT_SOFT_BODY_DEFAULT_SOLVER_H
#define BT_SOFT_BODY_DEFAULT_SOLVER_H


#include "BulletSoftBody/btSoftBodySolvers.h"
#include "btSoftBodySolverVertexBuffer.h"
struct btlCollisionObjectWrapper;

class btlDefaultSoftBodySolver : public btlSoftBodySolver
{
protected:		
	/** Variable to define whether we need to update solver constants on the next iteration */
	bool m_updateSolverConstants;

	btlAlignedObjectArray< btlSoftBody * > m_softBodySet;


public:
	btlDefaultSoftBodySolver();
	
	virtual ~btlDefaultSoftBodySolver();
	
	virtual SolverTypes getSolverType() const
	{
		return DEFAULT_SOLVER;
	}

	virtual bool checkInitialized();

	virtual void updateSoftBodies( );

	virtual void optimize( btlAlignedObjectArray< btlSoftBody * > &softBodies,bool forceUpdate=false );

	virtual void copyBackToSoftBodies(bool bMove = true);

	virtual void solveConstraints( float solverdt );

	virtual void predictMotion( float solverdt );

	virtual void copySoftBodyToVertexBuffer( const btlSoftBody *const softBody, btlVertexBufferDescriptor *vertexBuffer );

	virtual void processCollision( btlSoftBody *, const btlCollisionObjectWrapper* );

	virtual void processCollision( btlSoftBody*, btlSoftBody* );

};

#endif // #ifndef BT_ACCELERATED_SOFT_BODY_CPU_SOLVER_H
