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

#ifndef BT_SOFT_BODY_SOLVERS_H
#define BT_SOFT_BODY_SOLVERS_H

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"


class btlSoftBodyTriangleData;
class btlSoftBodyLinkData;
class btlSoftBodyVertexData;
class btlVertexBufferDescriptor;
class btlCollisionObject;
class btlSoftBody;


class btlSoftBodySolver
{
public:
	enum SolverTypes
	{
		DEFAULT_SOLVER,
		CPU_SOLVER,
		CL_SOLVER,
		CL_SIMD_SOLVER,
		DX_SOLVER,
		DX_SIMD_SOLVER
	};


protected:
	int m_numberOfPositionIterations;
	int m_numberOfVelocityIterations;
	// Simulation timescale
	float m_timeScale;
	
public:
	btlSoftBodySolver() :
		m_numberOfPositionIterations( 10 ),
		m_timeScale( 1 )
	{
		m_numberOfVelocityIterations = 0;
		m_numberOfPositionIterations = 5;
	}

	virtual ~btlSoftBodySolver()
	{
	}
	
	/**
	 * Return the type of the solver.
	 */
	virtual SolverTypes getSolverType() const = 0;


	/** Ensure that this solver is initialized. */
	virtual bool checkInitialized() = 0;

	/** Optimize soft bodies in this solver. */
	virtual void optimize( btlAlignedObjectArray< btlSoftBody * > &softBodies , bool forceUpdate=false) = 0;

	/** Copy necessary data back to the original soft body source objects. */
	virtual void copyBackToSoftBodies(bool bMove = true) = 0;

	/** Predict motion of soft bodies into next timestep */
	virtual void predictMotion( float solverdt ) = 0;

	/** Solve constraints for a set of soft bodies */
	virtual void solveConstraints( float solverdt ) = 0;

	/** Perform necessary per-step updates of soft bodies such as recomputing normals and bounding boxes */
	virtual void updateSoftBodies() = 0;

	/** Process a collision between one of the world's soft bodies and another collision object */
	virtual void processCollision( btlSoftBody *, const struct btlCollisionObjectWrapper* ) = 0;

	/** Process a collision between two soft bodies */
	virtual void processCollision( btlSoftBody*, btlSoftBody* ) = 0;

	/** Set the number of velocity constraint solver iterations this solver uses. */
	virtual void setNumberOfPositionIterations( int iterations )
	{
		m_numberOfPositionIterations = iterations;
	}

	/** Get the number of velocity constraint solver iterations this solver uses. */
	virtual int getNumberOfPositionIterations()
	{
		return m_numberOfPositionIterations;
	}

	/** Set the number of velocity constraint solver iterations this solver uses. */
	virtual void setNumberOfVelocityIterations( int iterations )
	{
		m_numberOfVelocityIterations = iterations;
	}

	/** Get the number of velocity constraint solver iterations this solver uses. */
	virtual int getNumberOfVelocityIterations()
	{
		return m_numberOfVelocityIterations;
	}

	/** Return the timescale that the simulation is using */
	float getTimeScale()
	{
		return m_timeScale;
	}

#if 0
	/**
	 * Add a collision object to be used by the indicated softbody.
	 */
	virtual void addCollisionObjectForSoftBody( int clothIdentifier, btlCollisionObject *collisionObject ) = 0;
#endif
};

/** 
 * Class to manage movement of data from a solver to a given target.
 * This version is abstract. Subclasses will have custom pairings for different combinations.
 */
class btlSoftBodySolverOutput
{
protected:

public:
	btlSoftBodySolverOutput()
	{
	}

	virtual ~btlSoftBodySolverOutput()
	{
	}


	/** Output current computed vertex data to the vertex buffers for all cloths in the solver. */
	virtual void copySoftBodyToVertexBuffer( const btlSoftBody * const softBody, btlVertexBufferDescriptor *vertexBuffer ) = 0;
};


#endif // #ifndef BT_SOFT_BODY_SOLVERS_H
