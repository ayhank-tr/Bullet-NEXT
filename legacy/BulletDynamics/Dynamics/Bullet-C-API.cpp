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

/*
	Draft high-level generic physics C-API. For low-level access, use the physics SDK native API's.
	Work in progress, functionality will be added on demand.

	If possible, use the richer Bullet C++ API, by including <src/btBulletDynamicsCommon.h>
*/

#include "Bullet-C-Api.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btConvexHullComputer.h"


#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"	
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/Gimpact/btTriangleShapeEx.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"


/*
	Create and Delete a Physics SDK	
*/

struct	btlPhysicsSdk
{

//	btDispatcher*				m_dispatcher;
//	btOverlappingPairCache*		m_pairCache;
//	btConstraintSolver*			m_constraintSolver

	btlVector3	m_worldAabbMin;
	btlVector3	m_worldAabbMax;


	//todo: version, hardware/optimization settings etc?
	btlPhysicsSdk()
		:m_worldAabbMin(-1000,-1000,-1000),
		m_worldAabbMax(1000,1000,1000)
	{

	}

	
};

plPhysicsSdkHandle	btl_plNewBulletSdk()
{
	void* mem = btlAlignedAlloc(sizeof(btlPhysicsSdk),16);
	return (plPhysicsSdkHandle)new (mem)btlPhysicsSdk;
}

void		btl_plDeletePhysicsSdk(plPhysicsSdkHandle	physicsSdk)
{
	btlPhysicsSdk* phys = reinterpret_cast<btlPhysicsSdk*>(physicsSdk);
	btlAlignedFree(phys);	
}


/* Dynamics World */
plDynamicsWorldHandle btl_plCreateDynamicsWorld(plPhysicsSdkHandle physicsSdkHandle)
{
	btlPhysicsSdk* physicsSdk = reinterpret_cast<btlPhysicsSdk*>(physicsSdkHandle);
	void* mem = btlAlignedAlloc(sizeof(btlDefaultCollisionConfiguration),16);
	btlDefaultCollisionConfiguration* collisionConfiguration = new (mem)btlDefaultCollisionConfiguration();
	mem = btlAlignedAlloc(sizeof(btlCollisionDispatcher),16);
	btlDispatcher*				dispatcher = new (mem)btlCollisionDispatcher(collisionConfiguration);
	mem = btlAlignedAlloc(sizeof(btlAxisSweep3),16);
	btlBroadphaseInterface*		pairCache = new (mem)btlAxisSweep3(physicsSdk->m_worldAabbMin,physicsSdk->m_worldAabbMax);
	mem = btlAlignedAlloc(sizeof(btlSequentialImpulseConstraintSolver),16);
	btlConstraintSolver*			constraintSolver = new(mem) btlSequentialImpulseConstraintSolver();

	mem = btlAlignedAlloc(sizeof(btlDiscreteDynamicsWorld),16);
	return (plDynamicsWorldHandle) new (mem)btlDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration);
}
void           btl_plDeleteDynamicsWorld(plDynamicsWorldHandle world)
{
	//todo: also clean up the other allocations, axisSweep, pairCache,dispatcher,constraintSolver,collisionConfiguration
	btlDynamicsWorld* dynamicsWorld = reinterpret_cast< btlDynamicsWorld* >(world);
	btlAlignedFree(dynamicsWorld);
}

void	btl_plStepSimulation(plDynamicsWorldHandle world,	plReal	timeStep)
{
	btlDynamicsWorld* dynamicsWorld = reinterpret_cast< btlDynamicsWorld* >(world);
	btlAssert(dynamicsWorld);
	dynamicsWorld->stepSimulation(timeStep);
}

void btl_plAddRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object)
{
	btlDynamicsWorld* dynamicsWorld = reinterpret_cast< btlDynamicsWorld* >(world);
	btlAssert(dynamicsWorld);
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);

	dynamicsWorld->addRigidBody(body);
}

void btl_plRemoveRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object)
{
	btlDynamicsWorld* dynamicsWorld = reinterpret_cast< btlDynamicsWorld* >(world);
	btlAssert(dynamicsWorld);
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);

	dynamicsWorld->removeRigidBody(body);
}

/* Rigid Body  */

plRigidBodyHandle btl_plCreateRigidBody(	void* user_data,  float mass, plCollisionShapeHandle cshape )
{
	btlTransform trans;
	trans.setIdentity();
	btlVector3 localInertia(0,0,0);
	btlCollisionShape* shape = reinterpret_cast<btlCollisionShape*>( cshape);
	btlAssert(shape);
	if (mass)
	{
		shape->calculateLocalInertia(mass,localInertia);
	}
	void* mem = btlAlignedAlloc(sizeof(btlRigidBody),16);
	btlRigidBody::btlRigidBodyConstructionInfo rbci(mass, 0,shape,localInertia);
	btlRigidBody* body = new (mem)btlRigidBody(rbci);
	body->setWorldTransform(trans);
	body->setUserPointer(user_data);
	return (plRigidBodyHandle) body;
}

void btl_plDeleteRigidBody(plRigidBodyHandle cbody)
{
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(cbody);
	btlAssert(body);
	btlAlignedFree( body);
}


/* Collision Shape definition */

plCollisionShapeHandle btl_plNewSphereShape(plReal radius)
{
	void* mem = btlAlignedAlloc(sizeof(btlSphereShape),16);
	return (plCollisionShapeHandle) new (mem)btlSphereShape(radius);
	
}
	
plCollisionShapeHandle btl_plNewBoxShape(plReal x, plReal y, plReal z)
{
	void* mem = btlAlignedAlloc(sizeof(btlBoxShape),16);
	return (plCollisionShapeHandle) new (mem)btlBoxShape(btlVector3(x,y,z));
}

plCollisionShapeHandle btl_plNewCapsuleShape(plReal radius, plReal height)
{
	//capsule is convex hull of 2 spheres, so use btMultiSphereShape
	
	const int numSpheres = 2;
	btlVector3 positions[numSpheres] = {btlVector3(0,height,0),btlVector3(0,-height,0)};
	btlScalar radi[numSpheres] = {radius,radius};
	void* mem = btlAlignedAlloc(sizeof(btlMultiSphereShape),16);
	return (plCollisionShapeHandle) new (mem)btlMultiSphereShape(positions,radi,numSpheres);
}
plCollisionShapeHandle btl_plNewConeShape(plReal radius, plReal height)
{
	void* mem = btlAlignedAlloc(sizeof(btlConeShape),16);
	return (plCollisionShapeHandle) new (mem)btlConeShape(radius,height);
}

plCollisionShapeHandle btl_plNewCylinderShape(plReal radius, plReal height)
{
	void* mem = btlAlignedAlloc(sizeof(btlCylinderShape),16);
	return (plCollisionShapeHandle) new (mem)btlCylinderShape(btlVector3(radius,height,radius));
}

/* Convex Meshes */
plCollisionShapeHandle btl_plNewConvexHullShape()
{
	void* mem = btlAlignedAlloc(sizeof(btlConvexHullShape),16);
	return (plCollisionShapeHandle) new (mem)btlConvexHullShape();
}


/* Concave static triangle meshes */
plMeshInterfaceHandle		   btl_plNewMeshInterface()
{
	return 0;
}

plCollisionShapeHandle btl_plNewCompoundShape()
{
	void* mem = btlAlignedAlloc(sizeof(btlCompoundShape),16);
	return (plCollisionShapeHandle) new (mem)btlCompoundShape();
}

void	btl_plAddChildShape(plCollisionShapeHandle compoundShapeHandle,plCollisionShapeHandle childShapeHandle, plVector3 childPos,plQuaternion childOrn)
{
	btlCollisionShape* colShape = reinterpret_cast<btlCollisionShape*>(compoundShapeHandle);
	btlAssert(colShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE);
	btlCompoundShape* compoundShape = reinterpret_cast<btlCompoundShape*>(colShape);
	btlCollisionShape* childShape = reinterpret_cast<btlCollisionShape*>(childShapeHandle);
	btlTransform	localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btlVector3(childPos[0],childPos[1],childPos[2]));
	localTrans.setRotation(btlQuaternion(childOrn[0],childOrn[1],childOrn[2],childOrn[3]));
	compoundShape->addChildShape(localTrans,childShape);
}

void btl_plSetEuler(plReal yaw,plReal pitch,plReal roll, plQuaternion orient)
{
	btlQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	orient[0] = orn.getX();
	orient[1] = orn.getY();
	orient[2] = orn.getZ();
	orient[3] = orn.getW();

}


//	extern  void		plAddTriangle(plMeshInterfaceHandle meshHandle, plVector3 v0,plVector3 v1,plVector3 v2);
//	extern  plCollisionShapeHandle plNewStaticTriangleMeshShape(plMeshInterfaceHandle);


void		btl_plAddVertex(plCollisionShapeHandle cshape, plReal x,plReal y,plReal z)
{
	btlCollisionShape* colShape = reinterpret_cast<btlCollisionShape*>( cshape);
	(void)colShape;
	btlAssert(colShape->getShapeType()==CONVEX_HULL_SHAPE_PROXYTYPE);
	btlConvexHullShape* convexHullShape = reinterpret_cast<btlConvexHullShape*>( cshape);
	convexHullShape->addPoint(btlVector3(x,y,z));

}

void btl_plDeleteShape(plCollisionShapeHandle cshape)
{
	btlCollisionShape* shape = reinterpret_cast<btlCollisionShape*>( cshape);
	btlAssert(shape);
	btlAlignedFree(shape);
}
void btl_plSetScaling(plCollisionShapeHandle cshape, plVector3 cscaling)
{
	btlCollisionShape* shape = reinterpret_cast<btlCollisionShape*>( cshape);
	btlAssert(shape);
	btlVector3 scaling(cscaling[0],cscaling[1],cscaling[2]);
	shape->setLocalScaling(scaling);	
}



void btl_plSetPosition(plRigidBodyHandle object, const plVector3 position)
{
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);
	btlVector3 pos(position[0],position[1],position[2]);
	btlTransform worldTrans = body->getWorldTransform();
	worldTrans.setOrigin(pos);
	body->setWorldTransform(worldTrans);
}

void btl_plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation)
{
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);
	btlQuaternion orn(orientation[0],orientation[1],orientation[2],orientation[3]);
	btlTransform worldTrans = body->getWorldTransform();
	worldTrans.setRotation(orn);
	body->setWorldTransform(worldTrans);
}

void	btl_plSetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix)
{
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);
	btlTransform& worldTrans = body->getWorldTransform();
	worldTrans.setFromOpenGLMatrix(matrix);
}

void	btl_plGetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix)
{
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);
	body->getWorldTransform().getOpenGLMatrix(matrix);

}

void	btl_plGetPosition(plRigidBodyHandle object,plVector3 position)
{
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);
	const btlVector3& pos = body->getWorldTransform().getOrigin();
	position[0] = pos.getX();
	position[1] = pos.getY();
	position[2] = pos.getZ();
}

void btl_plGetOrientation(plRigidBodyHandle object,plQuaternion orientation)
{
	btlRigidBody* body = reinterpret_cast< btlRigidBody* >(object);
	btlAssert(body);
	const btlQuaternion& orn = body->getWorldTransform().getRotation();
	orientation[0] = orn.getX();
	orientation[1] = orn.getY();
	orientation[2] = orn.getZ();
	orientation[3] = orn.getW();
}



//plRigidBodyHandle plRayCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);

//	extern  plRigidBodyHandle plObjectCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal);

double btl_plNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3])
{
	btlVector3 vp(p1[0], p1[1], p1[2]);
	btlTriangleShapeEx trishapeA(vp,
				  btlVector3(p2[0], p2[1], p2[2]), 
				  btlVector3(p3[0], p3[1], p3[2]));
	trishapeA.setMargin(0.000001f);
	btlVector3 vq(q1[0], q1[1], q1[2]);
	btlTriangleShapeEx trishapeB(vq,
				  btlVector3(q2[0], q2[1], q2[2]), 
				  btlVector3(q3[0], q3[1], q3[2]));
	trishapeB.setMargin(0.000001f);
	
	// btVoronoiSimplexSolver sGjkSimplexSolver;
	// btGjkEpaPenetrationDepthSolver penSolverPtr;	
	
	/*static*/ btlSimplexSolverInterface sGjkSimplexSolver;
	sGjkSimplexSolver.reset();
	
	/*static*/ btlGjkEpaPenetrationDepthSolver Solver0;
	/*static*/ btlMinkowskiPenetrationDepthSolver Solver1;
		
	btlConvexPenetrationDepthSolver* Solver = NULL;
	
	Solver = &Solver1;	
		
	btlGjkPairDetector convexConvex(&trishapeA ,&trishapeB,&sGjkSimplexSolver,Solver);
	
	convexConvex.m_catchDegeneracies = 1;
	
	// btGjkPairDetector convexConvex(&trishapeA ,&trishapeB,&sGjkSimplexSolver,0);
	
	btlPointCollector gjkOutput;
	btlGjkPairDetector::ClosestPointInput input;
	
		
	btlTransform tr;
	tr.setIdentity();
	
	input.m_transformA = tr;
	input.m_transformB = tr;
	
	convexConvex.getClosestPoints(input, gjkOutput, 0);
	
	
	if (gjkOutput.m_hasResult)
	{
		
		pb[0] = pa[0] = gjkOutput.m_pointInWorld[0];
		pb[1] = pa[1] = gjkOutput.m_pointInWorld[1];
		pb[2] = pa[2] = gjkOutput.m_pointInWorld[2];

		pb[0]+= gjkOutput.m_normalOnBInWorld[0] * gjkOutput.m_distance;
		pb[1]+= gjkOutput.m_normalOnBInWorld[1] * gjkOutput.m_distance;
		pb[2]+= gjkOutput.m_normalOnBInWorld[2] * gjkOutput.m_distance;
		
		normal[0] = gjkOutput.m_normalOnBInWorld[0];
		normal[1] = gjkOutput.m_normalOnBInWorld[1];
		normal[2] = gjkOutput.m_normalOnBInWorld[2];

		return gjkOutput.m_distance;
	}
	return -1.0f;	
}

// Convex hull
plConvexHull btl_plConvexHullCompute(float (*coords)[3], int count)
{
	btlConvexHullComputer *computer = new btlConvexHullComputer;
	computer->compute(reinterpret_cast< float* >(coords),
					  sizeof(*coords), count, 0, 0);
	return reinterpret_cast<plConvexHull>(computer);
}

void btl_plConvexHullDelete(plConvexHull hull)
{
	btlConvexHullComputer *computer(reinterpret_cast< btlConvexHullComputer* >(hull));
	delete computer;
}

int btl_plConvexHullNumVertices(plConvexHull hull)
{
	btlConvexHullComputer *computer(reinterpret_cast< btlConvexHullComputer* >(hull));
	return computer->vertices.size();
}

int btl_plConvexHullNumFaces(plConvexHull hull)
{
	btlConvexHullComputer *computer(reinterpret_cast< btlConvexHullComputer* >(hull));
	return computer->faces.size();
}

void btl_plConvexHullGetVertex(plConvexHull hull, int n, float coords[3],
						   int *original_index)
{
	btlConvexHullComputer *computer(reinterpret_cast< btlConvexHullComputer* >(hull));
	const btlVector3 &v(computer->vertices[n]);
	coords[0] = v[0];
	coords[1] = v[1];
	coords[2] = v[2];
	(*original_index) = computer->original_vertex_index[n];
}

int btl_plConvexHullGetFaceSize(plConvexHull hull, int n)
{
	btlConvexHullComputer *computer(reinterpret_cast< btlConvexHullComputer* >(hull));
	const btlConvexHullComputer::Edge *e_orig, *e;
	int count;

	for (e_orig = &computer->edges[computer->faces[n]], e = e_orig, count = 0;
		 count == 0 || e != e_orig;
		 e = e->getNextEdgeOfFace(), count++);
	return count;
}

void btl_plConvexHullGetFaceVertices(plConvexHull hull, int n, int *vertices)
{
	btlConvexHullComputer *computer(reinterpret_cast< btlConvexHullComputer* >(hull));
	const btlConvexHullComputer::Edge *e_orig, *e;
	int count;

	for (e_orig = &computer->edges[computer->faces[n]], e = e_orig, count = 0;
		 count == 0 || e != e_orig;
		 e = e->getNextEdgeOfFace(), count++)
	{
		vertices[count] = e->getTargetVertex();
	}
}
