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

	If possible, use the richer Bullet C++ API, by including "btBulletDynamicsCommon.h"
*/

#ifndef BULLET_C_API_H
#define BULLET_C_API_H

#define PL_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifdef BT_USE_DOUBLE_PRECISION
typedef double	plReal;
#else
typedef float	plReal;
#endif

typedef plReal	plVector3[3];
typedef plReal	plQuaternion[4];

#ifdef __cplusplus
extern "C" { 
#endif

/**	Particular physics SDK (C-API) */
	PL_DECLARE_HANDLE(plPhysicsSdkHandle);

/** 	Dynamics world, belonging to some physics SDK (C-API)*/
	PL_DECLARE_HANDLE(plDynamicsWorldHandle);

/** Rigid Body that can be part of a Dynamics World (C-API)*/	
	PL_DECLARE_HANDLE(plRigidBodyHandle);

/** 	Collision Shape/Geometry, property of a Rigid Body (C-API)*/
	PL_DECLARE_HANDLE(plCollisionShapeHandle);

/** Constraint for Rigid Bodies (C-API)*/
	PL_DECLARE_HANDLE(plConstraintHandle);

/** Triangle Mesh interface (C-API)*/
	PL_DECLARE_HANDLE(plMeshInterfaceHandle);

/** Broadphase Scene/Proxy Handles (C-API)*/
	PL_DECLARE_HANDLE(plCollisionBroadphaseHandle);
	PL_DECLARE_HANDLE(plBroadphaseProxyHandle);
	PL_DECLARE_HANDLE(plCollisionWorldHandle);

/**
	Create and Delete a Physics SDK	
*/

	extern	plPhysicsSdkHandle	btl_plNewBulletSdk(void); //this could be also another sdk, like ODE, PhysX etc.
	extern	void		btl_plDeletePhysicsSdk(plPhysicsSdkHandle	physicsSdk);

/** Collision World, not strictly necessary, you can also just create a Dynamics World with Rigid Bodies which internally manages the Collision World with Collision Objects */

	typedef void(*btlBroadphaseCallback)(void* clientData, void* object1,void* object2);

	extern plCollisionBroadphaseHandle	plCreateSapBroadphase(btlBroadphaseCallback beginCallback,btlBroadphaseCallback endCallback);

	extern void	plDestroyBroadphase(plCollisionBroadphaseHandle bp);

	extern 	plBroadphaseProxyHandle plCreateProxy(plCollisionBroadphaseHandle bp, void* clientData, plReal minX,plReal minY,plReal minZ, plReal maxX,plReal maxY, plReal maxZ);

	extern void plDestroyProxy(plCollisionBroadphaseHandle bp, plBroadphaseProxyHandle proxyHandle);

	extern void plSetBoundingBox(plBroadphaseProxyHandle proxyHandle, plReal minX,plReal minY,plReal minZ, plReal maxX,plReal maxY, plReal maxZ);

/* todo: add pair cache support with queries like add/remove/find pair */
	
	extern plCollisionWorldHandle plCreateCollisionWorld(plPhysicsSdkHandle physicsSdk);

/* todo: add/remove objects */
	

/* Dynamics World */

	extern  plDynamicsWorldHandle btl_plCreateDynamicsWorld(plPhysicsSdkHandle physicsSdk);

	extern  void           btl_plDeleteDynamicsWorld(plDynamicsWorldHandle world);

	extern	void	btl_plStepSimulation(plDynamicsWorldHandle,	plReal	timeStep);

	extern  void btl_plAddRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);

	extern  void btl_plRemoveRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);


/* Rigid Body  */

	extern  plRigidBodyHandle btl_plCreateRigidBody(	void* user_data,  float mass, plCollisionShapeHandle cshape );

	extern  void btl_plDeleteRigidBody(plRigidBodyHandle body);


/* Collision Shape definition */

	extern  plCollisionShapeHandle btl_plNewSphereShape(plReal radius);
	extern  plCollisionShapeHandle btl_plNewBoxShape(plReal x, plReal y, plReal z);
	extern  plCollisionShapeHandle btl_plNewCapsuleShape(plReal radius, plReal height);	
	extern  plCollisionShapeHandle btl_plNewConeShape(plReal radius, plReal height);
	extern  plCollisionShapeHandle btl_plNewCylinderShape(plReal radius, plReal height);
	extern	plCollisionShapeHandle btl_plNewCompoundShape(void);
	extern	void	btl_plAddChildShape(plCollisionShapeHandle compoundShape,plCollisionShapeHandle childShape, plVector3 childPos,plQuaternion childOrn);

	extern  void btl_plDeleteShape(plCollisionShapeHandle shape);

	/* Convex Meshes */
	extern  plCollisionShapeHandle btl_plNewConvexHullShape(void);
	extern  void		btl_plAddVertex(plCollisionShapeHandle convexHull, plReal x,plReal y,plReal z);
/* Concave static triangle meshes */
	extern  plMeshInterfaceHandle		   btl_plNewMeshInterface(void);
	extern  void		plAddTriangle(plMeshInterfaceHandle meshHandle, plVector3 v0,plVector3 v1,plVector3 v2);
	extern  plCollisionShapeHandle plNewStaticTriangleMeshShape(plMeshInterfaceHandle);

	extern  void btl_plSetScaling(plCollisionShapeHandle shape, plVector3 scaling);

/* SOLID has Response Callback/Table/Management */
/* PhysX has Triggers, User Callbacks and filtering */
/* ODE has the typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2); */

/*	typedef void plUpdatedPositionCallback(void* userData, plRigidBodyHandle	rbHandle, plVector3 pos); */
/*	typedef void plUpdatedOrientationCallback(void* userData, plRigidBodyHandle	rbHandle, plQuaternion orientation); */

	/* get world transform */
	extern void	btl_plGetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix);
	extern void	btl_plGetPosition(plRigidBodyHandle object,plVector3 position);
	extern void btl_plGetOrientation(plRigidBodyHandle object,plQuaternion orientation);

	/* set world transform (position/orientation) */
	extern  void btl_plSetPosition(plRigidBodyHandle object, const plVector3 position);
	extern  void btl_plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation);
	extern	void btl_plSetEuler(plReal yaw,plReal pitch,plReal roll, plQuaternion orient);
	extern	void btl_plSetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix);

	typedef struct plRayCastResult {
		plRigidBodyHandle		m_body;  
		plCollisionShapeHandle	m_shape; 		
		plVector3				m_positionWorld; 		
		plVector3				m_normalWorld;
	} plRayCastResult;

	extern  int plRayCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plRayCastResult res);

	/* Sweep API */

	/* extern  plRigidBodyHandle plObjectCast(plDynamicsWorldHandle world, const plVector3 rayStart, const plVector3 rayEnd, plVector3 hitpoint, plVector3 normal); */

	/* Continuous Collision Detection API */
	
	// needed for source/blender/blenkernel/intern/collision.c
	double btl_plNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3]);


	/* Convex Hull */
	PL_DECLARE_HANDLE(plConvexHull);
	plConvexHull btl_plConvexHullCompute(float (*coords)[3], int count);
	void btl_plConvexHullDelete(plConvexHull hull);
	int btl_plConvexHullNumVertices(plConvexHull hull);
	int btl_plConvexHullNumFaces(plConvexHull hull);
	void btl_plConvexHullGetVertex(plConvexHull hull, int n, float coords[3], int *original_index);
	int btl_plConvexHullGetFaceSize(plConvexHull hull, int n);
	void btl_plConvexHullGetFaceVertices(plConvexHull hull, int n, int *vertices);

#ifdef __cplusplus
}
#endif


#endif //BULLET_C_API_H

