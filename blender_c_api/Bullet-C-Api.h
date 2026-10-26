/*
Bullet NEXT - Blender C API shim (F1, C1.11).
Copyright (c) 2026 BGE NEXT contributors. Same zlib terms as Bullet.
*/

/* Blender's C callers (cloth collision, bmesh convex hull) use Bullet's C API under its
 * original names. Bullet NEXT serves that API from the frozen legacy backend, whose exported
 * names carry the btl_ prefix, so the original names are mapped here. */
#ifndef BN_BLENDER_C_API_H
#define BN_BLENDER_C_API_H

#define plAddChildShape btl_plAddChildShape
#define plAddRigidBody btl_plAddRigidBody
#define plAddVertex btl_plAddVertex
#define plConvexHullCompute btl_plConvexHullCompute
#define plConvexHullDelete btl_plConvexHullDelete
#define plConvexHullGetFaceSize btl_plConvexHullGetFaceSize
#define plConvexHullGetFaceVertices btl_plConvexHullGetFaceVertices
#define plConvexHullGetVertex btl_plConvexHullGetVertex
#define plConvexHullNumFaces btl_plConvexHullNumFaces
#define plConvexHullNumVertices btl_plConvexHullNumVertices
#define plCreateDynamicsWorld btl_plCreateDynamicsWorld
#define plCreateRigidBody btl_plCreateRigidBody
#define plDeleteDynamicsWorld btl_plDeleteDynamicsWorld
#define plDeletePhysicsSdk btl_plDeletePhysicsSdk
#define plDeleteRigidBody btl_plDeleteRigidBody
#define plDeleteShape btl_plDeleteShape
#define plGetOpenGLMatrix btl_plGetOpenGLMatrix
#define plGetOrientation btl_plGetOrientation
#define plGetPosition btl_plGetPosition
#define plNearestPoints btl_plNearestPoints
#define plNewBoxShape btl_plNewBoxShape
#define plNewBulletSdk btl_plNewBulletSdk
#define plNewCapsuleShape btl_plNewCapsuleShape
#define plNewCompoundShape btl_plNewCompoundShape
#define plNewConeShape btl_plNewConeShape
#define plNewConvexHullShape btl_plNewConvexHullShape
#define plNewCylinderShape btl_plNewCylinderShape
#define plNewMeshInterface btl_plNewMeshInterface
#define plNewSphereShape btl_plNewSphereShape
#define plRemoveRigidBody btl_plRemoveRigidBody
#define plSetEuler btl_plSetEuler
#define plSetOpenGLMatrix btl_plSetOpenGLMatrix
#define plSetOrientation btl_plSetOrientation
#define plSetPosition btl_plSetPosition
#define plSetScaling btl_plSetScaling
#define plStepSimulation btl_plStepSimulation

#include "../legacy/Bullet-C-Api.h"

#endif  /* BN_BLENDER_C_API_H */
