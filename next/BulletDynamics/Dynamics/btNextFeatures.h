/*
Bullet NEXT feature helpers (F1).
Copyright (c) 2026 BGE NEXT contributors. Same zlib terms as Bullet.

Each helper switches on one Bullet NEXT behaviour for one object or world. Nothing here runs
unless it is called, so plain upstream behaviour stays available.
*/
#ifndef BT_NEXT_FEATURES_H
#define BT_NEXT_FEATURES_H

#include "LinearMath/btScalar.h"

class btCollisionObject;
class btCollisionShape;
class btRigidBody;
class btDiscreteDynamicsWorld;

/// C1.1 - Blender semantics for constraint-linked bodies (collide again when the constraint is
/// disabled). Call before constraints are added.
void btNextSetLinkedCollisionWhileEnabled(bool enable);

/// C1.5 - Build polyhedral features so hull pairs use one-shot clipped manifolds. Recurses into
/// compound shapes. Returns the number of shapes prepared.
int btNextEnableHullClipping(btCollisionShape* shape);

/// C1.3 - Deterministic pair and island ordering, plus SAT clipping for convex vs triangle.
void btNextConfigureWorld(btDiscreteDynamicsWorld* world);

#endif
