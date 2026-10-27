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

/// C1.6 - Bake internal-edge data for a static triangle mesh or heightfield object (also meshes
/// inside a compound) and correct contact normals on internal edges automatically. Returns false
/// when the object has no mesh or heightfield.
bool btNextEnableActiveEdges(btCollisionObject* meshObject);

/// C1.6 option: also force the face normal where all three edges of a triangle are flat.
extern bool gBnFlatRegionRule;

/// C1.7 / C1.8 - Continuous collision for a fast body: motion threshold and swept radius are
/// derived from the shape's smallest half extent; with shapeCast the real convex shape is swept.
void btNextConfigureCcd(btRigidBody* body, bool shapeCast);

/// C1.3 - Deterministic pair and island ordering, plus SAT clipping for convex vs triangle.
void btNextConfigureWorld(btDiscreteDynamicsWorld* world);

/// Stage 0, CS.3 - World recentring for large worlds: moves every collision object (its world and
/// interpolation transforms, its motion state, its broadphase bounds) by `offset`, so a game can
/// keep the player near the origin where floats are precise. Constraints, contacts, vehicles and
/// characters follow, as they are relative to their bodies. Soft bodies are not moved here
/// (BulletDynamics does not know them): call btSoftBody::translate(offset) on each. Returns the
/// number of soft bodies skipped. Call between steps.
int btNextShiftWorld(btDiscreteDynamicsWorld* world, const class btVector3& offset);

#endif
