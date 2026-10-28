/*
Bullet NEXT feature helpers (F1).
Copyright (c) 2026 BGE NEXT contributors. Same zlib terms as Bullet.

Each helper switches on one Bullet NEXT behaviour for one object or world. Nothing here runs
unless it is called, so plain upstream behaviour stays available.
*/
#ifndef BT_NEXT_FEATURES_H
#define BT_NEXT_FEATURES_H

#include "LinearMath/btScalar.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"  // btNextCcdHits

class btCollisionObject;
class btCollisionShape;
class btRigidBody;

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
/// derived from the shape's smallest half extent; with shapeCast the real convex shape is swept
/// (Stage 1: or each convex child of a compound, such as a car body made of hulls).
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

/// Stage 1: true when continuous collision can sweep the shape itself: a convex shape, or a
/// compound whose children are all convex (each child is swept; the earliest hit counts).
bool btNextCcdCastable(const btCollisionShape* shape);

/// Crash report (Stage 1): the momentum continuous collision removed in the last internal step,
/// one entry per body it stopped (body, the object hit, point, normal and the impulse it amounts
/// to, in world space). With the contact impulses of the manifolds (m_appliedImpulse and the two
/// lateral ones), it accounts for every change of momentum a hit made. Read it in an internal
/// tick callback (once per internal step); it is in body order, the same at any thread count.
const btAlignedObjectArray<btDiscreteDynamicsWorld::BnCcdHit>& btNextCcdHits(const btDiscreteDynamicsWorld* world);

/// Crash report (Stage 1): the impulse the last step's solve gave body 0 of the manifold through
/// this point (normal and friction, world space; body 1 got the opposite), with either Bullet
/// NEXT solver. Zero for a point the solver left out (beyond the processing threshold), whose
/// stored impulse is from an earlier step.
btVector3 btNextContactImpulse(const class btPersistentManifold* manifold, int point);

#endif
