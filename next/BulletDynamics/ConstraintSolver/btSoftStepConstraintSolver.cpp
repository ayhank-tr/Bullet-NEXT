/*
Bullet NEXT - Soft Step constraint solver (F3c).
Copyright (c) 2026 Bullet NEXT contributors.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

The substep structure, the soft constraint coefficients and the relax and restitution passes
follow Box2D v3's soft step solver (Erin Catto, MIT licence); rebuilding joints every substep
follows Rapier. The code is written for Bullet's data structures.
*/

#include "btSoftStepConstraintSolver.h"

#include "btContactSolverInfo.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btTransformUtil.h"

#include <string.h>

namespace
{
// Soft constraint coefficients (Box2D v3, b2MakeSoft): a damped spring of the given frequency
// and damping ratio, solved implicitly over a substep of length h.
struct Softness
{
	btScalar biasRate, massScale, impulseScale;
};

Softness makeSoft(btScalar hertz, btScalar zeta, btScalar h)
{
	Softness s;
	if (hertz <= btScalar(0))
	{
		s.biasRate = 0;
		s.massScale = 1;
		s.impulseScale = 0;
		return s;
	}
	const btScalar omega = SIMD_2_PI * hertz;
	const btScalar a1 = btScalar(2) * zeta + h * omega;
	const btScalar a2 = h * omega * a1;
	const btScalar a3 = btScalar(1) / (btScalar(1) + a2);
	s.biasRate = omega / a1;
	s.massScale = a2 * a3;
	s.impulseScale = a3;
	return s;
}

typedef btSoftStepConstraintSolver::Body SsBody;

// Orientation after rotating at w for h seconds: Bullet's exponential map
// (btTransformUtil::integrateTransform), on the quaternion itself.
btQuaternion integrateRotation(const btQuaternion& q, const btVector3& w, btScalar h)
{
	btScalar angle2 = w.length2();
	btScalar angle = angle2 > SIMD_EPSILON ? btSqrt(angle2) : btScalar(0);
	if (angle * h > ANGULAR_MOTION_THRESHOLD)
		angle = ANGULAR_MOTION_THRESHOLD / h;
	btVector3 axis;
	if (angle < btScalar(0.001))
		axis = w * (btScalar(0.5) * h - (h * h * h) * btScalar(0.020833333333) * angle * angle);
	else
		axis = w * (btSin(btScalar(0.5) * angle * h) / angle);
	btQuaternion d(axis.x(), axis.y(), axis.z(), btCos(angle * h * btScalar(0.5)));
	btQuaternion r = d * q;
	r.safeNormalize();
	if (r.length2() > SIMD_EPSILON)
		return r;
	return q;
}

// btRigidBody::computeGyroscopicImpulseImplicit_Body, on the solver's orientation and spin:
// one implicit Euler (Newton) step of the body-frame equation I dw/dt = -w x I w.
btVector3 gyroscopicImplicitBody(const btQuaternion& q, const btVector3& w, const btVector3& inertia, btScalar h)
{
	const btVector3 wb = quatRotate(q.inverse(), w);
	btMatrix3x3 Ib;
	Ib.setValue(inertia.x(), 0, 0, 0, inertia.y(), 0, 0, 0, inertia.z());
	const btVector3 ibo = Ib * wb;
	const btVector3 f = h * wb.cross(ibo);
	btMatrix3x3 skew0, skew1;
	wb.getSkewSymmetricMatrix(&skew0[0], &skew0[1], &skew0[2]);
	ibo.getSkewSymmetricMatrix(&skew1[0], &skew1[1], &skew1[2]);
	const btMatrix3x3 J = Ib + (skew0 * Ib - skew1) * h;
	const btVector3 wb2 = wb - J.solve33(f);
	return quatRotate(q, wb2) - w;
}

void applyAnisotropic(const btCollisionObject* obj, btVector3& dir, int mode)
{
	if (obj && obj->hasAnisotropicFriction(mode))
	{
		btVector3 local = dir * obj->getWorldTransform().getBasis();
		local *= obj->getAnisotropicFriction();
		dir = obj->getWorldTransform().getBasis() * local;
	}
}

inline btScalar clampScalar(btScalar v, btScalar lo, btScalar hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

inline btScalar invMassOf(btScalar k)
{
	return k > SIMD_EPSILON ? btScalar(1) / k : btScalar(0);
}
}  // namespace

btSoftStepConstraintSolver::btSoftStepConstraintSolver()
	: m_fixedBody(-1),
	  m_maxGyroscopicForce(100),
	  m_h(0),
	  m_invH(0),
	  m_warmStart(true),
	  m_jointMassScale(1),
	  m_jointImpulseScale(0),
	  m_syncStamp(0)
{
}

btSoftStepConstraintSolver::~btSoftStepConstraintSolver()
{
}

int btSoftStepConstraintSolver::addBody(btCollisionObject* obj, btScalar timeStep)
{
	(void)timeStep;
	const int index = m_bodies.size();
	Body& b = m_bodies.expandNonInitializing();
	btRigidBody* rb = obj ? btRigidBody::upcast(obj) : 0;
	b.m_body = rb;
	b.m_dynamic = 0;
	b.m_gyro = 0;
	b.m_stamp = 0;
	b.m_v.setZero();
	b.m_w.setZero();
	b.m_dp.setZero();
	b.m_dR.setIdentity();
	b.m_invMass.setZero();
	b.m_invI.setValue(0, 0, 0, 0, 0, 0, 0, 0, 0);
	b.m_linAccel.setZero();
	b.m_angAccel.setZero();
	b.m_localInertia.setZero();
	b.m_angFactor.setValue(1, 1, 1);
	if (rb)
	{
		b.m_start = rb->getWorldTransform();
		b.m_x0 = b.m_start.getOrigin();
		b.m_start.getBasis().getRotation(b.m_q0);
		b.m_v = rb->getLinearVelocity();
		b.m_w = rb->getAngularVelocity();
		if (!rb->isStaticOrKinematicObject() && rb->getInvMass() > btScalar(0))
		{
			b.m_dynamic = 1;
			b.m_invMass = rb->getLinearFactor() * rb->getInvMass();
			const btVector3& a = rb->getAngularFactor();
			const btMatrix3x3& I = rb->getInvInertiaTensorWorld();
			for (int r = 0; r < 3; ++r)
				for (int c = 0; c < 3; ++c)
					b.m_invI[r][c] = I[r][c] * a[r] * a[c];
			// as btSequentialImpulseConstraintSolver::initSolverBody
			b.m_linAccel = rb->getTotalForce() * rb->getInvMass();
			b.m_angAccel = rb->getTotalTorque() * rb->getInvInertiaTensorWorld();
			b.m_gyro = rb->getFlags() & (BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT | BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD | BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY);
			b.m_localInertia = rb->getLocalInertia();
			b.m_angFactor = a;
		}
		else if (!rb->isKinematicObject())
		{
			b.m_v.setZero();
			b.m_w.setZero();
		}
	}
	else
	{
		b.m_start.setIdentity();
		b.m_x0.setZero();
		b.m_q0 = btQuaternion::getIdentity();
	}
	b.m_x = b.m_x0;
	b.m_q = b.m_q0;
	return index;
}

int btSoftStepConstraintSolver::bodyIndex(btCollisionObject* obj, btScalar timeStep)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb && !rb->isStaticOrKinematicObject() && rb->getInvMass() > btScalar(0))
	{
		// a dynamic body is in one island only, so its companion id is ours to use
		int id = obj->getCompanionId();
		if (id < 0 || id >= m_bodies.size() || m_bodies[id].m_body != rb)
		{
			id = addBody(obj, timeStep);
			obj->setCompanionId(id);
		}
		return id;
	}
	if (rb && rb->isKinematicObject())
	{
		// kinematic bodies can be in several islands at once: keep our own table
		const int unique = obj->getWorldArrayIndex();
		if (unique >= 0)
		{
			if (unique >= m_kinematicIndex.size())
				m_kinematicIndex.resize(unique + 1, -1);
			int id = m_kinematicIndex[unique];
			if (id < 0)
			{
				id = addBody(obj, timeStep);
				m_kinematicIndex[unique] = id;
				m_kinematicUsed.push_back(unique);
			}
			return id;
		}
		return addBody(obj, timeStep);
	}
	if (m_fixedBody < 0)
		m_fixedBody = addBody(0, timeStep);
	return m_fixedBody;
}

void btSoftStepConstraintSolver::preparePoints(btPersistentManifold** manifolds, int numManifolds, const btContactSolverInfo& info)
{
	const btScalar dt = info.m_timeStep;
	for (int m = 0; m < numManifolds; ++m)
	{
		btPersistentManifold* manifold = manifolds[m];
		btCollisionObject* obj0 = (btCollisionObject*)manifold->getBody0();
		btCollisionObject* obj1 = (btCollisionObject*)manifold->getBody1();
		const int ia = bodyIndex(obj0, dt);
		const int ib = bodyIndex(obj1, dt);
		if (!m_bodies[ia].m_dynamic && !m_bodies[ib].m_dynamic)
			continue;  // as the sequential solver: no response between two bodies that cannot move
		const int first = m_points.size();
		for (int k = 0; k < manifold->getNumContacts(); ++k)
		{
			btManifoldPoint& cp = manifold->getContactPoint(k);
			if (cp.getDistance() > manifold->getContactProcessingThreshold())
				continue;
			const Body& A = m_bodies[ia];
			const Body& B = m_bodies[ib];
			Point& p = m_points.expandNonInitializing();
			p.m_cp = &cp;
			p.m_motion1 = p.m_motion2 = 0;
			p.m_normalImpulse = p.m_tangentImpulse1 = p.m_tangentImpulse2 = 0;
			p.m_totalNormalImpulse = p.m_maxNormalImpulse = 0;
			p.m_separation = 0;
			p.m_a = ia;
			p.m_b = ib;
			p.m_static = (!A.m_dynamic || !B.m_dynamic) ? 1 : 0;
			p.m_n = cp.m_normalWorldOnB;
			p.m_rA = cp.getPositionWorldOnA() - A.m_x0;
			p.m_rB = cp.getPositionWorldOnB() - B.m_x0;
			p.m_baseSeparation = cp.getDistance() - p.m_n.dot(p.m_rA - p.m_rB);

			// friction directions: fixed for the step (from the normal), unless a contact callback set them
			if ((info.m_solverMode & SOLVER_ENABLE_FRICTION_DIRECTION_CACHING) && (cp.m_contactPointFlags & BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED))
			{
				p.m_t1 = cp.m_lateralFrictionDir1;
				p.m_t2 = cp.m_lateralFrictionDir2;
				p.m_motion1 = cp.m_contactMotion1;
				p.m_motion2 = cp.m_contactMotion2;
			}
			else
			{
				btPlaneSpace1(p.m_n, p.m_t1, p.m_t2);
				p.m_t1.normalize();
				p.m_t2.normalize();
				applyAnisotropic(obj0, p.m_t1, btCollisionObject::CF_ANISOTROPIC_FRICTION);
				applyAnisotropic(obj1, p.m_t1, btCollisionObject::CF_ANISOTROPIC_FRICTION);
				applyAnisotropic(obj0, p.m_t2, btCollisionObject::CF_ANISOTROPIC_FRICTION);
				applyAnisotropic(obj1, p.m_t2, btCollisionObject::CF_ANISOTROPIC_FRICTION);
				cp.m_lateralFrictionDir1 = p.m_t1;
				cp.m_lateralFrictionDir2 = p.m_t2;
			}

			// Jacobians and responses; the bodies that cannot move have zero inverse mass and inertia
			p.m_rAxN = p.m_rA.cross(p.m_n);
			p.m_rBxN = p.m_rB.cross(p.m_n);
			p.m_rAxT1 = p.m_rA.cross(p.m_t1);
			p.m_rBxT1 = p.m_rB.cross(p.m_t1);
			p.m_rAxT2 = p.m_rA.cross(p.m_t2);
			p.m_rBxT2 = p.m_rB.cross(p.m_t2);
			p.m_aAn = A.m_invI * p.m_rAxN;
			p.m_aBn = B.m_invI * p.m_rBxN;
			p.m_aAt1 = A.m_invI * p.m_rAxT1;
			p.m_aBt1 = B.m_invI * p.m_rBxT1;
			p.m_aAt2 = A.m_invI * p.m_rAxT2;
			p.m_aBt2 = B.m_invI * p.m_rBxT2;
			const btVector3 invMassAB = A.m_invMass + B.m_invMass;
			p.m_normalMass = invMassOf(p.m_n.dot(invMassAB * p.m_n) + p.m_rAxN.dot(p.m_aAn) + p.m_rBxN.dot(p.m_aBn));
			p.m_tangentMass1 = invMassOf(p.m_t1.dot(invMassAB * p.m_t1) + p.m_rAxT1.dot(p.m_aAt1) + p.m_rBxT1.dot(p.m_aBt1));
			p.m_tangentMass2 = invMassOf(p.m_t2.dot(invMassAB * p.m_t2) + p.m_rAxT2.dot(p.m_aAt2) + p.m_rBxT2.dot(p.m_aBt2));

			p.m_friction = cp.m_combinedFriction;
			p.m_restitution = cp.m_combinedRestitution;
			p.m_rolling = cp.m_combinedRollingFriction;
			p.m_spinning = cp.m_combinedSpinningFriction;
			p.m_rollingIndex = -1;
			if (p.m_rolling > 0 || p.m_spinning > 0)
			{
				p.m_rollingIndex = m_rolling.size();
				Rolling& rl = m_rolling.expandNonInitializing();
				rl.m_impulse[0] = rl.m_impulse[1] = rl.m_impulse[2] = 0;
				btVector3 r1, r2;
				btPlaneSpace1(p.m_n, r1, r2);
				r1.normalize();
				r2.normalize();
				applyAnisotropic(obj0, r1, btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
				applyAnisotropic(obj1, r1, btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
				applyAnisotropic(obj0, r2, btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
				applyAnisotropic(obj1, r2, btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
				rl.m_axis[0] = p.m_n;
				rl.m_axis[1] = r1;
				rl.m_axis[2] = r2;
				for (int a = 0; a < 3; ++a)
				{
					const btVector3& ax = rl.m_axis[a];
					rl.m_aA[a] = A.m_invI * ax;
					rl.m_aB[a] = B.m_invI * ax;
					rl.m_mass[a] = ax.length2() > btScalar(1e-6) ? invMassOf(ax.dot(rl.m_aA[a] + rl.m_aB[a])) : btScalar(0);
				}
			}

			// normal velocity before the step (restitution)
			p.m_relativeVelocity = p.m_n.dot(A.m_v - B.m_v) + p.m_rAxN.dot(A.m_w) - p.m_rBxN.dot(B.m_w);

			// warm start, in full, from the last substep of the previous step (substeps carry their
			// impulses over whole; Bullet's m_warmstartingFactor is for its own solver)
			if (m_warmStart)
			{
				p.m_normalImpulse = btMax(cp.m_prevRHS, btScalar(0)) * m_h;
				p.m_tangentImpulse1 = cp.m_appliedImpulseLateral1 * m_h;
				p.m_tangentImpulse2 = cp.m_appliedImpulseLateral2 * m_h;
			}
		}
		if (m_points.size() > first)
		{
			Group& g = m_groups.expandNonInitializing();
			g.m_first = first;
			g.m_count = m_points.size() - first;
			g.m_kind = (m_bodies[ia].m_dynamic ? 1 : 0) | (m_bodies[ib].m_dynamic ? 2 : 0);
		}
	}
}

void btSoftStepConstraintSolver::prepareJoints(btTypedConstraint** constraints, int numConstraints)
{
	int impulses = 0;
	for (int i = 0; i < numConstraints; ++i)
	{
		btTypedConstraint* c = constraints[i];
		btJointFeedback* fb = c->getJointFeedback();
		if (fb)
		{
			fb->m_appliedForceBodyA.setZero();
			fb->m_appliedTorqueBodyA.setZero();
			fb->m_appliedForceBodyB.setZero();
			fb->m_appliedTorqueBodyB.setZero();
		}
		c->internalSetAppliedImpulse(0);
		if (!c->isEnabled())
			continue;
		c->buildJacobian();
		Joint& j = m_joints.expandNonInitializing();
		j.m_constraint = c;
		j.m_a = bodyIndex(&c->getRigidBodyA(), 0);
		j.m_b = bodyIndex(&c->getRigidBodyB(), 0);
		j.m_firstRow = 0;
		j.m_numRows = 0;
		j.m_maxRows = btTypedConstraint::BN_MAX_WARM_ROWS;  // the most rows a btTypedConstraint has:
		                   // btGeneric6DofSpring2Constraint's limit, motor and spring on each of its six axes
		// warm start from the last substep of the previous step, while the rows are the same
		j.m_prevRows = c->m_bnWarmRows > 0 ? c->m_bnWarmRows : -1;
		j.m_firstImpulse = impulses;
		impulses += j.m_maxRows;
	}
	m_jointImpulses.resize(impulses);
	m_jointTotals.resize(impulses);
	for (int i = 0; i < impulses; ++i)
	{
		m_jointImpulses[i] = 0;
		m_jointTotals[i] = 0;
	}
	if (m_warmStart)
	{
		for (int i = 0; i < m_joints.size(); ++i)
		{
			const Joint& j = m_joints[i];
			const btTypedConstraint* c = j.m_constraint;
			for (int r = 0; r < c->m_bnWarmRows; ++r)
				m_jointImpulses[j.m_firstImpulse + r] = c->m_bnWarmForce[r] * m_h * m_settings.m_jointWarmStart;
		}
	}
}

// Joints read their bodies' transforms (getInfo1, getInfo2) and some their velocities (the
// springs and motors of the 6DOF constraints): give them the solver's current state.
void btSoftStepConstraintSolver::syncJointBodies()
{
	++m_syncStamp;
	for (int i = 0; i < m_joints.size(); ++i)
	{
		const Joint& j = m_joints[i];
		const int ids[2] = {j.m_a, j.m_b};
		for (int k = 0; k < 2; ++k)
		{
			Body& b = m_bodies[ids[k]];
			if (!b.m_dynamic || b.m_stamp == m_syncStamp)
				continue;
			b.m_stamp = m_syncStamp;
			b.m_body->setWorldTransform(btTransform(b.m_q, b.m_x));
			b.m_body->setLinearVelocity(b.m_v);
			b.m_body->setAngularVelocity(b.m_w);
		}
	}
}

void btSoftStepConstraintSolver::buildJointRows(const btContactSolverInfo& info, btScalar h)
{
	int total = 0;
	for (int i = 0; i < m_joints.size(); ++i)
	{
		Joint& j = m_joints[i];
		btTypedConstraint::btConstraintInfo1 info1;
		j.m_constraint->getInfo1(&info1);
		int rows = info1.m_numConstraintRows;
		if (rows > j.m_maxRows)
			rows = j.m_maxRows;  // a constraint with more rows than Bullet's own: the extra rows are dropped
		j.m_firstRow = total;
		j.m_numRows = rows;
		total += rows;
	}
	m_rows.resizeNoInitialize(total);
	m_rowsNoBias.resizeNoInitialize(btTypedConstraint::BN_MAX_WARM_ROWS);
	m_rowTarget0.resizeNoInitialize(total);
	m_rowK.resizeNoInitialize(total);
	m_rowEps.resizeNoInitialize(total);

	// Position correction. Soft (m_jointHertz > 0): getInfo2 gives the rows with erp 1 and 0;
	// their difference is each row's position error per substep, corrected at the soft spring's
	// rate (a correction speed that does not grow as the substeps shrink, unlike an ERP per
	// substep). Rows that keep their own ERP (a constraint's ERP parameter, stops) come out the
	// same both times and keep Bullet's correction.
	const bool softJoints = m_settings.m_jointHertz > 0;
	btScalar hertz = m_settings.m_jointHertz;
	if (hertz > btScalar(0.5) / h)
		hertz = btScalar(0.5) / h;
	const Softness soft = makeSoft(hertz, m_settings.m_jointDampingRatio, h);
	m_jointMassScale = softJoints ? soft.massScale : btScalar(1);
	m_jointImpulseScale = softJoints ? soft.impulseScale : btScalar(0);
	const btScalar biasErp = softJoints ? btScalar(1) : info.m_erp;
	for (int i = 0; i < m_joints.size(); ++i)
	{
		Joint& j = m_joints[i];
		if (!j.m_numRows)
			continue;
		btTypedConstraint* c = j.m_constraint;
		// the rows with position correction (solve), then without it (relax)
		for (int pass = 0; pass < 2; ++pass)
		{
			btSolverConstraint* row = pass == 0 ? &m_rows[j.m_firstRow] : &m_rowsNoBias[0];
			for (int r = 0; r < j.m_numRows; ++r)
			{
				memset(&row[r], 0, sizeof(btSolverConstraint));
				row[r].m_lowerLimit = -SIMD_INFINITY;
				row[r].m_upperLimit = SIMD_INFINITY;
				row[r].m_cfm = info.m_globalCfm;
			}
			btTypedConstraint::btConstraintInfo2 info2;
			info2.fps = btScalar(1) / h;
			info2.erp = pass == 0 ? biasErp : btScalar(0);
			info2.m_J1linearAxis = row->m_contactNormal1;
			info2.m_J1angularAxis = row->m_relpos1CrossNormal;
			info2.m_J2linearAxis = row->m_contactNormal2;
			info2.m_J2angularAxis = row->m_relpos2CrossNormal;
			info2.rowskip = sizeof(btSolverConstraint) / sizeof(btScalar);
			info2.m_constraintError = &row->m_rhs;
			info2.cfm = &row->m_cfm;
			info2.m_lowerLimit = &row->m_lowerLimit;
			info2.m_upperLimit = &row->m_upperLimit;
			info2.m_damping = info.m_damping;
			info2.m_numIterations = info.m_numIterations;
			c->getInfo2(&info2);
		}
		for (int r = 0; r < j.m_numRows; ++r)
			m_rowTarget0[j.m_firstRow + r] = m_rowsNoBias[r].m_rhs;
		const Body& A = m_bodies[j.m_a];
		const Body& B = m_bodies[j.m_b];
		const btScalar breaking = c->getBreakingImpulseThreshold();
		for (int r = 0; r < j.m_numRows; ++r)
		{
			const int idx = j.m_firstRow + r;
			btSolverConstraint& row = m_rows[idx];
			if (softJoints)
				row.m_rhs = m_rowTarget0[idx] + soft.biasRate * h * (row.m_rhs - m_rowTarget0[idx]);
			// m_frictionIndex (unused by joints) marks the rows without limits: they go to the block,
			// and a breakable joint's are not capped at the threshold (it breaks on the step's total)
			row.m_frictionIndex = (row.m_lowerLimit <= -SIMD_INFINITY && row.m_upperLimit >= SIMD_INFINITY) ? 1 : 0;
			if (!row.m_frictionIndex)
			{
				if (row.m_upperLimit >= breaking) row.m_upperLimit = breaking;
				if (row.m_lowerLimit <= -breaking) row.m_lowerLimit = -breaking;
			}
			// the responses go into the row's spare vectors: m_angularComponentA/B (invI * J angular)
			row.m_angularComponentA = A.m_invI * row.m_relpos1CrossNormal;
			row.m_angularComponentB = B.m_invI * row.m_relpos2CrossNormal;
			const btScalar k = row.m_contactNormal1.dot(A.m_invMass * row.m_contactNormal1) + row.m_relpos1CrossNormal.dot(row.m_angularComponentA) +
							   row.m_contactNormal2.dot(B.m_invMass * row.m_contactNormal2) + row.m_relpos2CrossNormal.dot(row.m_angularComponentB);
			m_rowK[idx] = k;
			// Bullet's joint cfm (an impulse relaxation, dlambda = m (target - Jv) - cfm lambda), as a
			// regularisation in units of k: (k + eps) dlambda = target - Jv - eps lambda
			m_rowEps[idx] = row.m_cfm * k;
		}
		factorBlock(j);
		// keep the impulses between substeps while the rows stay the same
		if (j.m_numRows != j.m_prevRows)
		{
			for (int r = 0; r < j.m_maxRows; ++r)
				m_jointImpulses[j.m_firstImpulse + r] = 0;
		}
		j.m_prevRows = j.m_numRows;
	}
	m_stats.m_jointRows += total;
}

// The rows without limits (the joint's bilateral part: a point's three axes, a hinge's two
// locked rotations...) are coupled through the bodies' rotation; one by one they converge
// slowly when the lever is long against the inertia. Solved together, one pass is exact.
void btSoftStepConstraintSolver::factorBlock(Joint& j)
{
	j.m_blockCount = 0;
	int idx[6];
	int n = 0;
	for (int r = 0; r < j.m_numRows; ++r)
	{
		const btSolverConstraint& row = m_rows[j.m_firstRow + r];
		if (row.m_frictionIndex && m_rowK[j.m_firstRow + r] > SIMD_EPSILON)
		{
			if (n == 6)
				return;  // more than six free rows cannot be independent
			idx[n++] = r;
		}
	}
	if (n < 2)
		return;
	const Body& A = m_bodies[j.m_a];
	const Body& B = m_bodies[j.m_b];
	// K = J M^-1 J^T, from each row's responses (a side that cannot move adds nothing)
	btVector3 linA[6], linB[6];
	for (int a = 0; a < n; ++a)
	{
		const btSolverConstraint& ra = m_rows[j.m_firstRow + idx[a]];
		linA[a] = A.m_invMass * ra.m_contactNormal1;
		linB[a] = B.m_invMass * ra.m_contactNormal2;
	}
	btScalar K[6][6];
	for (int a = 0; a < n; ++a)
	{
		const btSolverConstraint& ra = m_rows[j.m_firstRow + idx[a]];
		for (int b = 0; b <= a; ++b)
		{
			const btSolverConstraint& rb = m_rows[j.m_firstRow + idx[b]];
			btScalar k = 0;
			if (A.m_dynamic)
				k += ra.m_contactNormal1.dot(linA[b]) + ra.m_relpos1CrossNormal.dot(rb.m_angularComponentA);
			if (B.m_dynamic)
				k += ra.m_contactNormal2.dot(linB[b]) + ra.m_relpos2CrossNormal.dot(rb.m_angularComponentB);
			K[a][b] = k;
		}
	}
	// LDL^T of K + diag(eps)
	btScalar L[6][6], D[6];
	for (int a = 0; a < n; ++a)
	{
		const btScalar kaa = K[a][a] + m_rowEps[j.m_firstRow + idx[a]];
		btScalar d = kaa;
		for (int k = 0; k < a; ++k)
			d -= L[a][k] * L[a][k] * D[k];
		if (!(d > btScalar(1e-6) * kaa))
			return;  // redundant rows: leave them to the one-by-one solve
		D[a] = d;
		for (int b = a + 1; b < n; ++b)
		{
			btScalar v = K[b][a];
			for (int k = 0; k < a; ++k)
				v -= L[b][k] * L[a][k] * D[k];
			L[b][a] = v / d;
		}
	}
	int t = 0;
	for (int a = 0; a < n; ++a)
	{
		j.m_blockInvDiag[a] = btScalar(1) / D[a];
		for (int b = 0; b < a; ++b)
			j.m_blockL[t++] = L[a][b];
	}
	for (int a = 0; a < n; ++a)
		j.m_blockRow[a] = idx[a];
	j.m_blockCount = n;
}

void btSoftStepConstraintSolver::integrateVelocities(btScalar h)
{
	for (int i = 0; i < m_bodies.size(); ++i)
	{
		Body& b = m_bodies[i];
		if (!b.m_dynamic)
			continue;
		b.m_v += b.m_linAccel * h;
		b.m_w += b.m_angAccel * h;
		// the gyroscopic term is quadratic in the spin: nothing to add for a body barely turning
		if (b.m_gyro && b.m_w.length2() > btScalar(1e-4))
		{
			if (b.m_gyro & BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY)
				b.m_w += b.m_angFactor * gyroscopicImplicitBody(b.m_q, b.m_w, b.m_localInertia, h);
			if (b.m_gyro & (BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT | BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD))
			{
				// the rarer variants: from the body itself, given the solver's state
				btRigidBody* rb = b.m_body;
				rb->setWorldTransform(btTransform(b.m_q, b.m_x));
				rb->setAngularVelocity(b.m_w);
				if (b.m_gyro & BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT)
					b.m_w -= (rb->computeGyroscopicForceExplicit(m_maxGyroscopicForce) * rb->getInvInertiaTensorWorld()) * h;
				if (b.m_gyro & BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD)
					b.m_w += rb->computeGyroscopicImpulseImplicit_World(h);
			}
		}
	}
}

void btSoftStepConstraintSolver::integratePositions(btScalar h)
{
	for (int i = 0; i < m_bodies.size(); ++i)
	{
		Body& b = m_bodies[i];
		if (!b.m_dynamic)
			continue;
		b.m_x += b.m_v * h;
		b.m_q = integrateRotation(b.m_q, b.m_w, h);
		b.m_dp = b.m_x - b.m_x0;
		b.m_dR.setRotation(b.m_q * b.m_q0.inverse());
	}
}

void btSoftStepConstraintSolver::warmStart()
{
	for (int i = 0; i < m_points.size(); ++i)
	{
		const Point& p = m_points[i];
		Body& A = m_bodies[p.m_a];
		Body& B = m_bodies[p.m_b];
		const btScalar n = p.m_normalImpulse, t1 = p.m_tangentImpulse1, t2 = p.m_tangentImpulse2;
		const btVector3 P = p.m_n * n + p.m_t1 * t1 + p.m_t2 * t2;
		A.m_v += A.m_invMass * P;
		A.m_w += p.m_aAn * n + p.m_aAt1 * t1 + p.m_aAt2 * t2;
		B.m_v -= B.m_invMass * P;
		B.m_w -= p.m_aBn * n + p.m_aBt1 * t1 + p.m_aBt2 * t2;
		if (p.m_rollingIndex >= 0)
		{
			const Rolling& rl = m_rolling[p.m_rollingIndex];
			for (int a = 0; a < 3; ++a)
			{
				A.m_w += rl.m_aA[a] * rl.m_impulse[a];
				B.m_w -= rl.m_aB[a] * rl.m_impulse[a];
			}
		}
	}
	for (int i = 0; i < m_joints.size(); ++i)
	{
		const Joint& j = m_joints[i];
		Body& A = m_bodies[j.m_a];
		Body& B = m_bodies[j.m_b];
		for (int r = 0; r < j.m_numRows; ++r)
		{
			const btSolverConstraint& row = m_rows[j.m_firstRow + r];
			btScalar& lambda = m_jointImpulses[j.m_firstImpulse + r];
			lambda = clampScalar(lambda, row.m_lowerLimit, row.m_upperLimit);
			if (lambda == 0)
				continue;
			A.m_v += A.m_invMass * row.m_contactNormal1 * lambda;
			A.m_w += row.m_angularComponentA * lambda;
			B.m_v += B.m_invMass * row.m_contactNormal2 * lambda;
			B.m_w += row.m_angularComponentB * lambda;
		}
	}
}

void btSoftStepConstraintSolver::solveJoints(bool useBias)
{
	// soft (solve pass): dl = massScale K^-1 (target - Jv) - impulseScale l, Box2D's soft
	// constraint on the whole block; rigid (relax pass): dl = K^-1 (target - Jv)
	const btScalar ms = useBias ? m_jointMassScale : btScalar(1);
	const btScalar is = useBias ? m_jointImpulseScale : btScalar(0);
	for (int i = 0; i < m_joints.size(); ++i)
	{
		const Joint& j = m_joints[i];
		Body& A = m_bodies[j.m_a];
		Body& B = m_bodies[j.m_b];
		int inBlock = 0;  // bit per row
		if (j.m_blockCount)
		{
			// residuals, then (K + eps) y = residual with the factor: forward, diagonal, backward
			const int n = j.m_blockCount;
			btScalar x[6];
			for (int a = 0; a < n; ++a)
			{
				const int r = j.m_blockRow[a];
				const int idx = j.m_firstRow + r;
				const btSolverConstraint& row = m_rows[idx];
				const btScalar target = useBias ? row.m_rhs : m_rowTarget0[idx];
				const btScalar jv = row.m_contactNormal1.dot(A.m_v) + row.m_relpos1CrossNormal.dot(A.m_w) + row.m_contactNormal2.dot(B.m_v) + row.m_relpos2CrossNormal.dot(B.m_w);
				x[a] = target - jv - m_rowEps[idx] * m_jointImpulses[j.m_firstImpulse + r];
				inBlock |= 1 << r;
			}
			const btScalar* L = j.m_blockL;
			int t = 0;
			for (int a = 0; a < n; ++a)
				for (int b = 0; b < a; ++b)
					x[a] -= L[t++] * x[b];
			for (int a = 0; a < n; ++a)
				x[a] *= j.m_blockInvDiag[a];
			for (int a = n - 1; a >= 0; --a)
			{
				// L^T: column a of L below the diagonal is L[b][a] for b > a
				for (int b = a + 1; b < n; ++b)
					x[a] -= L[b * (b - 1) / 2 + a] * x[b];
			}
			for (int a = 0; a < n; ++a)
			{
				const int r = j.m_blockRow[a];
				const btSolverConstraint& row = m_rows[j.m_firstRow + r];
				btScalar& lambda = m_jointImpulses[j.m_firstImpulse + r];
				const btScalar d = ms * x[a] - is * lambda;
				lambda += d;
				A.m_v += A.m_invMass * row.m_contactNormal1 * d;
				A.m_w += row.m_angularComponentA * d;
				B.m_v += B.m_invMass * row.m_contactNormal2 * d;
				B.m_w += row.m_angularComponentB * d;
			}
		}
		for (int r = 0; r < j.m_numRows; ++r)
		{
			if (inBlock & (1 << r))
				continue;
			const int idx = j.m_firstRow + r;
			const btSolverConstraint& row = m_rows[idx];
			const btScalar k = m_rowK[idx] + m_rowEps[idx];
			if (!(k > SIMD_EPSILON))
				continue;
			const btScalar target = useBias ? row.m_rhs : m_rowTarget0[idx];
			const btScalar jv = row.m_contactNormal1.dot(A.m_v) + row.m_relpos1CrossNormal.dot(A.m_w) + row.m_contactNormal2.dot(B.m_v) + row.m_relpos2CrossNormal.dot(B.m_w);
			btScalar& lambda = m_jointImpulses[j.m_firstImpulse + r];
			const btScalar delta = ms * (target - jv - m_rowEps[idx] * lambda) / k - is * lambda;
			const btScalar next = clampScalar(lambda + delta, row.m_lowerLimit, row.m_upperLimit);
			const btScalar d = next - lambda;
			lambda = next;
			if (d == 0)
				continue;
			A.m_v += A.m_invMass * row.m_contactNormal1 * d;
			A.m_w += row.m_angularComponentA * d;
			B.m_v += B.m_invMass * row.m_contactNormal2 * d;
			B.m_w += row.m_angularComponentB * d;
		}
	}
}

namespace
{
struct SoftParams
{
	Softness soft, staticSoft;
	btScalar invH, pushMax;
	int passes;
	bool useBias;
};
}  // namespace

// One manifold. AD / BD: whether body A / B moves; the other side is fixed or kinematic, so its
// velocity is read but never changed and its anchor does not move.
template <bool AD, bool BD>
void btSoftStepConstraintSolver::solveGroupContacts(const Group& g, const void* params)
{
	const SoftParams& P = *static_cast<const SoftParams*>(params);
	const Softness& soft = P.soft;
	const Softness& staticSoft = P.staticSoft;
	const btScalar invH = P.invH;
	const btScalar pushMax = P.pushMax;
	const int passes = P.passes;
	const bool useBias = P.useBias;
	Point* pts = &m_points[g.m_first];
	Body& A = m_bodies[pts[0].m_a];
	Body& B = m_bodies[pts[0].m_b];

	// current separation from the anchors' motion since the start of the step
	for (int k = 0; k < g.m_count; ++k)
	{
		Point& p = pts[k];
		const btVector3 dA = AD ? A.m_dp + A.m_dR * p.m_rA : p.m_rA;
		const btVector3 dB = BD ? B.m_dp + B.m_dR * p.m_rB : p.m_rB;
		const btScalar sep = p.m_baseSeparation + p.m_n.dot(dA - dB);
		p.m_separation = sep;
		// this pass's target: speculative (closing a gap), soft (pushing out an overlap) or rigid
		if (sep > 0)
		{
			p.m_bias = sep * invH;  // may close the gap this substep, no more
			p.m_massScale = 1;
			p.m_impulseScale = 0;
		}
		else if (useBias)
		{
			const Softness& sk = p.m_static ? staticSoft : soft;
			p.m_bias = btMax(sk.biasRate * sep, -pushMax);
			p.m_massScale = sk.massScale;
			p.m_impulseScale = sk.impulseScale;
		}
		else
		{
			p.m_bias = 0;
			p.m_massScale = 1;
			p.m_impulseScale = 0;
		}
	}

	// Normal impulses over the manifold's points; repeated passes converge to the solution of
	// the manifold as one block (each pass is a Gauss-Seidel step on the same soft, regularised
	// equations, so the softness does not change with the passes).
	for (int pass = 0; pass < passes; ++pass)
	{
		for (int k = 0; k < g.m_count; ++k)
		{
			Point& p = pts[k];
			const btScalar vn = p.m_n.dot(A.m_v - B.m_v) + p.m_rAxN.dot(A.m_w) - p.m_rBxN.dot(B.m_w);
			const btScalar impulse = -p.m_normalMass * p.m_massScale * (vn + p.m_bias) - p.m_impulseScale * p.m_normalImpulse;
			const btScalar next = btMax(p.m_normalImpulse + impulse, btScalar(0));
			const btScalar d = next - p.m_normalImpulse;
			p.m_normalImpulse = next;
			if (d > p.m_maxNormalImpulse) p.m_maxNormalImpulse = d;
			if (AD) A.m_v += A.m_invMass * (p.m_n * d);
			if (AD) A.m_w += p.m_aAn * d;
			if (BD) B.m_v -= B.m_invMass * (p.m_n * d);
			if (BD) B.m_w -= p.m_aBn * d;
		}
	}

	// friction, once the normal impulses have settled
	for (int k = 0; k < g.m_count; ++k)
	{
		Point& p = pts[k];
		// friction: both directions, limited together to the friction cone
		if (p.m_friction > 0)
		{
			const btVector3 dv = A.m_v - B.m_v;
			const btScalar vt1 = p.m_t1.dot(dv) + p.m_rAxT1.dot(A.m_w) - p.m_rBxT1.dot(B.m_w) - p.m_motion1;
			const btScalar vt2 = p.m_t2.dot(dv) + p.m_rAxT2.dot(A.m_w) - p.m_rBxT2.dot(B.m_w) - p.m_motion2;
			btScalar l1 = p.m_tangentImpulse1 - p.m_tangentMass1 * vt1;
			btScalar l2 = p.m_tangentImpulse2 - p.m_tangentMass2 * vt2;
			const btScalar maxF = p.m_friction * p.m_normalImpulse;
			const btScalar len2 = l1 * l1 + l2 * l2;
			if (len2 > maxF * maxF)
			{
				const btScalar scale = len2 > SIMD_EPSILON ? maxF / btSqrt(len2) : btScalar(0);
				l1 *= scale;
				l2 *= scale;
			}
			const btScalar d1 = l1 - p.m_tangentImpulse1;
			const btScalar d2 = l2 - p.m_tangentImpulse2;
			p.m_tangentImpulse1 = l1;
			p.m_tangentImpulse2 = l2;
			const btVector3 P = p.m_t1 * d1 + p.m_t2 * d2;
			if (AD) A.m_v += A.m_invMass * P;
			if (AD) A.m_w += p.m_aAt1 * d1 + p.m_aAt2 * d2;
			if (BD) B.m_v -= B.m_invMass * P;
			if (BD) B.m_w -= p.m_aBt1 * d1 + p.m_aBt2 * d2;
		}
	}

	for (int k = 0; k < g.m_count; ++k)
	{
		Point& p = pts[k];
		// rolling and spinning friction: angular, limited by the normal impulse
		if (p.m_rollingIndex >= 0)
		{
			Rolling& rl = m_rolling[p.m_rollingIndex];
			for (int a = 0; a < 3; ++a)
			{
				const btScalar coef = a == 0 ? p.m_spinning : p.m_rolling;
				if (coef <= 0 || rl.m_mass[a] == 0)
					continue;
				const btScalar wr = rl.m_axis[a].dot(A.m_w - B.m_w);
				const btScalar maxL = coef * p.m_normalImpulse;
				const btScalar next = clampScalar(rl.m_impulse[a] - rl.m_mass[a] * wr, -maxL, maxL);
				const btScalar d = next - rl.m_impulse[a];
				rl.m_impulse[a] = next;
				if (AD) A.m_w += rl.m_aA[a] * d;
				if (BD) B.m_w -= rl.m_aB[a] * d;
			}
		}
	}
}

void btSoftStepConstraintSolver::solveContacts(btScalar h, bool useBias)
{
	SoftParams P;
	P.invH = btScalar(1) / h;
	btScalar hertz = m_settings.m_contactHertz > 0 ? m_settings.m_contactHertz : btScalar(0.125) * P.invH;
	if (hertz > btScalar(0.25) * P.invH)
		hertz = btScalar(0.25) * P.invH;
	P.soft = makeSoft(hertz, m_settings.m_contactDampingRatio, h);
	P.staticSoft = makeSoft(btScalar(2) * hertz, m_settings.m_contactDampingRatio, h);
	P.pushMax = m_settings.m_contactPushMaxVelocity;
	P.passes = m_settings.m_manifoldIterations > 0 ? m_settings.m_manifoldIterations : 1;
	P.useBias = useBias;
	for (int gi = 0; gi < m_groups.size(); ++gi)
	{
		const Group& g = m_groups[gi];
		switch (g.m_kind)
		{
			case 3: solveGroupContacts<true, true>(g, &P); break;
			case 1: solveGroupContacts<true, false>(g, &P); break;
			default: solveGroupContacts<false, true>(g, &P); break;
		}
	}
}

void btSoftStepConstraintSolver::applyRestitution(const btContactSolverInfo& info)
{
	const btScalar threshold = info.m_restitutionVelocityThreshold;
	for (int i = 0; i < m_points.size(); ++i)
	{
		Point& p = m_points[i];
		if (p.m_restitution <= 0 || p.m_relativeVelocity > -threshold || p.m_maxNormalImpulse <= 0)
			continue;
		Body& A = m_bodies[p.m_a];
		Body& B = m_bodies[p.m_b];
		const btScalar vn = p.m_n.dot(A.m_v - B.m_v) + p.m_rAxN.dot(A.m_w) - p.m_rBxN.dot(B.m_w);
		const btScalar impulse = -p.m_normalMass * (vn + p.m_restitution * p.m_relativeVelocity);
		const btScalar next = btMax(p.m_normalImpulse + impulse, btScalar(0));
		const btScalar d = next - p.m_normalImpulse;
		p.m_normalImpulse = next;
		p.m_totalNormalImpulse += d;
		A.m_v += A.m_invMass * (p.m_n * d);
		A.m_w += p.m_aAn * d;
		B.m_v -= B.m_invMass * (p.m_n * d);
		B.m_w -= p.m_aBn * d;
	}
}

void btSoftStepConstraintSolver::storeResults(const btContactSolverInfo& info)
{
	// contacts: the step's total impulse for game code; the last substep's impulses, as forces
	// (so that a change of substeps keeps them right), for warm starting
	for (int i = 0; i < m_points.size(); ++i)
	{
		const Point& p = m_points[i];
		btManifoldPoint& cp = *p.m_cp;
		cp.m_appliedImpulse = p.m_totalNormalImpulse;
		cp.m_prevRHS = p.m_normalImpulse * m_invH;
		cp.m_appliedImpulseLateral1 = p.m_tangentImpulse1 * m_invH;
		cp.m_appliedImpulseLateral2 = p.m_tangentImpulse2 * m_invH;
	}
	// joints: summed impulses, breaking, feedback
	const btScalar invDt = btScalar(1) / info.m_timeStep;
	for (int i = 0; i < m_joints.size(); ++i)
	{
		const Joint& j = m_joints[i];
		btTypedConstraint* c = j.m_constraint;
		btJointFeedback* fb = c->getJointFeedback();
		bool broken = false;
		for (int r = 0; r < j.m_numRows; ++r)
		{
			const btSolverConstraint& row = m_rows[j.m_firstRow + r];
			const btScalar total = m_jointTotals[j.m_firstImpulse + r];
			if (fb)
			{
				fb->m_appliedForceBodyA += row.m_contactNormal1 * total * c->getRigidBodyA().getLinearFactor() * invDt;
				fb->m_appliedForceBodyB += row.m_contactNormal2 * total * c->getRigidBodyB().getLinearFactor() * invDt;
				fb->m_appliedTorqueBodyA += row.m_relpos1CrossNormal * c->getRigidBodyA().getAngularFactor() * total * invDt;
				fb->m_appliedTorqueBodyB += row.m_relpos2CrossNormal * c->getRigidBodyB().getAngularFactor() * total * invDt;
			}
			c->internalSetAppliedImpulse(total);
			if (btFabs(total) >= c->getBreakingImpulseThreshold())
				broken = true;
			c->m_bnWarmForce[r] = m_jointImpulses[j.m_firstImpulse + r] * m_invH;
		}
		c->m_bnWarmRows = j.m_numRows;
		if (broken)
		{
			c->setEnabled(false);
		}
	}
	// bodies: back to the start transform; the world moves them to the end transform
	for (int i = 0; i < m_bodies.size(); ++i)
	{
		Body& b = m_bodies[i];
		if (!b.m_dynamic)
			continue;
		btRigidBody* rb = b.m_body;
		rb->setInterpolationWorldTransform(btTransform(b.m_q, b.m_x));
		rb->setWorldTransform(b.m_start);
		rb->setLinearVelocity(b.m_v);
		rb->setAngularVelocity(b.m_w);
		rb->setCompanionId(-1);
	}
}

btScalar btSoftStepConstraintSolver::solveGroup(btCollisionObject** bodies, int numBodies, btPersistentManifold** manifolds, int numManifolds,
												btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info,
												btIDebugDraw* debugDrawer, btDispatcher* dispatcher)
{
	(void)debugDrawer;
	(void)dispatcher;
	BT_PROFILE("btSoftStepConstraintSolver::solveGroup");
	if (numBodies <= 0 && numManifolds <= 0 && numConstraints <= 0)
		return 0;
	btClock clock;
	const btScalar dt = info.m_timeStep;
	int substeps = m_settings.m_substeps;
	if (substeps <= 0)
	{
		// the fewest substeps that reach the substep rate (a 1% margin keeps 60 Hz at 4 for 240 Hz)
		const btScalar want = dt * m_settings.m_substepRate * btScalar(0.99);
		substeps = int(want);
		if (btScalar(substeps) < want) ++substeps;
		if (substeps < 1) substeps = 1;
	}
	const btScalar h = dt / btScalar(substeps);
	m_h = h;
	m_invH = btScalar(1) / h;
	m_warmStart = (info.m_solverMode & SOLVER_USE_WARMSTARTING) != 0;

	m_bodies.resizeNoInitialize(0);
	m_points.resizeNoInitialize(0);
	m_rolling.resizeNoInitialize(0);
	m_groups.resizeNoInitialize(0);
	m_joints.resizeNoInitialize(0);
	m_fixedBody = -1;
	for (int i = 0; i < m_kinematicUsed.size(); ++i)
		m_kinematicIndex[m_kinematicUsed[i]] = -1;
	m_kinematicUsed.resizeNoInitialize(0);
	m_maxGyroscopicForce = info.m_maxGyroscopicForce;
	for (int i = 0; i < numBodies; ++i)
		bodies[i]->setCompanionId(-1);
	for (int i = 0; i < numBodies; ++i)
		bodyIndex(bodies[i], dt);
	preparePoints(manifolds, numManifolds, info);
	prepareJoints(constraints, numConstraints);
	m_stats.m_prepareMicros += double(clock.getTimeMicroseconds());

	clock.reset();
	for (int step = 0; step < substeps; ++step)
	{
		integrateVelocities(h);
		if (m_joints.size())
		{
			syncJointBodies();
			buildJointRows(info, h);
		}
		warmStart();
		for (int it = 0; it < m_settings.m_iterations; ++it)
		{
			solveContacts(h, true);
			for (int k = 0; k < m_settings.m_jointIterations; ++k)
				solveJoints(true);
		}
		integratePositions(h);
		for (int r = 0; r < m_settings.m_relaxIterations; ++r)
		{
			// contacts first here: the joints, solved last, end the substep held together
			solveContacts(h, false);
			for (int k = 0; k < m_settings.m_jointIterations; ++k)
				solveJoints(false);
		}
		for (int i = 0; i < m_points.size(); ++i)
			m_points[i].m_totalNormalImpulse += m_points[i].m_normalImpulse;
		for (int i = 0; i < m_joints.size(); ++i)
		{
			const Joint& j = m_joints[i];
			for (int r = 0; r < j.m_numRows; ++r)
				m_jointTotals[j.m_firstImpulse + r] += m_jointImpulses[j.m_firstImpulse + r];
		}
	}
	m_stats.m_substepMicros += double(clock.getTimeMicroseconds());

	clock.reset();
	applyRestitution(info);
	m_stats.m_restitutionMicros += double(clock.getTimeMicroseconds());

	clock.reset();
	storeResults(info);
	m_stats.m_storeMicros += double(clock.getTimeMicroseconds());
	m_stats.m_groups += 1;
	m_stats.m_bodies += m_bodies.size();
	m_stats.m_contactPoints += m_points.size();
	return 0;
}
