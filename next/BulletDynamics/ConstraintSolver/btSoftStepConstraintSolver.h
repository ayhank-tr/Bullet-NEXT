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
*/

#ifndef BT_SOFT_STEP_CONSTRAINT_SOLVER_H
#define BT_SOFT_STEP_CONSTRAINT_SOLVER_H

#include "btConstraintSolver.h"
#include "btSolverConstraint.h"
#include "btTypedConstraint.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"

class btRigidBody;
class btManifoldPoint;

/// Settings of the Soft Step solver. The structure follows Box2D v3's soft step solver; the
/// defaults were chosen in the Lab (F3c): a 20-box stack and a 1:100 mass ratio held still, resting
/// boxes sinking no more than 2.5 mm into each other, joints at least as tight as Bullet's
/// sequential impulse solver at 10 iterations.
struct btSoftStepSettings
{
	int m_substeps;                  ///< solver substeps per world step (collision runs once per step);
	                                 ///< 0: as many as m_substepRate asks for
	btScalar m_substepRate;          ///< substeps per second when m_substeps is 0 (a 60 Hz step at 540 Hz: 9)
	btScalar m_contactHertz;         ///< contact stiffness, as a spring frequency (capped at a quarter of the substep
	                                 ///< rate); 0: an eighth of the substep rate (stiffer contacts sink less
	                                 ///< under load, up to a quarter of the rate they start to shake)
	btScalar m_contactDampingRatio;  ///< contact damping ratio
	btScalar m_contactPushMaxVelocity;  ///< fastest speed at which overlap is pushed out (m/s)
	btScalar m_jointHertz;           ///< joint position correction, as a spring frequency (capped at half
	                                 ///< the substep rate); 0: Bullet's ERP (the world's m_erp) on every
	                                 ///< substep. At 540 substeps per second a grid of 764 joints starts
	                                 ///< to gain energy above about 220 Hz (Stage 0: 120 -> 200 Hz)
	btScalar m_jointDampingRatio;    ///< joint correction damping ratio
	int m_iterations;                ///< solve passes per substep (with position bias)
	int m_relaxIterations;           ///< relax passes per substep (velocity only, no position bias)
	int m_manifoldIterations;        ///< passes over each contact manifold within a solve or relax pass
	int m_jointIterations;           ///< passes over the joints within a solve or relax pass
	btScalar m_jointWarmStart;       ///< share of a joint's impulses carried into the next step (contacts carry all)

	btSoftStepSettings()
		: m_substeps(0),
		  m_substepRate(540),
		  m_contactHertz(0),
		  m_contactDampingRatio(10),
		  m_contactPushMaxVelocity(3),
		  m_jointHertz(200),
		  m_jointDampingRatio(5),
		  m_iterations(1),
		  m_relaxIterations(1),
		  m_manifoldIterations(2),
		  m_jointIterations(1),
		  m_jointWarmStart(1)
	{
	}
};

/// Timings and counts of the last solveGroup calls, summed until resetStats().
struct btSoftStepStats
{
	double m_prepareMicros, m_substepMicros, m_restitutionMicros, m_storeMicros;
	int m_groups, m_bodies, m_contactPoints, m_jointRows;
	btSoftStepStats() { reset(); }
	void reset()
	{
		m_prepareMicros = m_substepMicros = m_restitutionMicros = m_storeMicros = 0;
		m_groups = m_bodies = m_contactPoints = m_jointRows = 0;
	}
};

/// Soft Step: a substepping solver for contacts and joints, selectable next to
/// btSequentialImpulseConstraintSolver (Bullet NEXT, F3c).
///
/// Collision detection still runs once per world step. Inside solveGroup the solver runs
/// m_substeps substeps; each one integrates velocities (forces, gravity), warm starts, solves
/// the contacts with soft, mass-normalised position correction ("bias"), integrates
/// positions, then relaxes (solves again without bias). One restitution pass follows the
/// substeps. Contacts keep their anchors from the start of the step, and their separation is
/// updated from the bodies' motion, so no new collision query is needed. Joints are rebuilt
/// every substep from the bodies' current transforms (getInfo1 / getInfo2).
///
/// The solver moves the bodies itself. It leaves each body at its start transform and stores
/// the end transform in the body's interpolation transform; btDiscreteDynamicsWorld
/// (getSolverType() == BT_SOFT_STEP_SOLVER) moves the body there in integrateTransforms,
/// so continuous collision detection still applies.
ATTRIBUTE_ALIGNED16(class)
btSoftStepConstraintSolver : public btConstraintSolver
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btSoftStepConstraintSolver();
	virtual ~btSoftStepConstraintSolver();

	virtual btScalar solveGroup(btCollisionObject * *bodies, int numBodies, btPersistentManifold** manifold, int numManifolds,
								btTypedConstraint** constraints, int numConstraints, const btContactSolverInfo& info,
								class btIDebugDraw* debugDrawer, btDispatcher* dispatcher);

	virtual void reset() {}

	virtual btConstraintSolverType getSolverType() const { return BT_SOFT_STEP_SOLVER; }

	void setSettings(const btSoftStepSettings& s) { m_settings = s; }
	const btSoftStepSettings& getSettings() const { return m_settings; }

	const btSoftStepStats& getStats() const { return m_stats; }
	void resetStats() { m_stats.reset(); }

	/// A body as the solver sees it during one step.
	struct Body
	{
		btVector3 m_v, m_w;                 // velocity
		btTransform m_start;                // transform at the start of the step, restored at the end
		btVector3 m_x0;                     // centre of mass at the start of the step
		btQuaternion m_q0;                  // orientation at the start of the step
		btVector3 m_x;                      // current centre of mass
		btQuaternion m_q;                   // current orientation
		btVector3 m_dp;                     // m_x - m_x0
		btMatrix3x3 m_dR;                   // rotation since the start of the step
		btMatrix3x3 m_invI;                 // world inverse inertia (start of step), axis locks applied
		btVector3 m_invMass;                // inverse mass per axis, axis locks applied
		btVector3 m_linAccel, m_angAccel;   // external forces and torques, as accelerations
		btVector3 m_localInertia;           // principal moments (gyroscopic term)
		btVector3 m_angFactor;
		btRigidBody* m_body;                // null for the shared fixed body
		int m_dynamic;                      // 1: moved by this solver
		int m_gyro;                         // the body's gyroscopic flags
		int m_stamp;                        // last joint sync
	};

private:
	/// One contact point. The angular responses (invI * (r x direction)) are fixed for the step,
	/// so the inner loops only take dot products and add vectors. Kept small: the passes walk
	/// every point several times per substep.
	struct Point
	{
		btVector3 m_n;                      // contact normal, on B, pointing at A
		btVector3 m_rA, m_rB;               // anchors: contact points relative to the centres of mass (start of step)
		btVector3 m_t1, m_t2;               // friction directions
		btVector3 m_rAxN, m_rBxN, m_rAxT1, m_rBxT1, m_rAxT2, m_rBxT2;    // angular Jacobians
		btVector3 m_aAn, m_aBn, m_aAt1, m_aBt1, m_aAt2, m_aBt2;          // angular responses
		btScalar m_baseSeparation;          // separation minus the anchors' offset along the normal
		btScalar m_separation;              // this pass
		btScalar m_bias, m_massScale, m_impulseScale;  // this pass's normal target
		btScalar m_normalMass, m_tangentMass1, m_tangentMass2;
		btScalar m_normalImpulse, m_tangentImpulse1, m_tangentImpulse2;
		btScalar m_totalNormalImpulse;
		btScalar m_totalTangentImpulse1, m_totalTangentImpulse2;  // Stage 1: the step's friction (crash report)
		btScalar m_maxNormalImpulse;
		btScalar m_relativeVelocity;        // normal velocity at the start of the step (negative: approaching)
		btScalar m_friction, m_restitution;
		btScalar m_rolling, m_spinning;
		btScalar m_motion1, m_motion2;      // conveyor motion along the friction directions
		btManifoldPoint* m_cp;
		int m_a, m_b;                       // body indices
		int m_static;                       // one side cannot move
		int m_rollingIndex;                 // into m_rolling, -1: no rolling or spinning friction
	};

	/// Rolling and spinning friction of a point that has them.
	struct Rolling
	{
		btVector3 m_axis[3];                // [0]: spinning (about the normal), [1] [2]: rolling
		btVector3 m_aA[3], m_aB[3];         // angular responses
		btScalar m_mass[3];
		btScalar m_impulse[3];
	};

	struct Joint
	{
		btTypedConstraint* m_constraint;
		int m_a, m_b;
		int m_firstRow, m_numRows;          // into m_rows / m_rowTarget0 (this substep)
		int m_prevRows;                     // rows last substep (warm start only when unchanged)
		int m_firstImpulse;                 // into m_jointImpulses: per-row impulse kept between substeps
		int m_maxRows;
		int m_rigid;                        // bit per row: an angular row keeps Bullet's ERP, not the soft spring
		// the rows without limits, solved together as one block: LDL^T factor of
		// K = J M^-1 J^T (+ Bullet's cfm on the diagonal)
		int m_blockCount;                   // 0: every row on its own
		int m_blockRow[6];                  // row offsets within the joint
		btScalar m_blockL[15];              // unit lower triangle below the diagonal, row by row
		btScalar m_blockInvDiag[6];
	};

	btSoftStepSettings m_settings;
	btSoftStepStats m_stats;

	btAlignedObjectArray<Body> m_bodies;
	btAlignedObjectArray<Point> m_points;
	btAlignedObjectArray<Rolling> m_rolling;
	struct Group
	{
		int m_first, m_count;               // the points of one manifold
		int m_kind;                         // 1: A moves, 2: B moves, 3: both
	};
	btAlignedObjectArray<Group> m_groups;
	btAlignedObjectArray<Joint> m_joints;
	btAlignedObjectArray<btSolverConstraint> m_rows;     // joint rows, with bias
	btAlignedObjectArray<btSolverConstraint> m_rowsNoBias;  // one joint's rows with erp = 0 (scratch)
	btAlignedObjectArray<btScalar> m_rowTarget0;         // per row: its target without position correction (relax)
	btAlignedObjectArray<btScalar> m_rowK;               // effective inverse mass per row (J M^-1 J^T diagonal)
	btAlignedObjectArray<btScalar> m_rowEps;             // per-row Bullet cfm, in units of K
	btScalar m_jointMassScale, m_jointImpulseScale;      // joint softness this substep
	int m_syncStamp;
	btAlignedObjectArray<btScalar> m_jointImpulses;      // per joint row, kept between substeps
	btAlignedObjectArray<btScalar> m_jointTotals;        // per joint row, summed over the step
	btAlignedObjectArray<int> m_kinematicIndex;          // world array index -> body index
	btAlignedObjectArray<int> m_kinematicUsed;           // entries of m_kinematicIndex set this step
	int m_fixedBody;
	btScalar m_maxGyroscopicForce;
	btScalar m_h, m_invH;   // this step's substep length
	bool m_warmStart;

	int bodyIndex(btCollisionObject * obj, btScalar timeStep);
	int addBody(btCollisionObject * obj, btScalar timeStep);
	void preparePoints(btPersistentManifold * *manifolds, int numManifolds, const btContactSolverInfo& info);
	void prepareJoints(btTypedConstraint * *constraints, int numConstraints);
	void buildJointRows(const btContactSolverInfo& info, btScalar h);
	void factorBlock(Joint & j);
	void integrateVelocities(btScalar h);
	void syncJointBodies();
	void integratePositions(btScalar h);
	void warmStart();
	void solveJoints(bool useBias);
	void solveContacts(btScalar h, bool useBias);
	template <bool AD, bool BD>
	void solveGroupContacts(const Group& g, const void* params);
	void applyRestitution(const btContactSolverInfo& info);
	void storeResults(const btContactSolverInfo& info);
};

#endif  // BT_SOFT_STEP_CONSTRAINT_SOLVER_H
