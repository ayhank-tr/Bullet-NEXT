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
///btSoftBody implementation by Nathanael Presson

#ifndef _BT_SOFT_BODY_H
#define _BT_SOFT_BODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "btSparseSDF.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"

//#ifdef BT_USE_DOUBLE_PRECISION
//#define btRigidBodyData	btRigidBodyDoubleData
//#define btRigidBodyDataName	"btRigidBodyDoubleData"
//#else
#define btlSoftBodyData	btlSoftBodyFloatData
#define btlSoftBodyDataName	"btSoftBodyFloatData"
//#endif //BT_USE_DOUBLE_PRECISION

class btlBroadphaseInterface;
class btlDispatcher;
class btlSoftBodySolver;

/* btSoftBodyWorldInfo	*/ 
struct	btlSoftBodyWorldInfo
{
	btlScalar				air_density;
	btlScalar				water_density;
	btlScalar				water_offset;
	btlScalar				m_maxDisplacement;
	btlVector3				water_normal;
	btlBroadphaseInterface*	m_broadphase;
	btlDispatcher*	m_dispatcher;
	btlVector3				m_gravity;
	btlSparseSdf<3>			m_sparsesdf;

	btlSoftBodyWorldInfo()
		:air_density((btlScalar)1.2),
		water_density(0),
		water_offset(0),
		m_maxDisplacement(1000.f),//avoid soft body from 'exploding' so use some upper threshold of maximum motion that a node can travel per frame
		water_normal(0,0,0),
		m_broadphase(0),
		m_dispatcher(0),
		m_gravity(0,-10,0)
	{
	}
};	


///The btSoftBody is an class to simulate cloth and volumetric soft bodies. 
///There is two-way interaction between btSoftBody and btRigidBody/btCollisionObject.
class	btlSoftBody : public btlCollisionObject
{
public:
	btlAlignedObjectArray<const class btlCollisionObject*> m_collisionDisabledObjects;

	// The solver object that handles this soft body
	btlSoftBodySolver *m_softBodySolver;

	//
	// Enumerations
	//

	///eAeroModel 
	struct eAeroModel { enum _ {
		V_Point,			///Vertex normals are oriented toward velocity
		V_TwoSided,			///Vertex normals are flipped to match velocity	
		V_TwoSidedLiftDrag, ///Vertex normals are flipped to match velocity and lift and drag forces are applied
		V_OneSided,			///Vertex normals are taken as it is	
		F_TwoSided,			///Face normals are flipped to match velocity
		F_TwoSidedLiftDrag,	///Face normals are flipped to match velocity and lift and drag forces are applied 
		F_OneSided,			///Face normals are taken as it is		
		END
	};};

	///eVSolver : velocities solvers
	struct	eVSolver { enum _ {
		Linear,		///Linear solver
		END
	};};

	///ePSolver : positions solvers
	struct	ePSolver { enum _ {
		Linear,		///Linear solver
		Anchors,	///Anchor solver
		RContacts,	///Rigid contacts solver
		SContacts,	///Soft contacts solver
		END
	};};

	///eSolverPresets
	struct	eSolverPresets { enum _ {
		Positions,
		Velocities,
		Default	=	Positions,
		END
	};};

	///eFeature
	struct	eFeature { enum _ {
		None,
		Node,
		Link,
		Face,
		Tetra,
		END
	};};

	typedef btlAlignedObjectArray<eVSolver::_>	tVSolverArray;
	typedef btlAlignedObjectArray<ePSolver::_>	tPSolverArray;

	//
	// Flags
	//

	///fCollision
	struct fCollision { enum _ {
		RVSmask	=	0x000f,	///Rigid versus soft mask
		SDF_RS	=	0x0001,	///SDF based rigid vs soft
		CL_RS	=	0x0002, ///Cluster vs convex rigid vs soft

		SVSmask	=	0x0030,	///Rigid versus soft mask		
		VF_SS	=	0x0010,	///Vertex vs face soft vs soft handling
		CL_SS	=	0x0020, ///Cluster vs cluster soft vs soft handling
		CL_SELF =	0x0040, ///Cluster soft body self collision
		/* presets	*/ 
		Default	=	SDF_RS,
		END
	};};

	///fMaterial
	struct fMaterial { enum _ {
		DebugDraw	=	0x0001,	/// Enable debug draw
		/* presets	*/ 
		Default		=	DebugDraw,
		END
	};};

	//
	// API Types
	//

	/* sRayCast		*/ 
	struct sRayCast
	{
		btlSoftBody*	body;		/// soft body
		eFeature::_	feature;	/// feature type
		int			index;		/// feature index
		btlScalar	fraction;		/// time of impact fraction (rayorg+(rayto-rayfrom)*fraction)
	};

	/* ImplicitFn	*/ 
	struct	ImplicitFn
	{
		virtual ~ImplicitFn() {}
		virtual btlScalar	Eval(const btlVector3& x)=0;
	};

	//
	// Internal types
	//

	typedef btlAlignedObjectArray<btlScalar>	tScalarArray;
	typedef btlAlignedObjectArray<btlVector3>	tVector3Array;

	/* sCti is Softbody contact info	*/ 
	struct	sCti
	{
		const btlCollisionObject*	m_colObj;		/* Rigid body			*/ 
		btlVector3		m_normal;	/* Outward normal		*/ 
		btlScalar		m_offset;	/* Offset from origin	*/ 
	};	

	/* sMedium		*/ 
	struct	sMedium
	{
		btlVector3		m_velocity;	/* Velocity				*/ 
		btlScalar		m_pressure;	/* Pressure				*/ 
		btlScalar		m_density;	/* Density				*/ 
	};

	/* Base type	*/ 
	struct	Element
	{
		void*			m_tag;			// User data
		Element() : m_tag(0) {}
	};
	/* Material		*/ 
	struct	Material : Element
	{
		btlScalar				m_kLST;			// Linear stiffness coefficient [0,1]
		btlScalar				m_kAST;			// Area/Angular stiffness coefficient [0,1]
		btlScalar				m_kVST;			// Volume stiffness coefficient [0,1]
		int						m_flags;		// Flags
	};

	/* Feature		*/ 
	struct	Feature : Element
	{
		Material*				m_material;		// Material
	};
	/* Node			*/ 
	struct	Node : Feature
	{
		btlVector3				m_x;			// Position
		btlVector3				m_q;			// Previous step position
		btlVector3				m_v;			// Velocity
		btlVector3				m_f;			// Force accumulator
		btlVector3				m_n;			// Normal
		btlScalar				m_im;			// 1/mass
		btlScalar				m_area;			// Area
		btlDbvtNode*				m_leaf;			// Leaf data
		int						m_battach:1;	// Attached
	};
	/* Link			*/ 
	struct	Link : Feature
	{
		Node*					m_n[2];			// Node pointers
		btlScalar				m_rl;			// Rest length		
		int						m_bbending:1;	// Bending link
		btlScalar				m_c0;			// (ima+imb)*kLST
		btlScalar				m_c1;			// rl^2
		btlScalar				m_c2;			// |gradient|^2/c0
		btlVector3				m_c3;			// gradient
	};
	/* Face			*/ 
	struct	Face : Feature
	{
		Node*					m_n[3];			// Node pointers
		btlVector3				m_normal;		// Normal
		btlScalar				m_ra;			// Rest area
		btlDbvtNode*				m_leaf;			// Leaf data
	};
	/* Tetra		*/ 
	struct	Tetra : Feature
	{
		Node*					m_n[4];			// Node pointers		
		btlScalar				m_rv;			// Rest volume
		btlDbvtNode*				m_leaf;			// Leaf data
		btlVector3				m_c0[4];		// gradients
		btlScalar				m_c1;			// (4*kVST)/(im0+im1+im2+im3)
		btlScalar				m_c2;			// m_c1/sum(|g0..3|^2)
	};
	/* RContact		*/ 
	struct	RContact
	{
		sCti		m_cti;			// Contact infos
		Node*					m_node;			// Owner node
		btlMatrix3x3				m_c0;			// Impulse matrix
		btlVector3				m_c1;			// Relative anchor
		btlScalar				m_c2;			// ima*dt
		btlScalar				m_c3;			// Friction
		btlScalar				m_c4;			// Hardness
	};
	/* SContact		*/ 
	struct	SContact
	{
		Node*					m_node;			// Node
		Face*					m_face;			// Face
		btlVector3				m_weights;		// Weigths
		btlVector3				m_normal;		// Normal
		btlScalar				m_margin;		// Margin
		btlScalar				m_friction;		// Friction
		btlScalar				m_cfm[2];		// Constraint force mixing
	};
	/* Anchor		*/ 
	struct	Anchor
	{
		Node*					m_node;			// Node pointer
		btlVector3				m_local;		// Anchor position in body space
		btlRigidBody*			m_body;			// Body
		btlScalar				m_influence;
		btlMatrix3x3				m_c0;			// Impulse matrix
		btlVector3				m_c1;			// Relative anchor
		btlScalar				m_c2;			// ima*dt
	};
	/* Note			*/ 
	struct	Note : Element
	{
		const char*				m_text;			// Text
		btlVector3				m_offset;		// Offset
		int						m_rank;			// Rank
		Node*					m_nodes[4];		// Nodes
		btlScalar				m_coords[4];	// Coordinates
	};	
	/* Pose			*/ 
	struct	Pose
	{
		bool					m_bvolume;		// Is valid
		bool					m_bframe;		// Is frame
		btlScalar				m_volume;		// Rest volume
		tVector3Array			m_pos;			// Reference positions
		tScalarArray			m_wgh;			// Weights
		btlVector3				m_com;			// COM
		btlMatrix3x3				m_rot;			// Rotation
		btlMatrix3x3				m_scl;			// Scale
		btlMatrix3x3				m_aqq;			// Base scaling
	};
	/* Cluster		*/ 
	struct	Cluster
	{
		tScalarArray				m_masses;
		btlAlignedObjectArray<Node*>	m_nodes;		
		tVector3Array				m_framerefs;
		btlTransform					m_framexform;
		btlScalar					m_idmass;
		btlScalar					m_imass;
		btlMatrix3x3					m_locii;
		btlMatrix3x3					m_invwi;
		btlVector3					m_com;
		btlVector3					m_vimpulses[2];
		btlVector3					m_dimpulses[2];
		int							m_nvimpulses;
		int							m_ndimpulses;
		btlVector3					m_lv;
		btlVector3					m_av;
		btlDbvtNode*					m_leaf;
		btlScalar					m_ndamping;	/* Node damping		*/ 
		btlScalar					m_ldamping;	/* Linear damping	*/ 
		btlScalar					m_adamping;	/* Angular damping	*/ 
		btlScalar					m_matching;
		btlScalar					m_maxSelfCollisionImpulse;
		btlScalar					m_selfCollisionImpulseFactor;
		bool						m_containsAnchor;
		bool						m_collide;
		int							m_clusterIndex;
		Cluster() : m_leaf(0),m_ndamping(0),m_ldamping(0),m_adamping(0),m_matching(0) 
		,m_maxSelfCollisionImpulse(100.f),
		m_selfCollisionImpulseFactor(0.01f),
		m_containsAnchor(false)
		{}
	};
	/* Impulse		*/ 
	struct	Impulse
	{
		btlVector3					m_velocity;
		btlVector3					m_drift;
		int							m_asVelocity:1;
		int							m_asDrift:1;
		Impulse() : m_velocity(0,0,0),m_drift(0,0,0),m_asVelocity(0),m_asDrift(0)	{}
		Impulse						operator -() const
		{
			Impulse i=*this;
			i.m_velocity=-i.m_velocity;
			i.m_drift=-i.m_drift;
			return(i);
		}
		Impulse						operator*(btlScalar x) const
		{
			Impulse i=*this;
			i.m_velocity*=x;
			i.m_drift*=x;
			return(i);
		}
	};
	/* Body			*/ 
	struct	Body
	{
		Cluster*			m_soft;
		btlRigidBody*		m_rigid;
		const btlCollisionObject*	m_collisionObject;

		Body() : m_soft(0),m_rigid(0),m_collisionObject(0)				{}
		Body(Cluster* p) : m_soft(p),m_rigid(0),m_collisionObject(0)	{}
		Body(const btlCollisionObject* colObj) : m_soft(0),m_collisionObject(colObj)
		{
			m_rigid = (btlRigidBody*)btlRigidBody::upcast(m_collisionObject);
		}

		void						activate() const
		{
			if(m_rigid) 
				m_rigid->activate();
			if (m_collisionObject)
				m_collisionObject->activate();

		}
		const btlMatrix3x3&			invWorldInertia() const
		{
			static const btlMatrix3x3	iwi(0,0,0,0,0,0,0,0,0);
			if(m_rigid) return(m_rigid->getInvInertiaTensorWorld());
			if(m_soft)	return(m_soft->m_invwi);
			return(iwi);
		}
		btlScalar					invMass() const
		{
			if(m_rigid) return(m_rigid->getInvMass());
			if(m_soft)	return(m_soft->m_imass);
			return(0);
		}
		const btlTransform&			xform() const
		{
			static const btlTransform	identity=btlTransform::getIdentity();		
			if(m_collisionObject) return(m_collisionObject->getWorldTransform());
			if(m_soft)	return(m_soft->m_framexform);
			return(identity);
		}
		btlVector3					linearVelocity() const
		{
			if(m_rigid) return(m_rigid->getLinearVelocity());
			if(m_soft)	return(m_soft->m_lv);
			return(btlVector3(0,0,0));
		}
		btlVector3					angularVelocity(const btlVector3& rpos) const
		{			
			if(m_rigid) return(btlCross(m_rigid->getAngularVelocity(),rpos));
			if(m_soft)	return(btlCross(m_soft->m_av,rpos));
			return(btlVector3(0,0,0));
		}
		btlVector3					angularVelocity() const
		{			
			if(m_rigid) return(m_rigid->getAngularVelocity());
			if(m_soft)	return(m_soft->m_av);
			return(btlVector3(0,0,0));
		}
		btlVector3					velocity(const btlVector3& rpos) const
		{
			return(linearVelocity()+angularVelocity(rpos));
		}
		void						applyVImpulse(const btlVector3& impulse,const btlVector3& rpos) const
		{
			if(m_rigid)	m_rigid->applyImpulse(impulse,rpos);
			if(m_soft)	btlSoftBody::clusterVImpulse(m_soft,rpos,impulse);
		}
		void						applyDImpulse(const btlVector3& impulse,const btlVector3& rpos) const
		{
			if(m_rigid)	m_rigid->applyImpulse(impulse,rpos);
			if(m_soft)	btlSoftBody::clusterDImpulse(m_soft,rpos,impulse);
		}		
		void						applyImpulse(const Impulse& impulse,const btlVector3& rpos) const
		{
			if(impulse.m_asVelocity)	
			{
//				printf("impulse.m_velocity = %f,%f,%f\n",impulse.m_velocity.getX(),impulse.m_velocity.getY(),impulse.m_velocity.getZ());
				applyVImpulse(impulse.m_velocity,rpos);
			}
			if(impulse.m_asDrift)		
			{
//				printf("impulse.m_drift = %f,%f,%f\n",impulse.m_drift.getX(),impulse.m_drift.getY(),impulse.m_drift.getZ());
				applyDImpulse(impulse.m_drift,rpos);
			}
		}
		void						applyVAImpulse(const btlVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyTorqueImpulse(impulse);
			if(m_soft)	btlSoftBody::clusterVAImpulse(m_soft,impulse);
		}
		void						applyDAImpulse(const btlVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyTorqueImpulse(impulse);
			if(m_soft)	btlSoftBody::clusterDAImpulse(m_soft,impulse);
		}
		void						applyAImpulse(const Impulse& impulse) const
		{
			if(impulse.m_asVelocity)	applyVAImpulse(impulse.m_velocity);
			if(impulse.m_asDrift)		applyDAImpulse(impulse.m_drift);
		}
		void						applyDCImpulse(const btlVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyCentralImpulse(impulse);
			if(m_soft)	btlSoftBody::clusterDCImpulse(m_soft,impulse);
		}
	};
	/* Joint		*/ 
	struct	Joint
	{
		struct eType { enum _ {
			Linear=0,
			Angular,
			Contact
		};};
		struct Specs
		{
			Specs() : erp(1),cfm(1),split(1) {}
			btlScalar	erp;
			btlScalar	cfm;
			btlScalar	split;
		};
		Body						m_bodies[2];
		btlVector3					m_refs[2];
		btlScalar					m_cfm;
		btlScalar					m_erp;
		btlScalar					m_split;
		btlVector3					m_drift;
		btlVector3					m_sdrift;
		btlMatrix3x3					m_massmatrix;
		bool						m_delete;
		virtual						~Joint() {}
		Joint() : m_delete(false) {}
		virtual void				Prepare(btlScalar dt,int iterations);
		virtual void				Solve(btlScalar dt,btlScalar sor)=0;
		virtual void				Terminate(btlScalar dt)=0;
		virtual eType::_			Type() const=0;
	};
	/* LJoint		*/ 
	struct	LJoint : Joint
	{
		struct Specs : Joint::Specs
		{
			btlVector3	position;
		};		
		btlVector3					m_rpos[2];
		void						Prepare(btlScalar dt,int iterations);
		void						Solve(btlScalar dt,btlScalar sor);
		void						Terminate(btlScalar dt);
		eType::_					Type() const { return(eType::Linear); }
	};
	/* AJoint		*/ 
	struct	AJoint : Joint
	{
		struct IControl
		{
			virtual ~IControl() {}
			virtual void			Prepare(AJoint*)				{}
			virtual btlScalar		Speed(AJoint*,btlScalar current) { return(current); }
			static IControl*		Default()						{ static IControl def;return(&def); }
		};
		struct Specs : Joint::Specs
		{
			Specs() : icontrol(IControl::Default()) {}
			btlVector3	axis;
			IControl*	icontrol;
		};		
		btlVector3					m_axis[2];
		IControl*					m_icontrol;
		void						Prepare(btlScalar dt,int iterations);
		void						Solve(btlScalar dt,btlScalar sor);
		void						Terminate(btlScalar dt);
		eType::_					Type() const { return(eType::Angular); }
	};
	/* CJoint		*/ 
	struct	CJoint : Joint
	{		
		int							m_life;
		int							m_maxlife;
		btlVector3					m_rpos[2];
		btlVector3					m_normal;
		btlScalar					m_friction;
		void						Prepare(btlScalar dt,int iterations);
		void						Solve(btlScalar dt,btlScalar sor);
		void						Terminate(btlScalar dt);
		eType::_					Type() const { return(eType::Contact); }
	};
	/* Config		*/ 
	struct	Config
	{
		eAeroModel::_			aeromodel;		// Aerodynamic model (default: V_Point)
		btlScalar				kVCF;			// Velocities correction factor (Baumgarte)
		btlScalar				kDP;			// Damping coefficient [0,1]
		btlScalar				kDG;			// Drag coefficient [0,+inf]
		btlScalar				kLF;			// Lift coefficient [0,+inf]
		btlScalar				kPR;			// Pressure coefficient [-inf,+inf]
		btlScalar				kVC;			// Volume conversation coefficient [0,+inf]
		btlScalar				kDF;			// Dynamic friction coefficient [0,1]
		btlScalar				kMT;			// Pose matching coefficient [0,1]		
		btlScalar				kCHR;			// Rigid contacts hardness [0,1]
		btlScalar				kKHR;			// Kinetic contacts hardness [0,1]
		btlScalar				kSHR;			// Soft contacts hardness [0,1]
		btlScalar				kAHR;			// Anchors hardness [0,1]
		btlScalar				kSRHR_CL;		// Soft vs rigid hardness [0,1] (cluster only)
		btlScalar				kSKHR_CL;		// Soft vs kinetic hardness [0,1] (cluster only)
		btlScalar				kSSHR_CL;		// Soft vs soft hardness [0,1] (cluster only)
		btlScalar				kSR_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
		btlScalar				kSK_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
		btlScalar				kSS_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster only)
		btlScalar				maxvolume;		// Maximum volume ratio for pose
		btlScalar				timescale;		// Time scale
		int						viterations;	// Velocities solver iterations
		int						piterations;	// Positions solver iterations
		int						diterations;	// Drift solver iterations
		int						citerations;	// Cluster solver iterations
		int						collisions;		// Collisions flags
		tVSolverArray			m_vsequence;	// Velocity solvers sequence
		tPSolverArray			m_psequence;	// Position solvers sequence
		tPSolverArray			m_dsequence;	// Drift solvers sequence
	};
	/* SolverState	*/ 
	struct	SolverState
	{
		btlScalar				sdt;			// dt*timescale
		btlScalar				isdt;			// 1/sdt
		btlScalar				velmrg;			// velocity margin
		btlScalar				radmrg;			// radial margin
		btlScalar				updmrg;			// Update margin
	};	
	/// RayFromToCaster takes a ray from, ray to (instead of direction!)
	struct	RayFromToCaster : btlDbvt::ICollide
	{
		btlVector3			m_rayFrom;
		btlVector3			m_rayTo;
		btlVector3			m_rayNormalizedDirection;
		btlScalar			m_mint;
		Face*				m_face;
		int					m_tests;
		RayFromToCaster(const btlVector3& rayFrom,const btlVector3& rayTo,btlScalar mxt);
		void					Process(const btlDbvtNode* leaf);

		static inline btlScalar	rayFromToTriangle(const btlVector3& rayFrom,
			const btlVector3& rayTo,
			const btlVector3& rayNormalizedDirection,
			const btlVector3& a,
			const btlVector3& b,
			const btlVector3& c,
			btlScalar maxt=SIMD_INFINITY);
	};

	//
	// Typedefs
	//

	typedef void								(*psolver_t)(btlSoftBody*,btlScalar,btlScalar);
	typedef void								(*vsolver_t)(btlSoftBody*,btlScalar);
	typedef btlAlignedObjectArray<Cluster*>		tClusterArray;
	typedef btlAlignedObjectArray<Note>			tNoteArray;
	typedef btlAlignedObjectArray<Node>			tNodeArray;
	typedef btlAlignedObjectArray<btlDbvtNode*>	tLeafArray;
	typedef btlAlignedObjectArray<Link>			tLinkArray;
	typedef btlAlignedObjectArray<Face>			tFaceArray;
	typedef btlAlignedObjectArray<Tetra>			tTetraArray;
	typedef btlAlignedObjectArray<Anchor>		tAnchorArray;
	typedef btlAlignedObjectArray<RContact>		tRContactArray;
	typedef btlAlignedObjectArray<SContact>		tSContactArray;
	typedef btlAlignedObjectArray<Material*>		tMaterialArray;
	typedef btlAlignedObjectArray<Joint*>		tJointArray;
	typedef btlAlignedObjectArray<btlSoftBody*>	tSoftBodyArray;	

	//
	// Fields
	//

	Config					m_cfg;			// Configuration
	SolverState				m_sst;			// Solver state
	Pose					m_pose;			// Pose
	void*					m_tag;			// User data
	btlSoftBodyWorldInfo*	m_worldInfo;	// World info
	tNoteArray				m_notes;		// Notes
	tNodeArray				m_nodes;		// Nodes
	tLinkArray				m_links;		// Links
	tFaceArray				m_faces;		// Faces
	tTetraArray				m_tetras;		// Tetras
	tAnchorArray			m_anchors;		// Anchors
	tRContactArray			m_rcontacts;	// Rigid contacts
	tSContactArray			m_scontacts;	// Soft contacts
	tJointArray				m_joints;		// Joints
	tMaterialArray			m_materials;	// Materials
	btlScalar				m_timeacc;		// Time accumulator
	btlVector3				m_bounds[2];	// Spatial bounds	
	bool					m_bUpdateRtCst;	// Update runtime constants
	btlDbvt					m_ndbvt;		// Nodes tree
	btlDbvt					m_fdbvt;		// Faces tree
	btlDbvt					m_cdbvt;		// Clusters tree
	tClusterArray			m_clusters;		// Clusters

	btlAlignedObjectArray<bool>m_clusterConnectivity;//cluster connectivity, for self-collision

	btlTransform			m_initialWorldTransform;

	btlVector3			m_windVelocity;
	
	btlScalar        m_restLengthScale;
	
	//
	// Api
	//

	/* ctor																	*/ 
	btlSoftBody(	btlSoftBodyWorldInfo* worldInfo,int node_count,		const btlVector3* x,		const btlScalar* m);

	/* ctor																	*/ 
	btlSoftBody(	btlSoftBodyWorldInfo* worldInfo);

	void	initDefaults();

	/* dtor																	*/ 
	virtual ~btlSoftBody();
	/* Check for existing link												*/ 

	btlAlignedObjectArray<int>	m_userIndexMapping;

	btlSoftBodyWorldInfo*	getWorldInfo()
	{
		return m_worldInfo;
	}

	///@todo: avoid internal softbody shape hack and move collision code to collision library
	virtual void	setCollisionShape(btlCollisionShape* collisionShape)
	{
		
	}

	bool				checkLink(	int node0,
		int node1) const;
	bool				checkLink(	const Node* node0,
		const Node* node1) const;
	/* Check for existring face												*/ 
	bool				checkFace(	int node0,
		int node1,
		int node2) const;
	/* Append material														*/ 
	Material*			appendMaterial();
	/* Append note															*/ 
	void				appendNote(	const char* text,
		const btlVector3& o,
		const btlVector4& c=btlVector4(1,0,0,0),
		Node* n0=0,
		Node* n1=0,
		Node* n2=0,
		Node* n3=0);
	void				appendNote(	const char* text,
		const btlVector3& o,
		Node* feature);
	void				appendNote(	const char* text,
		const btlVector3& o,
		Link* feature);
	void				appendNote(	const char* text,
		const btlVector3& o,
		Face* feature);
	/* Append node															*/ 
	void				appendNode(	const btlVector3& x,btlScalar m);
	/* Append link															*/ 
	void				appendLink(int model=-1,Material* mat=0);
	void				appendLink(	int node0,
		int node1,
		Material* mat=0,
		bool bcheckexist=false);
	void				appendLink(	Node* node0,
		Node* node1,
		Material* mat=0,
		bool bcheckexist=false);
	/* Append face															*/ 
	void				appendFace(int model=-1,Material* mat=0);
	void				appendFace(	int node0,
		int node1,
		int node2,
		Material* mat=0);
	void			appendTetra(int model,Material* mat);
	//
	void			appendTetra(int node0,
										int node1,
										int node2,
										int node3,
										Material* mat=0);


	/* Append anchor														*/ 
	void				appendAnchor(	int node,
		btlRigidBody* body, bool disableCollisionBetweenLinkedBodies=false,btlScalar influence = 1);
	void			appendAnchor(int node,btlRigidBody* body, const btlVector3& localPivot,bool disableCollisionBetweenLinkedBodies=false,btlScalar influence = 1);
	/* Append linear joint													*/ 
	void				appendLinearJoint(const LJoint::Specs& specs,Cluster* body0,Body body1);
	void				appendLinearJoint(const LJoint::Specs& specs,Body body=Body());
	void				appendLinearJoint(const LJoint::Specs& specs,btlSoftBody* body);
	/* Append linear joint													*/ 
	void				appendAngularJoint(const AJoint::Specs& specs,Cluster* body0,Body body1);
	void				appendAngularJoint(const AJoint::Specs& specs,Body body=Body());
	void				appendAngularJoint(const AJoint::Specs& specs,btlSoftBody* body);
	/* Add force (or gravity) to the entire body							*/ 
	void				addForce(		const btlVector3& force);
	/* Add force (or gravity) to a node of the body							*/ 
	void				addForce(		const btlVector3& force,
		int node);
	/* Add aero force to a node of the body */
	void			    addAeroForceToNode(const btlVector3& windVelocity,int nodeIndex);

	/* Add aero force to a face of the body */
	void			    addAeroForceToFace(const btlVector3& windVelocity,int faceIndex);

	/* Add velocity to the entire body										*/ 
	void				addVelocity(	const btlVector3& velocity);

	/* Set velocity for the entire body										*/ 
	void				setVelocity(	const btlVector3& velocity);

	/* Add velocity to a node of the body									*/ 
	void				addVelocity(	const btlVector3& velocity,
		int node);
	/* Set mass																*/ 
	void				setMass(		int node,
		btlScalar mass);
	/* Get mass																*/ 
	btlScalar			getMass(		int node) const;
	/* Get total mass														*/ 
	btlScalar			getTotalMass() const;
	/* Set total mass (weighted by previous masses)							*/ 
	void				setTotalMass(	btlScalar mass,
		bool fromfaces=false);
	/* Set total density													*/ 
	void				setTotalDensity(btlScalar density);
	/* Set volume mass (using tetrahedrons)									*/
	void				setVolumeMass(		btlScalar mass);
	/* Set volume density (using tetrahedrons)								*/
	void				setVolumeDensity(	btlScalar density);
	/* Transform															*/ 
	void				transform(		const btlTransform& trs);
	/* Translate															*/ 
	void				translate(		const btlVector3& trs);
	/* Rotate															*/ 
	void				rotate(	const btlQuaternion& rot);
	/* Scale																*/ 
	void				scale(	const btlVector3& scl);
	/* Get link resting lengths scale										*/
	btlScalar			getRestLengthScale();
	/* Scale resting length of all springs									*/
	void				setRestLengthScale(btlScalar restLength);
	/* Set current state as pose											*/ 
	void				setPose(		bool bvolume,
		bool bframe);
	/* Set current link lengths as resting lengths							*/ 
	void				resetLinkRestLengths();
	/* Return the volume													*/ 
	btlScalar			getVolume() const;
	/* Cluster count														*/ 
	int					clusterCount() const;
	/* Cluster center of mass												*/ 
	static btlVector3	clusterCom(const Cluster* cluster);
	btlVector3			clusterCom(int cluster) const;
	/* Cluster velocity at rpos												*/ 
	static btlVector3	clusterVelocity(const Cluster* cluster,const btlVector3& rpos);
	/* Cluster impulse														*/ 
	static void			clusterVImpulse(Cluster* cluster,const btlVector3& rpos,const btlVector3& impulse);
	static void			clusterDImpulse(Cluster* cluster,const btlVector3& rpos,const btlVector3& impulse);
	static void			clusterImpulse(Cluster* cluster,const btlVector3& rpos,const Impulse& impulse);
	static void			clusterVAImpulse(Cluster* cluster,const btlVector3& impulse);
	static void			clusterDAImpulse(Cluster* cluster,const btlVector3& impulse);
	static void			clusterAImpulse(Cluster* cluster,const Impulse& impulse);
	static void			clusterDCImpulse(Cluster* cluster,const btlVector3& impulse);
	/* Generate bending constraints based on distance in the adjency graph	*/ 
	int					generateBendingConstraints(	int distance,
		Material* mat=0);
	/* Randomize constraints to reduce solver bias							*/ 
	void				randomizeConstraints();
	/* Release clusters														*/ 
	void				releaseCluster(int index);
	void				releaseClusters();
	/* Generate clusters (K-mean)											*/ 
	///generateClusters with k=0 will create a convex cluster for each tetrahedron or triangle
	///otherwise an approximation will be used (better performance)
	int					generateClusters(int k,int maxiterations=8192);
	/* Refine																*/ 
	void				refine(ImplicitFn* ifn,btlScalar accurary,bool cut);
	/* CutLink																*/ 
	bool				cutLink(int node0,int node1,btlScalar position);
	bool				cutLink(const Node* node0,const Node* node1,btlScalar position);

	///Ray casting using rayFrom and rayTo in worldspace, (not direction!)
	bool				rayTest(const btlVector3& rayFrom,
		const btlVector3& rayTo,
		sRayCast& results);
	/* Solver presets														*/ 
	void				setSolver(eSolverPresets::_ preset);
	/* predictMotion														*/ 
	void				predictMotion(btlScalar dt);
	/* solveConstraints														*/ 
	void				solveConstraints();
	/* staticSolve															*/ 
	void				staticSolve(int iterations);
	/* solveCommonConstraints												*/ 
	static void			solveCommonConstraints(btlSoftBody** bodies,int count,int iterations);
	/* solveClusters														*/ 
	static void			solveClusters(const btlAlignedObjectArray<btlSoftBody*>& bodies);
	/* integrateMotion														*/ 
	void				integrateMotion();
	/* defaultCollisionHandlers												*/ 
	void				defaultCollisionHandler(const btlCollisionObjectWrapper* pcoWrap);
	void				defaultCollisionHandler(btlSoftBody* psb);



	//
	// Functionality to deal with new accelerated solvers.
	//

	/**
	 * Set a wind velocity for interaction with the air.
	 */
	void setWindVelocity( const btlVector3 &velocity );


	/**
	 * Return the wind velocity for interaction with the air.
	 */
	const btlVector3& getWindVelocity();

	//
	// Set the solver that handles this soft body
	// Should not be allowed to get out of sync with reality
	// Currently called internally on addition to the world
	void setSoftBodySolver( btlSoftBodySolver *softBodySolver )
	{
		m_softBodySolver = softBodySolver;
	}

	//
	// Return the solver that handles this soft body
	// 
	btlSoftBodySolver *getSoftBodySolver()
	{
		return m_softBodySolver;
	}

	//
	// Return the solver that handles this soft body
	// 
	btlSoftBodySolver *getSoftBodySolver() const
	{
		return m_softBodySolver;
	}


	//
	// Cast
	//

	static const btlSoftBody*	upcast(const btlCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_SOFT_BODY)
			return (const btlSoftBody*)colObj;
		return 0;
	}
	static btlSoftBody*			upcast(btlCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_SOFT_BODY)
			return (btlSoftBody*)colObj;
		return 0;
	}

	//
	// ::btCollisionObject
	//

	virtual void getAabb(btlVector3& aabbMin,btlVector3& aabbMax) const
	{
		aabbMin = m_bounds[0];
		aabbMax = m_bounds[1];
	}
	//
	// Private
	//
	void				pointersToIndices();
	void				indicesToPointers(const int* map=0);

	int					rayTest(const btlVector3& rayFrom,const btlVector3& rayTo,
		btlScalar& mint,eFeature::_& feature,int& index,bool bcountonly) const;
	void				initializeFaceTree();
	btlVector3			evaluateCom() const;
	bool				checkContact(const btlCollisionObjectWrapper* colObjWrap,const btlVector3& x,btlScalar margin,btlSoftBody::sCti& cti) const;
	void				updateNormals();
	void				updateBounds();
	void				updatePose();
	void				updateConstants();
	void				updateLinkConstants();
	void				updateArea(bool averageArea = true);
	void				initializeClusters();
	void				updateClusters();
	void				cleanupClusters();
	void				prepareClusters(int iterations);
	void				solveClusters(btlScalar sor);
	void				applyClusters(bool drift);
	void				dampClusters();
	void				applyForces();	
	static void			PSolve_Anchors(btlSoftBody* psb,btlScalar kst,btlScalar ti);
	static void			PSolve_RContacts(btlSoftBody* psb,btlScalar kst,btlScalar ti);
	static void			PSolve_SContacts(btlSoftBody* psb,btlScalar,btlScalar ti);
	static void			PSolve_Links(btlSoftBody* psb,btlScalar kst,btlScalar ti);
	static void			VSolve_Links(btlSoftBody* psb,btlScalar kst);
	static psolver_t	getSolver(ePSolver::_ solver);
	static vsolver_t	getSolver(eVSolver::_ solver);


	virtual	int	calculateSerializeBufferSize()	const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer,  class btlSerializer* serializer) const;

	//virtual void serializeSingleObject(class btSerializer* serializer) const;


};




#endif //_BT_SOFT_BODY_H
