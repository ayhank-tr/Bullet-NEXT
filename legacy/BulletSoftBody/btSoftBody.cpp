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

#include "btSoftBodyInternals.h"
#include "BulletSoftBody/btSoftBodySolvers.h"
#include "btSoftBodyData.h"
#include "LinearMath/btSerializer.h"


//
btlSoftBody::btlSoftBody(btlSoftBodyWorldInfo*	worldInfo,int node_count,  const btlVector3* x,  const btlScalar* m)
:m_softBodySolver(0),m_worldInfo(worldInfo)
{	
	/* Init		*/ 
	initDefaults();

	/* Default material	*/ 
	Material*	pm=appendMaterial();
	pm->m_kLST	=	1;
	pm->m_kAST	=	1;
	pm->m_kVST	=	1;
	pm->m_flags	=	fMaterial::Default;

	/* Nodes			*/ 
	const btlScalar		margin=getCollisionShape()->getMargin();
	m_nodes.resize(node_count);
	for(int i=0,ni=node_count;i<ni;++i)
	{	
		Node&	n=m_nodes[i];
		ZeroInitialize(n);
		n.m_x		=	x?*x++:btlVector3(0,0,0);
		n.m_q		=	n.m_x;
		n.m_im		=	m?*m++:1;
		n.m_im		=	n.m_im>0?1/n.m_im:0;
		n.m_leaf	=	m_ndbvt.insert(btlDbvtVolume::FromCR(n.m_x,margin),&n);
		n.m_material=	pm;
	}
	updateBounds();	

}

btlSoftBody::btlSoftBody(btlSoftBodyWorldInfo*	worldInfo)
:m_worldInfo(worldInfo)
{
	initDefaults();
}


void	btlSoftBody::initDefaults()
{
	m_internalType		=	CO_SOFT_BODY;
	m_cfg.aeromodel		=	eAeroModel::V_Point;
	m_cfg.kVCF			=	1;
	m_cfg.kDG			=	0;
	m_cfg.kLF			=	0;
	m_cfg.kDP			=	0;
	m_cfg.kPR			=	0;
	m_cfg.kVC			=	0;
	m_cfg.kDF			=	(btlScalar)0.2;
	m_cfg.kMT			=	0;
	m_cfg.kCHR			=	(btlScalar)1.0;
	m_cfg.kKHR			=	(btlScalar)0.1;
	m_cfg.kSHR			=	(btlScalar)1.0;
	m_cfg.kAHR			=	(btlScalar)0.7;
	m_cfg.kSRHR_CL		=	(btlScalar)0.1;
	m_cfg.kSKHR_CL		=	(btlScalar)1;
	m_cfg.kSSHR_CL		=	(btlScalar)0.5;
	m_cfg.kSR_SPLT_CL	=	(btlScalar)0.5;
	m_cfg.kSK_SPLT_CL	=	(btlScalar)0.5;
	m_cfg.kSS_SPLT_CL	=	(btlScalar)0.5;
	m_cfg.maxvolume		=	(btlScalar)1;
	m_cfg.timescale		=	1;
	m_cfg.viterations	=	0;
	m_cfg.piterations	=	1;	
	m_cfg.diterations	=	0;
	m_cfg.citerations	=	4;
	m_cfg.collisions	=	fCollision::Default;
	m_pose.m_bvolume	=	false;
	m_pose.m_bframe		=	false;
	m_pose.m_volume		=	0;
	m_pose.m_com		=	btlVector3(0,0,0);
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();
	m_tag				=	0;
	m_timeacc			=	0;
	m_bUpdateRtCst		=	true;
	m_bounds[0]			=	btlVector3(0,0,0);
	m_bounds[1]			=	btlVector3(0,0,0);
	m_worldTransform.setIdentity();
	setSolver(eSolverPresets::Positions);
	
	/* Collision shape	*/ 
	///for now, create a collision shape internally
	m_collisionShape = new btlSoftBodyCollisionShape(this);
	m_collisionShape->setMargin(0.25f);
	
	m_initialWorldTransform.setIdentity();

	m_windVelocity = btlVector3(0,0,0);
	m_restLengthScale = btlScalar(1.0);
}

//
btlSoftBody::~btlSoftBody()
{
	//for now, delete the internal shape
	delete m_collisionShape;	
	int i;

	releaseClusters();
	for(i=0;i<m_materials.size();++i) 
		btlAlignedFree(m_materials[i]);
	for(i=0;i<m_joints.size();++i) 
		btlAlignedFree(m_joints[i]);
}

//
bool			btlSoftBody::checkLink(int node0,int node1) const
{
	return(checkLink(&m_nodes[node0],&m_nodes[node1]));
}

//
bool			btlSoftBody::checkLink(const Node* node0,const Node* node1) const
{
	const Node*	n[]={node0,node1};
	for(int i=0,ni=m_links.size();i<ni;++i)
	{
		const Link&	l=m_links[i];
		if(	(l.m_n[0]==n[0]&&l.m_n[1]==n[1])||
			(l.m_n[0]==n[1]&&l.m_n[1]==n[0]))
		{
			return(true);
		}
	}
	return(false);
}

//
bool			btlSoftBody::checkFace(int node0,int node1,int node2) const
{
	const Node*	n[]={	&m_nodes[node0],
		&m_nodes[node1],
		&m_nodes[node2]};
	for(int i=0,ni=m_faces.size();i<ni;++i)
	{
		const Face&	f=m_faces[i];
		int			c=0;
		for(int j=0;j<3;++j)
		{
			if(	(f.m_n[j]==n[0])||
				(f.m_n[j]==n[1])||
				(f.m_n[j]==n[2])) c|=1<<j; else break;
		}
		if(c==7) return(true);
	}
	return(false);
}

//
btlSoftBody::Material*		btlSoftBody::appendMaterial()
{
	Material*	pm=new(btlAlignedAlloc(sizeof(Material),16)) Material();
	if(m_materials.size()>0)
		*pm=*m_materials[0];
	else
		ZeroInitialize(*pm);
	m_materials.push_back(pm);
	return(pm);
}

//
void			btlSoftBody::appendNote(	const char* text,
									   const btlVector3& o,
									   const btlVector4& c,
									   Node* n0,
									   Node* n1,
									   Node* n2,
									   Node* n3)
{
	Note	n;
	ZeroInitialize(n);
	n.m_rank		=	0;
	n.m_text		=	text;
	n.m_offset		=	o;
	n.m_coords[0]	=	c.x();
	n.m_coords[1]	=	c.y();
	n.m_coords[2]	=	c.z();
	n.m_coords[3]	=	c.w();
	n.m_nodes[0]	=	n0;n.m_rank+=n0?1:0;
	n.m_nodes[1]	=	n1;n.m_rank+=n1?1:0;
	n.m_nodes[2]	=	n2;n.m_rank+=n2?1:0;
	n.m_nodes[3]	=	n3;n.m_rank+=n3?1:0;
	m_notes.push_back(n);
}

//
void			btlSoftBody::appendNote(	const char* text,
									   const btlVector3& o,
									   Node* feature)
{
	appendNote(text,o,btlVector4(1,0,0,0),feature);
}

//
void			btlSoftBody::appendNote(	const char* text,
									   const btlVector3& o,
									   Link* feature)
{
	static const btlScalar	w=1/(btlScalar)2;
	appendNote(text,o,btlVector4(w,w,0,0),	feature->m_n[0],
		feature->m_n[1]);
}

//
void			btlSoftBody::appendNote(	const char* text,
									   const btlVector3& o,
									   Face* feature)
{
	static const btlScalar	w=1/(btlScalar)3;
	appendNote(text,o,btlVector4(w,w,w,0),	feature->m_n[0],
		feature->m_n[1],
		feature->m_n[2]);
}

//
void			btlSoftBody::appendNode(	const btlVector3& x,btlScalar m)
{
	if(m_nodes.capacity()==m_nodes.size())
	{
		pointersToIndices();
		m_nodes.reserve(m_nodes.size()*2+1);
		indicesToPointers();
	}
	const btlScalar	margin=getCollisionShape()->getMargin();
	m_nodes.push_back(Node());
	Node&			n=m_nodes[m_nodes.size()-1];
	ZeroInitialize(n);
	n.m_x			=	x;
	n.m_q			=	n.m_x;
	n.m_im			=	m>0?1/m:0;
	n.m_material	=	m_materials[0];
	n.m_leaf		=	m_ndbvt.insert(btlDbvtVolume::FromCR(n.m_x,margin),&n);
}

//
void			btlSoftBody::appendLink(int model,Material* mat)
{
	Link	l;
	if(model>=0)
		l=m_links[model];
	else
	{ ZeroInitialize(l);l.m_material=mat?mat:m_materials[0]; }
	m_links.push_back(l);
}

//
void			btlSoftBody::appendLink(	int node0,
									   int node1,
									   Material* mat,
									   bool bcheckexist)
{
	appendLink(&m_nodes[node0],&m_nodes[node1],mat,bcheckexist);
}

//
void			btlSoftBody::appendLink(	Node* node0,
									   Node* node1,
									   Material* mat,
									   bool bcheckexist)
{
	if((!bcheckexist)||(!checkLink(node0,node1)))
	{
		appendLink(-1,mat);
		Link&	l=m_links[m_links.size()-1];
		l.m_n[0]		=	node0;
		l.m_n[1]		=	node1;
		l.m_rl			=	(l.m_n[0]->m_x-l.m_n[1]->m_x).length();
		m_bUpdateRtCst=true;
	}
}

//
void			btlSoftBody::appendFace(int model,Material* mat)
{
	Face	f;
	if(model>=0)
	{ f=m_faces[model]; }
	else
	{ ZeroInitialize(f);f.m_material=mat?mat:m_materials[0]; }
	m_faces.push_back(f);
}

//
void			btlSoftBody::appendFace(int node0,int node1,int node2,Material* mat)
{
	if (node0==node1)
		return;
	if (node1==node2)
		return;
	if (node2==node0)
		return;

	appendFace(-1,mat);
	Face&	f=m_faces[m_faces.size()-1];
	btlAssert(node0!=node1);
	btlAssert(node1!=node2);
	btlAssert(node2!=node0);
	f.m_n[0]	=	&m_nodes[node0];
	f.m_n[1]	=	&m_nodes[node1];
	f.m_n[2]	=	&m_nodes[node2];
	f.m_ra		=	AreaOf(	f.m_n[0]->m_x,
		f.m_n[1]->m_x,
		f.m_n[2]->m_x);	
	m_bUpdateRtCst=true;
}

//
void			btlSoftBody::appendTetra(int model,Material* mat)
{
Tetra	t;
if(model>=0)
	t=m_tetras[model];
	else
	{ ZeroInitialize(t);t.m_material=mat?mat:m_materials[0]; }
m_tetras.push_back(t);
}

//
void			btlSoftBody::appendTetra(int node0,
										int node1,
										int node2,
										int node3,
										Material* mat)
{
	appendTetra(-1,mat);
	Tetra&	t=m_tetras[m_tetras.size()-1];
	t.m_n[0]	=	&m_nodes[node0];
	t.m_n[1]	=	&m_nodes[node1];
	t.m_n[2]	=	&m_nodes[node2];
	t.m_n[3]	=	&m_nodes[node3];
	t.m_rv		=	VolumeOf(t.m_n[0]->m_x,t.m_n[1]->m_x,t.m_n[2]->m_x,t.m_n[3]->m_x);
	m_bUpdateRtCst=true;
}

//

void			btlSoftBody::appendAnchor(int node,btlRigidBody* body, bool disableCollisionBetweenLinkedBodies,btlScalar influence)
{
	btlVector3 local = body->getWorldTransform().inverse()*m_nodes[node].m_x;
	appendAnchor(node,body,local,disableCollisionBetweenLinkedBodies,influence);
}

//
void			btlSoftBody::appendAnchor(int node,btlRigidBody* body, const btlVector3& localPivot,bool disableCollisionBetweenLinkedBodies,btlScalar influence)
{
	if (disableCollisionBetweenLinkedBodies)
	{
		if (m_collisionDisabledObjects.findLinearSearch(body)==m_collisionDisabledObjects.size())
		{
			m_collisionDisabledObjects.push_back(body);
		}
	}

	Anchor	a;
	a.m_node			=	&m_nodes[node];
	a.m_body			=	body;
	a.m_local			=	localPivot;
	a.m_node->m_battach	=	1;
	a.m_influence = influence;
	m_anchors.push_back(a);
}

//
void			btlSoftBody::appendLinearJoint(const LJoint::Specs& specs,Cluster* body0,Body body1)
{
	LJoint*		pj	=	new(btlAlignedAlloc(sizeof(LJoint),16)) LJoint();
	pj->m_bodies[0]	=	body0;
	pj->m_bodies[1]	=	body1;
	pj->m_refs[0]	=	pj->m_bodies[0].xform().inverse()*specs.position;
	pj->m_refs[1]	=	pj->m_bodies[1].xform().inverse()*specs.position;
	pj->m_cfm		=	specs.cfm;
	pj->m_erp		=	specs.erp;
	pj->m_split		=	specs.split;
	m_joints.push_back(pj);
}

//
void			btlSoftBody::appendLinearJoint(const LJoint::Specs& specs,Body body)
{
	appendLinearJoint(specs,m_clusters[0],body);
}

//
void			btlSoftBody::appendLinearJoint(const LJoint::Specs& specs,btlSoftBody* body)
{
	appendLinearJoint(specs,m_clusters[0],body->m_clusters[0]);
}

//
void			btlSoftBody::appendAngularJoint(const AJoint::Specs& specs,Cluster* body0,Body body1)
{
	AJoint*		pj	=	new(btlAlignedAlloc(sizeof(AJoint),16)) AJoint();
	pj->m_bodies[0]	=	body0;
	pj->m_bodies[1]	=	body1;
	pj->m_refs[0]	=	pj->m_bodies[0].xform().inverse().getBasis()*specs.axis;
	pj->m_refs[1]	=	pj->m_bodies[1].xform().inverse().getBasis()*specs.axis;
	pj->m_cfm		=	specs.cfm;
	pj->m_erp		=	specs.erp;
	pj->m_split		=	specs.split;
	pj->m_icontrol	=	specs.icontrol;
	m_joints.push_back(pj);
}

//
void			btlSoftBody::appendAngularJoint(const AJoint::Specs& specs,Body body)
{
	appendAngularJoint(specs,m_clusters[0],body);
}

//
void			btlSoftBody::appendAngularJoint(const AJoint::Specs& specs,btlSoftBody* body)
{
	appendAngularJoint(specs,m_clusters[0],body->m_clusters[0]);
}

//
void			btlSoftBody::addForce(const btlVector3& force)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) addForce(force,i);
}

//
void			btlSoftBody::addForce(const btlVector3& force,int node)
{
	Node&	n=m_nodes[node];
	if(n.m_im>0)
	{
		n.m_f	+=	force;
	}
}

void			btlSoftBody::addAeroForceToNode(const btlVector3& windVelocity,int nodeIndex)
{
	btlAssert(nodeIndex >= 0 && nodeIndex < m_nodes.size());

	const btlScalar dt = m_sst.sdt;
	const btlScalar kLF = m_cfg.kLF;
	const btlScalar kDG = m_cfg.kDG;
	//const btScalar kPR = m_cfg.kPR;
	//const btScalar kVC = m_cfg.kVC;
	const bool as_lift = kLF>0;
	const bool as_drag = kDG>0;
	const bool as_aero = as_lift || as_drag;
	const bool as_vaero = as_aero && (m_cfg.aeromodel < btlSoftBody::eAeroModel::F_TwoSided);

	Node& n = m_nodes[nodeIndex];

	if( n.m_im>0 )
	{
		btlSoftBody::sMedium	medium;

		EvaluateMedium(m_worldInfo, n.m_x, medium);
		medium.m_velocity = windVelocity;
		medium.m_density = m_worldInfo->air_density;

		/* Aerodynamics			*/ 
		if(as_vaero)
		{				
			const btlVector3	rel_v = n.m_v - medium.m_velocity;					
			const btlScalar rel_v_len = rel_v.length();
			const btlScalar	rel_v2 = rel_v.length2();

			if(rel_v2>SIMD_EPSILON)
			{
				const btlVector3 rel_v_nrm = rel_v.normalized();
				btlVector3	nrm = n.m_n;						

				if (m_cfg.aeromodel == btlSoftBody::eAeroModel::V_TwoSidedLiftDrag)
				{
					nrm *= (btlScalar)( (btlDot(nrm,rel_v) < 0) ? -1 : +1);
					btlVector3 fDrag(0, 0, 0);
					btlVector3 fLift(0, 0, 0);

					btlScalar n_dot_v = nrm.dot(rel_v_nrm);
					btlScalar tri_area = 0.5f * n.m_area;
							
					fDrag = 0.5f * kDG * medium.m_density * rel_v2 * tri_area * n_dot_v * (-rel_v_nrm);
							
					// Check angle of attack
					// cos(10º) = 0.98480
					if ( 0 < n_dot_v && n_dot_v < 0.98480f)
						fLift = 0.5f * kLF * medium.m_density * rel_v_len * tri_area * btlSqrt(1.0f-n_dot_v*n_dot_v) * (nrm.cross(rel_v_nrm).cross(rel_v_nrm));

					// Check if the velocity change resulted by aero drag force exceeds the current velocity of the node.
					btlVector3 del_v_by_fDrag = fDrag*n.m_im*m_sst.sdt;										
					btlScalar del_v_by_fDrag_len2 = del_v_by_fDrag.length2();
					btlScalar v_len2 = n.m_v.length2();

					if (del_v_by_fDrag_len2 >= v_len2 && del_v_by_fDrag_len2 > 0)
					{
						btlScalar del_v_by_fDrag_len = del_v_by_fDrag.length();
						btlScalar v_len = n.m_v.length();
						fDrag *= btlScalar(0.8)*(v_len / del_v_by_fDrag_len);
					}

					n.m_f += fDrag;
					n.m_f += fLift;
				}
				else if (m_cfg.aeromodel == btlSoftBody::eAeroModel::V_Point || m_cfg.aeromodel == btlSoftBody::eAeroModel::V_OneSided || m_cfg.aeromodel == btlSoftBody::eAeroModel::V_TwoSided)
				{
					if (btlSoftBody::eAeroModel::V_TwoSided)
						nrm *= (btlScalar)( (btlDot(nrm,rel_v) < 0) ? -1 : +1);

					const btlScalar dvn = btlDot(rel_v,nrm);
					/* Compute forces	*/ 
					if(dvn>0)
					{
						btlVector3		force(0,0,0);
						const btlScalar	c0	=	n.m_area * dvn * rel_v2/2;
						const btlScalar	c1	=	c0 * medium.m_density;
						force	+=	nrm*(-c1*kLF);
						force	+=	rel_v.normalized() * (-c1 * kDG);
						ApplyClampedForce(n, force, dt);
					}
				}	
			}
		}
	}
}

void			btlSoftBody::addAeroForceToFace(const btlVector3& windVelocity,int faceIndex)
{
	const btlScalar dt = m_sst.sdt;
	const btlScalar kLF = m_cfg.kLF;
	const btlScalar kDG = m_cfg.kDG;
//	const btScalar kPR = m_cfg.kPR;
//	const btScalar kVC = m_cfg.kVC;
	const bool as_lift = kLF>0;
	const bool as_drag = kDG>0;
	const bool as_aero = as_lift || as_drag;
	const bool as_faero = as_aero && (m_cfg.aeromodel >= btlSoftBody::eAeroModel::F_TwoSided);

	if(as_faero)
	{
		btlSoftBody::Face&	f=m_faces[faceIndex];

		btlSoftBody::sMedium	medium;
		
		const btlVector3	v=(f.m_n[0]->m_v+f.m_n[1]->m_v+f.m_n[2]->m_v)/3;
		const btlVector3	x=(f.m_n[0]->m_x+f.m_n[1]->m_x+f.m_n[2]->m_x)/3;
		EvaluateMedium(m_worldInfo,x,medium);
		medium.m_velocity = windVelocity;
		medium.m_density = m_worldInfo->air_density;
		const btlVector3	rel_v=v-medium.m_velocity;
		const btlScalar rel_v_len = rel_v.length();
		const btlScalar	rel_v2=rel_v.length2();

		if(rel_v2>SIMD_EPSILON)
		{
			const btlVector3 rel_v_nrm = rel_v.normalized();
			btlVector3	nrm = f.m_normal;

			if (m_cfg.aeromodel == btlSoftBody::eAeroModel::F_TwoSidedLiftDrag)
			{
				nrm *= (btlScalar)( (btlDot(nrm,rel_v) < 0) ? -1 : +1);

				btlVector3 fDrag(0, 0, 0);
				btlVector3 fLift(0, 0, 0);

				btlScalar n_dot_v = nrm.dot(rel_v_nrm);
				btlScalar tri_area = 0.5f * f.m_ra;
					
				fDrag = 0.5f * kDG * medium.m_density * rel_v2 * tri_area * n_dot_v * (-rel_v_nrm);

				// Check angle of attack
				// cos(10º) = 0.98480
				if ( 0 < n_dot_v && n_dot_v < 0.98480f)
					fLift = 0.5f * kLF * medium.m_density * rel_v_len * tri_area * btlSqrt(1.0f-n_dot_v*n_dot_v) * (nrm.cross(rel_v_nrm).cross(rel_v_nrm));

				fDrag /= 3;
				fLift /= 3;

				for(int j=0;j<3;++j) 
				{
					if (f.m_n[j]->m_im>0)
					{
						// Check if the velocity change resulted by aero drag force exceeds the current velocity of the node.
						btlVector3 del_v_by_fDrag = fDrag*f.m_n[j]->m_im*m_sst.sdt;										
						btlScalar del_v_by_fDrag_len2 = del_v_by_fDrag.length2();
						btlScalar v_len2 = f.m_n[j]->m_v.length2();

						if (del_v_by_fDrag_len2 >= v_len2 && del_v_by_fDrag_len2 > 0)
						{
							btlScalar del_v_by_fDrag_len = del_v_by_fDrag.length();
							btlScalar v_len = f.m_n[j]->m_v.length();
							fDrag *= btlScalar(0.8)*(v_len / del_v_by_fDrag_len);
						}

						f.m_n[j]->m_f += fDrag; 
						f.m_n[j]->m_f += fLift;
					}
				}
			}
			else if (m_cfg.aeromodel == btlSoftBody::eAeroModel::F_OneSided || m_cfg.aeromodel == btlSoftBody::eAeroModel::F_TwoSided)
			{
				if (btlSoftBody::eAeroModel::F_TwoSided)
					nrm *= (btlScalar)( (btlDot(nrm,rel_v) < 0) ? -1 : +1);

				const btlScalar	dvn=btlDot(rel_v,nrm);
				/* Compute forces	*/ 
				if(dvn>0)
				{
					btlVector3		force(0,0,0);
					const btlScalar	c0	=	f.m_ra*dvn*rel_v2;
					const btlScalar	c1	=	c0*medium.m_density;
					force	+=	nrm*(-c1*kLF);
					force	+=	rel_v.normalized()*(-c1*kDG);
					force	/=	3;
					for(int j=0;j<3;++j) ApplyClampedForce(*f.m_n[j],force,dt);
				}
			}
		}
	}

}

//
void			btlSoftBody::addVelocity(const btlVector3& velocity)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) addVelocity(velocity,i);
}

/* Set velocity for the entire body										*/ 
void				btlSoftBody::setVelocity(	const btlVector3& velocity)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) 
	{
		Node&	n=m_nodes[i];
		if(n.m_im>0)
		{
			n.m_v	=	velocity;
		}
	}
}


//
void			btlSoftBody::addVelocity(const btlVector3& velocity,int node)
{
	Node&	n=m_nodes[node];
	if(n.m_im>0)
	{
		n.m_v	+=	velocity;
	}
}

//
void			btlSoftBody::setMass(int node,btlScalar mass)
{
	m_nodes[node].m_im=mass>0?1/mass:0;
	m_bUpdateRtCst=true;
}

//
btlScalar		btlSoftBody::getMass(int node) const
{
	return(m_nodes[node].m_im>0?1/m_nodes[node].m_im:0);
}

//
btlScalar		btlSoftBody::getTotalMass() const
{
	btlScalar	mass=0;
	for(int i=0;i<m_nodes.size();++i)
	{
		mass+=getMass(i);
	}
	return(mass);
}

//
void			btlSoftBody::setTotalMass(btlScalar mass,bool fromfaces)
{
	int i;

	if(fromfaces)
	{

		for(i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=0;
		}
		for(i=0;i<m_faces.size();++i)
		{
			const Face&		f=m_faces[i];
			const btlScalar	twicearea=AreaOf(	f.m_n[0]->m_x,
				f.m_n[1]->m_x,
				f.m_n[2]->m_x);
			for(int j=0;j<3;++j)
			{
				f.m_n[j]->m_im+=twicearea;
			}
		}
		for( i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=1/m_nodes[i].m_im;
		}
	}
	const btlScalar	tm=getTotalMass();
	const btlScalar	itm=1/tm;
	for( i=0;i<m_nodes.size();++i)
	{
		m_nodes[i].m_im/=itm*mass;
	}
	m_bUpdateRtCst=true;
}

//
void			btlSoftBody::setTotalDensity(btlScalar density)
{
	setTotalMass(getVolume()*density,true);
}

//
void			btlSoftBody::setVolumeMass(btlScalar mass)
{
btlAlignedObjectArray<btlScalar>	ranks;
ranks.resize(m_nodes.size(),0);
int i;

for(i=0;i<m_nodes.size();++i)
	{
	m_nodes[i].m_im=0;
	}
for(i=0;i<m_tetras.size();++i)
	{
	const Tetra& t=m_tetras[i];
	for(int j=0;j<4;++j)
		{
		t.m_n[j]->m_im+=btlFabs(t.m_rv);
		ranks[int(t.m_n[j]-&m_nodes[0])]+=1;
		}
	}
for( i=0;i<m_nodes.size();++i)
	{
	if(m_nodes[i].m_im>0)
		{
		m_nodes[i].m_im=ranks[i]/m_nodes[i].m_im;
		}
	}
setTotalMass(mass,false);
}

//
void			btlSoftBody::setVolumeDensity(btlScalar density)
{
btlScalar	volume=0;
for(int i=0;i<m_tetras.size();++i)
	{
	const Tetra& t=m_tetras[i];
	for(int j=0;j<4;++j)
		{
		volume+=btlFabs(t.m_rv);
		}
	}
setVolumeMass(volume*density/6);
}

//
void			btlSoftBody::transform(const btlTransform& trs)
{
	const btlScalar	margin=getCollisionShape()->getMargin();
	ATTRIBUTE_ALIGNED16(btlDbvtVolume)	vol;
	
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_x=trs*n.m_x;
		n.m_q=trs*n.m_q;
		n.m_n=trs.getBasis()*n.m_n;
		vol = btlDbvtVolume::FromCR(n.m_x,margin);
		
		m_ndbvt.update(n.m_leaf,vol);
	}
	updateNormals();
	updateBounds();
	updateConstants();
	m_initialWorldTransform = trs;
}

//
void			btlSoftBody::translate(const btlVector3& trs)
{
	btlTransform	t;
	t.setIdentity();
	t.setOrigin(trs);
	transform(t);
}

//
void			btlSoftBody::rotate(	const btlQuaternion& rot)
{
	btlTransform	t;
	t.setIdentity();
	t.setRotation(rot);
	transform(t);
}

//
void			btlSoftBody::scale(const btlVector3& scl)
{

	const btlScalar	margin=getCollisionShape()->getMargin();
	ATTRIBUTE_ALIGNED16(btlDbvtVolume)	vol;
	
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_x*=scl;
		n.m_q*=scl;
		vol = btlDbvtVolume::FromCR(n.m_x,margin);
		m_ndbvt.update(n.m_leaf,vol);
	}
	updateNormals();
	updateBounds();
	updateConstants();
}

//
btlScalar btlSoftBody::getRestLengthScale()
{
	return m_restLengthScale;
}

//
void btlSoftBody::setRestLengthScale(btlScalar restLengthScale)
{
	for(int i=0, ni=m_links.size(); i<ni; ++i)
	{
		Link&		l=m_links[i];
		l.m_rl	=	l.m_rl / m_restLengthScale * restLengthScale;
		l.m_c1	=	l.m_rl*l.m_rl;
	}
	m_restLengthScale = restLengthScale;
	
	if (getActivationState() == ISLAND_SLEEPING)
		activate();
}

//
void			btlSoftBody::setPose(bool bvolume,bool bframe)
{
	m_pose.m_bvolume	=	bvolume;
	m_pose.m_bframe		=	bframe;
	int i,ni;

	/* Weights		*/ 
	const btlScalar	omass=getTotalMass();
	const btlScalar	kmass=omass*m_nodes.size()*1000;
	btlScalar		tmass=omass;
	m_pose.m_wgh.resize(m_nodes.size());
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(m_nodes[i].m_im<=0) tmass+=kmass;
	}
	for( i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		m_pose.m_wgh[i]=	n.m_im>0					?
			1/(m_nodes[i].m_im*tmass)	:
		kmass/tmass;
	}
	/* Pos		*/ 
	const btlVector3	com=evaluateCom();
	m_pose.m_pos.resize(m_nodes.size());
	for( i=0,ni=m_nodes.size();i<ni;++i)
	{
		m_pose.m_pos[i]=m_nodes[i].m_x-com;
	}
	m_pose.m_volume	=	bvolume?getVolume():0;
	m_pose.m_com	=	com;
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();
	/* Aqq		*/ 
	m_pose.m_aqq[0]	=
		m_pose.m_aqq[1]	=
		m_pose.m_aqq[2]	=	btlVector3(0,0,0);
	for( i=0,ni=m_nodes.size();i<ni;++i)
	{
		const btlVector3&	q=m_pose.m_pos[i];
		const btlVector3		mq=m_pose.m_wgh[i]*q;
		m_pose.m_aqq[0]+=mq.x()*q;
		m_pose.m_aqq[1]+=mq.y()*q;
		m_pose.m_aqq[2]+=mq.z()*q;
	}
	m_pose.m_aqq=m_pose.m_aqq.inverse();
	
	updateConstants();
}

void				btlSoftBody::resetLinkRestLengths()
{
	for(int i=0, ni=m_links.size();i<ni;++i)
	{
		Link& l =	m_links[i];
		l.m_rl	=	(l.m_n[0]->m_x-l.m_n[1]->m_x).length();
		l.m_c1	=	l.m_rl*l.m_rl;
	}
}

//
btlScalar		btlSoftBody::getVolume() const
{
	btlScalar	vol=0;
	if(m_nodes.size()>0)
	{
		int i,ni;

		const btlVector3	org=m_nodes[0].m_x;
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			const Face&	f=m_faces[i];
			vol+=btlDot(f.m_n[0]->m_x-org,btlCross(f.m_n[1]->m_x-org,f.m_n[2]->m_x-org));
		}
		vol/=(btlScalar)6;
	}
	return(vol);
}

//
int				btlSoftBody::clusterCount() const
{
	return(m_clusters.size());
}

//
btlVector3		btlSoftBody::clusterCom(const Cluster* cluster)
{
	btlVector3		com(0,0,0);
	for(int i=0,ni=cluster->m_nodes.size();i<ni;++i)
	{
		com+=cluster->m_nodes[i]->m_x*cluster->m_masses[i];
	}
	return(com*cluster->m_imass);
}

//
btlVector3		btlSoftBody::clusterCom(int cluster) const
{
	return(clusterCom(m_clusters[cluster]));
}

//
btlVector3		btlSoftBody::clusterVelocity(const Cluster* cluster,const btlVector3& rpos)
{
	return(cluster->m_lv+btlCross(cluster->m_av,rpos));
}

//
void			btlSoftBody::clusterVImpulse(Cluster* cluster,const btlVector3& rpos,const btlVector3& impulse)
{
	const btlVector3	li=cluster->m_imass*impulse;
	const btlVector3	ai=cluster->m_invwi*btlCross(rpos,impulse);
	cluster->m_vimpulses[0]+=li;cluster->m_lv+=li;
	cluster->m_vimpulses[1]+=ai;cluster->m_av+=ai;
	cluster->m_nvimpulses++;
}

//
void			btlSoftBody::clusterDImpulse(Cluster* cluster,const btlVector3& rpos,const btlVector3& impulse)
{
	const btlVector3	li=cluster->m_imass*impulse;
	const btlVector3	ai=cluster->m_invwi*btlCross(rpos,impulse);
	cluster->m_dimpulses[0]+=li;
	cluster->m_dimpulses[1]+=ai;
	cluster->m_ndimpulses++;
}

//
void			btlSoftBody::clusterImpulse(Cluster* cluster,const btlVector3& rpos,const Impulse& impulse)
{
	if(impulse.m_asVelocity)	clusterVImpulse(cluster,rpos,impulse.m_velocity);
	if(impulse.m_asDrift)		clusterDImpulse(cluster,rpos,impulse.m_drift);
}

//
void			btlSoftBody::clusterVAImpulse(Cluster* cluster,const btlVector3& impulse)
{
	const btlVector3	ai=cluster->m_invwi*impulse;
	cluster->m_vimpulses[1]+=ai;cluster->m_av+=ai;
	cluster->m_nvimpulses++;
}

//
void			btlSoftBody::clusterDAImpulse(Cluster* cluster,const btlVector3& impulse)
{
	const btlVector3	ai=cluster->m_invwi*impulse;
	cluster->m_dimpulses[1]+=ai;
	cluster->m_ndimpulses++;
}

//
void			btlSoftBody::clusterAImpulse(Cluster* cluster,const Impulse& impulse)
{
	if(impulse.m_asVelocity)	clusterVAImpulse(cluster,impulse.m_velocity);
	if(impulse.m_asDrift)		clusterDAImpulse(cluster,impulse.m_drift);
}

//
void			btlSoftBody::clusterDCImpulse(Cluster* cluster,const btlVector3& impulse)
{
	cluster->m_dimpulses[0]+=impulse*cluster->m_imass;
	cluster->m_ndimpulses++;
}

struct btl_NodeLinks
{
    btlAlignedObjectArray<int> m_links;
};



//
int				btlSoftBody::generateBendingConstraints(int distance,Material* mat)
{
	int i,j;

	if(distance>1)
	{
		/* Build graph	*/ 
		const int		n=m_nodes.size();
		const unsigned	inf=(~(unsigned)0)>>1;
		unsigned*		adj=new unsigned[n*n];
		

#define IDX(_x_,_y_)	((_y_)*n+(_x_))
		for(j=0;j<n;++j)
		{
			for(i=0;i<n;++i)
			{
				if(i!=j)
				{
					adj[IDX(i,j)]=adj[IDX(j,i)]=inf;
				}
				else
				{
					adj[IDX(i,j)]=adj[IDX(j,i)]=0;
				}
			}
		}
		for( i=0;i<m_links.size();++i)
		{
			const int	ia=(int)(m_links[i].m_n[0]-&m_nodes[0]);
			const int	ib=(int)(m_links[i].m_n[1]-&m_nodes[0]);
			adj[IDX(ia,ib)]=1;
			adj[IDX(ib,ia)]=1;
		}


		//special optimized case for distance == 2
		if (distance == 2)
		{

			btlAlignedObjectArray<btl_NodeLinks> nodeLinks;


			/* Build node links */
			nodeLinks.resize(m_nodes.size());

			for( i=0;i<m_links.size();++i)
			{
				const int	ia=(int)(m_links[i].m_n[0]-&m_nodes[0]);
				const int	ib=(int)(m_links[i].m_n[1]-&m_nodes[0]);
				if (nodeLinks[ia].m_links.findLinearSearch(ib)==nodeLinks[ia].m_links.size())
					nodeLinks[ia].m_links.push_back(ib);

				if (nodeLinks[ib].m_links.findLinearSearch(ia)==nodeLinks[ib].m_links.size())
					nodeLinks[ib].m_links.push_back(ia);
			}
			for (int ii=0;ii<nodeLinks.size();ii++)
			{
				int i=ii;

				for (int jj=0;jj<nodeLinks[ii].m_links.size();jj++)
				{
					int k = nodeLinks[ii].m_links[jj];
					for (int kk=0;kk<nodeLinks[k].m_links.size();kk++)
					{
						int j = nodeLinks[k].m_links[kk];
						if (i!=j)
						{
							const unsigned	sum=adj[IDX(i,k)]+adj[IDX(k,j)];
							btlAssert(sum==2);
							if(adj[IDX(i,j)]>sum)
							{
								adj[IDX(i,j)]=adj[IDX(j,i)]=sum;
							}
						}

					}
				}
			}
		} else
		{
			///generic Floyd's algorithm
			for(int k=0;k<n;++k)
			{
				for(j=0;j<n;++j)
				{
					for(i=j+1;i<n;++i)
					{
						const unsigned	sum=adj[IDX(i,k)]+adj[IDX(k,j)];
						if(adj[IDX(i,j)]>sum)
						{
							adj[IDX(i,j)]=adj[IDX(j,i)]=sum;
						}
					}
				}
			}
		}


		/* Build links	*/ 
		int	nlinks=0;
		for(j=0;j<n;++j)
		{
			for(i=j+1;i<n;++i)
			{
				if(adj[IDX(i,j)]==(unsigned)distance)
				{
					appendLink(i,j,mat);
					m_links[m_links.size()-1].m_bbending=1;
					++nlinks;
				}
			}
		}
		delete[] adj;		
		return(nlinks);
	}
	return(0);
}

//
void			btlSoftBody::randomizeConstraints()
{
	unsigned long	seed=243703;
#define NEXTRAND (seed=(1664525L*seed+1013904223L)&0xffffffff)
	int i,ni;

	for(i=0,ni=m_links.size();i<ni;++i)
	{
		btlSwap(m_links[i],m_links[NEXTRAND%ni]);
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		btlSwap(m_faces[i],m_faces[NEXTRAND%ni]);
	}
#undef NEXTRAND
}

//
void			btlSoftBody::releaseCluster(int index)
{
	Cluster*	c=m_clusters[index];
	if(c->m_leaf) m_cdbvt.remove(c->m_leaf);
	c->~Cluster();
	btlAlignedFree(c);
	m_clusters.remove(c);
}

//
void			btlSoftBody::releaseClusters()
{
	while(m_clusters.size()>0) releaseCluster(0);
}

//
int				btlSoftBody::generateClusters(int k,int maxiterations)
{
	int i;
	releaseClusters();
	m_clusters.resize(btlMin(k,m_nodes.size()));
	for(i=0;i<m_clusters.size();++i)
	{
		m_clusters[i]			=	new(btlAlignedAlloc(sizeof(Cluster),16)) Cluster();
		m_clusters[i]->m_collide=	true;
	}
	k=m_clusters.size();
	if(k>0)
	{
		/* Initialize		*/ 
		btlAlignedObjectArray<btlVector3>	centers;
		btlVector3						cog(0,0,0);
		int								i;
		for(i=0;i<m_nodes.size();++i)
		{
			cog+=m_nodes[i].m_x;
			m_clusters[(i*29873)%m_clusters.size()]->m_nodes.push_back(&m_nodes[i]);
		}
		cog/=(btlScalar)m_nodes.size();
		centers.resize(k,cog);
		/* Iterate			*/ 
		const btlScalar	slope=16;
		bool			changed;
		int				iterations=0;
		do	{
			const btlScalar	w=2-btlMin<btlScalar>(1,iterations/slope);
			changed=false;
			iterations++;	
			int i;

			for(i=0;i<k;++i)
			{
				btlVector3	c(0,0,0);
				for(int j=0;j<m_clusters[i]->m_nodes.size();++j)
				{
					c+=m_clusters[i]->m_nodes[j]->m_x;
				}
				if(m_clusters[i]->m_nodes.size())
				{
					c			/=	(btlScalar)m_clusters[i]->m_nodes.size();
					c			=	centers[i]+(c-centers[i])*w;
					changed		|=	((c-centers[i]).length2()>SIMD_EPSILON);
					centers[i]	=	c;
					m_clusters[i]->m_nodes.resize(0);
				}			
			}
			for(i=0;i<m_nodes.size();++i)
			{
				const btlVector3	nx=m_nodes[i].m_x;
				int				kbest=0;
				btlScalar		kdist=ClusterMetric(centers[0],nx);
				for(int j=1;j<k;++j)
				{
					const btlScalar	d=ClusterMetric(centers[j],nx);
					if(d<kdist)
					{
						kbest=j;
						kdist=d;
					}
				}
				m_clusters[kbest]->m_nodes.push_back(&m_nodes[i]);
			}		
		} while(changed&&(iterations<maxiterations));
		/* Merge		*/ 
		btlAlignedObjectArray<int>	cids;
		cids.resize(m_nodes.size(),-1);
		for(i=0;i<m_clusters.size();++i)
		{
			for(int j=0;j<m_clusters[i]->m_nodes.size();++j)
			{
				cids[int(m_clusters[i]->m_nodes[j]-&m_nodes[0])]=i;
			}
		}
		for(i=0;i<m_faces.size();++i)
		{
			const int idx[]={	int(m_faces[i].m_n[0]-&m_nodes[0]),
				int(m_faces[i].m_n[1]-&m_nodes[0]),
				int(m_faces[i].m_n[2]-&m_nodes[0])};
			for(int j=0;j<3;++j)
			{
				const int cid=cids[idx[j]];
				for(int q=1;q<3;++q)
				{
					const int kid=idx[(j+q)%3];
					if(cids[kid]!=cid)
					{
						if(m_clusters[cid]->m_nodes.findLinearSearch(&m_nodes[kid])==m_clusters[cid]->m_nodes.size())
						{
							m_clusters[cid]->m_nodes.push_back(&m_nodes[kid]);
						}
					}
				}
			}
		}
		/* Master		*/ 
		if(m_clusters.size()>1)
		{
			Cluster*	pmaster=new(btlAlignedAlloc(sizeof(Cluster),16)) Cluster();
			pmaster->m_collide	=	false;
			pmaster->m_nodes.reserve(m_nodes.size());
			for(int i=0;i<m_nodes.size();++i) pmaster->m_nodes.push_back(&m_nodes[i]);
			m_clusters.push_back(pmaster);
			btlSwap(m_clusters[0],m_clusters[m_clusters.size()-1]);
		}
		/* Terminate	*/ 
		for(i=0;i<m_clusters.size();++i)
		{
			if(m_clusters[i]->m_nodes.size()==0)
			{
				releaseCluster(i--);
			}
		}
	} else
	{
		//create a cluster for each tetrahedron (if tetrahedra exist) or each face
		if (m_tetras.size())
		{
			m_clusters.resize(m_tetras.size());
			for(i=0;i<m_clusters.size();++i)
			{
				m_clusters[i]			=	new(btlAlignedAlloc(sizeof(Cluster),16)) Cluster();
				m_clusters[i]->m_collide=	true;
			}
			for (i=0;i<m_tetras.size();i++)
			{
				for (int j=0;j<4;j++)
				{
					m_clusters[i]->m_nodes.push_back(m_tetras[i].m_n[j]);
				}
			}

		} else
		{
			m_clusters.resize(m_faces.size());
			for(i=0;i<m_clusters.size();++i)
			{
				m_clusters[i]			=	new(btlAlignedAlloc(sizeof(Cluster),16)) Cluster();
				m_clusters[i]->m_collide=	true;
			}

			for(i=0;i<m_faces.size();++i)
			{
				for(int j=0;j<3;++j)
				{
					m_clusters[i]->m_nodes.push_back(m_faces[i].m_n[j]);
				}
			}
		}
	}

	if (m_clusters.size())
	{
		initializeClusters();
		updateClusters();


		//for self-collision
		m_clusterConnectivity.resize(m_clusters.size()*m_clusters.size());
		{
			for (int c0=0;c0<m_clusters.size();c0++)
			{
				m_clusters[c0]->m_clusterIndex=c0;
				for (int c1=0;c1<m_clusters.size();c1++)
				{
					
					bool connected=false;
					Cluster* cla = m_clusters[c0];
					Cluster* clb = m_clusters[c1];
					for (int i=0;!connected&&i<cla->m_nodes.size();i++)
					{
						for (int j=0;j<clb->m_nodes.size();j++)
						{
							if (cla->m_nodes[i] == clb->m_nodes[j])
							{
								connected=true;
								break;
							}
						}
					}
					m_clusterConnectivity[c0+c1*m_clusters.size()]=connected;
				}
			}
		}
	}

	return(m_clusters.size());
}

//
void			btlSoftBody::refine(ImplicitFn* ifn,btlScalar accurary,bool cut)
{
	const Node*			nbase = &m_nodes[0];
	int					ncount = m_nodes.size();
	btlSymMatrix<int>	edges(ncount,-2);
	int					newnodes=0;
	int i,j,k,ni;

	/* Filter out		*/ 
	for(i=0;i<m_links.size();++i)
	{
		Link&	l=m_links[i];
		if(l.m_bbending)
		{
			if(!SameSign(ifn->Eval(l.m_n[0]->m_x),ifn->Eval(l.m_n[1]->m_x)))
			{
				btlSwap(m_links[i],m_links[m_links.size()-1]);
				m_links.pop_back();--i;
			}
		}	
	}
	/* Fill edges		*/ 
	for(i=0;i<m_links.size();++i)
	{
		Link&	l=m_links[i];
		edges(int(l.m_n[0]-nbase),int(l.m_n[1]-nbase))=-1;
	}
	for(i=0;i<m_faces.size();++i)
	{	
		Face&	f=m_faces[i];
		edges(int(f.m_n[0]-nbase),int(f.m_n[1]-nbase))=-1;
		edges(int(f.m_n[1]-nbase),int(f.m_n[2]-nbase))=-1;
		edges(int(f.m_n[2]-nbase),int(f.m_n[0]-nbase))=-1;
	}
	/* Intersect		*/ 
	for(i=0;i<ncount;++i)
	{
		for(j=i+1;j<ncount;++j)
		{
			if(edges(i,j)==-1)
			{
				Node&			a=m_nodes[i];
				Node&			b=m_nodes[j];
				const btlScalar	t=ImplicitSolve(ifn,a.m_x,b.m_x,accurary);
				if(t>0)
				{
					const btlVector3	x=Lerp(a.m_x,b.m_x,t);
					const btlVector3	v=Lerp(a.m_v,b.m_v,t);
					btlScalar		m=0;
					if(a.m_im>0)
					{
						if(b.m_im>0)
						{
							const btlScalar	ma=1/a.m_im;
							const btlScalar	mb=1/b.m_im;
							const btlScalar	mc=Lerp(ma,mb,t);
							const btlScalar	f=(ma+mb)/(ma+mb+mc);
							a.m_im=1/(ma*f);
							b.m_im=1/(mb*f);
							m=mc*f;
						}
						else
						{ a.m_im/=0.5f;m=1/a.m_im; }
					}
					else
					{
						if(b.m_im>0)
						{ b.m_im/=0.5f;m=1/b.m_im; }
						else
							m=0;
					}
					appendNode(x,m);
					edges(i,j)=m_nodes.size()-1;
					m_nodes[edges(i,j)].m_v=v;
					++newnodes;
				}
			}
		}
	}
	nbase=&m_nodes[0];
	/* Refine links		*/ 
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		Link&		feat=m_links[i];
		const int	idx[]={	int(feat.m_n[0]-nbase),
			int(feat.m_n[1]-nbase)};
		if((idx[0]<ncount)&&(idx[1]<ncount))
		{
			const int ni=edges(idx[0],idx[1]);
			if(ni>0)
			{
				appendLink(i);
				Link*		pft[]={	&m_links[i],
					&m_links[m_links.size()-1]};			
				pft[0]->m_n[0]=&m_nodes[idx[0]];
				pft[0]->m_n[1]=&m_nodes[ni];
				pft[1]->m_n[0]=&m_nodes[ni];
				pft[1]->m_n[1]=&m_nodes[idx[1]];
			}
		}
	}
	/* Refine faces		*/ 
	for(i=0;i<m_faces.size();++i)
	{
		const Face&	feat=m_faces[i];
		const int	idx[]={	int(feat.m_n[0]-nbase),
			int(feat.m_n[1]-nbase),
			int(feat.m_n[2]-nbase)};
		for(j=2,k=0;k<3;j=k++)
		{
			if((idx[j]<ncount)&&(idx[k]<ncount))
			{
				const int ni=edges(idx[j],idx[k]);
				if(ni>0)
				{
					appendFace(i);
					const int	l=(k+1)%3;
					Face*		pft[]={	&m_faces[i],
						&m_faces[m_faces.size()-1]};
					pft[0]->m_n[0]=&m_nodes[idx[l]];
					pft[0]->m_n[1]=&m_nodes[idx[j]];
					pft[0]->m_n[2]=&m_nodes[ni];
					pft[1]->m_n[0]=&m_nodes[ni];
					pft[1]->m_n[1]=&m_nodes[idx[k]];
					pft[1]->m_n[2]=&m_nodes[idx[l]];
					appendLink(ni,idx[l],pft[0]->m_material);
					--i;break;
				}
			}
		}
	}
	/* Cut				*/ 
	if(cut)
	{	
		btlAlignedObjectArray<int>	cnodes;
		const int					pcount=ncount;
		int							i;
		ncount=m_nodes.size();
		cnodes.resize(ncount,0);
		/* Nodes		*/ 
		for(i=0;i<ncount;++i)
		{
			const btlVector3	x=m_nodes[i].m_x;
			if((i>=pcount)||(btlFabs(ifn->Eval(x))<accurary))
			{
				const btlVector3	v=m_nodes[i].m_v;
				btlScalar		m=getMass(i);
				if(m>0) { m*=0.5f;m_nodes[i].m_im/=0.5f; }
				appendNode(x,m);
				cnodes[i]=m_nodes.size()-1;
				m_nodes[cnodes[i]].m_v=v;
			}
		}
		nbase=&m_nodes[0];
		/* Links		*/ 
		for(i=0,ni=m_links.size();i<ni;++i)
		{
			const int		id[]={	int(m_links[i].m_n[0]-nbase),
				int(m_links[i].m_n[1]-nbase)};
			int				todetach=0;
			if(cnodes[id[0]]&&cnodes[id[1]])
			{
				appendLink(i);
				todetach=m_links.size()-1;
			}
			else
			{
				if((	(ifn->Eval(m_nodes[id[0]].m_x)<accurary)&&
					(ifn->Eval(m_nodes[id[1]].m_x)<accurary)))
					todetach=i;
			}
			if(todetach)
			{
				Link&	l=m_links[todetach];
				for(int j=0;j<2;++j)
				{
					int cn=cnodes[int(l.m_n[j]-nbase)];
					if(cn) l.m_n[j]=&m_nodes[cn];
				}			
			}
		}
		/* Faces		*/ 
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			Node**			n=	m_faces[i].m_n;
			if(	(ifn->Eval(n[0]->m_x)<accurary)&&
				(ifn->Eval(n[1]->m_x)<accurary)&&
				(ifn->Eval(n[2]->m_x)<accurary))
			{
				for(int j=0;j<3;++j)
				{
					int cn=cnodes[int(n[j]-nbase)];
					if(cn) n[j]=&m_nodes[cn];
				}
			}
		}
		/* Clean orphans	*/ 
		int							nnodes=m_nodes.size();
		btlAlignedObjectArray<int>	ranks;
		btlAlignedObjectArray<int>	todelete;
		ranks.resize(nnodes,0);
		for(i=0,ni=m_links.size();i<ni;++i)
		{
			for(int j=0;j<2;++j) ranks[int(m_links[i].m_n[j]-nbase)]++;
		}
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			for(int j=0;j<3;++j) ranks[int(m_faces[i].m_n[j]-nbase)]++;
		}
		for(i=0;i<m_links.size();++i)
		{
			const int	id[]={	int(m_links[i].m_n[0]-nbase),
				int(m_links[i].m_n[1]-nbase)};
			const bool	sg[]={	ranks[id[0]]==1,
				ranks[id[1]]==1};
			if(sg[0]||sg[1])
			{
				--ranks[id[0]];
				--ranks[id[1]];
				btlSwap(m_links[i],m_links[m_links.size()-1]);
				m_links.pop_back();--i;
			}
		}
#if 0	
		for(i=nnodes-1;i>=0;--i)
		{
			if(!ranks[i]) todelete.push_back(i);
		}	
		if(todelete.size())
		{		
			btlAlignedObjectArray<int>&	map=ranks;
			for(int i=0;i<nnodes;++i) map[i]=i;
			PointersToIndices(this);
			for(int i=0,ni=todelete.size();i<ni;++i)
			{
				int		j=todelete[i];
				int&	a=map[j];
				int&	b=map[--nnodes];
				m_ndbvt.remove(m_nodes[a].m_leaf);m_nodes[a].m_leaf=0;
				btlSwap(m_nodes[a],m_nodes[b]);
				j=a;a=b;b=j;			
			}
			IndicesToPointers(this,&map[0]);
			m_nodes.resize(nnodes);
		}
#endif
	}
	m_bUpdateRtCst=true;
}

//
bool			btlSoftBody::cutLink(const Node* node0,const Node* node1,btlScalar position)
{
	return(cutLink(int(node0-&m_nodes[0]),int(node1-&m_nodes[0]),position));
}

//
bool			btlSoftBody::cutLink(int node0,int node1,btlScalar position)
{
	bool			done=false;
	int i,ni;
//	const btVector3	d=m_nodes[node0].m_x-m_nodes[node1].m_x;
	const btlVector3	x=Lerp(m_nodes[node0].m_x,m_nodes[node1].m_x,position);
	const btlVector3	v=Lerp(m_nodes[node0].m_v,m_nodes[node1].m_v,position);
	const btlScalar	m=1;
	appendNode(x,m);
	appendNode(x,m);
	Node*			pa=&m_nodes[node0];
	Node*			pb=&m_nodes[node1];
	Node*			pn[2]={	&m_nodes[m_nodes.size()-2],
		&m_nodes[m_nodes.size()-1]};
	pn[0]->m_v=v;
	pn[1]->m_v=v;
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		const int mtch=MatchEdge(m_links[i].m_n[0],m_links[i].m_n[1],pa,pb);
		if(mtch!=-1)
		{
			appendLink(i);
			Link*	pft[]={&m_links[i],&m_links[m_links.size()-1]};
			pft[0]->m_n[1]=pn[mtch];
			pft[1]->m_n[0]=pn[1-mtch];
			done=true;
		}
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		for(int k=2,l=0;l<3;k=l++)
		{
			const int mtch=MatchEdge(m_faces[i].m_n[k],m_faces[i].m_n[l],pa,pb);
			if(mtch!=-1)
			{
				appendFace(i);
				Face*	pft[]={&m_faces[i],&m_faces[m_faces.size()-1]};
				pft[0]->m_n[l]=pn[mtch];
				pft[1]->m_n[k]=pn[1-mtch];
				appendLink(pn[0],pft[0]->m_n[(l+1)%3],pft[0]->m_material,true);
				appendLink(pn[1],pft[0]->m_n[(l+1)%3],pft[0]->m_material,true);
			}
		}
	}
	if(!done)
	{
		m_ndbvt.remove(pn[0]->m_leaf);
		m_ndbvt.remove(pn[1]->m_leaf);
		m_nodes.pop_back();
		m_nodes.pop_back();
	}
	return(done);
}

//
bool			btlSoftBody::rayTest(const btlVector3& rayFrom,
									const btlVector3& rayTo,
									sRayCast& results)
{
	if(m_faces.size()&&m_fdbvt.empty()) 
		initializeFaceTree();

	results.body	=	this;
	results.fraction = 1.f;
	results.feature	=	eFeature::None;
	results.index	=	-1;

	return(rayTest(rayFrom,rayTo,results.fraction,results.feature,results.index,false)!=0);
}

//
void			btlSoftBody::setSolver(eSolverPresets::_ preset)
{
	m_cfg.m_vsequence.clear();
	m_cfg.m_psequence.clear();
	m_cfg.m_dsequence.clear();
	switch(preset)
	{
	case	eSolverPresets::Positions:
		m_cfg.m_psequence.push_back(ePSolver::Anchors);
		m_cfg.m_psequence.push_back(ePSolver::RContacts);
		m_cfg.m_psequence.push_back(ePSolver::SContacts);
		m_cfg.m_psequence.push_back(ePSolver::Linear);	
		break;	
	case	eSolverPresets::Velocities:
		m_cfg.m_vsequence.push_back(eVSolver::Linear);

		m_cfg.m_psequence.push_back(ePSolver::Anchors);
		m_cfg.m_psequence.push_back(ePSolver::RContacts);
		m_cfg.m_psequence.push_back(ePSolver::SContacts);

		m_cfg.m_dsequence.push_back(ePSolver::Linear);
		break;
	}
}

//
void			btlSoftBody::predictMotion(btlScalar dt)
{

	int i,ni;

	/* Update				*/ 
	if(m_bUpdateRtCst)
	{
		m_bUpdateRtCst=false;
		updateConstants();
		m_fdbvt.clear();
		if(m_cfg.collisions&fCollision::VF_SS)
		{
			initializeFaceTree();			
		}
	}

	/* Prepare				*/ 
	m_sst.sdt		=	dt*m_cfg.timescale;
	m_sst.isdt		=	1/m_sst.sdt;
	m_sst.velmrg	=	m_sst.sdt*3;
	m_sst.radmrg	=	getCollisionShape()->getMargin();
	m_sst.updmrg	=	m_sst.radmrg*(btlScalar)0.25;
	/* Forces				*/ 
	addVelocity(m_worldInfo->m_gravity*m_sst.sdt);
	applyForces();
	/* Integrate			*/ 
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		n.m_q	=	n.m_x;
		btlVector3 deltaV = n.m_f*n.m_im*m_sst.sdt;
		{
			btlScalar maxDisplacement = m_worldInfo->m_maxDisplacement;
			btlScalar clampDeltaV = maxDisplacement/m_sst.sdt;
			for (int c=0;c<3;c++)
			{
				if (deltaV[c]>clampDeltaV)
				{
					deltaV[c] = clampDeltaV;
				}
				if (deltaV[c]<-clampDeltaV)
				{
					deltaV[c]=-clampDeltaV;
				}
			}
		}
		n.m_v	+=	deltaV;
		n.m_x	+=	n.m_v*m_sst.sdt;
		n.m_f	=	btlVector3(0,0,0);
	}
	/* Clusters				*/ 
	updateClusters();
	/* Bounds				*/ 
	updateBounds();	
	/* Nodes				*/ 
	ATTRIBUTE_ALIGNED16(btlDbvtVolume)	vol;
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		Node&	n=m_nodes[i];
		vol = btlDbvtVolume::FromCR(n.m_x,m_sst.radmrg);
		m_ndbvt.update(	n.m_leaf,
			vol,
			n.m_v*m_sst.velmrg,
			m_sst.updmrg);
	}
	/* Faces				*/ 
	if(!m_fdbvt.empty())
	{
		for(int i=0;i<m_faces.size();++i)
		{
			Face&			f=m_faces[i];
			const btlVector3	v=(	f.m_n[0]->m_v+
				f.m_n[1]->m_v+
				f.m_n[2]->m_v)/3;
			vol = VolumeOf(f,m_sst.radmrg);
			m_fdbvt.update(	f.m_leaf,
				vol,
				v*m_sst.velmrg,
				m_sst.updmrg);
		}
	}
	/* Pose					*/ 
	updatePose();
	/* Match				*/ 
	if(m_pose.m_bframe&&(m_cfg.kMT>0))
	{
		const btlMatrix3x3	posetrs=m_pose.m_rot;
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			Node&	n=m_nodes[i];
			if(n.m_im>0)
			{
				const btlVector3	x=posetrs*m_pose.m_pos[i]+m_pose.m_com;
				n.m_x=Lerp(n.m_x,x,m_cfg.kMT);
			}
		}
	}
	/* Clear contacts		*/ 
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	/* Optimize dbvt's		*/ 
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
	m_cdbvt.optimizeIncremental(1);
}

//
void			btlSoftBody::solveConstraints()
{

	/* Apply clusters		*/ 
	applyClusters(false);
	/* Prepare links		*/ 

	int i,ni;

	for(i=0,ni=m_links.size();i<ni;++i)
	{
		Link&	l=m_links[i];
		l.m_c3		=	l.m_n[1]->m_q-l.m_n[0]->m_q;
		l.m_c2		=	1/(l.m_c3.length2()*l.m_c0);
	}
	/* Prepare anchors		*/ 
	for(i=0,ni=m_anchors.size();i<ni;++i)
	{
		Anchor&			a=m_anchors[i];
		const btlVector3	ra=a.m_body->getWorldTransform().getBasis()*a.m_local;
		a.m_c0	=	ImpulseMatrix(	m_sst.sdt,
			a.m_node->m_im,
			a.m_body->getInvMass(),
			a.m_body->getInvInertiaTensorWorld(),
			ra);
		a.m_c1	=	ra;
		a.m_c2	=	m_sst.sdt*a.m_node->m_im;
		a.m_body->activate();
	}
	/* Solve velocities		*/ 
	if(m_cfg.viterations>0)
	{
		/* Solve			*/ 
		for(int isolve=0;isolve<m_cfg.viterations;++isolve)
		{
			for(int iseq=0;iseq<m_cfg.m_vsequence.size();++iseq)
			{
				getSolver(m_cfg.m_vsequence[iseq])(this,1);
			}
		}
		/* Update			*/ 
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			Node&	n=m_nodes[i];
			n.m_x	=	n.m_q+n.m_v*m_sst.sdt;
		}
	}
	/* Solve positions		*/ 
	if(m_cfg.piterations>0)
	{
		for(int isolve=0;isolve<m_cfg.piterations;++isolve)
		{
			const btlScalar ti=isolve/(btlScalar)m_cfg.piterations;
			for(int iseq=0;iseq<m_cfg.m_psequence.size();++iseq)
			{
				getSolver(m_cfg.m_psequence[iseq])(this,1,ti);
			}
		}
		const btlScalar	vc=m_sst.isdt*(1-m_cfg.kDP);
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			Node&	n=m_nodes[i];
			n.m_v	=	(n.m_x-n.m_q)*vc;
			n.m_f	=	btlVector3(0,0,0);		
		}
	}
	/* Solve drift			*/ 
	if(m_cfg.diterations>0)
	{
		const btlScalar	vcf=m_cfg.kVCF*m_sst.isdt;
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			Node&	n=m_nodes[i];
			n.m_q	=	n.m_x;
		}
		for(int idrift=0;idrift<m_cfg.diterations;++idrift)
		{
			for(int iseq=0;iseq<m_cfg.m_dsequence.size();++iseq)
			{
				getSolver(m_cfg.m_dsequence[iseq])(this,1,0);
			}
		}
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			Node&	n=m_nodes[i];
			n.m_v	+=	(n.m_x-n.m_q)*vcf;
		}
	}
	/* Apply clusters		*/ 
	dampClusters();
	applyClusters(true);
}

//
void			btlSoftBody::staticSolve(int iterations)
{
	for(int isolve=0;isolve<iterations;++isolve)
	{
		for(int iseq=0;iseq<m_cfg.m_psequence.size();++iseq)
		{
			getSolver(m_cfg.m_psequence[iseq])(this,1,0);
		}
	}
}

//
void			btlSoftBody::solveCommonConstraints(btlSoftBody** /*bodies*/,int /*count*/,int /*iterations*/)
{
	/// placeholder
}

//
void			btlSoftBody::solveClusters(const btlAlignedObjectArray<btlSoftBody*>& bodies)
{
	const int	nb=bodies.size();
	int			iterations=0;
	int i;

	for(i=0;i<nb;++i)
	{
		iterations=btlMax(iterations,bodies[i]->m_cfg.citerations);
	}
	for(i=0;i<nb;++i)
	{
		bodies[i]->prepareClusters(iterations);
	}
	for(i=0;i<iterations;++i)
	{
		const btlScalar sor=1;
		for(int j=0;j<nb;++j)
		{
			bodies[j]->solveClusters(sor);
		}
	}
	for(i=0;i<nb;++i)
	{
		bodies[i]->cleanupClusters();
	}
}

//
void			btlSoftBody::integrateMotion()
{
	/* Update			*/ 
	updateNormals();
}

//
btlSoftBody::RayFromToCaster::RayFromToCaster(const btlVector3& rayFrom,const btlVector3& rayTo,btlScalar mxt)
{
	m_rayFrom = rayFrom;
	m_rayNormalizedDirection = (rayTo-rayFrom);
	m_rayTo = rayTo;
	m_mint	=	mxt;
	m_face	=	0;
	m_tests	=	0;
}

//
void				btlSoftBody::RayFromToCaster::Process(const btlDbvtNode* leaf)
{
	btlSoftBody::Face&	f=*(btlSoftBody::Face*)leaf->data;
	const btlScalar		t=rayFromToTriangle(	m_rayFrom,m_rayTo,m_rayNormalizedDirection,
		f.m_n[0]->m_x,
		f.m_n[1]->m_x,
		f.m_n[2]->m_x,
		m_mint);
	if((t>0)&&(t<m_mint)) 
	{ 
		m_mint=t;m_face=&f; 
	}
	++m_tests;
}

//
btlScalar			btlSoftBody::RayFromToCaster::rayFromToTriangle(	const btlVector3& rayFrom,
																   const btlVector3& rayTo,
																   const btlVector3& rayNormalizedDirection,
																   const btlVector3& a,
																   const btlVector3& b,
																   const btlVector3& c,
																   btlScalar maxt)
{
	static const btlScalar	ceps=-SIMD_EPSILON*10;
	static const btlScalar	teps=SIMD_EPSILON*10;

	const btlVector3			n=btlCross(b-a,c-a);
	const btlScalar			d=btlDot(a,n);
	const btlScalar			den=btlDot(rayNormalizedDirection,n);
	if(!btlFuzzyZero(den))
	{
		const btlScalar		num=btlDot(rayFrom,n)-d;
		const btlScalar		t=-num/den;
		if((t>teps)&&(t<maxt))
		{
			const btlVector3	hit=rayFrom+rayNormalizedDirection*t;
			if(	(btlDot(n,btlCross(a-hit,b-hit))>ceps)	&&			
				(btlDot(n,btlCross(b-hit,c-hit))>ceps)	&&
				(btlDot(n,btlCross(c-hit,a-hit))>ceps))
			{
				return(t);
			}
		}
	}
	return(-1);
}

//
void				btlSoftBody::pointersToIndices()
{
#define	PTR2IDX(_p_,_b_)	reinterpret_cast<btlSoftBody::Node*>((_p_)-(_b_))
	btlSoftBody::Node*	base=m_nodes.size() ? &m_nodes[0] : 0;
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(m_nodes[i].m_leaf)
		{
			m_nodes[i].m_leaf->data=*(void**)&i;
		}
	}
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		m_links[i].m_n[0]=PTR2IDX(m_links[i].m_n[0],base);
		m_links[i].m_n[1]=PTR2IDX(m_links[i].m_n[1],base);
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		m_faces[i].m_n[0]=PTR2IDX(m_faces[i].m_n[0],base);
		m_faces[i].m_n[1]=PTR2IDX(m_faces[i].m_n[1],base);
		m_faces[i].m_n[2]=PTR2IDX(m_faces[i].m_n[2],base);
		if(m_faces[i].m_leaf)
		{
			m_faces[i].m_leaf->data=*(void**)&i;
		}
	}
	for(i=0,ni=m_anchors.size();i<ni;++i)
	{
		m_anchors[i].m_node=PTR2IDX(m_anchors[i].m_node,base);
	}
	for(i=0,ni=m_notes.size();i<ni;++i)
	{
		for(int j=0;j<m_notes[i].m_rank;++j)
		{
			m_notes[i].m_nodes[j]=PTR2IDX(m_notes[i].m_nodes[j],base);
		}
	}
#undef	PTR2IDX
}

//
void				btlSoftBody::indicesToPointers(const int* map)
{
#define	IDX2PTR(_p_,_b_)	map?(&(_b_)[map[(((char*)_p_)-(char*)0)]]):	\
	(&(_b_)[(((char*)_p_)-(char*)0)])
	btlSoftBody::Node*	base=m_nodes.size() ? &m_nodes[0]:0;
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(m_nodes[i].m_leaf)
		{
			m_nodes[i].m_leaf->data=&m_nodes[i];
		}
	}
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		m_links[i].m_n[0]=IDX2PTR(m_links[i].m_n[0],base);
		m_links[i].m_n[1]=IDX2PTR(m_links[i].m_n[1],base);
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		m_faces[i].m_n[0]=IDX2PTR(m_faces[i].m_n[0],base);
		m_faces[i].m_n[1]=IDX2PTR(m_faces[i].m_n[1],base);
		m_faces[i].m_n[2]=IDX2PTR(m_faces[i].m_n[2],base);
		if(m_faces[i].m_leaf)
		{
			m_faces[i].m_leaf->data=&m_faces[i];
		}
	}
	for(i=0,ni=m_anchors.size();i<ni;++i)
	{
		m_anchors[i].m_node=IDX2PTR(m_anchors[i].m_node,base);
	}
	for(i=0,ni=m_notes.size();i<ni;++i)
	{
		for(int j=0;j<m_notes[i].m_rank;++j)
		{
			m_notes[i].m_nodes[j]=IDX2PTR(m_notes[i].m_nodes[j],base);
		}
	}
#undef	IDX2PTR
}

//
int					btlSoftBody::rayTest(const btlVector3& rayFrom,const btlVector3& rayTo,
										btlScalar& mint,eFeature::_& feature,int& index,bool bcountonly) const
{
	int	cnt=0;
	btlVector3 dir = rayTo-rayFrom;
	

	if(bcountonly||m_fdbvt.empty())
	{/* Full search	*/ 
		
		for(int i=0,ni=m_faces.size();i<ni;++i)
		{
			const btlSoftBody::Face&	f=m_faces[i];

			const btlScalar			t=RayFromToCaster::rayFromToTriangle(	rayFrom,rayTo,dir,
				f.m_n[0]->m_x,
				f.m_n[1]->m_x,
				f.m_n[2]->m_x,
				mint);
			if(t>0)
			{
				++cnt;
				if(!bcountonly)
				{
					feature=btlSoftBody::eFeature::Face;
					index=i;
					mint=t;
				}
			}
		}
	}
	else
	{/* Use dbvt	*/ 
		RayFromToCaster	collider(rayFrom,rayTo,mint);

		btlDbvt::rayTest(m_fdbvt.m_root,rayFrom,rayTo,collider);
		if(collider.m_face)
		{
			mint=collider.m_mint;
			feature=btlSoftBody::eFeature::Face;
			index=(int)(collider.m_face-&m_faces[0]);
			cnt=1;
		}
	}

	for (int i=0;i<m_tetras.size();i++)
	{
		const btlSoftBody::Tetra& tet = m_tetras[i];
		int tetfaces[4][3] = {{0,1,2},{0,1,3},{1,2,3},{0,2,3}};
		for (int f=0;f<4;f++)
		{

			int index0=tetfaces[f][0];
			int index1=tetfaces[f][1];
			int index2=tetfaces[f][2];
			btlVector3 v0=tet.m_n[index0]->m_x;
			btlVector3 v1=tet.m_n[index1]->m_x;
			btlVector3 v2=tet.m_n[index2]->m_x;


		const btlScalar			t=RayFromToCaster::rayFromToTriangle(	rayFrom,rayTo,dir,
			v0,v1,v2,
				mint);
		if(t>0)
			{
				++cnt;
				if(!bcountonly)
				{
					feature=btlSoftBody::eFeature::Tetra;
					index=i;
					mint=t;
				}
			}
		}
	}
	return(cnt);
}

//
void			btlSoftBody::initializeFaceTree()
{
	m_fdbvt.clear();
	for(int i=0;i<m_faces.size();++i)
	{
		Face&	f=m_faces[i];
		f.m_leaf=m_fdbvt.insert(VolumeOf(f,0),&f);
	}
}

//
btlVector3		btlSoftBody::evaluateCom() const
{
	btlVector3	com(0,0,0);
	if(m_pose.m_bframe)
	{
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			com+=m_nodes[i].m_x*m_pose.m_wgh[i];
		}
	}
	return(com);
}

//
bool				btlSoftBody::checkContact(	const btlCollisionObjectWrapper* colObjWrap,
											 const btlVector3& x,
											 btlScalar margin,
											 btlSoftBody::sCti& cti) const
{
	btlVector3 nrm;
	const btlCollisionShape *shp = colObjWrap->getCollisionShape();
//	const btRigidBody *tmpRigid = btRigidBody::upcast(colObjWrap->getCollisionObject());
	//const btTransform &wtr = tmpRigid ? tmpRigid->getWorldTransform() : colObjWrap->getWorldTransform();
	const btlTransform &wtr = colObjWrap->getWorldTransform();
	//todo: check which transform is needed here

	btlScalar dst = 
		m_worldInfo->m_sparsesdf.Evaluate(	
			wtr.invXform(x),
			shp,
			nrm,
			margin);
	if(dst<0)
	{
		cti.m_colObj = colObjWrap->getCollisionObject();
		cti.m_normal = wtr.getBasis()*nrm;
		cti.m_offset = -btlDot( cti.m_normal, x - cti.m_normal * dst );
		return(true);
	}
	return(false);
}

//
void					btlSoftBody::updateNormals()
{

	const btlVector3	zv(0,0,0);
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		m_nodes[i].m_n=zv;
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		btlSoftBody::Face&	f=m_faces[i];
		const btlVector3		n=btlCross(f.m_n[1]->m_x-f.m_n[0]->m_x,
			f.m_n[2]->m_x-f.m_n[0]->m_x);
		f.m_normal=n.normalized();
		f.m_n[0]->m_n+=n;
		f.m_n[1]->m_n+=n;
		f.m_n[2]->m_n+=n;
	}
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		btlScalar len = m_nodes[i].m_n.length();
		if (len>SIMD_EPSILON)
			m_nodes[i].m_n /= len;
	}
}

//
void					btlSoftBody::updateBounds()
{
	/*if( m_acceleratedSoftBody )
	{
		// If we have an accelerated softbody we need to obtain the bounds correctly
		// For now (slightly hackily) just have a very large AABB
		// TODO: Write get bounds kernel
		// If that is updating in place, atomic collisions might be low (when the cloth isn't perfectly aligned to an axis) and we could
		// probably do a test and exchange reasonably efficiently.

		m_bounds[0] = btVector3(-1000, -1000, -1000);
		m_bounds[1] = btVector3(1000, 1000, 1000);

	} else {*/
		if(m_ndbvt.m_root)
		{
			const btlVector3&	mins=m_ndbvt.m_root->volume.Mins();
			const btlVector3&	maxs=m_ndbvt.m_root->volume.Maxs();
			const btlScalar		csm=getCollisionShape()->getMargin();
			const btlVector3		mrg=btlVector3(	csm,
				csm,
				csm)*1; // ??? to investigate...
			m_bounds[0]=mins-mrg;
			m_bounds[1]=maxs+mrg;
			if(0!=getBroadphaseHandle())
			{					
				m_worldInfo->m_broadphase->setAabb(	getBroadphaseHandle(),
					m_bounds[0],
					m_bounds[1],
					m_worldInfo->m_dispatcher);
			}
		}
		else
		{
			m_bounds[0]=
				m_bounds[1]=btlVector3(0,0,0);
		}		
	//}
}


//
void					btlSoftBody::updatePose()
{
	if(m_pose.m_bframe)
	{
		btlSoftBody::Pose&	pose=m_pose;
		const btlVector3		com=evaluateCom();
		/* Com			*/ 
		pose.m_com	=	com;
		/* Rotation		*/ 
		btlMatrix3x3		Apq;
		const btlScalar	eps=SIMD_EPSILON;
		Apq[0]=Apq[1]=Apq[2]=btlVector3(0,0,0);
		Apq[0].setX(eps);Apq[1].setY(eps*2);Apq[2].setZ(eps*3);
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			const btlVector3		a=pose.m_wgh[i]*(m_nodes[i].m_x-com);
			const btlVector3&	b=pose.m_pos[i];
			Apq[0]+=a.x()*b;
			Apq[1]+=a.y()*b;
			Apq[2]+=a.z()*b;
		}
		btlMatrix3x3		r,s;
		PolarDecompose(Apq,r,s);
		pose.m_rot=r;
		pose.m_scl=pose.m_aqq*r.transpose()*Apq;
		if(m_cfg.maxvolume>1)
		{
			const btlScalar	idet=Clamp<btlScalar>(	1/pose.m_scl.determinant(),
				1,m_cfg.maxvolume);
			pose.m_scl=Mul(pose.m_scl,idet);
		}

	}
}

//
void				btlSoftBody::updateArea(bool averageArea)
{
	int i,ni;

	/* Face area		*/ 
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		Face&		f=m_faces[i];
		f.m_ra	=	AreaOf(f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x);
	}
	
	/* Node area		*/ 

	if (averageArea)
	{
		btlAlignedObjectArray<int>	counts;
		counts.resize(m_nodes.size(),0);
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			m_nodes[i].m_area	=	0;
		}
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			btlSoftBody::Face&	f=m_faces[i];
			for(int j=0;j<3;++j)
			{
				const int index=(int)(f.m_n[j]-&m_nodes[0]);
				counts[index]++;
				f.m_n[j]->m_area+=btlFabs(f.m_ra);
			}
		}
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			if(counts[i]>0)
				m_nodes[i].m_area/=(btlScalar)counts[i];
			else
				m_nodes[i].m_area=0;
		}
	}
	else
	{
		// initialize node area as zero
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			m_nodes[i].m_area=0;	
		}

		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			btlSoftBody::Face&	f=m_faces[i];

			for(int j=0;j<3;++j)
			{
				f.m_n[j]->m_area += f.m_ra;
			}
		}

		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			m_nodes[i].m_area *= 0.3333333f;
		}
	}
}


void				btlSoftBody::updateLinkConstants()
{	
	int i,ni;

	/* Links		*/ 
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		Link&		l=m_links[i];
		Material&	m=*l.m_material;
		l.m_c0	=	(l.m_n[0]->m_im+l.m_n[1]->m_im)/m.m_kLST;
	}
}

void				btlSoftBody::updateConstants()
{
	resetLinkRestLengths();
	updateLinkConstants();
	updateArea();
}



//
void					btlSoftBody::initializeClusters()
{
	int i;

	for( i=0;i<m_clusters.size();++i)
	{
		Cluster&	c=*m_clusters[i];
		c.m_imass=0;
		c.m_masses.resize(c.m_nodes.size());
		for(int j=0;j<c.m_nodes.size();++j)
		{
			if (c.m_nodes[j]->m_im==0)
			{
				c.m_containsAnchor = true;
				c.m_masses[j]	=	BT_LARGE_FLOAT;
			} else
			{
				c.m_masses[j]	=	btlScalar(1.)/c.m_nodes[j]->m_im;
			}
			c.m_imass		+=	c.m_masses[j];
		}
		c.m_imass		=	btlScalar(1.)/c.m_imass;
		c.m_com			=	btlSoftBody::clusterCom(&c);
		c.m_lv			=	btlVector3(0,0,0);
		c.m_av			=	btlVector3(0,0,0);
		c.m_leaf		=	0;
		/* Inertia	*/ 
		btlMatrix3x3&	ii=c.m_locii;
		ii[0]=ii[1]=ii[2]=btlVector3(0,0,0);
		{
			int i,ni;

			for(i=0,ni=c.m_nodes.size();i<ni;++i)
			{
				const btlVector3	k=c.m_nodes[i]->m_x-c.m_com;
				const btlVector3	q=k*k;
				const btlScalar	m=c.m_masses[i];
				ii[0][0]	+=	m*(q[1]+q[2]);
				ii[1][1]	+=	m*(q[0]+q[2]);
				ii[2][2]	+=	m*(q[0]+q[1]);
				ii[0][1]	-=	m*k[0]*k[1];
				ii[0][2]	-=	m*k[0]*k[2];
				ii[1][2]	-=	m*k[1]*k[2];
			}
		}
		ii[1][0]=ii[0][1];
		ii[2][0]=ii[0][2];
		ii[2][1]=ii[1][2];
		
		ii = ii.inverse();

		/* Frame	*/ 
		c.m_framexform.setIdentity();
		c.m_framexform.setOrigin(c.m_com);
		c.m_framerefs.resize(c.m_nodes.size());
		{
			int i;
			for(i=0;i<c.m_framerefs.size();++i)
			{
				c.m_framerefs[i]=c.m_nodes[i]->m_x-c.m_com;
			}
		}
	}
}

//
void					btlSoftBody::updateClusters()
{
	BT_PROFILE("UpdateClusters");
	int i;

	for(i=0;i<m_clusters.size();++i)
	{
		btlSoftBody::Cluster&	c=*m_clusters[i];
		const int				n=c.m_nodes.size();
		//const btScalar			invn=1/(btScalar)n;
		if(n)
		{
			/* Frame				*/ 
			const btlScalar	eps=btlScalar(0.0001);
			btlMatrix3x3		m,r,s;
			m[0]=m[1]=m[2]=btlVector3(0,0,0);
			m[0][0]=eps*1;
			m[1][1]=eps*2;
			m[2][2]=eps*3;
			c.m_com=clusterCom(&c);
			for(int i=0;i<c.m_nodes.size();++i)
			{
				const btlVector3		a=c.m_nodes[i]->m_x-c.m_com;
				const btlVector3&	b=c.m_framerefs[i];
				m[0]+=a[0]*b;m[1]+=a[1]*b;m[2]+=a[2]*b;
			}
			PolarDecompose(m,r,s);
			c.m_framexform.setOrigin(c.m_com);
			c.m_framexform.setBasis(r);		
			/* Inertia			*/ 
#if 1/* Constant	*/ 
			c.m_invwi=c.m_framexform.getBasis()*c.m_locii*c.m_framexform.getBasis().transpose();
#else
#if 0/* Sphere	*/ 
			const btlScalar	rk=(2*c.m_extents.length2())/(5*c.m_imass);
			const btlVector3	inertia(rk,rk,rk);
			const btlVector3	iin(btlFabs(inertia[0])>SIMD_EPSILON?1/inertia[0]:0,
				btlFabs(inertia[1])>SIMD_EPSILON?1/inertia[1]:0,
				btlFabs(inertia[2])>SIMD_EPSILON?1/inertia[2]:0);

			c.m_invwi=c.m_xform.getBasis().scaled(iin)*c.m_xform.getBasis().transpose();
#else/* Actual	*/ 		
			c.m_invwi[0]=c.m_invwi[1]=c.m_invwi[2]=btlVector3(0,0,0);
			for(int i=0;i<n;++i)
			{
				const btlVector3	k=c.m_nodes[i]->m_x-c.m_com;
				const btlVector3		q=k*k;
				const btlScalar		m=1/c.m_nodes[i]->m_im;
				c.m_invwi[0][0]	+=	m*(q[1]+q[2]);
				c.m_invwi[1][1]	+=	m*(q[0]+q[2]);
				c.m_invwi[2][2]	+=	m*(q[0]+q[1]);
				c.m_invwi[0][1]	-=	m*k[0]*k[1];
				c.m_invwi[0][2]	-=	m*k[0]*k[2];
				c.m_invwi[1][2]	-=	m*k[1]*k[2];
			}
			c.m_invwi[1][0]=c.m_invwi[0][1];
			c.m_invwi[2][0]=c.m_invwi[0][2];
			c.m_invwi[2][1]=c.m_invwi[1][2];
			c.m_invwi=c.m_invwi.inverse();
#endif
#endif
			/* Velocities			*/ 
			c.m_lv=btlVector3(0,0,0);
			c.m_av=btlVector3(0,0,0);
			{
				int i;

				for(i=0;i<n;++i)
				{
					const btlVector3	v=c.m_nodes[i]->m_v*c.m_masses[i];
					c.m_lv	+=	v;
					c.m_av	+=	btlCross(c.m_nodes[i]->m_x-c.m_com,v);
				}
			}
			c.m_lv=c.m_imass*c.m_lv*(1-c.m_ldamping);
			c.m_av=c.m_invwi*c.m_av*(1-c.m_adamping);
			c.m_vimpulses[0]	=
				c.m_vimpulses[1]	= btlVector3(0,0,0);
			c.m_dimpulses[0]	=
				c.m_dimpulses[1]	= btlVector3(0,0,0);
			c.m_nvimpulses		= 0;
			c.m_ndimpulses		= 0;
			/* Matching				*/ 
			if(c.m_matching>0)
			{
				for(int j=0;j<c.m_nodes.size();++j)
				{
					Node&			n=*c.m_nodes[j];
					const btlVector3	x=c.m_framexform*c.m_framerefs[j];
					n.m_x=Lerp(n.m_x,x,c.m_matching);
				}
			}			
			/* Dbvt					*/ 
			if(c.m_collide)
			{
				btlVector3	mi=c.m_nodes[0]->m_x;
				btlVector3	mx=mi;
				for(int j=1;j<n;++j)
				{
					mi.setMin(c.m_nodes[j]->m_x);
					mx.setMax(c.m_nodes[j]->m_x);
				}			
				ATTRIBUTE_ALIGNED16(btlDbvtVolume)	bounds=btlDbvtVolume::FromMM(mi,mx);
				if(c.m_leaf)
					m_cdbvt.update(c.m_leaf,bounds,c.m_lv*m_sst.sdt*3,m_sst.radmrg);
				else
					c.m_leaf=m_cdbvt.insert(bounds,&c);
			}
		}
	}


}




//
void					btlSoftBody::cleanupClusters()
{
	for(int i=0;i<m_joints.size();++i)
	{
		m_joints[i]->Terminate(m_sst.sdt);
		if(m_joints[i]->m_delete)
		{
			btlAlignedFree(m_joints[i]);
			m_joints.remove(m_joints[i--]);
		}	
	}
}

//
void					btlSoftBody::prepareClusters(int iterations)
{
	for(int i=0;i<m_joints.size();++i)
	{
		m_joints[i]->Prepare(m_sst.sdt,iterations);
	}
}


//
void					btlSoftBody::solveClusters(btlScalar sor)
{
	for(int i=0,ni=m_joints.size();i<ni;++i)
	{
		m_joints[i]->Solve(m_sst.sdt,sor);
	}
}

//
void					btlSoftBody::applyClusters(bool drift)
{
	BT_PROFILE("ApplyClusters");
//	const btScalar					f0=m_sst.sdt;
	//const btScalar					f1=f0/2;
	btlAlignedObjectArray<btlVector3> deltas;
	btlAlignedObjectArray<btlScalar> weights;
	deltas.resize(m_nodes.size(),btlVector3(0,0,0));
	weights.resize(m_nodes.size(),0);
	int i;

	if(drift)
	{
		for(i=0;i<m_clusters.size();++i)
		{
			Cluster&	c=*m_clusters[i];
			if(c.m_ndimpulses)
			{
				c.m_dimpulses[0]/=(btlScalar)c.m_ndimpulses;
				c.m_dimpulses[1]/=(btlScalar)c.m_ndimpulses;
			}
		}
	}
	
	for(i=0;i<m_clusters.size();++i)
	{
		Cluster&	c=*m_clusters[i];	
		if(0<(drift?c.m_ndimpulses:c.m_nvimpulses))
		{
			const btlVector3		v=(drift?c.m_dimpulses[0]:c.m_vimpulses[0])*m_sst.sdt;
			const btlVector3		w=(drift?c.m_dimpulses[1]:c.m_vimpulses[1])*m_sst.sdt;
			for(int j=0;j<c.m_nodes.size();++j)
			{
				const int			idx=int(c.m_nodes[j]-&m_nodes[0]);
				const btlVector3&	x=c.m_nodes[j]->m_x;
				const btlScalar		q=c.m_masses[j];
				deltas[idx]		+=	(v+btlCross(w,x-c.m_com))*q;
				weights[idx]	+=	q;
			}
		}
	}
	for(i=0;i<deltas.size();++i)
	{
		if(weights[i]>0) 
		{
			m_nodes[i].m_x+=deltas[i]/weights[i];
		}
	}
}

//
void					btlSoftBody::dampClusters()
{
	int i;

	for(i=0;i<m_clusters.size();++i)
	{
		Cluster&	c=*m_clusters[i];	
		if(c.m_ndamping>0)
		{
			for(int j=0;j<c.m_nodes.size();++j)
			{
				Node&			n=*c.m_nodes[j];
				if(n.m_im>0)
				{
					const btlVector3	vx=c.m_lv+btlCross(c.m_av,c.m_nodes[j]->m_q-c.m_com);
					if(vx.length2()<=n.m_v.length2())
						{
						n.m_v	+=	c.m_ndamping*(vx-n.m_v);
						}
				}
			}
		}
	}
}

//
void				btlSoftBody::Joint::Prepare(btlScalar dt,int)
{
	m_bodies[0].activate();
	m_bodies[1].activate();
}

//
void				btlSoftBody::LJoint::Prepare(btlScalar dt,int iterations)
{
	static const btlScalar	maxdrift=4;
	Joint::Prepare(dt,iterations);
	m_rpos[0]		=	m_bodies[0].xform()*m_refs[0];
	m_rpos[1]		=	m_bodies[1].xform()*m_refs[1];
	m_drift			=	Clamp(m_rpos[0]-m_rpos[1],maxdrift)*m_erp/dt;
	m_rpos[0]		-=	m_bodies[0].xform().getOrigin();
	m_rpos[1]		-=	m_bodies[1].xform().getOrigin();
	m_massmatrix	=	ImpulseMatrix(	m_bodies[0].invMass(),m_bodies[0].invWorldInertia(),m_rpos[0],
		m_bodies[1].invMass(),m_bodies[1].invWorldInertia(),m_rpos[1]);
	if(m_split>0)
	{
		m_sdrift	=	m_massmatrix*(m_drift*m_split);
		m_drift		*=	1-m_split;
	}
	m_drift	/=(btlScalar)iterations;
}

//
void				btlSoftBody::LJoint::Solve(btlScalar dt,btlScalar sor)
{
	const btlVector3		va=m_bodies[0].velocity(m_rpos[0]);
	const btlVector3		vb=m_bodies[1].velocity(m_rpos[1]);
	const btlVector3		vr=va-vb;
	btlSoftBody::Impulse	impulse;
	impulse.m_asVelocity	=	1;
	impulse.m_velocity		=	m_massmatrix*(m_drift+vr*m_cfm)*sor;
	m_bodies[0].applyImpulse(-impulse,m_rpos[0]);
	m_bodies[1].applyImpulse( impulse,m_rpos[1]);
}

//
void				btlSoftBody::LJoint::Terminate(btlScalar dt)
{
	if(m_split>0)
	{
		m_bodies[0].applyDImpulse(-m_sdrift,m_rpos[0]);
		m_bodies[1].applyDImpulse( m_sdrift,m_rpos[1]);
	}
}

//
void				btlSoftBody::AJoint::Prepare(btlScalar dt,int iterations)
{
	static const btlScalar	maxdrift=SIMD_PI/16;
	m_icontrol->Prepare(this);
	Joint::Prepare(dt,iterations);
	m_axis[0]	=	m_bodies[0].xform().getBasis()*m_refs[0];
	m_axis[1]	=	m_bodies[1].xform().getBasis()*m_refs[1];
	m_drift		=	NormalizeAny(btlCross(m_axis[1],m_axis[0]));
	m_drift		*=	btlMin(maxdrift,btlAcos(Clamp<btlScalar>(btlDot(m_axis[0],m_axis[1]),-1,+1)));
	m_drift		*=	m_erp/dt;
	m_massmatrix=	AngularImpulseMatrix(m_bodies[0].invWorldInertia(),m_bodies[1].invWorldInertia());
	if(m_split>0)
	{
		m_sdrift	=	m_massmatrix*(m_drift*m_split);
		m_drift		*=	1-m_split;
	}
	m_drift	/=(btlScalar)iterations;
}

//
void				btlSoftBody::AJoint::Solve(btlScalar dt,btlScalar sor)
{
	const btlVector3		va=m_bodies[0].angularVelocity();
	const btlVector3		vb=m_bodies[1].angularVelocity();
	const btlVector3		vr=va-vb;
	const btlScalar		sp=btlDot(vr,m_axis[0]);
	const btlVector3		vc=vr-m_axis[0]*m_icontrol->Speed(this,sp);
	btlSoftBody::Impulse	impulse;
	impulse.m_asVelocity	=	1;
	impulse.m_velocity		=	m_massmatrix*(m_drift+vc*m_cfm)*sor;
	m_bodies[0].applyAImpulse(-impulse);
	m_bodies[1].applyAImpulse( impulse);
}

//
void				btlSoftBody::AJoint::Terminate(btlScalar dt)
{
	if(m_split>0)
	{
		m_bodies[0].applyDAImpulse(-m_sdrift);
		m_bodies[1].applyDAImpulse( m_sdrift);
	}
}

//
void				btlSoftBody::CJoint::Prepare(btlScalar dt,int iterations)
{
	Joint::Prepare(dt,iterations);
	const bool	dodrift=(m_life==0);
	m_delete=(++m_life)>m_maxlife;
	if(dodrift)
	{
		m_drift=m_drift*m_erp/dt;
		if(m_split>0)
		{
			m_sdrift	=	m_massmatrix*(m_drift*m_split);
			m_drift		*=	1-m_split;
		}
		m_drift/=(btlScalar)iterations;
	}
	else
	{
		m_drift=m_sdrift=btlVector3(0,0,0);
	}
}

//
void				btlSoftBody::CJoint::Solve(btlScalar dt,btlScalar sor)
{
	const btlVector3		va=m_bodies[0].velocity(m_rpos[0]);
	const btlVector3		vb=m_bodies[1].velocity(m_rpos[1]);
	const btlVector3		vrel=va-vb;
	const btlScalar		rvac=btlDot(vrel,m_normal);
	btlSoftBody::Impulse	impulse;
	impulse.m_asVelocity	=	1;
	impulse.m_velocity		=	m_drift;
	if(rvac<0)
	{
		const btlVector3	iv=m_normal*rvac;
		const btlVector3	fv=vrel-iv;
		impulse.m_velocity	+=	iv+fv*m_friction;
	}
	impulse.m_velocity=m_massmatrix*impulse.m_velocity*sor;
	
	if (m_bodies[0].m_soft==m_bodies[1].m_soft)
	{
		if ((impulse.m_velocity.getX() ==impulse.m_velocity.getX())&&(impulse.m_velocity.getY() ==impulse.m_velocity.getY())&&
			(impulse.m_velocity.getZ() ==impulse.m_velocity.getZ()))
		{
			if (impulse.m_asVelocity)
			{
				if (impulse.m_velocity.length() <m_bodies[0].m_soft->m_maxSelfCollisionImpulse)
				{
					
				} else
				{
					m_bodies[0].applyImpulse(-impulse*m_bodies[0].m_soft->m_selfCollisionImpulseFactor,m_rpos[0]);
					m_bodies[1].applyImpulse( impulse*m_bodies[0].m_soft->m_selfCollisionImpulseFactor,m_rpos[1]);
				}
			}
		}
	} else
	{
		m_bodies[0].applyImpulse(-impulse,m_rpos[0]);
		m_bodies[1].applyImpulse( impulse,m_rpos[1]);
	}
}

//
void				btlSoftBody::CJoint::Terminate(btlScalar dt)
{
	if(m_split>0)
	{
		m_bodies[0].applyDImpulse(-m_sdrift,m_rpos[0]);
		m_bodies[1].applyDImpulse( m_sdrift,m_rpos[1]);
	}
}

//
void				btlSoftBody::applyForces()
{

	BT_PROFILE("SoftBody applyForces");
//	const btScalar					dt =			m_sst.sdt;
	const btlScalar					kLF =			m_cfg.kLF;
	const btlScalar					kDG =			m_cfg.kDG;
	const btlScalar					kPR =			m_cfg.kPR;
	const btlScalar					kVC =			m_cfg.kVC;
	const bool						as_lift =		kLF>0;
	const bool						as_drag =		kDG>0;
	const bool						as_pressure =	kPR!=0;
	const bool						as_volume =		kVC>0;
	const bool						as_aero =		as_lift	||
													as_drag		;
	//const bool						as_vaero =		as_aero	&&
	//												(m_cfg.aeromodel < btSoftBody::eAeroModel::F_TwoSided);
	//const bool						as_faero =		as_aero	&&
	//												(m_cfg.aeromodel >= btSoftBody::eAeroModel::F_TwoSided);
	const bool						use_medium =	as_aero;
	const bool						use_volume =	as_pressure	||
		as_volume	;
	btlScalar						volume =		0;
	btlScalar						ivolumetp =		0;
	btlScalar						dvolumetv =		0;
	btlSoftBody::sMedium	medium;
	if(use_volume)
	{
		volume		=	getVolume();
		ivolumetp	=	1/btlFabs(volume)*kPR;
		dvolumetv	=	(m_pose.m_volume-volume)*kVC;
	}
	/* Per vertex forces			*/ 
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		btlSoftBody::Node&	n=m_nodes[i];
		if(n.m_im>0)
		{
			if(use_medium)
			{
				/* Aerodynamics			*/ 
				addAeroForceToNode(m_windVelocity, i);
			}
			/* Pressure				*/ 
			if(as_pressure)
			{
				n.m_f	+=	n.m_n*(n.m_area*ivolumetp);
			}
			/* Volume				*/ 
			if(as_volume)
			{
				n.m_f	+=	n.m_n*(n.m_area*dvolumetv);
			}
		}
	}

	/* Per face forces				*/ 
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
	//	btSoftBody::Face&	f=m_faces[i];

		/* Aerodynamics			*/ 
		addAeroForceToFace(m_windVelocity, i);	
	}
}

//
void				btlSoftBody::PSolve_Anchors(btlSoftBody* psb,btlScalar kst,btlScalar ti)
{
	const btlScalar	kAHR=psb->m_cfg.kAHR*kst;
	const btlScalar	dt=psb->m_sst.sdt;
	for(int i=0,ni=psb->m_anchors.size();i<ni;++i)
	{
		const Anchor&		a=psb->m_anchors[i];
		const btlTransform&	t=a.m_body->getWorldTransform();
		Node&				n=*a.m_node;
		const btlVector3		wa=t*a.m_local;
		const btlVector3		va=a.m_body->getVelocityInLocalPoint(a.m_c1)*dt;
		const btlVector3		vb=n.m_x-n.m_q;
		const btlVector3		vr=(va-vb)+(wa-n.m_x)*kAHR;
		const btlVector3		impulse=a.m_c0*vr*a.m_influence;
		n.m_x+=impulse*a.m_c2;
		a.m_body->applyImpulse(-impulse,a.m_c1);
	}
}

//
void btlSoftBody::PSolve_RContacts(btlSoftBody* psb, btlScalar kst, btlScalar ti)
{
	const btlScalar	dt = psb->m_sst.sdt;
	const btlScalar	mrg = psb->getCollisionShape()->getMargin();
	for(int i=0,ni=psb->m_rcontacts.size();i<ni;++i)
	{
		const RContact&		c = psb->m_rcontacts[i];
		const sCti&			cti = c.m_cti;	
		if (cti.m_colObj->hasContactResponse()) 
		{
			btlRigidBody* tmpRigid = (btlRigidBody*)btlRigidBody::upcast(cti.m_colObj);
			const btlVector3		va = tmpRigid ? tmpRigid->getVelocityInLocalPoint(c.m_c1)*dt : btlVector3(0,0,0);
			const btlVector3		vb = c.m_node->m_x-c.m_node->m_q;	
			const btlVector3		vr = vb-va;
			const btlScalar		dn = btlDot(vr, cti.m_normal);		
			if(dn<=SIMD_EPSILON)
			{
				const btlScalar		dp = btlMin( (btlDot(c.m_node->m_x, cti.m_normal) + cti.m_offset), mrg );
				const btlVector3		fv = vr - (cti.m_normal * dn);
				// c0 is the impulse matrix, c3 is 1 - the friction coefficient or 0, c4 is the contact hardness coefficient
				const btlVector3		impulse = c.m_c0 * ( (vr - (fv * c.m_c3) + (cti.m_normal * (dp * c.m_c4))) * kst );
				c.m_node->m_x -= impulse * c.m_c2;
				if (tmpRigid)
					tmpRigid->applyImpulse(impulse,c.m_c1);
			}
		}
	}
}

//
void				btlSoftBody::PSolve_SContacts(btlSoftBody* psb,btlScalar,btlScalar ti)
{
	for(int i=0,ni=psb->m_scontacts.size();i<ni;++i)
	{
		const SContact&		c=psb->m_scontacts[i];
		const btlVector3&	nr=c.m_normal;
		Node&				n=*c.m_node;
		Face&				f=*c.m_face;
		const btlVector3		p=BaryEval(	f.m_n[0]->m_x,
			f.m_n[1]->m_x,
			f.m_n[2]->m_x,
			c.m_weights);
		const btlVector3		q=BaryEval(	f.m_n[0]->m_q,
			f.m_n[1]->m_q,
			f.m_n[2]->m_q,
			c.m_weights);											
		const btlVector3		vr=(n.m_x-n.m_q)-(p-q);
		btlVector3			corr(0,0,0);
		btlScalar dot = btlDot(vr,nr);
		if(dot<0)
		{
			const btlScalar	j=c.m_margin-(btlDot(nr,n.m_x)-btlDot(nr,p));
			corr+=c.m_normal*j;
		}
		corr			-=	ProjectOnPlane(vr,nr)*c.m_friction;
		n.m_x			+=	corr*c.m_cfm[0];
		f.m_n[0]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.x());
		f.m_n[1]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.y());
		f.m_n[2]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.z());
	}
}

//
void				btlSoftBody::PSolve_Links(btlSoftBody* psb,btlScalar kst,btlScalar ti)
{
	for(int i=0,ni=psb->m_links.size();i<ni;++i)
	{			
		Link&	l=psb->m_links[i];
		if(l.m_c0>0)
		{
			Node&			a=*l.m_n[0];
			Node&			b=*l.m_n[1];
			const btlVector3	del=b.m_x-a.m_x;
			const btlScalar	len=del.length2();
			if (l.m_c1+len > SIMD_EPSILON)
			{
				const btlScalar	k=((l.m_c1-len)/(l.m_c0*(l.m_c1+len)))*kst;
				a.m_x-=del*(k*a.m_im);
				b.m_x+=del*(k*b.m_im);
			}
		}
	}
}

//
void				btlSoftBody::VSolve_Links(btlSoftBody* psb,btlScalar kst)
{
	for(int i=0,ni=psb->m_links.size();i<ni;++i)
	{			
		Link&			l=psb->m_links[i];
		Node**			n=l.m_n;
		const btlScalar	j=-btlDot(l.m_c3,n[0]->m_v-n[1]->m_v)*l.m_c2*kst;
		n[0]->m_v+=	l.m_c3*(j*n[0]->m_im);
		n[1]->m_v-=	l.m_c3*(j*n[1]->m_im);
	}
}

//
btlSoftBody::psolver_t	btlSoftBody::getSolver(ePSolver::_ solver)
{
	switch(solver)
	{
	case	ePSolver::Anchors:		
		return(&btlSoftBody::PSolve_Anchors);
	case	ePSolver::Linear:		
		return(&btlSoftBody::PSolve_Links);
	case	ePSolver::RContacts:	
		return(&btlSoftBody::PSolve_RContacts);
	case	ePSolver::SContacts:	
		return(&btlSoftBody::PSolve_SContacts);	
		default:
		{
		}
	}
	return(0);
}

//
btlSoftBody::vsolver_t	btlSoftBody::getSolver(eVSolver::_ solver)
{
	switch(solver)
	{
	case	eVSolver::Linear:		return(&btlSoftBody::VSolve_Links);
		default:
		{
		}
	}
	return(0);
}

//
void			btlSoftBody::defaultCollisionHandler(const btlCollisionObjectWrapper* pcoWrap)
{

	switch(m_cfg.collisions&fCollision::RVSmask)
	{
	case	fCollision::SDF_RS:
		{
			btlSoftColliders::CollideSDF_RS	docollide;		
			btlRigidBody*		prb1=(btlRigidBody*) btlRigidBody::upcast(pcoWrap->getCollisionObject());
			btlTransform	wtr=pcoWrap->getWorldTransform();

			const btlTransform	ctr=pcoWrap->getWorldTransform();
			const btlScalar		timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
			const btlScalar		basemargin=getCollisionShape()->getMargin();
			btlVector3			mins;
			btlVector3			maxs;
			ATTRIBUTE_ALIGNED16(btlDbvtVolume)		volume;
			pcoWrap->getCollisionShape()->getAabb(	pcoWrap->getWorldTransform(),
				mins,
				maxs);
			volume=btlDbvtVolume::FromMM(mins,maxs);
			volume.Expand(btlVector3(basemargin,basemargin,basemargin));		
			docollide.psb		=	this;
			docollide.m_colObj1Wrap = pcoWrap;
			docollide.m_rigidBody = prb1;

			docollide.dynmargin	=	basemargin+timemargin;
			docollide.stamargin	=	basemargin;
			m_ndbvt.collideTV(m_ndbvt.m_root,volume,docollide);
		}
		break;
	case	fCollision::CL_RS:
		{
			btlSoftColliders::CollideCL_RS	collider;
			collider.ProcessColObj(this,pcoWrap);
		}
		break;
	}
}

//
void			btlSoftBody::defaultCollisionHandler(btlSoftBody* psb)
{
	const int cf=m_cfg.collisions&psb->m_cfg.collisions;
	switch(cf&fCollision::SVSmask)
	{
	case	fCollision::CL_SS:
		{
			
			//support self-collision if CL_SELF flag set
			if (this!=psb || psb->m_cfg.collisions&fCollision::CL_SELF)
			{
				btlSoftColliders::CollideCL_SS	docollide;
				docollide.ProcessSoftSoft(this,psb);
			}
			
		}
		break;
	case	fCollision::VF_SS:
		{
			//only self-collision for Cluster, not Vertex-Face yet
			if (this!=psb)
			{
				btlSoftColliders::CollideVF_SS	docollide;
				/* common					*/ 
				docollide.mrg=	getCollisionShape()->getMargin()+
					psb->getCollisionShape()->getMargin();
				/* psb0 nodes vs psb1 faces	*/ 
				docollide.psb[0]=this;
				docollide.psb[1]=psb;
				docollide.psb[0]->m_ndbvt.collideTT(	docollide.psb[0]->m_ndbvt.m_root,
					docollide.psb[1]->m_fdbvt.m_root,
					docollide);
				/* psb1 nodes vs psb0 faces	*/ 
				docollide.psb[0]=psb;
				docollide.psb[1]=this;
				docollide.psb[0]->m_ndbvt.collideTT(	docollide.psb[0]->m_ndbvt.m_root,
					docollide.psb[1]->m_fdbvt.m_root,
					docollide);
			}
		}
		break;
	default:
		{
			
		}
	}
}



void btlSoftBody::setWindVelocity( const btlVector3 &velocity )
{
	m_windVelocity = velocity;
}


const btlVector3& btlSoftBody::getWindVelocity()
{
	return m_windVelocity;
}



int	btlSoftBody::calculateSerializeBufferSize()	const
{
	int sz = sizeof(btlSoftBodyData);
	return sz;
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlSoftBody::serialize(void* dataBuffer, class btlSerializer* serializer) const
{
	btlSoftBodyData* sbd = (btlSoftBodyData*) dataBuffer;

	btlCollisionObject::serialize(&sbd->m_collisionObjectData, serializer);

	btlHashMap<btlHashPtr,int>	m_nodeIndexMap;

	sbd->m_numMaterials = m_materials.size();
	sbd->m_materials = sbd->m_numMaterials? (SoftBodyMaterialData**) serializer->getUniquePointer((void*)&m_materials): 0;

	if (sbd->m_materials)
	{
		int sz = sizeof(SoftBodyMaterialData*);
		int numElem = sbd->m_numMaterials;
		btlChunk* chunk = serializer->allocate(sz,numElem);
		//SoftBodyMaterialData** memPtr = chunk->m_oldPtr;
		SoftBodyMaterialData** memPtr = (SoftBodyMaterialData**)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			btlSoftBody::Material* mat = m_materials[i];
			*memPtr = mat ? (SoftBodyMaterialData*)serializer->getUniquePointer((void*)mat) : 0;
			if (!serializer->findPointer(mat))
			{
				//serialize it here
				btlChunk* chunk = serializer->allocate(sizeof(SoftBodyMaterialData),1);
				SoftBodyMaterialData* memPtr = (SoftBodyMaterialData*)chunk->m_oldPtr;
				memPtr->m_flags = mat->m_flags;
				memPtr->m_angularStiffness = mat->m_kAST;
				memPtr->m_linearStiffness = mat->m_kLST;
				memPtr->m_volumeStiffness = mat->m_kVST;
				serializer->finalizeChunk(chunk,"SoftBodyMaterialData",BT_SBMATERIAL_CODE,mat);
			}
		}
		serializer->finalizeChunk(chunk,"SoftBodyMaterialData",BT_ARRAY_CODE,(void*) &m_materials);
	}


	

	sbd->m_numNodes = m_nodes.size();
	sbd->m_nodes = sbd->m_numNodes ? (SoftBodyNodeData*)serializer->getUniquePointer((void*)&m_nodes): 0;
	if (sbd->m_nodes)
	{
		int sz = sizeof(SoftBodyNodeData);
		int numElem = sbd->m_numNodes;
		btlChunk* chunk = serializer->allocate(sz,numElem);
		SoftBodyNodeData* memPtr = (SoftBodyNodeData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			m_nodes[i].m_f.serializeFloat( memPtr->m_accumulatedForce);
			memPtr->m_area = m_nodes[i].m_area;
			memPtr->m_attach = m_nodes[i].m_battach;
			memPtr->m_inverseMass = m_nodes[i].m_im;
			memPtr->m_material = m_nodes[i].m_material? (SoftBodyMaterialData*)serializer->getUniquePointer((void*) m_nodes[i].m_material):0;
			m_nodes[i].m_n.serializeFloat(memPtr->m_normal);
			m_nodes[i].m_x.serializeFloat(memPtr->m_position);
			m_nodes[i].m_q.serializeFloat(memPtr->m_previousPosition);
			m_nodes[i].m_v.serializeFloat(memPtr->m_velocity);
			m_nodeIndexMap.insert(&m_nodes[i],i);
		}
		serializer->finalizeChunk(chunk,"SoftBodyNodeData",BT_SBNODE_CODE,(void*) &m_nodes);
	}

	sbd->m_numLinks = m_links.size();
	sbd->m_links = sbd->m_numLinks? (SoftBodyLinkData*) serializer->getUniquePointer((void*)&m_links[0]):0;
	if (sbd->m_links)
	{
		int sz = sizeof(SoftBodyLinkData);
		int numElem = sbd->m_numLinks;
		btlChunk* chunk = serializer->allocate(sz,numElem);
		SoftBodyLinkData* memPtr = (SoftBodyLinkData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			memPtr->m_bbending = m_links[i].m_bbending;
			memPtr->m_material = m_links[i].m_material? (SoftBodyMaterialData*)serializer->getUniquePointer((void*) m_links[i].m_material):0;
			memPtr->m_nodeIndices[0] = m_links[i].m_n[0] ? m_links[i].m_n[0] - &m_nodes[0]: -1;
			memPtr->m_nodeIndices[1] = m_links[i].m_n[1] ? m_links[i].m_n[1] - &m_nodes[0]: -1;
			btlAssert(memPtr->m_nodeIndices[0]<m_nodes.size());
			btlAssert(memPtr->m_nodeIndices[1]<m_nodes.size());
			memPtr->m_restLength = m_links[i].m_rl;
		}
		serializer->finalizeChunk(chunk,"SoftBodyLinkData",BT_ARRAY_CODE,(void*) &m_links[0]);

	}


	sbd->m_numFaces = m_faces.size();
	sbd->m_faces = sbd->m_numFaces? (SoftBodyFaceData*) serializer->getUniquePointer((void*)&m_faces[0]):0;
	if (sbd->m_faces)
	{
		int sz = sizeof(SoftBodyFaceData);
		int numElem = sbd->m_numFaces;
		btlChunk* chunk = serializer->allocate(sz,numElem);
		SoftBodyFaceData* memPtr = (SoftBodyFaceData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			memPtr->m_material = m_faces[i].m_material ?  (SoftBodyMaterialData*) serializer->getUniquePointer((void*)m_faces[i].m_material): 0;
			m_faces[i].m_normal.serializeFloat(	memPtr->m_normal);
			for (int j=0;j<3;j++)
			{
				memPtr->m_nodeIndices[j] = m_faces[i].m_n[j]? m_faces[i].m_n[j] - &m_nodes[0]: -1;
			}
			memPtr->m_restArea = m_faces[i].m_ra;
		}
		serializer->finalizeChunk(chunk,"SoftBodyFaceData",BT_ARRAY_CODE,(void*) &m_faces[0]);
	}


	sbd->m_numTetrahedra = m_tetras.size();
	sbd->m_tetrahedra = sbd->m_numTetrahedra ? (SoftBodyTetraData*) serializer->getUniquePointer((void*)&m_tetras[0]):0;
	if (sbd->m_tetrahedra)
	{
		int sz = sizeof(SoftBodyTetraData);
		int numElem = sbd->m_numTetrahedra;
		btlChunk* chunk = serializer->allocate(sz,numElem);
		SoftBodyTetraData* memPtr = (SoftBodyTetraData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			for (int j=0;j<4;j++)
			{
				m_tetras[i].m_c0[j].serializeFloat(	memPtr->m_c0[j] );
				memPtr->m_nodeIndices[j] = m_tetras[j].m_n[j]? m_tetras[j].m_n[j]-&m_nodes[0] : -1;
			}
			memPtr->m_c1 = m_tetras[i].m_c1;
			memPtr->m_c2 = m_tetras[i].m_c2;
			memPtr->m_material = m_tetras[i].m_material ? (SoftBodyMaterialData*)serializer->getUniquePointer((void*) m_tetras[i].m_material): 0;
			memPtr->m_restVolume = m_tetras[i].m_rv;
		}
		serializer->finalizeChunk(chunk,"SoftBodyTetraData",BT_ARRAY_CODE,(void*) &m_tetras[0]);
	}

	sbd->m_numAnchors = m_anchors.size();
	sbd->m_anchors = sbd->m_numAnchors ? (SoftRigidAnchorData*) serializer->getUniquePointer((void*)&m_anchors[0]):0;
	if (sbd->m_anchors)
	{
		int sz = sizeof(SoftRigidAnchorData);
		int numElem = sbd->m_numAnchors;
		btlChunk* chunk = serializer->allocate(sz,numElem);
		SoftRigidAnchorData* memPtr = (SoftRigidAnchorData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			m_anchors[i].m_c0.serializeFloat(memPtr->m_c0);
			m_anchors[i].m_c1.serializeFloat(memPtr->m_c1);
			memPtr->m_c2 = m_anchors[i].m_c2;
			m_anchors[i].m_local.serializeFloat(memPtr->m_localFrame);
			memPtr->m_nodeIndex = m_anchors[i].m_node? m_anchors[i].m_node-&m_nodes[0]: -1;
			
			memPtr->m_rigidBody = m_anchors[i].m_body? (btlRigidBodyData*)  serializer->getUniquePointer((void*)m_anchors[i].m_body): 0;
			btlAssert(memPtr->m_nodeIndex < m_nodes.size());
		}
		serializer->finalizeChunk(chunk,"SoftRigidAnchorData",BT_ARRAY_CODE,(void*) &m_anchors[0]);
	}
	

	sbd->m_config.m_dynamicFriction = m_cfg.kDF;
	sbd->m_config.m_baumgarte = m_cfg.kVCF;
	sbd->m_config.m_pressure = m_cfg.kPR;
	sbd->m_config.m_aeroModel = this->m_cfg.aeromodel;
	sbd->m_config.m_lift = m_cfg.kLF;
	sbd->m_config.m_drag = m_cfg.kDG;
	sbd->m_config.m_positionIterations = m_cfg.piterations;
	sbd->m_config.m_driftIterations = m_cfg.diterations;
	sbd->m_config.m_clusterIterations = m_cfg.citerations;
	sbd->m_config.m_velocityIterations = m_cfg.viterations;
	sbd->m_config.m_maxVolume = m_cfg.maxvolume;
	sbd->m_config.m_damping = m_cfg.kDP;
	sbd->m_config.m_poseMatch = m_cfg.kMT;
	sbd->m_config.m_collisionFlags = m_cfg.collisions;
	sbd->m_config.m_volume = m_cfg.kVC;
	sbd->m_config.m_rigidContactHardness = m_cfg.kCHR;
	sbd->m_config.m_kineticContactHardness = m_cfg.kKHR;
	sbd->m_config.m_softContactHardness = m_cfg.kSHR;
	sbd->m_config.m_anchorHardness = m_cfg.kAHR;
	sbd->m_config.m_timeScale = m_cfg.timescale;
	sbd->m_config.m_maxVolume = m_cfg.maxvolume;
	sbd->m_config.m_softRigidClusterHardness = m_cfg.kSRHR_CL;
	sbd->m_config.m_softKineticClusterHardness = m_cfg.kSKHR_CL;
	sbd->m_config.m_softSoftClusterHardness = m_cfg.kSSHR_CL;
	sbd->m_config.m_softRigidClusterImpulseSplit = m_cfg.kSR_SPLT_CL;
	sbd->m_config.m_softKineticClusterImpulseSplit = m_cfg.kSK_SPLT_CL;
	sbd->m_config.m_softSoftClusterImpulseSplit = m_cfg.kSS_SPLT_CL;

	//pose for shape matching
	{
		sbd->m_pose = (SoftBodyPoseData*)serializer->getUniquePointer((void*)&m_pose);

		int sz = sizeof(SoftBodyPoseData);
		btlChunk* chunk = serializer->allocate(sz,1);
		SoftBodyPoseData* memPtr = (SoftBodyPoseData*)chunk->m_oldPtr;
		
		m_pose.m_aqq.serializeFloat(memPtr->m_aqq);
		memPtr->m_bframe = m_pose.m_bframe;
		memPtr->m_bvolume = m_pose.m_bvolume;
		m_pose.m_com.serializeFloat(memPtr->m_com);
		
		memPtr->m_numPositions = m_pose.m_pos.size();
		memPtr->m_positions = memPtr->m_numPositions ? (btlVector3FloatData*)serializer->getUniquePointer((void*)&m_pose.m_pos[0]): 0;
		if (memPtr->m_numPositions)
		{
			int numElem = memPtr->m_numPositions;
			int sz = sizeof(btlVector3Data);
			btlChunk* chunk = serializer->allocate(sz,numElem);
			btlVector3FloatData* memPtr = (btlVector3FloatData*)chunk->m_oldPtr;
			for (int i=0;i<numElem;i++,memPtr++)
			{
				m_pose.m_pos[i].serializeFloat(*memPtr);
			}
			serializer->finalizeChunk(chunk,"btVector3FloatData",BT_ARRAY_CODE,(void*)&m_pose.m_pos[0]);
		}
		memPtr->m_restVolume = m_pose.m_volume;
		m_pose.m_rot.serializeFloat(memPtr->m_rot);
		m_pose.m_scl.serializeFloat(memPtr->m_scale);

		memPtr->m_numWeigts = m_pose.m_wgh.size();
		memPtr->m_weights = memPtr->m_numWeigts? (float*) serializer->getUniquePointer((void*) &m_pose.m_wgh[0]) : 0;
		if (memPtr->m_numWeigts)
		{
			
			int numElem = memPtr->m_numWeigts;
			int sz = sizeof(float);
			btlChunk* chunk = serializer->allocate(sz,numElem);
			float* memPtr = (float*) chunk->m_oldPtr;
			for (int i=0;i<numElem;i++,memPtr++)
			{
				*memPtr = m_pose.m_wgh[i];
			}
			serializer->finalizeChunk(chunk,"float",BT_ARRAY_CODE,(void*)&m_pose.m_wgh[0]);
		}

		serializer->finalizeChunk(chunk,"SoftBodyPoseData",BT_ARRAY_CODE,(void*)&m_pose);
	}

	//clusters for convex-cluster collision detection

	sbd->m_numClusters = m_clusters.size();
	sbd->m_clusters = sbd->m_numClusters? (SoftBodyClusterData*) serializer->getUniquePointer((void*)m_clusters[0]) : 0;
	if (sbd->m_numClusters)
	{
		int numElem = sbd->m_numClusters;
		int sz = sizeof(SoftBodyClusterData);
		btlChunk* chunk = serializer->allocate(sz,numElem);
		SoftBodyClusterData* memPtr = (SoftBodyClusterData*) chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			memPtr->m_adamping= m_clusters[i]->m_adamping;
			m_clusters[i]->m_av.serializeFloat(memPtr->m_av);
			memPtr->m_clusterIndex = m_clusters[i]->m_clusterIndex;
			memPtr->m_collide = m_clusters[i]->m_collide;
			m_clusters[i]->m_com.serializeFloat(memPtr->m_com);
			memPtr->m_containsAnchor = m_clusters[i]->m_containsAnchor;
			m_clusters[i]->m_dimpulses[0].serializeFloat(memPtr->m_dimpulses[0]);
			m_clusters[i]->m_dimpulses[1].serializeFloat(memPtr->m_dimpulses[1]);
			m_clusters[i]->m_framexform.serializeFloat(memPtr->m_framexform);
			memPtr->m_idmass = m_clusters[i]->m_idmass;
			memPtr->m_imass = m_clusters[i]->m_imass;
			m_clusters[i]->m_invwi.serializeFloat(memPtr->m_invwi);
			memPtr->m_ldamping = m_clusters[i]->m_ldamping;
			m_clusters[i]->m_locii.serializeFloat(memPtr->m_locii);
			m_clusters[i]->m_lv.serializeFloat(memPtr->m_lv);
			memPtr->m_matching = m_clusters[i]->m_matching;
			memPtr->m_maxSelfCollisionImpulse = m_clusters[i]->m_maxSelfCollisionImpulse;
			memPtr->m_ndamping = m_clusters[i]->m_ndamping;
			memPtr->m_ldamping = m_clusters[i]->m_ldamping;
			memPtr->m_adamping = m_clusters[i]->m_adamping;
			memPtr->m_selfCollisionImpulseFactor = m_clusters[i]->m_selfCollisionImpulseFactor;

			memPtr->m_numFrameRefs = m_clusters[i]->m_framerefs.size();
			memPtr->m_numMasses = m_clusters[i]->m_masses.size();
			memPtr->m_numNodes = m_clusters[i]->m_nodes.size();

			memPtr->m_nvimpulses = m_clusters[i]->m_nvimpulses;
			m_clusters[i]->m_vimpulses[0].serializeFloat(memPtr->m_vimpulses[0]);
			m_clusters[i]->m_vimpulses[1].serializeFloat(memPtr->m_vimpulses[1]);
			memPtr->m_ndimpulses = m_clusters[i]->m_ndimpulses;

			

			memPtr->m_framerefs = memPtr->m_numFrameRefs? (btlVector3FloatData*)serializer->getUniquePointer((void*)&m_clusters[i]->m_framerefs[0]) : 0;
			if (memPtr->m_framerefs)
			{
				int numElem = memPtr->m_numFrameRefs;
				int sz = sizeof(btlVector3FloatData);
				btlChunk* chunk = serializer->allocate(sz,numElem);
				btlVector3FloatData* memPtr = (btlVector3FloatData*) chunk->m_oldPtr;
				for (int j=0;j<numElem;j++,memPtr++)
				{
					m_clusters[i]->m_framerefs[j].serializeFloat(*memPtr);
				}
				serializer->finalizeChunk(chunk,"btVector3FloatData",BT_ARRAY_CODE,(void*)&m_clusters[i]->m_framerefs[0]);
			}
			
			memPtr->m_masses = memPtr->m_numMasses ? (float*) serializer->getUniquePointer((void*)&m_clusters[i]->m_masses[0]): 0;
			if (memPtr->m_masses)
			{
				int numElem = memPtr->m_numMasses;
				int sz = sizeof(float);
				btlChunk* chunk = serializer->allocate(sz,numElem);
				float* memPtr = (float*) chunk->m_oldPtr;
				for (int j=0;j<numElem;j++,memPtr++)
				{
					*memPtr = m_clusters[i]->m_masses[j];
				}
				serializer->finalizeChunk(chunk,"float",BT_ARRAY_CODE,(void*)&m_clusters[i]->m_masses[0]);
			}

			memPtr->m_nodeIndices  = memPtr->m_numNodes ? (int*) serializer->getUniquePointer((void*) &m_clusters[i]->m_nodes) : 0;
			if (memPtr->m_nodeIndices )
			{
				int numElem = memPtr->m_numMasses;
				int sz = sizeof(int);
				btlChunk* chunk = serializer->allocate(sz,numElem);
				int* memPtr = (int*) chunk->m_oldPtr;
				for (int j=0;j<numElem;j++,memPtr++)
				{
					int* indexPtr = m_nodeIndexMap.find(m_clusters[i]->m_nodes[j]);
					btlAssert(indexPtr);
					*memPtr = *indexPtr;
				}
				serializer->finalizeChunk(chunk,"int",BT_ARRAY_CODE,(void*)&m_clusters[i]->m_nodes);
			}
		}
		serializer->finalizeChunk(chunk,"SoftBodyClusterData",BT_ARRAY_CODE,(void*)m_clusters[0]);

	}
	

	
	sbd->m_numJoints = m_joints.size();
	sbd->m_joints = m_joints.size()? (btlSoftBodyJointData*) serializer->getUniquePointer((void*)&m_joints[0]) : 0;

	if (sbd->m_joints)
	{
		int sz = sizeof(btlSoftBodyJointData);
		int numElem = m_joints.size();
		btlChunk* chunk = serializer->allocate(sz,numElem);
		btlSoftBodyJointData* memPtr = (btlSoftBodyJointData*)chunk->m_oldPtr;

		for (int i=0;i<numElem;i++,memPtr++)
		{
			memPtr->m_jointType = (int)m_joints[i]->Type();
			m_joints[i]->m_refs[0].serializeFloat(memPtr->m_refs[0]);
			m_joints[i]->m_refs[1].serializeFloat(memPtr->m_refs[1]);
			memPtr->m_cfm = m_joints[i]->m_cfm;
			memPtr->m_erp = float(m_joints[i]->m_erp);
			memPtr->m_split = float(m_joints[i]->m_split);
			memPtr->m_delete = m_joints[i]->m_delete;
			
			for (int j=0;j<4;j++)
			{
				memPtr->m_relPosition[0].m_floats[j] = 0.f;
				memPtr->m_relPosition[1].m_floats[j] = 0.f;
			}
			memPtr->m_bodyA = 0;
			memPtr->m_bodyB = 0;
			if (m_joints[i]->m_bodies[0].m_soft)
			{
				memPtr->m_bodyAtype = BT_JOINT_SOFT_BODY_CLUSTER;
				memPtr->m_bodyA = serializer->getUniquePointer((void*)m_joints[i]->m_bodies[0].m_soft);
			}
			if (m_joints[i]->m_bodies[0].m_collisionObject)
			{
				memPtr->m_bodyAtype = BT_JOINT_COLLISION_OBJECT;
				memPtr->m_bodyA = serializer->getUniquePointer((void*)m_joints[i]->m_bodies[0].m_collisionObject);
			}
			if (m_joints[i]->m_bodies[0].m_rigid)
			{
				memPtr->m_bodyAtype = BT_JOINT_RIGID_BODY;
				memPtr->m_bodyA = serializer->getUniquePointer((void*)m_joints[i]->m_bodies[0].m_rigid);
			}

			if (m_joints[i]->m_bodies[1].m_soft)
			{
				memPtr->m_bodyBtype = BT_JOINT_SOFT_BODY_CLUSTER;
				memPtr->m_bodyB = serializer->getUniquePointer((void*)m_joints[i]->m_bodies[1].m_soft);
			}
			if (m_joints[i]->m_bodies[1].m_collisionObject)
			{
				memPtr->m_bodyBtype = BT_JOINT_COLLISION_OBJECT;
				memPtr->m_bodyB = serializer->getUniquePointer((void*)m_joints[i]->m_bodies[1].m_collisionObject);
			}
			if (m_joints[i]->m_bodies[1].m_rigid)
			{
				memPtr->m_bodyBtype = BT_JOINT_RIGID_BODY;
				memPtr->m_bodyB = serializer->getUniquePointer((void*)m_joints[i]->m_bodies[1].m_rigid);
			}
		}
		serializer->finalizeChunk(chunk,"btSoftBodyJointData",BT_ARRAY_CODE,(void*) &m_joints[0]);
	}


	return btlSoftBodyDataName;
}

