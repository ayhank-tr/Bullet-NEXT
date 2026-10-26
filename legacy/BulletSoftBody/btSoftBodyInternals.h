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

#ifndef _BT_SOFT_BODY_INTERNALS_H
#define _BT_SOFT_BODY_INTERNALS_H

#include "btSoftBody.h"


#include "LinearMath/btQuickprof.h"
#include "LinearMath/btPolarDecomposition.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include <string.h> //for memset
//
// btSymMatrix
//
template <typename T>
struct btlSymMatrix
{
	btlSymMatrix() : dim(0)					{}
	btlSymMatrix(int n,const T& init=T())	{ resize(n,init); }
	void					resize(int n,const T& init=T())			{ dim=n;store.resize((n*(n+1))/2,init); }
	int						index(int c,int r) const				{ if(c>r) btlSwap(c,r);btlAssert(r<dim);return((r*(r+1))/2+c); }
	T&						operator()(int c,int r)					{ return(store[index(c,r)]); }
	const T&				operator()(int c,int r) const			{ return(store[index(c,r)]); }
	btlAlignedObjectArray<T>	store;
	int						dim;
};	

//
// btSoftBodyCollisionShape
//
class btlSoftBodyCollisionShape : public btlConcaveShape
{
public:
	btlSoftBody*						m_body;

	btlSoftBodyCollisionShape(btlSoftBody* backptr)
	{
		m_shapeType = SOFTBODY_SHAPE_PROXYTYPE;
		m_body=backptr;
	}

	virtual ~btlSoftBodyCollisionShape()
	{

	}

	void	processAllTriangles(btlTriangleCallback* /*callback*/,const btlVector3& /*aabbMin*/,const btlVector3& /*aabbMax*/) const
	{
		//not yet
		btlAssert(0);
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const
	{
		/* t is usually identity, except when colliding against btCompoundShape. See Issue 512 */
		const btlVector3	mins=m_body->m_bounds[0];
		const btlVector3	maxs=m_body->m_bounds[1];
		const btlVector3	crns[]={t*btlVector3(mins.x(),mins.y(),mins.z()),
			t*btlVector3(maxs.x(),mins.y(),mins.z()),
			t*btlVector3(maxs.x(),maxs.y(),mins.z()),
			t*btlVector3(mins.x(),maxs.y(),mins.z()),
			t*btlVector3(mins.x(),mins.y(),maxs.z()),
			t*btlVector3(maxs.x(),mins.y(),maxs.z()),
			t*btlVector3(maxs.x(),maxs.y(),maxs.z()),
			t*btlVector3(mins.x(),maxs.y(),maxs.z())};
		aabbMin=aabbMax=crns[0];
		for(int i=1;i<8;++i)
		{
			aabbMin.setMin(crns[i]);
			aabbMax.setMax(crns[i]);
		}
	}


	virtual void	setLocalScaling(const btlVector3& /*scaling*/)
	{		
		///na
	}
	virtual const btlVector3& getLocalScaling() const
	{
		static const btlVector3 dummy(1,1,1);
		return dummy;
	}
	virtual void	calculateLocalInertia(btlScalar /*mass*/,btlVector3& /*inertia*/) const
	{
		///not yet
		btlAssert(0);
	}
	virtual const char*	getName()const
	{
		return "SoftBody";
	}

};

//
// btSoftClusterCollisionShape
//
class btlSoftClusterCollisionShape : public btlConvexInternalShape
{
public:
	const btlSoftBody::Cluster*	m_cluster;

	btlSoftClusterCollisionShape (const btlSoftBody::Cluster* cluster) : m_cluster(cluster) { setMargin(0); }


	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec) const
	{
		btlSoftBody::Node* const *						n=&m_cluster->m_nodes[0];
		btlScalar										d=btlDot(vec,n[0]->m_x);
		int												j=0;
		for(int i=1,ni=m_cluster->m_nodes.size();i<ni;++i)
		{
			const btlScalar	k=btlDot(vec,n[i]->m_x);
			if(k>d) { d=k;j=i; }
		}
		return(n[j]->m_x);
	}
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
	{
		return(localGetSupportingVertex(vec));
	}
	//notice that the vectors should be unit length
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
	{}


	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
	{}

	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const
	{}

	virtual int	getShapeType() const { return SOFTBODY_SHAPE_PROXYTYPE; }

	//debugging
	virtual const char*	getName()const {return "SOFTCLUSTER";}

	virtual void	setMargin(btlScalar margin)
	{
		btlConvexInternalShape::setMargin(margin);
	}
	virtual btlScalar	getMargin() const
	{
		return btlConvexInternalShape::getMargin();
	}
};

//
// Inline's
//

//
template <typename T>
static inline void			ZeroInitialize(T& value)
{
	memset(&value,0,sizeof(T));
}
//
template <typename T>
static inline bool			CompLess(const T& a,const T& b)
{ return(a<b); }
//
template <typename T>
static inline bool			CompGreater(const T& a,const T& b)
{ return(a>b); }
//
template <typename T>
static inline T				Lerp(const T& a,const T& b,btlScalar t)
{ return(a+(b-a)*t); }
//
template <typename T>
static inline T				InvLerp(const T& a,const T& b,btlScalar t)
{ return((b+a*t-b*t)/(a*b)); }
//
static inline btlMatrix3x3	Lerp(	const btlMatrix3x3& a,
								 const btlMatrix3x3& b,
								 btlScalar t)
{
	btlMatrix3x3	r;
	r[0]=Lerp(a[0],b[0],t);
	r[1]=Lerp(a[1],b[1],t);
	r[2]=Lerp(a[2],b[2],t);
	return(r);
}
//
static inline btlVector3		Clamp(const btlVector3& v,btlScalar maxlength)
{
	const btlScalar sql=v.length2();
	if(sql>(maxlength*maxlength))
		return((v*maxlength)/btlSqrt(sql));
	else
		return(v);
}
//
template <typename T>
static inline T				Clamp(const T& x,const T& l,const T& h)
{ return(x<l?l:x>h?h:x); }
//
template <typename T>
static inline T				Sq(const T& x)
{ return(x*x); }
//
template <typename T>
static inline T				Cube(const T& x)
{ return(x*x*x); }
//
template <typename T>
static inline T				Sign(const T& x)
{ return((T)(x<0?-1:+1)); }
//
template <typename T>
static inline bool			SameSign(const T& x,const T& y)
{ return((x*y)>0); }
//
static inline btlScalar		ClusterMetric(const btlVector3& x,const btlVector3& y)
{
	const btlVector3	d=x-y;
	return(btlFabs(d[0])+btlFabs(d[1])+btlFabs(d[2]));
}
//
static inline btlMatrix3x3	ScaleAlongAxis(const btlVector3& a,btlScalar s)
{
	const btlScalar	xx=a.x()*a.x();
	const btlScalar	yy=a.y()*a.y();
	const btlScalar	zz=a.z()*a.z();
	const btlScalar	xy=a.x()*a.y();
	const btlScalar	yz=a.y()*a.z();
	const btlScalar	zx=a.z()*a.x();
	btlMatrix3x3		m;
	m[0]=btlVector3(1-xx+xx*s,xy*s-xy,zx*s-zx);
	m[1]=btlVector3(xy*s-xy,1-yy+yy*s,yz*s-yz);
	m[2]=btlVector3(zx*s-zx,yz*s-yz,1-zz+zz*s);
	return(m);
}
//
static inline btlMatrix3x3	Cross(const btlVector3& v)
{
	btlMatrix3x3	m;
	m[0]=btlVector3(0,-v.z(),+v.y());
	m[1]=btlVector3(+v.z(),0,-v.x());
	m[2]=btlVector3(-v.y(),+v.x(),0);
	return(m);
}
//
static inline btlMatrix3x3	Diagonal(btlScalar x)
{
	btlMatrix3x3	m;
	m[0]=btlVector3(x,0,0);
	m[1]=btlVector3(0,x,0);
	m[2]=btlVector3(0,0,x);
	return(m);
}
//
static inline btlMatrix3x3	Add(const btlMatrix3x3& a,
								const btlMatrix3x3& b)
{
	btlMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]+b[i];
	return(r);
}
//
static inline btlMatrix3x3	Sub(const btlMatrix3x3& a,
								const btlMatrix3x3& b)
{
	btlMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]-b[i];
	return(r);
}
//
static inline btlMatrix3x3	Mul(const btlMatrix3x3& a,
								btlScalar b)
{
	btlMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]*b;
	return(r);
}
//
static inline void			Orthogonalize(btlMatrix3x3& m)
{
	m[2]=btlCross(m[0],m[1]).normalized();
	m[1]=btlCross(m[2],m[0]).normalized();
	m[0]=btlCross(m[1],m[2]).normalized();
}
//
static inline btlMatrix3x3	MassMatrix(btlScalar im,const btlMatrix3x3& iwi,const btlVector3& r)
{
	const btlMatrix3x3	cr=Cross(r);
	return(Sub(Diagonal(im),cr*iwi*cr));
}

//
static inline btlMatrix3x3	ImpulseMatrix(	btlScalar dt,
										  btlScalar ima,
										  btlScalar imb,
										  const btlMatrix3x3& iwi,
										  const btlVector3& r)
{
	return(Diagonal(1/dt)*Add(Diagonal(ima),MassMatrix(imb,iwi,r)).inverse());
}

//
static inline btlMatrix3x3	ImpulseMatrix(	btlScalar ima,const btlMatrix3x3& iia,const btlVector3& ra,
										  btlScalar imb,const btlMatrix3x3& iib,const btlVector3& rb)	
{
	return(Add(MassMatrix(ima,iia,ra),MassMatrix(imb,iib,rb)).inverse());
}

//
static inline btlMatrix3x3	AngularImpulseMatrix(	const btlMatrix3x3& iia,
												 const btlMatrix3x3& iib)
{
	return(Add(iia,iib).inverse());
}

//
static inline btlVector3		ProjectOnAxis(	const btlVector3& v,
										  const btlVector3& a)
{
	return(a*btlDot(v,a));
}
//
static inline btlVector3		ProjectOnPlane(	const btlVector3& v,
										   const btlVector3& a)
{
	return(v-ProjectOnAxis(v,a));
}

//
static inline void			ProjectOrigin(	const btlVector3& a,
										  const btlVector3& b,
										  btlVector3& prj,
										  btlScalar& sqd)
{
	const btlVector3	d=b-a;
	const btlScalar	m2=d.length2();
	if(m2>SIMD_EPSILON)
	{	
		const btlScalar	t=Clamp<btlScalar>(-btlDot(a,d)/m2,0,1);
		const btlVector3	p=a+d*t;
		const btlScalar	l2=p.length2();
		if(l2<sqd)
		{
			prj=p;
			sqd=l2;
		}
	}
}
//
static inline void			ProjectOrigin(	const btlVector3& a,
										  const btlVector3& b,
										  const btlVector3& c,
										  btlVector3& prj,
										  btlScalar& sqd)
{
	const btlVector3&	q=btlCross(b-a,c-a);
	const btlScalar		m2=q.length2();
	if(m2>SIMD_EPSILON)
	{
		const btlVector3	n=q/btlSqrt(m2);
		const btlScalar	k=btlDot(a,n);
		const btlScalar	k2=k*k;
		if(k2<sqd)
		{
			const btlVector3	p=n*k;
			if(	(btlDot(btlCross(a-p,b-p),q)>0)&&
				(btlDot(btlCross(b-p,c-p),q)>0)&&
				(btlDot(btlCross(c-p,a-p),q)>0))
			{			
				prj=p;
				sqd=k2;
			}
			else
			{
				ProjectOrigin(a,b,prj,sqd);
				ProjectOrigin(b,c,prj,sqd);
				ProjectOrigin(c,a,prj,sqd);
			}
		}
	}
}

//
template <typename T>
static inline T				BaryEval(		const T& a,
									 const T& b,
									 const T& c,
									 const btlVector3& coord)
{
	return(a*coord.x()+b*coord.y()+c*coord.z());
}
//
static inline btlVector3		BaryCoord(	const btlVector3& a,
									  const btlVector3& b,
									  const btlVector3& c,
									  const btlVector3& p)
{
	const btlScalar	w[]={	btlCross(a-p,b-p).length(),
		btlCross(b-p,c-p).length(),
		btlCross(c-p,a-p).length()};
	const btlScalar	isum=1/(w[0]+w[1]+w[2]);
	return(btlVector3(w[1]*isum,w[2]*isum,w[0]*isum));
}

//
static inline btlScalar				ImplicitSolve(	btlSoftBody::ImplicitFn* fn,
										  const btlVector3& a,
										  const btlVector3& b,
										  const btlScalar accuracy,
										  const int maxiterations=256)
{
	btlScalar	span[2]={0,1};
	btlScalar	values[2]={fn->Eval(a),fn->Eval(b)};
	if(values[0]>values[1])
	{
		btlSwap(span[0],span[1]);
		btlSwap(values[0],values[1]);
	}
	if(values[0]>-accuracy) return(-1);
	if(values[1]<+accuracy) return(-1);
	for(int i=0;i<maxiterations;++i)
	{
		const btlScalar	t=Lerp(span[0],span[1],values[0]/(values[0]-values[1]));
		const btlScalar	v=fn->Eval(Lerp(a,b,t));
		if((t<=0)||(t>=1))		break;
		if(btlFabs(v)<accuracy)	return(t);
		if(v<0)
		{ span[0]=t;values[0]=v; }
		else
		{ span[1]=t;values[1]=v; }
	}
	return(-1);
}

//
static inline btlVector3		NormalizeAny(const btlVector3& v)
{
	const btlScalar l=v.length();
	if(l>SIMD_EPSILON)
		return(v/l);
	else
		return(btlVector3(0,0,0));
}

//
static inline btlDbvtVolume	VolumeOf(	const btlSoftBody::Face& f,
									 btlScalar margin)
{
	const btlVector3*	pts[]={	&f.m_n[0]->m_x,
		&f.m_n[1]->m_x,
		&f.m_n[2]->m_x};
	btlDbvtVolume		vol=btlDbvtVolume::FromPoints(pts,3);
	vol.Expand(btlVector3(margin,margin,margin));
	return(vol);
}

//
static inline btlVector3			CenterOf(	const btlSoftBody::Face& f)
{
	return((f.m_n[0]->m_x+f.m_n[1]->m_x+f.m_n[2]->m_x)/3);
}

//
static inline btlScalar			AreaOf(		const btlVector3& x0,
									   const btlVector3& x1,
									   const btlVector3& x2)
{
	const btlVector3	a=x1-x0;
	const btlVector3	b=x2-x0;
	const btlVector3	cr=btlCross(a,b);
	const btlScalar	area=cr.length();
	return(area);
}

//
static inline btlScalar		VolumeOf(	const btlVector3& x0,
									 const btlVector3& x1,
									 const btlVector3& x2,
									 const btlVector3& x3)
{
	const btlVector3	a=x1-x0;
	const btlVector3	b=x2-x0;
	const btlVector3	c=x3-x0;
	return(btlDot(a,btlCross(b,c)));
}

//
static inline void					EvaluateMedium(	const btlSoftBodyWorldInfo* wfi,
										   const btlVector3& x,
										   btlSoftBody::sMedium& medium)
{
	medium.m_velocity	=	btlVector3(0,0,0);
	medium.m_pressure	=	0;
	medium.m_density	=	wfi->air_density;
	if(wfi->water_density>0)
	{
		const btlScalar	depth=-(btlDot(x,wfi->water_normal)+wfi->water_offset);
		if(depth>0)
		{
			medium.m_density	=	wfi->water_density;
			medium.m_pressure	=	depth*wfi->water_density*wfi->m_gravity.length();
		}
	}
}

//
static inline void			ApplyClampedForce(	btlSoftBody::Node& n,
											  const btlVector3& f,
											  btlScalar dt)
{
	const btlScalar	dtim=dt*n.m_im;
	if((f*dtim).length2()>n.m_v.length2())
	{/* Clamp	*/ 
		n.m_f-=ProjectOnAxis(n.m_v,f.normalized())/dtim;						
	}
	else
	{/* Apply	*/ 
		n.m_f+=f;
	}
}

//
static inline int		MatchEdge(	const btlSoftBody::Node* a,
								  const btlSoftBody::Node* b,
								  const btlSoftBody::Node* ma,
								  const btlSoftBody::Node* mb)
{
	if((a==ma)&&(b==mb)) return(0);
	if((a==mb)&&(b==ma)) return(1);
	return(-1);
}

//
// btEigen : Extract eigen system,
// straitforward implementation of http://math.fullerton.edu/mathews/n2003/JacobiMethodMod.html
// outputs are NOT sorted.
//
struct	btlEigen
{
	static int			system(btlMatrix3x3& a,btlMatrix3x3* vectors,btlVector3* values=0)
	{
		static const int		maxiterations=16;
		static const btlScalar	accuracy=(btlScalar)0.0001;
		btlMatrix3x3&			v=*vectors;
		int						iterations=0;
		vectors->setIdentity();
		do	{
			int				p=0,q=1;
			if(btlFabs(a[p][q])<btlFabs(a[0][2])) { p=0;q=2; }
			if(btlFabs(a[p][q])<btlFabs(a[1][2])) { p=1;q=2; }
			if(btlFabs(a[p][q])>accuracy)
			{
				const btlScalar	w=(a[q][q]-a[p][p])/(2*a[p][q]);
				const btlScalar	z=btlFabs(w);
				const btlScalar	t=w/(z*(btlSqrt(1+w*w)+z));
				if(t==t)/* [WARNING] let hope that one does not get thrown aways by some compilers... */ 
				{
					const btlScalar	c=1/btlSqrt(t*t+1);
					const btlScalar	s=c*t;
					mulPQ(a,c,s,p,q);
					mulTPQ(a,c,s,p,q);
					mulPQ(v,c,s,p,q);
				} else break;
			} else break;
		} while((++iterations)<maxiterations);
		if(values)
		{
			*values=btlVector3(a[0][0],a[1][1],a[2][2]);
		}
		return(iterations);
	}
private:
	static inline void	mulTPQ(btlMatrix3x3& a,btlScalar c,btlScalar s,int p,int q)
	{
		const btlScalar	m[2][3]={	{a[p][0],a[p][1],a[p][2]},
		{a[q][0],a[q][1],a[q][2]}};
		int i;

		for(i=0;i<3;++i) a[p][i]=c*m[0][i]-s*m[1][i];
		for(i=0;i<3;++i) a[q][i]=c*m[1][i]+s*m[0][i];
	}
	static inline void	mulPQ(btlMatrix3x3& a,btlScalar c,btlScalar s,int p,int q)
	{
		const btlScalar	m[2][3]={	{a[0][p],a[1][p],a[2][p]},
		{a[0][q],a[1][q],a[2][q]}};
		int i;

		for(i=0;i<3;++i) a[i][p]=c*m[0][i]-s*m[1][i];
		for(i=0;i<3;++i) a[i][q]=c*m[1][i]+s*m[0][i];
	}
};

//
// Polar decomposition,
// "Computing the Polar Decomposition with Applications", Nicholas J. Higham, 1986.
//
static inline int			PolarDecompose(	const btlMatrix3x3& m,btlMatrix3x3& q,btlMatrix3x3& s)
{
	static const btlPolarDecomposition polar;  
	return polar.decompose(m, q, s);
}

//
// btSoftColliders
//
struct btlSoftColliders
{
	//
	// ClusterBase
	//
	struct	ClusterBase : btlDbvt::ICollide
	{
		btlScalar			erp;
		btlScalar			idt;
		btlScalar			m_margin;
		btlScalar			friction;
		btlScalar			threshold;
		ClusterBase()
		{
			erp			=(btlScalar)1;
			idt			=0;
			m_margin		=0;
			friction	=0;
			threshold	=(btlScalar)0;
		}
		bool				SolveContact(	const btlGjkEpaSolver2::sResults& res,
			btlSoftBody::Body ba,const btlSoftBody::Body bb,
			btlSoftBody::CJoint& joint)
		{
			if(res.distance<m_margin)
			{
				btlVector3 norm = res.normal;
				norm.normalize();//is it necessary?

				const btlVector3		ra=res.witnesses[0]-ba.xform().getOrigin();
				const btlVector3		rb=res.witnesses[1]-bb.xform().getOrigin();
				const btlVector3		va=ba.velocity(ra);
				const btlVector3		vb=bb.velocity(rb);
				const btlVector3		vrel=va-vb;
				const btlScalar		rvac=btlDot(vrel,norm);
				 btlScalar		depth=res.distance-m_margin;
				
//				printf("depth=%f\n",depth);
				const btlVector3		iv=norm*rvac;
				const btlVector3		fv=vrel-iv;
				joint.m_bodies[0]	=	ba;
				joint.m_bodies[1]	=	bb;
				joint.m_refs[0]		=	ra*ba.xform().getBasis();
				joint.m_refs[1]		=	rb*bb.xform().getBasis();
				joint.m_rpos[0]		=	ra;
				joint.m_rpos[1]		=	rb;
				joint.m_cfm			=	1;
				joint.m_erp			=	1;
				joint.m_life		=	0;
				joint.m_maxlife		=	0;
				joint.m_split		=	1;
				
				joint.m_drift		=	depth*norm;

				joint.m_normal		=	norm;
//				printf("normal=%f,%f,%f\n",res.normal.getX(),res.normal.getY(),res.normal.getZ());
				joint.m_delete		=	false;
				joint.m_friction	=	fv.length2()<(rvac*friction*rvac*friction)?1:friction;
				joint.m_massmatrix	=	ImpulseMatrix(	ba.invMass(),ba.invWorldInertia(),joint.m_rpos[0],
					bb.invMass(),bb.invWorldInertia(),joint.m_rpos[1]);

				return(true);
			}
			return(false);
		}
	};
	//
	// CollideCL_RS
	//
	struct	CollideCL_RS : ClusterBase
	{
		btlSoftBody*		psb;
		const btlCollisionObjectWrapper*	m_colObjWrap;

		void		Process(const btlDbvtNode* leaf)
		{
			btlSoftBody::Cluster*		cluster=(btlSoftBody::Cluster*)leaf->data;
			btlSoftClusterCollisionShape	cshape(cluster);
			
			const btlConvexShape*		rshape=(const btlConvexShape*)m_colObjWrap->getCollisionShape();

			///don't collide an anchored cluster with a static/kinematic object
			if(m_colObjWrap->getCollisionObject()->isStaticOrKinematicObject() && cluster->m_containsAnchor)
				return;

			btlGjkEpaSolver2::sResults	res;		
			if(btlGjkEpaSolver2::SignedDistance(	&cshape,btlTransform::getIdentity(),
				rshape,m_colObjWrap->getWorldTransform(),
				btlVector3(1,0,0),res))
			{
				btlSoftBody::CJoint	joint;
				if(SolveContact(res,cluster,m_colObjWrap->getCollisionObject(),joint))//prb,joint))
				{
					btlSoftBody::CJoint*	pj=new(btlAlignedAlloc(sizeof(btlSoftBody::CJoint),16)) btlSoftBody::CJoint();
					*pj=joint;psb->m_joints.push_back(pj);
					if(m_colObjWrap->getCollisionObject()->isStaticOrKinematicObject())
					{
						pj->m_erp	*=	psb->m_cfg.kSKHR_CL;
						pj->m_split	*=	psb->m_cfg.kSK_SPLT_CL;
					}
					else
					{
						pj->m_erp	*=	psb->m_cfg.kSRHR_CL;
						pj->m_split	*=	psb->m_cfg.kSR_SPLT_CL;
					}
				}
			}
		}
		void		ProcessColObj(btlSoftBody* ps,const btlCollisionObjectWrapper* colObWrap)
		{
			psb			=	ps;
			m_colObjWrap			=	colObWrap;
			idt			=	ps->m_sst.isdt;
			m_margin		=	m_colObjWrap->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin();
			///Bullet rigid body uses multiply instead of minimum to determine combined friction. Some customization would be useful.
			friction	=	btlMin(psb->m_cfg.kDF,m_colObjWrap->getCollisionObject()->getFriction());
			btlVector3			mins;
			btlVector3			maxs;

			ATTRIBUTE_ALIGNED16(btlDbvtVolume)		volume;
			colObWrap->getCollisionShape()->getAabb(colObWrap->getWorldTransform(),mins,maxs);
			volume=btlDbvtVolume::FromMM(mins,maxs);
			volume.Expand(btlVector3(1,1,1)*m_margin);
			ps->m_cdbvt.collideTV(ps->m_cdbvt.m_root,volume,*this);
		}	
	};
	//
	// CollideCL_SS
	//
	struct	CollideCL_SS : ClusterBase
	{
		btlSoftBody*	bodies[2];
		void		Process(const btlDbvtNode* la,const btlDbvtNode* lb)
		{
			btlSoftBody::Cluster*		cla=(btlSoftBody::Cluster*)la->data;
			btlSoftBody::Cluster*		clb=(btlSoftBody::Cluster*)lb->data;


			bool connected=false;
			if ((bodies[0]==bodies[1])&&(bodies[0]->m_clusterConnectivity.size()))
			{
				connected = bodies[0]->m_clusterConnectivity[cla->m_clusterIndex+bodies[0]->m_clusters.size()*clb->m_clusterIndex];
			}

			if (!connected)
			{
				btlSoftClusterCollisionShape	csa(cla);
				btlSoftClusterCollisionShape	csb(clb);
				btlGjkEpaSolver2::sResults	res;		
				if(btlGjkEpaSolver2::SignedDistance(	&csa,btlTransform::getIdentity(),
					&csb,btlTransform::getIdentity(),
					cla->m_com-clb->m_com,res))
				{
					btlSoftBody::CJoint	joint;
					if(SolveContact(res,cla,clb,joint))
					{
						btlSoftBody::CJoint*	pj=new(btlAlignedAlloc(sizeof(btlSoftBody::CJoint),16)) btlSoftBody::CJoint();
						*pj=joint;bodies[0]->m_joints.push_back(pj);
						pj->m_erp	*=	btlMax(bodies[0]->m_cfg.kSSHR_CL,bodies[1]->m_cfg.kSSHR_CL);
						pj->m_split	*=	(bodies[0]->m_cfg.kSS_SPLT_CL+bodies[1]->m_cfg.kSS_SPLT_CL)/2;
					}
				}
			} else
			{
				static int count=0;
				count++;
				//printf("count=%d\n",count);
				
			}
		}
		void		ProcessSoftSoft(btlSoftBody* psa,btlSoftBody* psb)
		{
			idt			=	psa->m_sst.isdt;
			//m_margin		=	(psa->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin())/2;
			m_margin		=	(psa->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin());
			friction	=	btlMin(psa->m_cfg.kDF,psb->m_cfg.kDF);
			bodies[0]	=	psa;
			bodies[1]	=	psb;
			psa->m_cdbvt.collideTT(psa->m_cdbvt.m_root,psb->m_cdbvt.m_root,*this);
		}	
	};
	//
	// CollideSDF_RS
	//
	struct	CollideSDF_RS : btlDbvt::ICollide
	{
		void		Process(const btlDbvtNode* leaf)
		{
			btlSoftBody::Node*	node=(btlSoftBody::Node*)leaf->data;
			DoNode(*node);
		}
		void		DoNode(btlSoftBody::Node& n) const
		{
			const btlScalar			m=n.m_im>0?dynmargin:stamargin;
			btlSoftBody::RContact	c;

			if(	(!n.m_battach)&&
				psb->checkContact(m_colObj1Wrap,n.m_x,m,c.m_cti))
			{
				const btlScalar	ima=n.m_im;
				const btlScalar	imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
				const btlScalar	ms=ima+imb;
				if(ms>0)
				{
					const btlTransform&	wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1Wrap->getCollisionObject()->getWorldTransform();
					static const btlMatrix3x3	iwiStatic(0,0,0,0,0,0,0,0,0);
					const btlMatrix3x3&	iwi=m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
					const btlVector3		ra=n.m_x-wtr.getOrigin();
					const btlVector3		va=m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra)*psb->m_sst.sdt : btlVector3(0,0,0);
					const btlVector3		vb=n.m_x-n.m_q;	
					const btlVector3		vr=vb-va;
					const btlScalar		dn=btlDot(vr,c.m_cti.m_normal);
					const btlVector3		fv=vr-c.m_cti.m_normal*dn;
					const btlScalar		fc=psb->m_cfg.kDF*m_colObj1Wrap->getCollisionObject()->getFriction();
					c.m_node	=	&n;
					c.m_c0		=	ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
					c.m_c1		=	ra;
					c.m_c2		=	ima*psb->m_sst.sdt;
			        c.m_c3		=	fv.length2()<(dn*fc*dn*fc)?0:1-fc;
					c.m_c4		=	m_colObj1Wrap->getCollisionObject()->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
					psb->m_rcontacts.push_back(c);
					if (m_rigidBody)
						m_rigidBody->activate();
				}
			}
		}
		btlSoftBody*		psb;
		const btlCollisionObjectWrapper*	m_colObj1Wrap;
		btlRigidBody*	m_rigidBody;
		btlScalar		dynmargin;
		btlScalar		stamargin;
	};
	//
	// CollideVF_SS
	//
	struct	CollideVF_SS : btlDbvt::ICollide
	{
		void		Process(const btlDbvtNode* lnode,
			const btlDbvtNode* lface)
		{
			btlSoftBody::Node*	node=(btlSoftBody::Node*)lnode->data;
			btlSoftBody::Face*	face=(btlSoftBody::Face*)lface->data;
			btlVector3			o=node->m_x;
			btlVector3			p;
			btlScalar			d=SIMD_INFINITY;
			ProjectOrigin(	face->m_n[0]->m_x-o,
				face->m_n[1]->m_x-o,
				face->m_n[2]->m_x-o,
				p,d);
			const btlScalar	m=mrg+(o-node->m_q).length()*2;
			if(d<(m*m))
			{
				const btlSoftBody::Node*	n[]={face->m_n[0],face->m_n[1],face->m_n[2]};
				const btlVector3			w=BaryCoord(n[0]->m_x,n[1]->m_x,n[2]->m_x,p+o);
				const btlScalar			ma=node->m_im;
				btlScalar				mb=BaryEval(n[0]->m_im,n[1]->m_im,n[2]->m_im,w);
				if(	(n[0]->m_im<=0)||
					(n[1]->m_im<=0)||
					(n[2]->m_im<=0))
				{
					mb=0;
				}
				const btlScalar	ms=ma+mb;
				if(ms>0)
				{
					btlSoftBody::SContact	c;
					c.m_normal		=	p/-btlSqrt(d);
					c.m_margin		=	m;
					c.m_node		=	node;
					c.m_face		=	face;
					c.m_weights		=	w;
					c.m_friction	=	btlMax(psb[0]->m_cfg.kDF,psb[1]->m_cfg.kDF);
					c.m_cfm[0]		=	ma/ms*psb[0]->m_cfg.kSHR;
					c.m_cfm[1]		=	mb/ms*psb[1]->m_cfg.kSHR;
					psb[0]->m_scontacts.push_back(c);
				}
			}	
		}
		btlSoftBody*		psb[2];
		btlScalar		mrg;
	};
};

#endif //_BT_SOFT_BODY_INTERNALS_H
