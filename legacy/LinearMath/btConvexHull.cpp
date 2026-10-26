/*
Stan Melax Convex Hull Computation
Copyright (c) 2003-2006 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <string.h>

#include "btConvexHull.h"
#include "btAlignedObjectArray.h"
#include "btMinMax.h"
#include "btVector3.h"





//----------------------------------

class btl_int3  
{
public:
	int x,y,z;
	btl_int3(){};
	btl_int3(int _x,int _y, int _z){x=_x;y=_y;z=_z;}
	const int& operator[](int i) const {return (&x)[i];}
	int& operator[](int i) {return (&x)[i];}
};


//------- btPlane ----------


inline btlPlane PlaneFlip(const btlPlane &plane){return btlPlane(-plane.normal,-plane.dist);}
inline int operator==( const btlPlane &a, const btlPlane &b ) { return (a.normal==b.normal && a.dist==b.dist); }
inline int coplanar( const btlPlane &a, const btlPlane &b ) { return (a==b || a==PlaneFlip(b)); }


//--------- Utility Functions ------

btlVector3  btl_PlaneLineIntersection(const btlPlane &plane, const btlVector3 &p0, const btlVector3 &p1);
btlVector3  btl_PlaneProject(const btlPlane &plane, const btlVector3 &point);

btlVector3  btl_ThreePlaneIntersection(const btlPlane &p0,const btlPlane &p1, const btlPlane &p2);
btlVector3  btl_ThreePlaneIntersection(const btlPlane &p0,const btlPlane &p1, const btlPlane &p2)
{
	btlVector3 N1 = p0.normal;
	btlVector3 N2 = p1.normal;
	btlVector3 N3 = p2.normal;

	btlVector3 n2n3; n2n3 = N2.cross(N3);
	btlVector3 n3n1; n3n1 = N3.cross(N1);
	btlVector3 n1n2; n1n2 = N1.cross(N2);

	btlScalar quotient = (N1.dot(n2n3));

	btlAssert(btlFabs(quotient) > btlScalar(0.000001));
	
	quotient = btlScalar(-1.) / quotient;
	n2n3 *= p0.dist;
	n3n1 *= p1.dist;
	n1n2 *= p2.dist;
	btlVector3 potentialVertex = n2n3;
	potentialVertex += n3n1;
	potentialVertex += n1n2;
	potentialVertex *= quotient;

	btlVector3 result(potentialVertex.getX(),potentialVertex.getY(),potentialVertex.getZ());
	return result;

}

btlScalar   btl_DistanceBetweenLines(const btlVector3 &ustart, const btlVector3 &udir, const btlVector3 &vstart, const btlVector3 &vdir, btlVector3 *upoint=NULL, btlVector3 *vpoint=NULL);
btlVector3  btl_TriNormal(const btlVector3 &v0, const btlVector3 &v1, const btlVector3 &v2);
btlVector3  NormalOf(const btlVector3 *vert, const int n);


btlVector3 btl_PlaneLineIntersection(const btlPlane &plane, const btlVector3 &p0, const btlVector3 &p1)
{
	// returns the point where the line p0-p1 intersects the plane n&d
				static btlVector3 dif;
		dif = p1-p0;
				btlScalar dn= btlDot(plane.normal,dif);
				btlScalar t = -(plane.dist+btlDot(plane.normal,p0) )/dn;
				return p0 + (dif*t);
}

btlVector3 btl_PlaneProject(const btlPlane &plane, const btlVector3 &point)
{
	return point - plane.normal * (btlDot(point,plane.normal)+plane.dist);
}

btlVector3 btl_TriNormal(const btlVector3 &v0, const btlVector3 &v1, const btlVector3 &v2)
{
	// return the normal of the triangle
	// inscribed by v0, v1, and v2
	btlVector3 cp=btlCross(v1-v0,v2-v1);
	btlScalar m=cp.length();
	if(m==0) return btlVector3(1,0,0);
	return cp*(btlScalar(1.0)/m);
}


btlScalar btl_DistanceBetweenLines(const btlVector3 &ustart, const btlVector3 &udir, const btlVector3 &vstart, const btlVector3 &vdir, btlVector3 *upoint, btlVector3 *vpoint)
{
	static btlVector3 cp;
	cp = btlCross(udir,vdir).normalized();

	btlScalar distu = -btlDot(cp,ustart);
	btlScalar distv = -btlDot(cp,vstart);
	btlScalar dist = (btlScalar)fabs(distu-distv);
	if(upoint) 
		{
		btlPlane plane;
		plane.normal = btlCross(vdir,cp).normalized();
		plane.dist = -btlDot(plane.normal,vstart);
		*upoint = btl_PlaneLineIntersection(plane,ustart,ustart+udir);
	}
	if(vpoint) 
		{
		btlPlane plane;
		plane.normal = btlCross(udir,cp).normalized();
		plane.dist = -btlDot(plane.normal,ustart);
		*vpoint = btl_PlaneLineIntersection(plane,vstart,vstart+vdir);
	}
	return dist;
}







#define COPLANAR   (0)
#define UNDER      (1)
#define OVER       (2)
#define SPLIT      (OVER|UNDER)
#define PAPERWIDTH (btlScalar(0.001))

btlScalar btl_planetestepsilon = PAPERWIDTH;



typedef btl_ConvexH::HalfEdge HalfEdge;

btl_ConvexH::btl_ConvexH(int vertices_size,int edges_size,int facets_size)
{
	vertices.resize(vertices_size);
	edges.resize(edges_size);
	facets.resize(facets_size);
}


int btl_PlaneTest(const btlPlane &p, const btlVector3 &v);
int btl_PlaneTest(const btlPlane &p, const btlVector3 &v) {
	btlScalar a  = btlDot(v,p.normal)+p.dist;
	int   flag = (a>btl_planetestepsilon)?OVER:((a<-btl_planetestepsilon)?UNDER:COPLANAR);
	return flag;
}

int btl_SplitTest(btl_ConvexH &convex,const btlPlane &plane);
int btl_SplitTest(btl_ConvexH &convex,const btlPlane &plane) {
	int flag=0;
	for(int i=0;i<convex.vertices.size();i++) {
		flag |= btl_PlaneTest(plane,convex.vertices[i]);
	}
	return flag;
}

class VertFlag
{
public:
	unsigned char planetest;
	unsigned char junk;
	unsigned char undermap;
	unsigned char overmap;
};
class EdgeFlag 
{
public:
	unsigned char planetest;
	unsigned char fixes;
	short undermap;
	short overmap;
};
class PlaneFlag
{
public:
	unsigned char undermap;
	unsigned char overmap;
};
class Coplanar{
public:
	unsigned short ea;
	unsigned char v0;
	unsigned char v1;
};








template<class T>
int maxdirfiltered(const T *p,int count,const T &dir,btlAlignedObjectArray<int> &allow)
{
	btlAssert(count);
	int m=-1;
	for(int i=0;i<count;i++) 
		if(allow[i])
		{
			if(m==-1 || btlDot(p[i],dir)>btlDot(p[m],dir))
				m=i;
		}
	btlAssert(m!=-1);
	return m;
} 

btlVector3 btl_orth(const btlVector3 &v);
btlVector3 btl_orth(const btlVector3 &v)
{
	btlVector3 a=btlCross(v,btlVector3(0,0,1));
	btlVector3 b=btlCross(v,btlVector3(0,1,0));
	if (a.length() > b.length())
	{
		return a.normalized();
	} else {
		return b.normalized();
	}
}


template<class T>
int btl_maxdirsterid(const T *p,int count,const T &dir,btlAlignedObjectArray<int> &allow)
{
	int m=-1;
	while(m==-1)
	{
		m = maxdirfiltered(p,count,dir,allow);
		if(allow[m]==3) return m;
		T u = btl_orth(dir);
		T v = btlCross(u,dir);
		int ma=-1;
		for(btlScalar x = btlScalar(0.0) ; x<= btlScalar(360.0) ; x+= btlScalar(45.0))
		{
			btlScalar s = btlSin(SIMD_RADS_PER_DEG*(x));
			btlScalar c = btlCos(SIMD_RADS_PER_DEG*(x));
			int mb = maxdirfiltered(p,count,dir+(u*s+v*c)*btlScalar(0.025),allow);
			if(ma==m && mb==m)
			{
				allow[m]=3;
				return m;
			}
			if(ma!=-1 && ma!=mb)  // Yuck - this is really ugly
			{
				int mc = ma;
				for(btlScalar xx = x-btlScalar(40.0) ; xx <= x ; xx+= btlScalar(5.0))
				{
					btlScalar s = btlSin(SIMD_RADS_PER_DEG*(xx));
					btlScalar c = btlCos(SIMD_RADS_PER_DEG*(xx));
					int md = maxdirfiltered(p,count,dir+(u*s+v*c)*btlScalar(0.025),allow);
					if(mc==m && md==m)
					{
						allow[m]=3;
						return m;
					}
					mc=md;
				}
			}
			ma=mb;
		}
		allow[m]=0;
		m=-1;
	}
	btlAssert(0);
	return m;
} 




int operator ==(const btl_int3 &a,const btl_int3 &b);
int operator ==(const btl_int3 &a,const btl_int3 &b) 
{
	for(int i=0;i<3;i++) 
	{
		if(a[i]!=b[i]) return 0;
	}
	return 1;
}


int btl_above(btlVector3* vertices,const btl_int3& t, const btlVector3 &p, btlScalar epsilon);
int btl_above(btlVector3* vertices,const btl_int3& t, const btlVector3 &p, btlScalar epsilon) 
{
	btlVector3 n=btl_TriNormal(vertices[t[0]],vertices[t[1]],vertices[t[2]]);
	return (btlDot(n,p-vertices[t[0]]) > epsilon); // EPSILON???
}
int btl_hasedge(const btl_int3 &t, int a,int b);
int btl_hasedge(const btl_int3 &t, int a,int b)
{
	for(int i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(t[i]==a && t[i1]==b) return 1;
	}
	return 0;
}
int btl_hasvert(const btl_int3 &t, int v);
int btl_hasvert(const btl_int3 &t, int v)
{
	return (t[0]==v || t[1]==v || t[2]==v) ;
}
int btl_shareedge(const btl_int3 &a,const btl_int3 &b);
int btl_shareedge(const btl_int3 &a,const btl_int3 &b)
{
	int i;
	for(i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(btl_hasedge(a,b[i1],b[i])) return 1;
	}
	return 0;
}

class btlHullTriangle;



class btlHullTriangle : public btl_int3
{
public:
	btl_int3 n;
	int id;
	int vmax;
	btlScalar rise;
	btlHullTriangle(int a,int b,int c):btl_int3(a,b,c),n(-1,-1,-1)
	{
		vmax=-1;
		rise = btlScalar(0.0);
	}
	~btlHullTriangle()
	{
	}
	int &neib(int a,int b);
};


int &btlHullTriangle::neib(int a,int b)
{
	static int er=-1;
	int i;
	for(i=0;i<3;i++) 
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		if((*this)[i]==a && (*this)[i1]==b) return n[i2];
		if((*this)[i]==b && (*this)[i1]==a) return n[i2];
	}
	btlAssert(0);
	return er;
}
void btl_HullLibrary::b2bfix(btlHullTriangle* s,btlHullTriangle*t)
{
	int i;
	for(i=0;i<3;i++) 
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int a = (*s)[i1];
		int b = (*s)[i2];
		btlAssert(m_tris[s->neib(a,b)]->neib(b,a) == s->id);
		btlAssert(m_tris[t->neib(a,b)]->neib(b,a) == t->id);
		m_tris[s->neib(a,b)]->neib(b,a) = t->neib(b,a);
		m_tris[t->neib(b,a)]->neib(a,b) = s->neib(a,b);
	}
}

void btl_HullLibrary::removeb2b(btlHullTriangle* s,btlHullTriangle*t)
{
	b2bfix(s,t);
	deAllocateTriangle(s);

	deAllocateTriangle(t);
}

void btl_HullLibrary::checkit(btlHullTriangle *t)
{
	(void)t;

	int i;
	btlAssert(m_tris[t->id]==t);
	for(i=0;i<3;i++)
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int a = (*t)[i1];
		int b = (*t)[i2];

		// release compile fix
		(void)i1;
		(void)i2;
		(void)a;
		(void)b;

		btlAssert(a!=b);
		btlAssert( m_tris[t->n[i]]->neib(b,a) == t->id);
	}
}

btlHullTriangle*	btl_HullLibrary::allocateTriangle(int a,int b,int c)
{
	void* mem = btlAlignedAlloc(sizeof(btlHullTriangle),16);
	btlHullTriangle* tr = new (mem)btlHullTriangle(a,b,c);
	tr->id = m_tris.size();
	m_tris.push_back(tr);

	return tr;
}

void	btl_HullLibrary::deAllocateTriangle(btlHullTriangle* tri)
{
	btlAssert(m_tris[tri->id]==tri);
	m_tris[tri->id]=NULL;
	tri->~btlHullTriangle();
	btlAlignedFree(tri);
}


void btl_HullLibrary::extrude(btlHullTriangle *t0,int v)
{
	btl_int3 t= *t0;
	int n = m_tris.size();
	btlHullTriangle* ta = allocateTriangle(v,t[1],t[2]);
	ta->n = btl_int3(t0->n[0],n+1,n+2);
	m_tris[t0->n[0]]->neib(t[1],t[2]) = n+0;
	btlHullTriangle* tb = allocateTriangle(v,t[2],t[0]);
	tb->n = btl_int3(t0->n[1],n+2,n+0);
	m_tris[t0->n[1]]->neib(t[2],t[0]) = n+1;
	btlHullTriangle* tc = allocateTriangle(v,t[0],t[1]);
	tc->n = btl_int3(t0->n[2],n+0,n+1);
	m_tris[t0->n[2]]->neib(t[0],t[1]) = n+2;
	checkit(ta);
	checkit(tb);
	checkit(tc);
	if(btl_hasvert(*m_tris[ta->n[0]],v)) removeb2b(ta,m_tris[ta->n[0]]);
	if(btl_hasvert(*m_tris[tb->n[0]],v)) removeb2b(tb,m_tris[tb->n[0]]);
	if(btl_hasvert(*m_tris[tc->n[0]],v)) removeb2b(tc,m_tris[tc->n[0]]);
	deAllocateTriangle(t0);

}

btlHullTriangle* btl_HullLibrary::extrudable(btlScalar epsilon)
{
	int i;
	btlHullTriangle *t=NULL;
	for(i=0;i<m_tris.size();i++)
	{
		if(!t || (m_tris[i] && t->rise<m_tris[i]->rise))
		{
			t = m_tris[i];
		}
	}
	return (t->rise >epsilon)?t:NULL ;
}




btl_int4 btl_HullLibrary::FindSimplex(btlVector3 *verts,int verts_count,btlAlignedObjectArray<int> &allow)
{
	btlVector3 basis[3];
	basis[0] = btlVector3( btlScalar(0.01), btlScalar(0.02), btlScalar(1.0) );      
	int p0 = btl_maxdirsterid(verts,verts_count, basis[0],allow);   
	int	p1 = btl_maxdirsterid(verts,verts_count,-basis[0],allow);
	basis[0] = verts[p0]-verts[p1];
	if(p0==p1 || basis[0]==btlVector3(0,0,0)) 
		return btl_int4(-1,-1,-1,-1);
	basis[1] = btlCross(btlVector3(     btlScalar(1),btlScalar(0.02), btlScalar(0)),basis[0]);
	basis[2] = btlCross(btlVector3(btlScalar(-0.02),     btlScalar(1), btlScalar(0)),basis[0]);
	if (basis[1].length() > basis[2].length())
	{
		basis[1].normalize();
	} else {
		basis[1] = basis[2];
		basis[1].normalize ();
	}
	int p2 = btl_maxdirsterid(verts,verts_count,basis[1],allow);
	if(p2 == p0 || p2 == p1)
	{
		p2 = btl_maxdirsterid(verts,verts_count,-basis[1],allow);
	}
	if(p2 == p0 || p2 == p1) 
		return btl_int4(-1,-1,-1,-1);
	basis[1] = verts[p2] - verts[p0];
	basis[2] = btlCross(basis[1],basis[0]).normalized();
	int p3 = btl_maxdirsterid(verts,verts_count,basis[2],allow);
	if(p3==p0||p3==p1||p3==p2) p3 = btl_maxdirsterid(verts,verts_count,-basis[2],allow);
	if(p3==p0||p3==p1||p3==p2) 
		return btl_int4(-1,-1,-1,-1);
	btlAssert(!(p0==p1||p0==p2||p0==p3||p1==p2||p1==p3||p2==p3));
	if(btlDot(verts[p3]-verts[p0],btlCross(verts[p1]-verts[p0],verts[p2]-verts[p0])) <0) {btlSwap(p2,p3);}
	return btl_int4(p0,p1,p2,p3);
}

int btl_HullLibrary::calchullgen(btlVector3 *verts,int verts_count, int vlimit)
{
	if(verts_count <4) return 0;
	if(vlimit==0) vlimit=1000000000;
	int j;
	btlVector3 bmin(*verts),bmax(*verts);
	btlAlignedObjectArray<int> isextreme;
	isextreme.reserve(verts_count);
	btlAlignedObjectArray<int> allow;
	allow.reserve(verts_count);

	for(j=0;j<verts_count;j++) 
	{
		allow.push_back(1);
		isextreme.push_back(0);
		bmin.setMin (verts[j]);
		bmax.setMax (verts[j]);
	}
	btlScalar epsilon = (bmax-bmin).length() * btlScalar(0.001);
	btlAssert (epsilon != 0.0);


	btl_int4 p = FindSimplex(verts,verts_count,allow);
	if(p.x==-1) return 0; // simplex failed



	btlVector3 center = (verts[p[0]]+verts[p[1]]+verts[p[2]]+verts[p[3]]) / btlScalar(4.0);  // a valid interior point
	btlHullTriangle *t0 = allocateTriangle(p[2],p[3],p[1]); t0->n=btl_int3(2,3,1);
	btlHullTriangle *t1 = allocateTriangle(p[3],p[2],p[0]); t1->n=btl_int3(3,2,0);
	btlHullTriangle *t2 = allocateTriangle(p[0],p[1],p[3]); t2->n=btl_int3(0,1,3);
	btlHullTriangle *t3 = allocateTriangle(p[1],p[0],p[2]); t3->n=btl_int3(1,0,2);
	isextreme[p[0]]=isextreme[p[1]]=isextreme[p[2]]=isextreme[p[3]]=1;
	checkit(t0);checkit(t1);checkit(t2);checkit(t3);

	for(j=0;j<m_tris.size();j++)
	{
		btlHullTriangle *t=m_tris[j];
		btlAssert(t);
		btlAssert(t->vmax<0);
		btlVector3 n=btl_TriNormal(verts[(*t)[0]],verts[(*t)[1]],verts[(*t)[2]]);
		t->vmax = btl_maxdirsterid(verts,verts_count,n,allow);
		t->rise = btlDot(n,verts[t->vmax]-verts[(*t)[0]]);
	}
	btlHullTriangle *te;
	vlimit-=4;
	while(vlimit >0 && ((te=extrudable(epsilon)) != 0))
	{
		//int3 ti=*te;
		int v=te->vmax;
		btlAssert(v != -1);
		btlAssert(!isextreme[v]);  // wtf we've already done this vertex
		isextreme[v]=1;
		//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
		j=m_tris.size();
		while(j--) {
			if(!m_tris[j]) continue;
			btl_int3 t=*m_tris[j];
			if(btl_above(verts,t,verts[v],btlScalar(0.01)*epsilon)) 
			{
				extrude(m_tris[j],v);
			}
		}
		// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
		j=m_tris.size();
		while(j--)
		{
			if(!m_tris[j]) continue;
			if(!btl_hasvert(*m_tris[j],v)) break;
			btl_int3 nt=*m_tris[j];
			if(btl_above(verts,nt,center,btlScalar(0.01)*epsilon)  || btlCross(verts[nt[1]]-verts[nt[0]],verts[nt[2]]-verts[nt[1]]).length()< epsilon*epsilon*btlScalar(0.1) )
			{
				btlHullTriangle *nb = m_tris[m_tris[j]->n[0]];
				btlAssert(nb);btlAssert(!btl_hasvert(*nb,v));btlAssert(nb->id<j);
				extrude(nb,v);
				j=m_tris.size(); 
			}
		} 
		j=m_tris.size();
		while(j--)
		{
			btlHullTriangle *t=m_tris[j];
			if(!t) continue;
			if(t->vmax>=0) break;
			btlVector3 n=btl_TriNormal(verts[(*t)[0]],verts[(*t)[1]],verts[(*t)[2]]);
			t->vmax = btl_maxdirsterid(verts,verts_count,n,allow);
			if(isextreme[t->vmax]) 
			{
				t->vmax=-1; // already done that vertex - algorithm needs to be able to terminate.
			}
			else
			{
				t->rise = btlDot(n,verts[t->vmax]-verts[(*t)[0]]);
			}
		}
		vlimit --;
	}
	return 1;
}

int btl_HullLibrary::calchull(btlVector3 *verts,int verts_count, TUIntArray& tris_out, int &tris_count,int vlimit) 
{
	int rc=calchullgen(verts,verts_count,  vlimit) ;
	if(!rc) return 0;
	btlAlignedObjectArray<int> ts;
	int i;

	for(i=0;i<m_tris.size();i++)
	{
		if(m_tris[i])
		{
			for(int j=0;j<3;j++)
				ts.push_back((*m_tris[i])[j]);
			deAllocateTriangle(m_tris[i]);
		}
	}
	tris_count = ts.size()/3;
	tris_out.resize(ts.size());
	
	for (i=0;i<ts.size();i++)
	{
		tris_out[i] = static_cast<unsigned int>(ts[i]);
	}
	m_tris.resize(0);

	return 1;
}





bool btl_HullLibrary::ComputeHull(unsigned int vcount,const btlVector3 *vertices,btl_PHullResult &result,unsigned int vlimit)
{
	
	int    tris_count;
	int ret = calchull( (btlVector3 *) vertices, (int) vcount, result.m_Indices, tris_count, static_cast<int>(vlimit) );
	if(!ret) return false;
	result.mIndexCount = (unsigned int) (tris_count*3);
	result.mFaceCount  = (unsigned int) tris_count;
	result.mVertices   = (btlVector3*) vertices;
	result.mVcount     = (unsigned int) vcount;
	return true;

}


void btl_ReleaseHull(btl_PHullResult &result);
void btl_ReleaseHull(btl_PHullResult &result)
{
	if ( result.m_Indices.size() )
	{
		result.m_Indices.clear();
	}

	result.mVcount = 0;
	result.mIndexCount = 0;
	result.mVertices = 0;
}


//*********************************************************************
//*********************************************************************
//********  HullLib header
//*********************************************************************
//*********************************************************************

//*********************************************************************
//*********************************************************************
//********  HullLib implementation
//*********************************************************************
//*********************************************************************

HullError btl_HullLibrary::CreateConvexHull(const btl_HullDesc       &desc,           // describes the input request
																					btl_HullResult           &result)         // contains the resulst
{
	HullError ret = QE_FAIL;


	btl_PHullResult hr;

	unsigned int vcount = desc.mVcount;
	if ( vcount < 8 ) vcount = 8;

	btlAlignedObjectArray<btlVector3> vertexSource;
	vertexSource.resize(static_cast<int>(vcount));

	btlVector3 scale;

	unsigned int ovcount;

	bool ok = CleanupVertices(desc.mVcount,desc.mVertices, desc.mVertexStride, ovcount, &vertexSource[0], desc.mNormalEpsilon, scale ); // normalize point cloud, remove duplicates!

	if ( ok )
	{


//		if ( 1 ) // scale vertices back to their original size.
		{
			for (unsigned int i=0; i<ovcount; i++)
			{
				btlVector3& v = vertexSource[static_cast<int>(i)];
				v[0]*=scale[0];
				v[1]*=scale[1];
				v[2]*=scale[2];
			}
		}

		ok = ComputeHull(ovcount,&vertexSource[0],hr,desc.mMaxVertices);

		if ( ok )
		{

			// re-index triangle mesh so it refers to only used vertices, rebuild a new vertex table.
			btlAlignedObjectArray<btlVector3>	vertexScratch;
			vertexScratch.resize(static_cast<int>(hr.mVcount));

			BringOutYourDead(hr.mVertices,hr.mVcount, &vertexScratch[0], ovcount, &hr.m_Indices[0], hr.mIndexCount );

			ret = QE_OK;

			if ( desc.HasHullFlag(QF_TRIANGLES) ) // if he wants the results as triangle!
			{
				result.mPolygons          = false;
				result.mNumOutputVertices = ovcount;
				result.m_OutputVertices.resize(static_cast<int>(ovcount));
				result.mNumFaces          = hr.mFaceCount;
				result.mNumIndices        = hr.mIndexCount;

				result.m_Indices.resize(static_cast<int>(hr.mIndexCount));

				memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(btlVector3)*ovcount );

  			if ( desc.HasHullFlag(QF_REVERSE_ORDER) )
				{

					const unsigned int *source = &hr.m_Indices[0];
					unsigned int *dest   = &result.m_Indices[0];

					for (unsigned int i=0; i<hr.mFaceCount; i++)
					{
						dest[0] = source[2];
						dest[1] = source[1];
						dest[2] = source[0];
						dest+=3;
						source+=3;
					}

				}
				else
				{
					memcpy(&result.m_Indices[0], &hr.m_Indices[0], sizeof(unsigned int)*hr.mIndexCount);
				}
			}
			else
			{
				result.mPolygons          = true;
				result.mNumOutputVertices = ovcount;
				result.m_OutputVertices.resize(static_cast<int>(ovcount));
				result.mNumFaces          = hr.mFaceCount;
				result.mNumIndices        = hr.mIndexCount+hr.mFaceCount;
				result.m_Indices.resize(static_cast<int>(result.mNumIndices));
				memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(btlVector3)*ovcount );

//				if ( 1 )
				{
					const unsigned int *source = &hr.m_Indices[0];
					unsigned int *dest   = &result.m_Indices[0];
					for (unsigned int i=0; i<hr.mFaceCount; i++)
					{
						dest[0] = 3;
						if ( desc.HasHullFlag(QF_REVERSE_ORDER) )
						{
							dest[1] = source[2];
							dest[2] = source[1];
							dest[3] = source[0];
						}
						else
						{
							dest[1] = source[0];
							dest[2] = source[1];
							dest[3] = source[2];
						}

						dest+=4;
						source+=3;
					}
				}
			}
			btl_ReleaseHull(hr);
		}
	}

	return ret;
}



HullError btl_HullLibrary::ReleaseResult(btl_HullResult &result) // release memory allocated for this result, we are done with it.
{
	if ( result.m_OutputVertices.size())
	{
		result.mNumOutputVertices=0;
		result.m_OutputVertices.clear();
	}
	if ( result.m_Indices.size() )
	{
		result.mNumIndices=0;
		result.m_Indices.clear();
	}
	return QE_OK;
}


static void addPoint(unsigned int &vcount,btlVector3 *p,btlScalar x,btlScalar y,btlScalar z)
{
	// XXX, might be broken
	btlVector3& dest = p[vcount];
	dest[0] = x;
	dest[1] = y;
	dest[2] = z;
	vcount++;
}

btlScalar btl_GetDist(btlScalar px,btlScalar py,btlScalar pz,const btlScalar *p2);
btlScalar btl_GetDist(btlScalar px,btlScalar py,btlScalar pz,const btlScalar *p2)
{

	btlScalar dx = px - p2[0];
	btlScalar dy = py - p2[1];
	btlScalar dz = pz - p2[2];

	return dx*dx+dy*dy+dz*dz;
}



bool  btl_HullLibrary::CleanupVertices(unsigned int svcount,
				   const btlVector3 *svertices,
				   unsigned int stride,
				   unsigned int &vcount,       // output number of vertices
				   btlVector3 *vertices,                 // location to store the results.
				   btlScalar  normalepsilon,
				   btlVector3& scale)
{
	if ( svcount == 0 ) return false;

	m_vertexIndexMapping.resize(0);


#define EPSILON btlScalar(0.000001) /* close enough to consider two btScalaring point numbers to be 'the same'. */

	vcount = 0;

	btlScalar recip[3]={0.f,0.f,0.f};

	if ( scale )
	{
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
	}

	btlScalar bmin[3] = {  FLT_MAX,  FLT_MAX,  FLT_MAX };
	btlScalar bmax[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	const char *vtx = (const char *) svertices;

//	if ( 1 )
	{
		for (unsigned int i=0; i<svcount; i++)
		{
			const btlScalar *p = (const btlScalar *) vtx;

			vtx+=stride;

			for (int j=0; j<3; j++)
			{
				if ( p[j] < bmin[j] ) bmin[j] = p[j];
				if ( p[j] > bmax[j] ) bmax[j] = p[j];
			}
		}
	}

	btlScalar dx = bmax[0] - bmin[0];
	btlScalar dy = bmax[1] - bmin[1];
	btlScalar dz = bmax[2] - bmin[2];

	btlVector3 center;

	center[0] = dx*btlScalar(0.5) + bmin[0];
	center[1] = dy*btlScalar(0.5) + bmin[1];
	center[2] = dz*btlScalar(0.5) + bmin[2];

	if ( dx < EPSILON || dy < EPSILON || dz < EPSILON || svcount < 3 )
	{

		btlScalar len = FLT_MAX;

		if ( dx > EPSILON && dx < len ) len = dx;
		if ( dy > EPSILON && dy < len ) len = dy;
		if ( dz > EPSILON && dz < len ) len = dz;

		if ( len == FLT_MAX )
		{
			dx = dy = dz = btlScalar(0.01); // one centimeter
		}
		else
		{
			if ( dx < EPSILON ) dx = len * btlScalar(0.05); // 1/5th the shortest non-zero edge.
			if ( dy < EPSILON ) dy = len * btlScalar(0.05);
			if ( dz < EPSILON ) dz = len * btlScalar(0.05);
		}

		btlScalar x1 = center[0] - dx;
		btlScalar x2 = center[0] + dx;

		btlScalar y1 = center[1] - dy;
		btlScalar y2 = center[1] + dy;

		btlScalar z1 = center[2] - dz;
		btlScalar z2 = center[2] + dz;

		addPoint(vcount,vertices,x1,y1,z1);
		addPoint(vcount,vertices,x2,y1,z1);
		addPoint(vcount,vertices,x2,y2,z1);
		addPoint(vcount,vertices,x1,y2,z1);
		addPoint(vcount,vertices,x1,y1,z2);
		addPoint(vcount,vertices,x2,y1,z2);
		addPoint(vcount,vertices,x2,y2,z2);
		addPoint(vcount,vertices,x1,y2,z2);

		return true; // return cube


	}
	else
	{
		if ( scale )
		{
			scale[0] = dx;
			scale[1] = dy;
			scale[2] = dz;

			recip[0] = 1 / dx;
			recip[1] = 1 / dy;
			recip[2] = 1 / dz;

			center[0]*=recip[0];
			center[1]*=recip[1];
			center[2]*=recip[2];

		}

	}



	vtx = (const char *) svertices;

	for (unsigned int i=0; i<svcount; i++)
	{
		const btlVector3 *p = (const btlVector3 *)vtx;
		vtx+=stride;

		btlScalar px = p->getX();
		btlScalar py = p->getY();
		btlScalar pz = p->getZ();

		if ( scale )
		{
			px = px*recip[0]; // normalize
			py = py*recip[1]; // normalize
			pz = pz*recip[2]; // normalize
		}

//		if ( 1 )
		{
			unsigned int j;

			for (j=0; j<vcount; j++)
			{
				/// XXX might be broken
				btlVector3& v = vertices[j];

				btlScalar x = v[0];
				btlScalar y = v[1];
				btlScalar z = v[2];

				btlScalar dx = btlFabs(x - px );
				btlScalar dy = btlFabs(y - py );
				btlScalar dz = btlFabs(z - pz );

				if ( dx < normalepsilon && dy < normalepsilon && dz < normalepsilon )
				{
					// ok, it is close enough to the old one
					// now let us see if it is further from the center of the point cloud than the one we already recorded.
					// in which case we keep this one instead.

					btlScalar dist1 = btl_GetDist(px,py,pz,center);
					btlScalar dist2 = btl_GetDist(v[0],v[1],v[2],center);

					if ( dist1 > dist2 )
					{
						v[0] = px;
						v[1] = py;
						v[2] = pz;
						
					}

					break;
				}
			}

			if ( j == vcount )
			{
				btlVector3& dest = vertices[vcount];
				dest[0] = px;
				dest[1] = py;
				dest[2] = pz;
				vcount++;
			}
			m_vertexIndexMapping.push_back(j);
		}
	}

	// ok..now make sure we didn't prune so many vertices it is now invalid.
//	if ( 1 )
	{
		btlScalar bmin[3] = {  FLT_MAX,  FLT_MAX,  FLT_MAX };
		btlScalar bmax[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

		for (unsigned int i=0; i<vcount; i++)
		{
			const btlVector3& p = vertices[i];
			for (int j=0; j<3; j++)
			{
				if ( p[j] < bmin[j] ) bmin[j] = p[j];
				if ( p[j] > bmax[j] ) bmax[j] = p[j];
			}
		}

		btlScalar dx = bmax[0] - bmin[0];
		btlScalar dy = bmax[1] - bmin[1];
		btlScalar dz = bmax[2] - bmin[2];

		if ( dx < EPSILON || dy < EPSILON || dz < EPSILON || vcount < 3)
		{
			btlScalar cx = dx*btlScalar(0.5) + bmin[0];
			btlScalar cy = dy*btlScalar(0.5) + bmin[1];
			btlScalar cz = dz*btlScalar(0.5) + bmin[2];

			btlScalar len = FLT_MAX;

			if ( dx >= EPSILON && dx < len ) len = dx;
			if ( dy >= EPSILON && dy < len ) len = dy;
			if ( dz >= EPSILON && dz < len ) len = dz;

			if ( len == FLT_MAX )
			{
				dx = dy = dz = btlScalar(0.01); // one centimeter
			}
			else
			{
				if ( dx < EPSILON ) dx = len * btlScalar(0.05); // 1/5th the shortest non-zero edge.
				if ( dy < EPSILON ) dy = len * btlScalar(0.05);
				if ( dz < EPSILON ) dz = len * btlScalar(0.05);
			}

			btlScalar x1 = cx - dx;
			btlScalar x2 = cx + dx;

			btlScalar y1 = cy - dy;
			btlScalar y2 = cy + dy;

			btlScalar z1 = cz - dz;
			btlScalar z2 = cz + dz;

			vcount = 0; // add box

			addPoint(vcount,vertices,x1,y1,z1);
			addPoint(vcount,vertices,x2,y1,z1);
			addPoint(vcount,vertices,x2,y2,z1);
			addPoint(vcount,vertices,x1,y2,z1);
			addPoint(vcount,vertices,x1,y1,z2);
			addPoint(vcount,vertices,x2,y1,z2);
			addPoint(vcount,vertices,x2,y2,z2);
			addPoint(vcount,vertices,x1,y2,z2);

			return true;
		}
	}

	return true;
}

void btl_HullLibrary::BringOutYourDead(const btlVector3* verts,unsigned int vcount, btlVector3* overts,unsigned int &ocount,unsigned int *indices,unsigned indexcount)
{
	btlAlignedObjectArray<int>tmpIndices;
	tmpIndices.resize(m_vertexIndexMapping.size());
	int i;

	for (i=0;i<m_vertexIndexMapping.size();i++)
	{
		tmpIndices[i] = m_vertexIndexMapping[i];
	}

	TUIntArray usedIndices;
	usedIndices.resize(static_cast<int>(vcount));
	memset(&usedIndices[0],0,sizeof(unsigned int)*vcount);

	ocount = 0;

	for (i=0; i<int (indexcount); i++)
	{
		unsigned int v = indices[i]; // original array index

		btlAssert( v >= 0 && v < vcount );

		if ( usedIndices[static_cast<int>(v)] ) // if already remapped
		{
			indices[i] = usedIndices[static_cast<int>(v)]-1; // index to new array
		}
		else
		{

			indices[i] = ocount;      // new index mapping

			overts[ocount][0] = verts[v][0]; // copy old vert to new vert array
			overts[ocount][1] = verts[v][1];
			overts[ocount][2] = verts[v][2];

			for (int k=0;k<m_vertexIndexMapping.size();k++)
			{
				if (tmpIndices[k]==int(v))
					m_vertexIndexMapping[k]=ocount;
			}

			ocount++; // increment output vert count

			btlAssert( ocount >=0 && ocount <= vcount );

			usedIndices[static_cast<int>(v)] = ocount; // assign new index remapping

		
		}
	}

	
}
