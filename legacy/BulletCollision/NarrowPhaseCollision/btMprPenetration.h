
/***
 * ---------------------------------
 * Copyright (c)2012 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file was ported from mpr.c file, part of libccd.
 *  The Minkoski Portal Refinement implementation was ported 
 *  to OpenCL by Erwin Coumans for the Bullet 3 Physics library.
 *  The original MPR idea and implementation is by Gary Snethen
 *  in XenoCollide, see http://github.com/erwincoumans/xenocollide
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see <http://www.opensource.org/licenses/bsd-license.php>.
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

///2014 Oct, Erwin Coumans, Use templates to avoid void* casts

#ifndef BT_MPR_PENETRATION_H
#define BT_MPR_PENETRATION_H

#define BT_DEBUG_MPR1

#include "LinearMath/btTransform.h"
#include "LinearMath/btAlignedObjectArray.h"

//#define MPR_AVERAGE_CONTACT_POSITIONS


struct btlMprCollisionDescription
{
    btlVector3	m_firstDir;
    int			m_maxGjkIterations;
    btlScalar	m_maximumDistanceSquared;
    btlScalar	m_gjkRelError2;
   
    btlMprCollisionDescription()
    :	m_firstDir(0,1,0),
        m_maxGjkIterations(1000),
        m_maximumDistanceSquared(1e30f),
        m_gjkRelError2(1.0e-6)
    {
    }
    virtual ~btlMprCollisionDescription()
    {
    }
};

struct btlMprDistanceInfo
{
    btlVector3	m_pointOnA;
    btlVector3	m_pointOnB;
    btlVector3	m_normalBtoA;
    btlScalar	m_distance;
};

#ifdef __cplusplus
#define BT_MPR_SQRT sqrtf
#else
#define BT_MPR_SQRT sqrt
#endif
#define BT_MPR_FMIN(x, y) ((x) < (y) ? (x) : (y))
#define BT_MPR_FABS fabs

#define BT_MPR_TOLERANCE 1E-6f
#define BT_MPR_MAX_ITERATIONS 1000

struct _btMprSupport_t 
{
    btlVector3 v;  //!< Support point in minkowski sum
    btlVector3 v1; //!< Support point in obj1
    btlVector3 v2; //!< Support point in obj2
};
typedef struct _btMprSupport_t btlMprSupport_t;

struct _btMprSimplex_t 
{
    btlMprSupport_t ps[4];
    int last; //!< index of last added point
};
typedef struct _btMprSimplex_t btlMprSimplex_t;

inline btlMprSupport_t* btlMprSimplexPointW(btlMprSimplex_t *s, int idx)
{
    return &s->ps[idx];
}

inline void btlMprSimplexSetSize(btlMprSimplex_t *s, int size)
{
    s->last = size - 1;
}

#ifdef DEBUG_MPR
inline void btlPrintPortalVertex(_btMprSimplex_t* portal, int index)
{
    printf("portal[%d].v = %f,%f,%f, v1=%f,%f,%f, v2=%f,%f,%f\n", index, portal->ps[index].v.x(),portal->ps[index].v.y(),portal->ps[index].v.z(),
           portal->ps[index].v1.x(),portal->ps[index].v1.y(),portal->ps[index].v1.z(),
           portal->ps[index].v2.x(),portal->ps[index].v2.y(),portal->ps[index].v2.z());
}
#endif //DEBUG_MPR




inline int btlMprSimplexSize(const btlMprSimplex_t *s)
{
    return s->last + 1;
}


inline const btlMprSupport_t* btlMprSimplexPoint(const btlMprSimplex_t* s, int idx)
{
    // here is no check on boundaries
    return &s->ps[idx];
}

inline void btlMprSupportCopy(btlMprSupport_t *d, const btlMprSupport_t *s)
{
    *d = *s;
}

inline void btlMprSimplexSet(btlMprSimplex_t *s, size_t pos, const btlMprSupport_t *a)
{
    btlMprSupportCopy(s->ps + pos, a);
}


inline void btlMprSimplexSwap(btlMprSimplex_t *s, size_t pos1, size_t pos2)
{
    btlMprSupport_t supp;

    btlMprSupportCopy(&supp, &s->ps[pos1]);
    btlMprSupportCopy(&s->ps[pos1], &s->ps[pos2]);
    btlMprSupportCopy(&s->ps[pos2], &supp);
}


inline int btlMprIsZero(float val)
{
    return BT_MPR_FABS(val) < FLT_EPSILON;
}



inline int btlMprEq(float _a, float _b)
{
    float ab;
    float a, b;

    ab = BT_MPR_FABS(_a - _b);
    if (BT_MPR_FABS(ab) < FLT_EPSILON)
        return 1;

    a = BT_MPR_FABS(_a);
    b = BT_MPR_FABS(_b);
    if (b > a){
        return ab < FLT_EPSILON * b;
    }else{
        return ab < FLT_EPSILON * a;
    }
}


inline int btlMprVec3Eq(const btlVector3* a, const btlVector3 *b)
{
    return btlMprEq((*a).x(), (*b).x())
            && btlMprEq((*a).y(), (*b).y())
            && btlMprEq((*a).z(), (*b).z());
}











template <typename btlConvexTemplate>
inline void btlFindOrigin(const btlConvexTemplate& a, const btlConvexTemplate& b, const btlMprCollisionDescription& colDesc,btlMprSupport_t *center)
{

	center->v1 = a.getObjectCenterInWorld();
    center->v2 = b.getObjectCenterInWorld();
    center->v = center->v1 - center->v2;
}

inline void btlMprVec3Set(btlVector3 *v, float x, float y, float z)
{
	v->setValue(x,y,z);
}

inline void btlMprVec3Add(btlVector3 *v, const btlVector3 *w)
{
	*v += *w;
}

inline void btlMprVec3Copy(btlVector3 *v, const btlVector3 *w)
{
    *v = *w;
}

inline void btlMprVec3Scale(btlVector3 *d, float k)
{
    *d *= k;
}

inline float btlMprVec3Dot(const btlVector3 *a, const btlVector3 *b)
{
    float dot;

	dot = btlDot(*a,*b);
    return dot;
}


inline float btlMprVec3Len2(const btlVector3 *v)
{
    return btlMprVec3Dot(v, v);
}

inline void btlMprVec3Normalize(btlVector3 *d)
{
    float k = 1.f / BT_MPR_SQRT(btlMprVec3Len2(d));
    btlMprVec3Scale(d, k);
}

inline void btlMprVec3Cross(btlVector3 *d, const btlVector3 *a, const btlVector3 *b)
{
	*d = btlCross(*a,*b);
	
}


inline void btlMprVec3Sub2(btlVector3 *d, const btlVector3 *v, const btlVector3 *w)
{
	*d = *v - *w;
}

inline void btlPortalDir(const btlMprSimplex_t *portal, btlVector3 *dir)
{
    btlVector3 v2v1, v3v1;

    btlMprVec3Sub2(&v2v1, &btlMprSimplexPoint(portal, 2)->v,
                       &btlMprSimplexPoint(portal, 1)->v);
    btlMprVec3Sub2(&v3v1, &btlMprSimplexPoint(portal, 3)->v,
                       &btlMprSimplexPoint(portal, 1)->v);
    btlMprVec3Cross(dir, &v2v1, &v3v1);
    btlMprVec3Normalize(dir);
}


inline int portalEncapsulesOrigin(const btlMprSimplex_t *portal,
                                       const btlVector3 *dir)
{
    float dot;
    dot = btlMprVec3Dot(dir, &btlMprSimplexPoint(portal, 1)->v);
    return btlMprIsZero(dot) || dot > 0.f;
}

inline int portalReachTolerance(const btlMprSimplex_t *portal,
                                     const btlMprSupport_t *v4,
                                     const btlVector3 *dir)
{
    float dv1, dv2, dv3, dv4;
    float dot1, dot2, dot3;

    // find the smallest dot product of dir and {v1-v4, v2-v4, v3-v4}

    dv1 = btlMprVec3Dot(&btlMprSimplexPoint(portal, 1)->v, dir);
    dv2 = btlMprVec3Dot(&btlMprSimplexPoint(portal, 2)->v, dir);
    dv3 = btlMprVec3Dot(&btlMprSimplexPoint(portal, 3)->v, dir);
    dv4 = btlMprVec3Dot(&v4->v, dir);

    dot1 = dv4 - dv1;
    dot2 = dv4 - dv2;
    dot3 = dv4 - dv3;

    dot1 = BT_MPR_FMIN(dot1, dot2);
    dot1 = BT_MPR_FMIN(dot1, dot3);

    return btlMprEq(dot1, BT_MPR_TOLERANCE) || dot1 < BT_MPR_TOLERANCE;
}

inline int portalCanEncapsuleOrigin(const btlMprSimplex_t *portal,
                                         const btlMprSupport_t *v4,
                                         const btlVector3 *dir)
{
    float dot;
    dot = btlMprVec3Dot(&v4->v, dir);
    return btlMprIsZero(dot) || dot > 0.f;
}

inline void btlExpandPortal(btlMprSimplex_t *portal,
                              const btlMprSupport_t *v4)
{
    float dot;
    btlVector3 v4v0;

    btlMprVec3Cross(&v4v0, &v4->v, &btlMprSimplexPoint(portal, 0)->v);
    dot = btlMprVec3Dot(&btlMprSimplexPoint(portal, 1)->v, &v4v0);
    if (dot > 0.f){
        dot = btlMprVec3Dot(&btlMprSimplexPoint(portal, 2)->v, &v4v0);
        if (dot > 0.f){
            btlMprSimplexSet(portal, 1, v4);
        }else{
            btlMprSimplexSet(portal, 3, v4);
        }
    }else{
        dot = btlMprVec3Dot(&btlMprSimplexPoint(portal, 3)->v, &v4v0);
        if (dot > 0.f){
            btlMprSimplexSet(portal, 2, v4);
        }else{
            btlMprSimplexSet(portal, 1, v4);
        }
    }
}
template <typename btlConvexTemplate>
inline void btlMprSupport(const btlConvexTemplate& a, const btlConvexTemplate& b,
                         const btlMprCollisionDescription& colDesc,
													const btlVector3& dir, btlMprSupport_t *supp)
{
	btlVector3 seperatingAxisInA = dir* a.getWorldTransform().getBasis();
	btlVector3 seperatingAxisInB = -dir* b.getWorldTransform().getBasis();

	btlVector3 pInA = a.getLocalSupportWithMargin(seperatingAxisInA);
	btlVector3 qInB = b.getLocalSupportWithMargin(seperatingAxisInB);

	supp->v1 = a.getWorldTransform()(pInA);
	supp->v2 = b.getWorldTransform()(qInB);
	supp->v = supp->v1 - supp->v2;
}


template <typename btlConvexTemplate>
static int btlDiscoverPortal(const btlConvexTemplate& a, const btlConvexTemplate& b,
                            const btlMprCollisionDescription& colDesc,
													btlMprSimplex_t *portal)
{
    btlVector3 dir, va, vb;
    float dot;
    int cont;
	
	

    // vertex 0 is center of portal
    btlFindOrigin(a,b,colDesc, btlMprSimplexPointW(portal, 0));
   
    
    // vertex 0 is center of portal
    btlMprSimplexSetSize(portal, 1);
    


	btlVector3 zero = btlVector3(0,0,0);
	btlVector3* org = &zero;

    if (btlMprVec3Eq(&btlMprSimplexPoint(portal, 0)->v, org)){
        // Portal's center lies on origin (0,0,0) => we know that objects
        // intersect but we would need to know penetration info.
        // So move center little bit...
        btlMprVec3Set(&va, FLT_EPSILON * 10.f, 0.f, 0.f);
        btlMprVec3Add(&btlMprSimplexPointW(portal, 0)->v, &va);
    }


    // vertex 1 = support in direction of origin
    btlMprVec3Copy(&dir, &btlMprSimplexPoint(portal, 0)->v);
    btlMprVec3Scale(&dir, -1.f);
    btlMprVec3Normalize(&dir);


    btlMprSupport(a,b,colDesc, dir, btlMprSimplexPointW(portal, 1));
 
    btlMprSimplexSetSize(portal, 2);

    // test if origin isn't outside of v1
    dot = btlMprVec3Dot(&btlMprSimplexPoint(portal, 1)->v, &dir);
	

    if (btlMprIsZero(dot) || dot < 0.f)
        return -1;


    // vertex 2
    btlMprVec3Cross(&dir, &btlMprSimplexPoint(portal, 0)->v,
                       &btlMprSimplexPoint(portal, 1)->v);
    if (btlMprIsZero(btlMprVec3Len2(&dir))){
        if (btlMprVec3Eq(&btlMprSimplexPoint(portal, 1)->v, org)){
            // origin lies on v1
            return 1;
        }else{
            // origin lies on v0-v1 segment
            return 2;
        }
    }

    btlMprVec3Normalize(&dir);
    btlMprSupport(a,b,colDesc, dir, btlMprSimplexPointW(portal, 2));
    
    
    
    dot = btlMprVec3Dot(&btlMprSimplexPoint(portal, 2)->v, &dir);
    if (btlMprIsZero(dot) || dot < 0.f)
        return -1;

    btlMprSimplexSetSize(portal, 3);

    // vertex 3 direction
    btlMprVec3Sub2(&va, &btlMprSimplexPoint(portal, 1)->v,
                     &btlMprSimplexPoint(portal, 0)->v);
    btlMprVec3Sub2(&vb, &btlMprSimplexPoint(portal, 2)->v,
                     &btlMprSimplexPoint(portal, 0)->v);
    btlMprVec3Cross(&dir, &va, &vb);
    btlMprVec3Normalize(&dir);

    // it is better to form portal faces to be oriented "outside" origin
    dot = btlMprVec3Dot(&dir, &btlMprSimplexPoint(portal, 0)->v);
    if (dot > 0.f){
        btlMprSimplexSwap(portal, 1, 2);
        btlMprVec3Scale(&dir, -1.f);
    }

    while (btlMprSimplexSize(portal) < 4){
		 btlMprSupport(a,b,colDesc, dir, btlMprSimplexPointW(portal, 3));
        
        dot = btlMprVec3Dot(&btlMprSimplexPoint(portal, 3)->v, &dir);
        if (btlMprIsZero(dot) || dot < 0.f)
            return -1;

        cont = 0;

        // test if origin is outside (v1, v0, v3) - set v2 as v3 and
        // continue
        btlMprVec3Cross(&va, &btlMprSimplexPoint(portal, 1)->v,
                          &btlMprSimplexPoint(portal, 3)->v);
        dot = btlMprVec3Dot(&va, &btlMprSimplexPoint(portal, 0)->v);
        if (dot < 0.f && !btlMprIsZero(dot)){
            btlMprSimplexSet(portal, 2, btlMprSimplexPoint(portal, 3));
            cont = 1;
        }

        if (!cont){
            // test if origin is outside (v3, v0, v2) - set v1 as v3 and
            // continue
            btlMprVec3Cross(&va, &btlMprSimplexPoint(portal, 3)->v,
                              &btlMprSimplexPoint(portal, 2)->v);
            dot = btlMprVec3Dot(&va, &btlMprSimplexPoint(portal, 0)->v);
            if (dot < 0.f && !btlMprIsZero(dot)){
                btlMprSimplexSet(portal, 1, btlMprSimplexPoint(portal, 3));
                cont = 1;
            }
        }

        if (cont){
            btlMprVec3Sub2(&va, &btlMprSimplexPoint(portal, 1)->v,
                             &btlMprSimplexPoint(portal, 0)->v);
            btlMprVec3Sub2(&vb, &btlMprSimplexPoint(portal, 2)->v,
                             &btlMprSimplexPoint(portal, 0)->v);
            btlMprVec3Cross(&dir, &va, &vb);
            btlMprVec3Normalize(&dir);
        }else{
            btlMprSimplexSetSize(portal, 4);
        }
    }

    return 0;
}

template <typename btlConvexTemplate>
static int btlRefinePortal(const btlConvexTemplate& a, const btlConvexTemplate& b,const btlMprCollisionDescription& colDesc,
							btlMprSimplex_t *portal)
{
    btlVector3 dir;
    btlMprSupport_t v4;

	for (int i=0;i<BT_MPR_MAX_ITERATIONS;i++)
    //while (1)
	{
        // compute direction outside the portal (from v0 throught v1,v2,v3
        // face)
        btlPortalDir(portal, &dir);

        // test if origin is inside the portal
        if (portalEncapsulesOrigin(portal, &dir))
            return 0;

        // get next support point
        
		 btlMprSupport(a,b,colDesc, dir, &v4);


        // test if v4 can expand portal to contain origin and if portal
        // expanding doesn't reach given tolerance
        if (!portalCanEncapsuleOrigin(portal, &v4, &dir)
                || portalReachTolerance(portal, &v4, &dir))
		{
            return -1;
        }

        // v1-v2-v3 triangle must be rearranged to face outside Minkowski
        // difference (direction from v0).
        btlExpandPortal(portal, &v4);
    }

    return -1;
}

static void btlFindPos(const btlMprSimplex_t *portal, btlVector3 *pos)
{

	btlVector3 zero = btlVector3(0,0,0);
	btlVector3* origin = &zero;

    btlVector3 dir;
    size_t i;
    float b[4], sum, inv;
    btlVector3 vec, p1, p2;

    btlPortalDir(portal, &dir);

    // use barycentric coordinates of tetrahedron to find origin
    btlMprVec3Cross(&vec, &btlMprSimplexPoint(portal, 1)->v,
                       &btlMprSimplexPoint(portal, 2)->v);
    b[0] = btlMprVec3Dot(&vec, &btlMprSimplexPoint(portal, 3)->v);

    btlMprVec3Cross(&vec, &btlMprSimplexPoint(portal, 3)->v,
                       &btlMprSimplexPoint(portal, 2)->v);
    b[1] = btlMprVec3Dot(&vec, &btlMprSimplexPoint(portal, 0)->v);

    btlMprVec3Cross(&vec, &btlMprSimplexPoint(portal, 0)->v,
                       &btlMprSimplexPoint(portal, 1)->v);
    b[2] = btlMprVec3Dot(&vec, &btlMprSimplexPoint(portal, 3)->v);

    btlMprVec3Cross(&vec, &btlMprSimplexPoint(portal, 2)->v,
                       &btlMprSimplexPoint(portal, 1)->v);
    b[3] = btlMprVec3Dot(&vec, &btlMprSimplexPoint(portal, 0)->v);

	sum = b[0] + b[1] + b[2] + b[3];

    if (btlMprIsZero(sum) || sum < 0.f){
		b[0] = 0.f;

        btlMprVec3Cross(&vec, &btlMprSimplexPoint(portal, 2)->v,
                           &btlMprSimplexPoint(portal, 3)->v);
        b[1] = btlMprVec3Dot(&vec, &dir);
        btlMprVec3Cross(&vec, &btlMprSimplexPoint(portal, 3)->v,
                           &btlMprSimplexPoint(portal, 1)->v);
        b[2] = btlMprVec3Dot(&vec, &dir);
        btlMprVec3Cross(&vec, &btlMprSimplexPoint(portal, 1)->v,
                           &btlMprSimplexPoint(portal, 2)->v);
        b[3] = btlMprVec3Dot(&vec, &dir);

		sum = b[1] + b[2] + b[3];
	}

	inv = 1.f / sum;

    btlMprVec3Copy(&p1, origin);
    btlMprVec3Copy(&p2, origin);
    for (i = 0; i < 4; i++){
        btlMprVec3Copy(&vec, &btlMprSimplexPoint(portal, i)->v1);
        btlMprVec3Scale(&vec, b[i]);
        btlMprVec3Add(&p1, &vec);

        btlMprVec3Copy(&vec, &btlMprSimplexPoint(portal, i)->v2);
        btlMprVec3Scale(&vec, b[i]);
        btlMprVec3Add(&p2, &vec);
    }
    btlMprVec3Scale(&p1, inv);
    btlMprVec3Scale(&p2, inv);
#ifdef MPR_AVERAGE_CONTACT_POSITIONS
    btlMprVec3Copy(pos, &p1);
    btlMprVec3Add(pos, &p2);
    btlMprVec3Scale(pos, 0.5);
#else
    btlMprVec3Copy(pos, &p2);
#endif//MPR_AVERAGE_CONTACT_POSITIONS
}

inline float btlMprVec3Dist2(const btlVector3 *a, const btlVector3 *b)
{
    btlVector3 ab;
    btlMprVec3Sub2(&ab, a, b);
    return btlMprVec3Len2(&ab);
}

inline float _btMprVec3PointSegmentDist2(const btlVector3 *P,
                                                  const btlVector3 *x0,
                                                  const btlVector3 *b,
                                                  btlVector3 *witness)
{
    // The computation comes from solving equation of segment:
    //      S(t) = x0 + t.d
    //          where - x0 is initial point of segment
    //                - d is direction of segment from x0 (|d| > 0)
    //                - t belongs to <0, 1> interval
    // 
    // Than, distance from a segment to some point P can be expressed:
    //      D(t) = |x0 + t.d - P|^2
    //          which is distance from any point on segment. Minimization
    //          of this function brings distance from P to segment.
    // Minimization of D(t) leads to simple quadratic equation that's
    // solving is straightforward.
    //
    // Bonus of this method is witness point for free.

    float dist, t;
    btlVector3 d, a;

    // direction of segment
    btlMprVec3Sub2(&d, b, x0);

    // precompute vector from P to x0
    btlMprVec3Sub2(&a, x0, P);

    t  = -1.f * btlMprVec3Dot(&a, &d);
    t /= btlMprVec3Len2(&d);

    if (t < 0.f || btlMprIsZero(t)){
        dist = btlMprVec3Dist2(x0, P);
        if (witness)
            btlMprVec3Copy(witness, x0);
    }else if (t > 1.f || btlMprEq(t, 1.f)){
        dist = btlMprVec3Dist2(b, P);
        if (witness)
            btlMprVec3Copy(witness, b);
    }else{
        if (witness){
            btlMprVec3Copy(witness, &d);
            btlMprVec3Scale(witness, t);
            btlMprVec3Add(witness, x0);
            dist = btlMprVec3Dist2(witness, P);
        }else{
            // recycling variables
            btlMprVec3Scale(&d, t);
            btlMprVec3Add(&d, &a);
            dist = btlMprVec3Len2(&d);
        }
    }

    return dist;
}



inline float btlMprVec3PointTriDist2(const btlVector3 *P,
                                const btlVector3 *x0, const btlVector3 *B,
                                const btlVector3 *C,
                                btlVector3 *witness)
{
    // Computation comes from analytic expression for triangle (x0, B, C)
    //      T(s, t) = x0 + s.d1 + t.d2, where d1 = B - x0 and d2 = C - x0 and
    // Then equation for distance is:
    //      D(s, t) = | T(s, t) - P |^2
    // This leads to minimization of quadratic function of two variables.
    // The solution from is taken only if s is between 0 and 1, t is
    // between 0 and 1 and t + s < 1, otherwise distance from segment is
    // computed.

    btlVector3 d1, d2, a;
    float u, v, w, p, q, r;
    float s, t, dist, dist2;
    btlVector3 witness2;

    btlMprVec3Sub2(&d1, B, x0);
    btlMprVec3Sub2(&d2, C, x0);
    btlMprVec3Sub2(&a, x0, P);

    u = btlMprVec3Dot(&a, &a);
    v = btlMprVec3Dot(&d1, &d1);
    w = btlMprVec3Dot(&d2, &d2);
    p = btlMprVec3Dot(&a, &d1);
    q = btlMprVec3Dot(&a, &d2);
    r = btlMprVec3Dot(&d1, &d2);

	btlScalar div = (w * v - r * r);
	if (btlMprIsZero(div))
	{
		s=-1;
	} else
	{
		s = (q * r - w * p) / div;
		t = (-s * r - q) / w;
	}

    if ((btlMprIsZero(s) || s > 0.f)
            && (btlMprEq(s, 1.f) || s < 1.f)
            && (btlMprIsZero(t) || t > 0.f)
            && (btlMprEq(t, 1.f) || t < 1.f)
            && (btlMprEq(t + s, 1.f) || t + s < 1.f)){

        if (witness){
            btlMprVec3Scale(&d1, s);
            btlMprVec3Scale(&d2, t);
            btlMprVec3Copy(witness, x0);
            btlMprVec3Add(witness, &d1);
            btlMprVec3Add(witness, &d2);

            dist = btlMprVec3Dist2(witness, P);
        }else{
            dist  = s * s * v;
            dist += t * t * w;
            dist += 2.f * s * t * r;
            dist += 2.f * s * p;
            dist += 2.f * t * q;
            dist += u;
        }
    }else{
        dist = _btMprVec3PointSegmentDist2(P, x0, B, witness);

        dist2 = _btMprVec3PointSegmentDist2(P, x0, C, &witness2);
        if (dist2 < dist){
            dist = dist2;
            if (witness)
                btlMprVec3Copy(witness, &witness2);
        }

        dist2 = _btMprVec3PointSegmentDist2(P, B, C, &witness2);
        if (dist2 < dist){
            dist = dist2;
            if (witness)
                btlMprVec3Copy(witness, &witness2);
        }
    }

    return dist;
}

template <typename btlConvexTemplate>
static void btlFindPenetr(const btlConvexTemplate& a, const btlConvexTemplate& b,
                         const btlMprCollisionDescription& colDesc,
                         btlMprSimplex_t *portal,
                         float *depth, btlVector3 *pdir, btlVector3 *pos)
{
    btlVector3 dir;
    btlMprSupport_t v4;
    unsigned long iterations;

	btlVector3 zero = btlVector3(0,0,0);
	btlVector3* origin = &zero;


    iterations = 1UL;
	for (int i=0;i<BT_MPR_MAX_ITERATIONS;i++)
    //while (1)
	{
        // compute portal direction and obtain next support point
        btlPortalDir(portal, &dir);
        
		 btlMprSupport(a,b,colDesc, dir, &v4);


        // reached tolerance -> find penetration info
        if (portalReachTolerance(portal, &v4, &dir)
                || iterations ==BT_MPR_MAX_ITERATIONS)
		{
            *depth = btlMprVec3PointTriDist2(origin,&btlMprSimplexPoint(portal, 1)->v,&btlMprSimplexPoint(portal, 2)->v,&btlMprSimplexPoint(portal, 3)->v,pdir);
            *depth = BT_MPR_SQRT(*depth);
			
			if (btlMprIsZero((*pdir).x()) && btlMprIsZero((*pdir).y()) && btlMprIsZero((*pdir).z()))
			{
				
				*pdir = dir;
			} 
			btlMprVec3Normalize(pdir);
			
            // barycentric coordinates:
            btlFindPos(portal, pos);


            return;
        }

        btlExpandPortal(portal, &v4);

        iterations++;
    }
}

static void btlFindPenetrTouch(btlMprSimplex_t *portal,float *depth, btlVector3 *dir, btlVector3 *pos)
{
    // Touching contact on portal's v1 - so depth is zero and direction
    // is unimportant and pos can be guessed
    *depth = 0.f;
    btlVector3 zero = btlVector3(0,0,0);
	btlVector3* origin = &zero;


	btlMprVec3Copy(dir, origin);
#ifdef MPR_AVERAGE_CONTACT_POSITIONS
    btlMprVec3Copy(pos, &btlMprSimplexPoint(portal, 1)->v1);
    btlMprVec3Add(pos, &btlMprSimplexPoint(portal, 1)->v2);
    btlMprVec3Scale(pos, 0.5);
#else
     btlMprVec3Copy(pos, &btlMprSimplexPoint(portal, 1)->v2);
#endif
}

static void btlFindPenetrSegment(btlMprSimplex_t *portal,
                              float *depth, btlVector3 *dir, btlVector3 *pos)
{
    
    // Origin lies on v0-v1 segment.
    // Depth is distance to v1, direction also and position must be
    // computed
#ifdef MPR_AVERAGE_CONTACT_POSITIONS
    btlMprVec3Copy(pos, &btlMprSimplexPoint(portal, 1)->v1);
    btlMprVec3Add(pos, &btlMprSimplexPoint(portal, 1)->v2);
    btlMprVec3Scale(pos, 0.5f);
#else
     btlMprVec3Copy(pos, &btlMprSimplexPoint(portal, 1)->v2);
#endif//MPR_AVERAGE_CONTACT_POSITIONS

    btlMprVec3Copy(dir, &btlMprSimplexPoint(portal, 1)->v);
    *depth = BT_MPR_SQRT(btlMprVec3Len2(dir));
    btlMprVec3Normalize(dir);

    
}


template <typename btlConvexTemplate>
inline int btlMprPenetration( const btlConvexTemplate& a, const btlConvexTemplate& b,
                            const btlMprCollisionDescription& colDesc,
					float *depthOut, btlVector3* dirOut, btlVector3* posOut)
{
	
	 btlMprSimplex_t portal;


    // Phase 1: Portal discovery
    int result = btlDiscoverPortal(a,b,colDesc, &portal);
	
	  
	//sepAxis[pairIndex] = *pdir;//or -dir?

	switch (result)
	{
	case 0:
		{
			// Phase 2: Portal refinement
		
			result = btlRefinePortal(a,b,colDesc, &portal);
			if (result < 0)
				return -1;

			// Phase 3. Penetration info
			btlFindPenetr(a,b,colDesc, &portal, depthOut, dirOut, posOut);
			
			
			break;
		}
	case 1:
		{
			 // Touching contact on portal's v1.
			btlFindPenetrTouch(&portal, depthOut, dirOut, posOut);
			result=0;
			break;
		}
	case 2:
		{
			
			btlFindPenetrSegment( &portal, depthOut, dirOut, posOut);
			result=0;
			break;
		}
	default:
		{
			//if (res < 0)
			//{
				// Origin isn't inside portal - no collision.
				result = -1;
			//}
		}
	};
	
	return result;
};


template<typename btlConvexTemplate, typename btlMprDistanceTemplate>
inline int	btlComputeMprPenetration( const btlConvexTemplate& a, const btlConvexTemplate& b, const
                                    btlMprCollisionDescription& colDesc, btlMprDistanceTemplate* distInfo)
{
	btlVector3 dir,pos;
	float depth;

	int res = btlMprPenetration(a,b,colDesc,&depth, &dir, &pos);
	if (res==0)
	{
		distInfo->m_distance = -depth;
		distInfo->m_pointOnB = pos;
		distInfo->m_normalBtoA = -dir;
		distInfo->m_pointOnA = pos-distInfo->m_distance*dir;
		return 0;
	}

	return -1;
}



#endif //BT_MPR_PENETRATION_H
