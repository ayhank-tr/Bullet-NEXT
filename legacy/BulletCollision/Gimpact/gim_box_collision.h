#ifndef btl_GIM_BOX_COLLISION_H_INCLUDED
#define btl_GIM_BOX_COLLISION_H_INCLUDED

/*! \file gim_box_collision.h
\author Francisco Leon Najera
*/
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/
#include "gim_basic_geometry_operations.h"
#include "LinearMath/btTransform.h"



//SIMD_FORCE_INLINE bool test_cross_edge_box(
//	const btVector3 & edge,
//	const btVector3 & absolute_edge,
//	const btVector3 & pointa,
//	const btVector3 & pointb, const btVector3 & extend,
//	int dir_index0,
//	int dir_index1
//	int component_index0,
//	int component_index1)
//{
//	// dir coords are -z and y
//
//	const btScalar dir0 = -edge[dir_index0];
//	const btScalar dir1 = edge[dir_index1];
//	btScalar pmin = pointa[component_index0]*dir0 + pointa[component_index1]*dir1;
//	btScalar pmax = pointb[component_index0]*dir0 + pointb[component_index1]*dir1;
//	//find minmax
//	if(pmin>pmax)
//	{
//		GIM_SWAP_NUMBERS(pmin,pmax);
//	}
//	//find extends
//	const btScalar rad = extend[component_index0] * absolute_edge[dir_index0] +
//					extend[component_index1] * absolute_edge[dir_index1];
//
//	if(pmin>rad || -rad>pmax) return false;
//	return true;
//}
//
//SIMD_FORCE_INLINE bool test_cross_edge_box_X_axis(
//	const btVector3 & edge,
//	const btVector3 & absolute_edge,
//	const btVector3 & pointa,
//	const btVector3 & pointb, btVector3 & extend)
//{
//
//	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,2,1,1,2);
//}
//
//
//SIMD_FORCE_INLINE bool test_cross_edge_box_Y_axis(
//	const btVector3 & edge,
//	const btVector3 & absolute_edge,
//	const btVector3 & pointa,
//	const btVector3 & pointb, btVector3 & extend)
//{
//
//	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,0,2,2,0);
//}
//
//SIMD_FORCE_INLINE bool test_cross_edge_box_Z_axis(
//	const btVector3 & edge,
//	const btVector3 & absolute_edge,
//	const btVector3 & pointa,
//	const btVector3 & pointb, btVector3 & extend)
//{
//
//	return test_cross_edge_box(edge,absolute_edge,pointa,pointb,extend,1,0,0,1);
//}

#define TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,i_dir_0,i_dir_1,i_comp_0,i_comp_1)\
{\
	const btlScalar dir0 = -edge[i_dir_0];\
	const btlScalar dir1 = edge[i_dir_1];\
	btlScalar pmin = pointa[i_comp_0]*dir0 + pointa[i_comp_1]*dir1;\
	btlScalar pmax = pointb[i_comp_0]*dir0 + pointb[i_comp_1]*dir1;\
	if(pmin>pmax)\
	{\
		btl_GIM_SWAP_NUMBERS(pmin,pmax); \
	}\
	const btlScalar abs_dir0 = absolute_edge[i_dir_0];\
	const btlScalar abs_dir1 = absolute_edge[i_dir_1];\
	const btlScalar rad = _extend[i_comp_0] * abs_dir0 + _extend[i_comp_1] * abs_dir1;\
	if(pmin>rad || -rad>pmax) return false;\
}\


#define TEST_CROSS_EDGE_BOX_X_AXIS_MCR(edge,absolute_edge,pointa,pointb,_extend)\
{\
	TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,2,1,1,2);\
}\

#define TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(edge,absolute_edge,pointa,pointb,_extend)\
{\
	TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,0,2,2,0);\
}\

#define TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(edge,absolute_edge,pointa,pointb,_extend)\
{\
	TEST_CROSS_EDGE_BOX_MCR(edge,absolute_edge,pointa,pointb,_extend,1,0,0,1);\
}\



//!  Class for transforming a model1 to the space of model0
class btl_GIM_BOX_BOX_TRANSFORM_CACHE
{
public:
    btlVector3  m_T1to0;//!< Transforms translation of model1 to model 0
	btlMatrix3x3 m_R1to0;//!< Transforms Rotation of model1 to model 0, equal  to R0' * R1
	btlMatrix3x3 m_AR;//!< Absolute value of m_R1to0

	SIMD_FORCE_INLINE void calc_absolute_matrix()
	{
		static const btlVector3 vepsi(1e-6f,1e-6f,1e-6f);
		m_AR[0] = vepsi + m_R1to0[0].absolute();
		m_AR[1] = vepsi + m_R1to0[1].absolute();
		m_AR[2] = vepsi + m_R1to0[2].absolute();
	}

	btl_GIM_BOX_BOX_TRANSFORM_CACHE()
	{
	}


	btl_GIM_BOX_BOX_TRANSFORM_CACHE(mat4f  trans1_to_0)
	{
		COPY_MATRIX_3X3(m_R1to0,trans1_to_0)
        MAT_GET_TRANSLATION(trans1_to_0,m_T1to0)
		calc_absolute_matrix();
	}

	//! Calc the transformation relative  1 to 0. Inverts matrics by transposing
	SIMD_FORCE_INLINE void calc_from_homogenic(const btlTransform & trans0,const btlTransform & trans1)
	{

		m_R1to0 = trans0.getBasis().transpose();
		m_T1to0 = m_R1to0 * (-trans0.getOrigin());

		m_T1to0 += m_R1to0*trans1.getOrigin();
		m_R1to0 *= trans1.getBasis();

		calc_absolute_matrix();
	}

	//! Calcs the full invertion of the matrices. Useful for scaling matrices
	SIMD_FORCE_INLINE void calc_from_full_invert(const btlTransform & trans0,const btlTransform & trans1)
	{
		m_R1to0 = trans0.getBasis().inverse();
		m_T1to0 = m_R1to0 * (-trans0.getOrigin());

		m_T1to0 += m_R1to0*trans1.getOrigin();
		m_R1to0 *= trans1.getBasis();

		calc_absolute_matrix();
	}

	SIMD_FORCE_INLINE btlVector3 transform(const btlVector3 & point)
	{
        return point.dot3(m_R1to0[0], m_R1to0[1], m_R1to0[2]) + m_T1to0;
	}
};


#define BOX_PLANE_EPSILON 0.000001f

//! Axis aligned box
class btl_GIM_AABB
{
public:
	btlVector3 m_min;
	btlVector3 m_max;

	btl_GIM_AABB()
	{}


	btl_GIM_AABB(const btlVector3 & V1,
			 const btlVector3 & V2,
			 const btlVector3 & V3)
	{
		m_min[0] = btl_GIM_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = btl_GIM_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = btl_GIM_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = btl_GIM_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = btl_GIM_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = btl_GIM_MAX3(V1[2],V2[2],V3[2]);
	}

	btl_GIM_AABB(const btlVector3 & V1,
			 const btlVector3 & V2,
			 const btlVector3 & V3,
			 GREAL margin)
	{
		m_min[0] = btl_GIM_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = btl_GIM_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = btl_GIM_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = btl_GIM_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = btl_GIM_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = btl_GIM_MAX3(V1[2],V2[2],V3[2]);

		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	btl_GIM_AABB(const btl_GIM_AABB &other):
		m_min(other.m_min),m_max(other.m_max)
	{
	}

	btl_GIM_AABB(const btl_GIM_AABB &other,btlScalar margin ):
		m_min(other.m_min),m_max(other.m_max)
	{
		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	SIMD_FORCE_INLINE void invalidate()
	{
		m_min[0] = G_REAL_INFINITY;
		m_min[1] = G_REAL_INFINITY;
		m_min[2] = G_REAL_INFINITY;
		m_max[0] = -G_REAL_INFINITY;
		m_max[1] = -G_REAL_INFINITY;
		m_max[2] = -G_REAL_INFINITY;
	}

	SIMD_FORCE_INLINE void increment_margin(btlScalar margin)
	{
		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	SIMD_FORCE_INLINE void copy_with_margin(const btl_GIM_AABB &other, btlScalar margin)
	{
		m_min[0] = other.m_min[0] - margin;
		m_min[1] = other.m_min[1] - margin;
		m_min[2] = other.m_min[2] - margin;

		m_max[0] = other.m_max[0] + margin;
		m_max[1] = other.m_max[1] + margin;
		m_max[2] = other.m_max[2] + margin;
	}

	template<typename CLASS_POINT>
	SIMD_FORCE_INLINE void calc_from_triangle(
							const CLASS_POINT & V1,
							const CLASS_POINT & V2,
							const CLASS_POINT & V3)
	{
		m_min[0] = btl_GIM_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = btl_GIM_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = btl_GIM_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = btl_GIM_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = btl_GIM_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = btl_GIM_MAX3(V1[2],V2[2],V3[2]);
	}

	template<typename CLASS_POINT>
	SIMD_FORCE_INLINE void calc_from_triangle_margin(
							const CLASS_POINT & V1,
							const CLASS_POINT & V2,
							const CLASS_POINT & V3, btlScalar margin)
	{
		m_min[0] = btl_GIM_MIN3(V1[0],V2[0],V3[0]);
		m_min[1] = btl_GIM_MIN3(V1[1],V2[1],V3[1]);
		m_min[2] = btl_GIM_MIN3(V1[2],V2[2],V3[2]);

		m_max[0] = btl_GIM_MAX3(V1[0],V2[0],V3[0]);
		m_max[1] = btl_GIM_MAX3(V1[1],V2[1],V3[1]);
		m_max[2] = btl_GIM_MAX3(V1[2],V2[2],V3[2]);

		m_min[0] -= margin;
		m_min[1] -= margin;
		m_min[2] -= margin;
		m_max[0] += margin;
		m_max[1] += margin;
		m_max[2] += margin;
	}

	//! Apply a transform to an AABB
	SIMD_FORCE_INLINE void appy_transform(const btlTransform & trans)
	{
		btlVector3 center = (m_max+m_min)*0.5f;
		btlVector3 extends = m_max - center;
		// Compute new center
		center = trans(center);

        btlVector3 textends = extends.dot3(trans.getBasis().getRow(0).absolute(), 
                                          trans.getBasis().getRow(1).absolute(), 
                                          trans.getBasis().getRow(2).absolute());
        
		m_min = center - textends;
		m_max = center + textends;
	}

	//! Merges a Box
	SIMD_FORCE_INLINE void merge(const btl_GIM_AABB & box)
	{
		m_min[0] = btl_GIM_MIN(m_min[0],box.m_min[0]);
		m_min[1] = btl_GIM_MIN(m_min[1],box.m_min[1]);
		m_min[2] = btl_GIM_MIN(m_min[2],box.m_min[2]);

		m_max[0] = btl_GIM_MAX(m_max[0],box.m_max[0]);
		m_max[1] = btl_GIM_MAX(m_max[1],box.m_max[1]);
		m_max[2] = btl_GIM_MAX(m_max[2],box.m_max[2]);
	}

	//! Merges a point
	template<typename CLASS_POINT>
	SIMD_FORCE_INLINE void merge_point(const CLASS_POINT & point)
	{
		m_min[0] = btl_GIM_MIN(m_min[0],point[0]);
		m_min[1] = btl_GIM_MIN(m_min[1],point[1]);
		m_min[2] = btl_GIM_MIN(m_min[2],point[2]);

		m_max[0] = btl_GIM_MAX(m_max[0],point[0]);
		m_max[1] = btl_GIM_MAX(m_max[1],point[1]);
		m_max[2] = btl_GIM_MAX(m_max[2],point[2]);
	}

	//! Gets the extend and center
	SIMD_FORCE_INLINE void get_center_extend(btlVector3 & center,btlVector3 & extend)  const
	{
		center = (m_max+m_min)*0.5f;
		extend = m_max - center;
	}

	//! Finds the intersecting box between this box and the other.
	SIMD_FORCE_INLINE void find_intersection(const btl_GIM_AABB & other, btl_GIM_AABB & intersection)  const
	{
		intersection.m_min[0] = btl_GIM_MAX(other.m_min[0],m_min[0]);
		intersection.m_min[1] = btl_GIM_MAX(other.m_min[1],m_min[1]);
		intersection.m_min[2] = btl_GIM_MAX(other.m_min[2],m_min[2]);

		intersection.m_max[0] = btl_GIM_MIN(other.m_max[0],m_max[0]);
		intersection.m_max[1] = btl_GIM_MIN(other.m_max[1],m_max[1]);
		intersection.m_max[2] = btl_GIM_MIN(other.m_max[2],m_max[2]);
	}


	SIMD_FORCE_INLINE bool has_collision(const btl_GIM_AABB & other) const
	{
		if(m_min[0] > other.m_max[0] ||
		   m_max[0] < other.m_min[0] ||
		   m_min[1] > other.m_max[1] ||
		   m_max[1] < other.m_min[1] ||
		   m_min[2] > other.m_max[2] ||
		   m_max[2] < other.m_min[2])
		{
			return false;
		}
		return true;
	}

	/*! \brief Finds the Ray intersection parameter.
	\param aabb Aligned box
	\param vorigin A vec3f with the origin of the ray
	\param vdir A vec3f with the direction of the ray
	*/
	SIMD_FORCE_INLINE bool collide_ray(const btlVector3 & vorigin,const btlVector3 & vdir)
	{
		btlVector3 extents,center;
		this->get_center_extend(center,extents);;

		btlScalar Dx = vorigin[0] - center[0];
		if(btl_GIM_GREATER(Dx, extents[0]) && Dx*vdir[0]>=0.0f)	return false;
		btlScalar Dy = vorigin[1] - center[1];
		if(btl_GIM_GREATER(Dy, extents[1]) && Dy*vdir[1]>=0.0f)	return false;
		btlScalar Dz = vorigin[2] - center[2];
		if(btl_GIM_GREATER(Dz, extents[2]) && Dz*vdir[2]>=0.0f)	return false;


		btlScalar f = vdir[1] * Dz - vdir[2] * Dy;
		if(btlFabs(f) > extents[1]*btlFabs(vdir[2]) + extents[2]*btlFabs(vdir[1])) return false;
		f = vdir[2] * Dx - vdir[0] * Dz;
		if(btlFabs(f) > extents[0]*btlFabs(vdir[2]) + extents[2]*btlFabs(vdir[0]))return false;
		f = vdir[0] * Dy - vdir[1] * Dx;
		if(btlFabs(f) > extents[0]*btlFabs(vdir[1]) + extents[1]*btlFabs(vdir[0]))return false;
		return true;
	}


	SIMD_FORCE_INLINE void projection_interval(const btlVector3 & direction, btlScalar &vmin, btlScalar &vmax) const
	{
		btlVector3 center = (m_max+m_min)*0.5f;
		btlVector3 extend = m_max-center;

		btlScalar _fOrigin =  direction.dot(center);
		btlScalar _fMaximumExtent = extend.dot(direction.absolute());
		vmin = _fOrigin - _fMaximumExtent;
		vmax = _fOrigin + _fMaximumExtent;
	}

	SIMD_FORCE_INLINE ePLANE_INTERSECTION_TYPE plane_classify(const btlVector4 &plane) const
	{
		btlScalar _fmin,_fmax;
		this->projection_interval(plane,_fmin,_fmax);

		if(plane[3] > _fmax + BOX_PLANE_EPSILON)
		{
			return G_BACK_PLANE; // 0
		}

		if(plane[3]+BOX_PLANE_EPSILON >=_fmin)
		{
			return G_COLLIDE_PLANE; //1
		}
		return G_FRONT_PLANE;//2
	}

	SIMD_FORCE_INLINE bool overlapping_trans_conservative(const btl_GIM_AABB & box, btlTransform & trans1_to_0)
	{
		btl_GIM_AABB tbox = box;
		tbox.appy_transform(trans1_to_0);
		return has_collision(tbox);
	}

	//! transcache is the transformation cache from box to this AABB
	SIMD_FORCE_INLINE bool overlapping_trans_cache(
		const btl_GIM_AABB & box,const btl_GIM_BOX_BOX_TRANSFORM_CACHE & transcache, bool fulltest)
	{

		//Taken from OPCODE
		btlVector3 ea,eb;//extends
		btlVector3 ca,cb;//extends
		get_center_extend(ca,ea);
		box.get_center_extend(cb,eb);


		btlVector3 T;
		btlScalar t,t2;
		int i;

		// Class I : A's basis vectors
		for(i=0;i<3;i++)
		{
			T[i] =  transcache.m_R1to0[i].dot(cb) + transcache.m_T1to0[i] - ca[i];
			t = transcache.m_AR[i].dot(eb) + ea[i];
			if(btl_GIM_GREATER(T[i], t))	return false;
		}
		// Class II : B's basis vectors
		for(i=0;i<3;i++)
		{
			t = MAT_DOT_COL(transcache.m_R1to0,T,i);
			t2 = MAT_DOT_COL(transcache.m_AR,ea,i) + eb[i];
			if(btl_GIM_GREATER(t,t2))	return false;
		}
		// Class III : 9 cross products
		if(fulltest)
		{
			int j,m,n,o,p,q,r;
			for(i=0;i<3;i++)
			{
				m = (i+1)%3;
				n = (i+2)%3;
				o = i==0?1:0;
				p = i==2?1:2;
				for(j=0;j<3;j++)
				{
					q = j==2?1:2;
					r = j==0?1:0;
					t = T[n]*transcache.m_R1to0[m][j] - T[m]*transcache.m_R1to0[n][j];
					t2 = ea[o]*transcache.m_AR[p][j] + ea[p]*transcache.m_AR[o][j] +
						eb[r]*transcache.m_AR[i][q] + eb[q]*transcache.m_AR[i][r];
					if(btl_GIM_GREATER(t,t2))	return false;
				}
			}
		}
		return true;
	}

	//! Simple test for planes.
	SIMD_FORCE_INLINE bool collide_plane(
		const btlVector4 & plane)
	{
		ePLANE_INTERSECTION_TYPE classify = plane_classify(plane);
		return (classify == G_COLLIDE_PLANE);
	}

	//! test for a triangle, with edges
	SIMD_FORCE_INLINE bool collide_triangle_exact(
		const btlVector3 & p1,
		const btlVector3 & p2,
		const btlVector3 & p3,
		const btlVector4 & triangle_plane)
	{
		if(!collide_plane(triangle_plane)) return false;

		btlVector3 center,extends;
		this->get_center_extend(center,extends);

		const btlVector3 v1(p1 - center);
		const btlVector3 v2(p2 - center);
		const btlVector3 v3(p3 - center);

		//First axis
		btlVector3 diff(v2 - v1);
		btlVector3 abs_diff = diff.absolute();
		//Test With X axis
		TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff,abs_diff,v1,v3,extends);
		//Test With Y axis
		TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff,abs_diff,v1,v3,extends);
		//Test With Z axis
		TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff,abs_diff,v1,v3,extends);


		diff = v3 - v2;
		abs_diff = diff.absolute();
		//Test With X axis
		TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff,abs_diff,v2,v1,extends);
		//Test With Y axis
		TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff,abs_diff,v2,v1,extends);
		//Test With Z axis
		TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff,abs_diff,v2,v1,extends);

		diff = v1 - v3;
		abs_diff = diff.absolute();
		//Test With X axis
		TEST_CROSS_EDGE_BOX_X_AXIS_MCR(diff,abs_diff,v3,v2,extends);
		//Test With Y axis
		TEST_CROSS_EDGE_BOX_Y_AXIS_MCR(diff,abs_diff,v3,v2,extends);
		//Test With Z axis
		TEST_CROSS_EDGE_BOX_Z_AXIS_MCR(diff,abs_diff,v3,v2,extends);

		return true;
	}
};


//! Compairison of transformation objects
SIMD_FORCE_INLINE bool btlCompareTransformsEqual(const btlTransform & t1,const btlTransform & t2)
{
	if(!(t1.getOrigin() == t2.getOrigin()) ) return false;

	if(!(t1.getBasis().getRow(0) == t2.getBasis().getRow(0)) ) return false;
	if(!(t1.getBasis().getRow(1) == t2.getBasis().getRow(1)) ) return false;
	if(!(t1.getBasis().getRow(2) == t2.getBasis().getRow(2)) ) return false;
	return true;
}



#endif // GIM_BOX_COLLISION_H_INCLUDED
