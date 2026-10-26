/*! \file btGImpactShape.h
\author Francisco Leon Najera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef GIMPACT_TRIANGLE_SHAPE_EX_H
#define GIMPACT_TRIANGLE_SHAPE_EX_H

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "btBoxCollision.h"
#include "btClipPolygon.h"
#include "btGeometryOperations.h"


#define MAX_TRI_CLIPPING 16

//! Structure for collision
struct btl_GIM_TRIANGLE_CONTACT
{
    btlScalar m_penetration_depth;
    int m_point_count;
    btlVector4 m_separating_normal;
    btlVector3 m_points[MAX_TRI_CLIPPING];

	SIMD_FORCE_INLINE void copy_from(const btl_GIM_TRIANGLE_CONTACT& other)
	{
		m_penetration_depth = other.m_penetration_depth;
		m_separating_normal = other.m_separating_normal;
		m_point_count = other.m_point_count;
		int i = m_point_count;
		while(i--)
		{
			m_points[i] = other.m_points[i];
		}
	}

	btl_GIM_TRIANGLE_CONTACT()
	{
	}

	btl_GIM_TRIANGLE_CONTACT(const btl_GIM_TRIANGLE_CONTACT& other)
	{
		copy_from(other);
	}

    //! classify points that are closer
    void merge_points(const btlVector4 & plane,
    				btlScalar margin, const btlVector3 * points, int point_count);

};



class btlPrimitiveTriangle
{
public:
	btlVector3 m_vertices[3];
	btlVector4 m_plane;
	btlScalar m_margin;
	btlScalar m_dummy;
	btlPrimitiveTriangle():m_margin(0.01f)
	{

	}


	SIMD_FORCE_INLINE void buildTriPlane()
	{
		btlVector3 normal = (m_vertices[1]-m_vertices[0]).cross(m_vertices[2]-m_vertices[0]);
		normal.normalize();
		m_plane.setValue(normal[0],normal[1],normal[2],m_vertices[0].dot(normal));
	}

	//! Test if triangles could collide
	bool overlap_test_conservative(const btlPrimitiveTriangle& other);

	//! Calcs the plane which is paralele to the edge and perpendicular to the triangle plane
	/*!
	\pre this triangle must have its plane calculated.
	*/
	SIMD_FORCE_INLINE void get_edge_plane(int edge_index, btlVector4 &plane)  const
    {
		const btlVector3 & e0 = m_vertices[edge_index];
		const btlVector3 & e1 = m_vertices[(edge_index+1)%3];
		btl_edge_plane(e0,e1,m_plane,plane);
    }

    void applyTransform(const btlTransform& t)
	{
		m_vertices[0] = t(m_vertices[0]);
		m_vertices[1] = t(m_vertices[1]);
		m_vertices[2] = t(m_vertices[2]);
	}

	//! Clips the triangle against this
	/*!
	\pre clipped_points must have MAX_TRI_CLIPPING size, and this triangle must have its plane calculated.
	\return the number of clipped points
	*/
    int clip_triangle(btlPrimitiveTriangle & other, btlVector3 * clipped_points );

	//! Find collision using the clipping method
	/*!
	\pre this triangle and other must have their triangles calculated
	*/
    bool find_triangle_collision_clip_method(btlPrimitiveTriangle & other, btl_GIM_TRIANGLE_CONTACT & contacts);
};



//! Helper class for colliding Bullet Triangle Shapes
/*!
This class implements a better getAabb method than the previous btTriangleShape class
*/
class btlTriangleShapeEx: public btlTriangleShape
{
public:

	btlTriangleShapeEx():btlTriangleShape(btlVector3(0,0,0),btlVector3(0,0,0),btlVector3(0,0,0))
	{
	}

	btlTriangleShapeEx(const btlVector3& p0,const btlVector3& p1,const btlVector3& p2):	btlTriangleShape(p0,p1,p2)
	{
	}

	btlTriangleShapeEx(const btlTriangleShapeEx & other):	btlTriangleShape(other.m_vertices1[0],other.m_vertices1[1],other.m_vertices1[2])
	{
	}

	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax)const
	{
		btlVector3 tv0 = t(m_vertices1[0]);
		btlVector3 tv1 = t(m_vertices1[1]);
		btlVector3 tv2 = t(m_vertices1[2]);

		btlAABB trianglebox(tv0,tv1,tv2,m_collisionMargin);
		aabbMin = trianglebox.m_min;
		aabbMax = trianglebox.m_max;
	}

	void applyTransform(const btlTransform& t)
	{
		m_vertices1[0] = t(m_vertices1[0]);
		m_vertices1[1] = t(m_vertices1[1]);
		m_vertices1[2] = t(m_vertices1[2]);
	}

	SIMD_FORCE_INLINE void buildTriPlane(btlVector4 & plane) const
	{
		btlVector3 normal = (m_vertices1[1]-m_vertices1[0]).cross(m_vertices1[2]-m_vertices1[0]);
		normal.normalize();
		plane.setValue(normal[0],normal[1],normal[2],m_vertices1[0].dot(normal));
	}

	bool overlap_test_conservative(const btlTriangleShapeEx& other);
};


#endif //GIMPACT_TRIANGLE_MESH_SHAPE_H
