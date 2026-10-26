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

#include "LinearMath/btScalar.h"
#include "SphereTriangleDetector.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"


btl_SphereTriangleDetector::btl_SphereTriangleDetector(btlSphereShape* sphere,btlTriangleShape* triangle,btlScalar contactBreakingThreshold)
:m_sphere(sphere),
m_triangle(triangle),
m_contactBreakingThreshold(contactBreakingThreshold)
{

}

void	btl_SphereTriangleDetector::getClosestPoints(const ClosestPointInput& input,Result& output,class btlIDebugDraw* debugDraw,bool swapResults)
{

	(void)debugDraw;
	const btlTransform& transformA = input.m_transformA;
	const btlTransform& transformB = input.m_transformB;

	btlVector3 point,normal;
	btlScalar timeOfImpact = btlScalar(1.);
	btlScalar depth = btlScalar(0.);
//	output.m_distance = btScalar(BT_LARGE_FLOAT);
	//move sphere into triangle space
	btlTransform	sphereInTr = transformB.inverseTimes(transformA);

	if (collide(sphereInTr.getOrigin(),point,normal,depth,timeOfImpact,m_contactBreakingThreshold))
	{
		if (swapResults)
		{
			btlVector3 normalOnB = transformB.getBasis()*normal;
			btlVector3 normalOnA = -normalOnB;
			btlVector3 pointOnA = transformB*point+normalOnB*depth;
			output.addContactPoint(normalOnA,pointOnA,depth);
		} else
		{
			output.addContactPoint(transformB.getBasis()*normal,transformB*point,depth);
		}
	}

}



// See also geometrictools.com
// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
btlScalar btl_SegmentSqrDistance(const btlVector3& from, const btlVector3& to,const btlVector3 &p, btlVector3 &nearest);

btlScalar btl_SegmentSqrDistance(const btlVector3& from, const btlVector3& to,const btlVector3 &p, btlVector3 &nearest) {
	btlVector3 diff = p - from;
	btlVector3 v = to - from;
	btlScalar t = v.dot(diff);
	
	if (t > 0) {
		btlScalar dotVV = v.dot(v);
		if (t < dotVV) {
			t /= dotVV;
			diff -= t*v;
		} else {
			t = 1;
			diff -= v;
		}
	} else
		t = 0;

	nearest = from + t*v;
	return diff.dot(diff);	
}

bool btl_SphereTriangleDetector::facecontains(const btlVector3 &p,const btlVector3* vertices,btlVector3& normal)  {
	btlVector3 lp(p);
	btlVector3 lnormal(normal);
	
	return pointInTriangle(vertices, lnormal, &lp);
}

bool btl_SphereTriangleDetector::collide(const btlVector3& sphereCenter,btlVector3 &point, btlVector3& resultNormal, btlScalar& depth, btlScalar &timeOfImpact, btlScalar contactBreakingThreshold)
{

	const btlVector3* vertices = &m_triangle->getVertexPtr(0);
	
	btlScalar radius = m_sphere->getRadius();
	btlScalar radiusWithThreshold = radius + contactBreakingThreshold;

	btlVector3 normal = (vertices[1]-vertices[0]).cross(vertices[2]-vertices[0]);
	normal.normalize();
	btlVector3 p1ToCentre = sphereCenter - vertices[0];
	btlScalar distanceFromPlane = p1ToCentre.dot(normal);

	if (distanceFromPlane < btlScalar(0.))
	{
		//triangle facing the other way
		distanceFromPlane *= btlScalar(-1.);
		normal *= btlScalar(-1.);
	}

	bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;
	
	// Check for contact / intersection
	bool hasContact = false;
	btlVector3 contactPoint;
	if (isInsideContactPlane) {
		if (facecontains(sphereCenter,vertices,normal)) {
			// Inside the contact wedge - touches a point on the shell plane
			hasContact = true;
			contactPoint = sphereCenter - normal*distanceFromPlane;
		} else {
			// Could be inside one of the contact capsules
			btlScalar contactCapsuleRadiusSqr = radiusWithThreshold*radiusWithThreshold;
			btlVector3 nearestOnEdge;
			for (int i = 0; i < m_triangle->getNumEdges(); i++) {
				
				btlVector3 pa;
				btlVector3 pb;
				
				m_triangle->getEdge(i,pa,pb);

				btlScalar distanceSqr = btl_SegmentSqrDistance(pa,pb,sphereCenter, nearestOnEdge);
				if (distanceSqr < contactCapsuleRadiusSqr) {
					// Yep, we're inside a capsule
					hasContact = true;
					contactPoint = nearestOnEdge;
				}
				
			}
		}
	}

	if (hasContact) {
		btlVector3 contactToCentre = sphereCenter - contactPoint;
		btlScalar distanceSqr = contactToCentre.length2();

		if (distanceSqr < radiusWithThreshold*radiusWithThreshold)
		{
			if (distanceSqr>SIMD_EPSILON)
			{
				btlScalar distance = btlSqrt(distanceSqr);
				resultNormal = contactToCentre;
				resultNormal.normalize();
				point = contactPoint;
				depth = -(radius-distance);
			} else
			{
				resultNormal = normal;
				point = contactPoint;
				depth = -radius;
			}
			return true;
		}
	}
	
	return false;
}


bool btl_SphereTriangleDetector::pointInTriangle(const btlVector3 vertices[], const btlVector3 &normal, btlVector3 *p )
{
	const btlVector3* p1 = &vertices[0];
	const btlVector3* p2 = &vertices[1];
	const btlVector3* p3 = &vertices[2];

	btlVector3 edge1( *p2 - *p1 );
	btlVector3 edge2( *p3 - *p2 );
	btlVector3 edge3( *p1 - *p3 );

	btlVector3 p1_to_p( *p - *p1 );
	btlVector3 p2_to_p( *p - *p2 );
	btlVector3 p3_to_p( *p - *p3 );

	btlVector3 edge1_normal( edge1.cross(normal));
	btlVector3 edge2_normal( edge2.cross(normal));
	btlVector3 edge3_normal( edge3.cross(normal));
	
	btlScalar r1, r2, r3;
	r1 = edge1_normal.dot( p1_to_p );
	r2 = edge2_normal.dot( p2_to_p );
	r3 = edge3_normal.dot( p3_to_p );
	if ( ( r1 > 0 && r2 > 0 && r3 > 0 ) ||
	     ( r1 <= 0 && r2 <= 0 && r3 <= 0 ) )
		return true;
	return false;

}
