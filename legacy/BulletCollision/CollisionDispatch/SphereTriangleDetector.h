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

#ifndef BT_SPHERE_TRIANGLE_DETECTOR_H
#define BT_SPHERE_TRIANGLE_DETECTOR_H

#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"



class btlSphereShape;
class btlTriangleShape;



/// sphere-triangle to match the btDiscreteCollisionDetectorInterface
struct btl_SphereTriangleDetector : public btlDiscreteCollisionDetectorInterface
{
	virtual void	getClosestPoints(const ClosestPointInput& input,Result& output,class btlIDebugDraw* debugDraw,bool swapResults=false);

	btl_SphereTriangleDetector(btlSphereShape* sphere,btlTriangleShape* triangle, btlScalar contactBreakingThreshold);

	virtual ~btl_SphereTriangleDetector() {};

	bool collide(const btlVector3& sphereCenter,btlVector3 &point, btlVector3& resultNormal, btlScalar& depth, btlScalar &timeOfImpact, btlScalar	contactBreakingThreshold);

private:

	
	bool pointInTriangle(const btlVector3 vertices[], const btlVector3 &normal, btlVector3 *p );
	bool facecontains(const btlVector3 &p,const btlVector3* vertices,btlVector3& normal);

	btlSphereShape* m_sphere;
	btlTriangleShape* m_triangle;
	btlScalar	m_contactBreakingThreshold;
	
};
#endif //BT_SPHERE_TRIANGLE_DETECTOR_H

