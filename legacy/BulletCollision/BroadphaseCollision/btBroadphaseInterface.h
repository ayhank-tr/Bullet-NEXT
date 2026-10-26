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

#ifndef		BT_BROADPHASE_INTERFACE_H
#define 	BT_BROADPHASE_INTERFACE_H



struct btlDispatcherInfo;
class btlDispatcher;
#include "btBroadphaseProxy.h"

class btlOverlappingPairCache;



struct	btlBroadphaseAabbCallback
{
	virtual ~btlBroadphaseAabbCallback() {}
	virtual bool	process(const btlBroadphaseProxy* proxy) = 0;
};


struct	btlBroadphaseRayCallback : public btlBroadphaseAabbCallback
{
	///added some cached data to accelerate ray-AABB tests
	btlVector3		m_rayDirectionInverse;
	unsigned int	m_signs[3];
	btlScalar		m_lambda_max;

	virtual ~btlBroadphaseRayCallback() {}
};

#include "LinearMath/btVector3.h"

///The btBroadphaseInterface class provides an interface to detect aabb-overlapping object pairs.
///Some implementations for this broadphase interface include btAxisSweep3, bt32BitAxisSweep3 and btDbvtBroadphase.
///The actual overlapping pair management, storage, adding and removing of pairs is dealt by the btOverlappingPairCache class.
class btlBroadphaseInterface
{
public:
	virtual ~btlBroadphaseInterface() {}

	virtual btlBroadphaseProxy*	createProxy(  const btlVector3& aabbMin,  const btlVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, btlDispatcher* dispatcher,void* multiSapProxy) =0;
	virtual void	destroyProxy(btlBroadphaseProxy* proxy,btlDispatcher* dispatcher)=0;
	virtual void	setAabb(btlBroadphaseProxy* proxy,const btlVector3& aabbMin,const btlVector3& aabbMax, btlDispatcher* dispatcher)=0;
	virtual void	getAabb(btlBroadphaseProxy* proxy,btlVector3& aabbMin, btlVector3& aabbMax ) const =0;

	virtual void	rayTest(const btlVector3& rayFrom,const btlVector3& rayTo, btlBroadphaseRayCallback& rayCallback, const btlVector3& aabbMin=btlVector3(0,0,0), const btlVector3& aabbMax = btlVector3(0,0,0)) = 0;

	virtual void	aabbTest(const btlVector3& aabbMin, const btlVector3& aabbMax, btlBroadphaseAabbCallback& callback) = 0;

	///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
	virtual void	calculateOverlappingPairs(btlDispatcher* dispatcher)=0;

	virtual	btlOverlappingPairCache*	getOverlappingPairCache()=0;
	virtual	const btlOverlappingPairCache*	getOverlappingPairCache() const =0;

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(btlVector3& aabbMin,btlVector3& aabbMax) const =0;

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(btlDispatcher* dispatcher) { (void) dispatcher; };

	virtual void	printStats() = 0;

};

#endif //BT_BROADPHASE_INTERFACE_H
