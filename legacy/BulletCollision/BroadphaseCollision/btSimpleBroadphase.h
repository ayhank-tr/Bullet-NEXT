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

#ifndef BT_SIMPLE_BROADPHASE_H
#define BT_SIMPLE_BROADPHASE_H


#include "btOverlappingPairCache.h"


struct btlSimpleBroadphaseProxy : public btlBroadphaseProxy
{
	int			m_nextFree;
	
//	int			m_handleId;

	
	btlSimpleBroadphaseProxy() {};

	btlSimpleBroadphaseProxy(const btlVector3& minpt,const btlVector3& maxpt,int shapeType,void* userPtr,short int collisionFilterGroup,short int collisionFilterMask,void* multiSapProxy)
	:btlBroadphaseProxy(minpt,maxpt,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy)
	{
		(void)shapeType;
	}
	
	
	SIMD_FORCE_INLINE void SetNextFree(int next) {m_nextFree = next;}
	SIMD_FORCE_INLINE int GetNextFree() const {return m_nextFree;}

	


};

///The SimpleBroadphase is just a unit-test for btAxisSweep3, bt32BitAxisSweep3, or btDbvtBroadphase, so use those classes instead.
///It is a brute force aabb culling broadphase based on O(n^2) aabb checks
class btlSimpleBroadphase : public btlBroadphaseInterface
{

protected:

	int		m_numHandles;						// number of active handles
	int		m_maxHandles;						// max number of handles
	int		m_LastHandleIndex;							
	
	btlSimpleBroadphaseProxy* m_pHandles;						// handles pool

	void* m_pHandlesRawPtr;
	int		m_firstFreeHandle;		// free handles list
	
	int allocHandle()
	{
		btlAssert(m_numHandles < m_maxHandles);
		int freeHandle = m_firstFreeHandle;
		m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
		m_numHandles++;
		if(freeHandle > m_LastHandleIndex)
		{
			m_LastHandleIndex = freeHandle;
		}
		return freeHandle;
	}

	void freeHandle(btlSimpleBroadphaseProxy* proxy)
	{
		int handle = int(proxy-m_pHandles);
		btlAssert(handle >= 0 && handle < m_maxHandles);
		if(handle == m_LastHandleIndex)
		{
			m_LastHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		proxy->m_clientObject = 0;

		m_numHandles--;
	}

	btlOverlappingPairCache*	m_pairCache;
	bool	m_ownsPairCache;

	int	m_invalidPair;

	
	
	inline btlSimpleBroadphaseProxy*	getSimpleProxyFromProxy(btlBroadphaseProxy* proxy)
	{
		btlSimpleBroadphaseProxy* proxy0 = static_cast<btlSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	inline const btlSimpleBroadphaseProxy*	getSimpleProxyFromProxy(btlBroadphaseProxy* proxy) const
	{
		const btlSimpleBroadphaseProxy* proxy0 = static_cast<const btlSimpleBroadphaseProxy*>(proxy);
		return proxy0;
	}

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(btlDispatcher* dispatcher);


	void	validate();

protected:


	

public:
	btlSimpleBroadphase(int maxProxies=16384,btlOverlappingPairCache* overlappingPairCache=0);
	virtual ~btlSimpleBroadphase();


		static bool	aabbOverlap(btlSimpleBroadphaseProxy* proxy0,btlSimpleBroadphaseProxy* proxy1);


	virtual btlBroadphaseProxy*	createProxy(  const btlVector3& aabbMin,  const btlVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btlDispatcher* dispatcher,void* multiSapProxy);

	virtual void	calculateOverlappingPairs(btlDispatcher* dispatcher);

	virtual void	destroyProxy(btlBroadphaseProxy* proxy,btlDispatcher* dispatcher);
	virtual void	setAabb(btlBroadphaseProxy* proxy,const btlVector3& aabbMin,const btlVector3& aabbMax, btlDispatcher* dispatcher);
	virtual void	getAabb(btlBroadphaseProxy* proxy,btlVector3& aabbMin, btlVector3& aabbMax ) const;

	virtual void	rayTest(const btlVector3& rayFrom,const btlVector3& rayTo, btlBroadphaseRayCallback& rayCallback, const btlVector3& aabbMin=btlVector3(0,0,0),const btlVector3& aabbMax=btlVector3(0,0,0));
	virtual void	aabbTest(const btlVector3& aabbMin, const btlVector3& aabbMax, btlBroadphaseAabbCallback& callback);
		
	btlOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_pairCache;
	}
	const btlOverlappingPairCache*	getOverlappingPairCache() const
	{
		return m_pairCache;
	}

	bool	testAabbOverlap(btlBroadphaseProxy* proxy0,btlBroadphaseProxy* proxy1);


	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(btlVector3& aabbMin,btlVector3& aabbMax) const
	{
		aabbMin.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
		aabbMax.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	}

	virtual void	printStats()
	{
//		printf("btSimpleBroadphase.h\n");
//		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
	}
};



#endif //BT_SIMPLE_BROADPHASE_H

