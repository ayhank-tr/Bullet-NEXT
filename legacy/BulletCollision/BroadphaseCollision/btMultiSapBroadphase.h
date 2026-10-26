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
#ifndef BT_MULTI_SAP_BROADPHASE
#define BT_MULTI_SAP_BROADPHASE

#include "btBroadphaseInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btOverlappingPairCache.h"


class btlBroadphaseInterface;
class btlSimpleBroadphase;


typedef btlAlignedObjectArray<btlBroadphaseInterface*> btlSapBroadphaseArray;

///The btMultiSapBroadphase is a research project, not recommended to use in production. Use btAxisSweep3 or btDbvtBroadphase instead.
///The btMultiSapBroadphase is a broadphase that contains multiple SAP broadphases.
///The user can add SAP broadphases that cover the world. A btBroadphaseProxy can be in multiple child broadphases at the same time.
///A btQuantizedBvh acceleration structures finds overlapping SAPs for each btBroadphaseProxy.
///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=328
///and http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=1329
class btlMultiSapBroadphase :public btlBroadphaseInterface
{
	btlSapBroadphaseArray	m_sapBroadphases;
	
	btlSimpleBroadphase*		m_simpleBroadphase;

	btlOverlappingPairCache*	m_overlappingPairs;

	class btlQuantizedBvh*			m_optimizedAabbTree;


	bool					m_ownsPairCache;
	
	btlOverlapFilterCallback*	m_filterCallback;

	int			m_invalidPair;

	struct	btlBridgeProxy
	{
		btlBroadphaseProxy*		m_childProxy;
		btlBroadphaseInterface*	m_childBroadphase;
	};


public:

	struct	btlMultiSapProxy	: public btlBroadphaseProxy
	{

		///array with all the entries that this proxy belongs to
		btlAlignedObjectArray<btlBridgeProxy*> m_bridgeProxies;
		btlVector3	m_aabbMin;
		btlVector3	m_aabbMax;

		int	m_shapeType;

/*		void*	m_userPtr;
		short int	m_collisionFilterGroup;
		short int	m_collisionFilterMask;
*/
		btlMultiSapProxy(const btlVector3& aabbMin,  const btlVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask)
			:btlBroadphaseProxy(aabbMin,aabbMax,userPtr,collisionFilterGroup,collisionFilterMask),
			m_aabbMin(aabbMin),
			m_aabbMax(aabbMax),
			m_shapeType(shapeType)
		{
			m_multiSapParentProxy =this;
		}

		
	};

protected:


	btlAlignedObjectArray<btlMultiSapProxy*> m_multiSapProxies;

public:

	btlMultiSapBroadphase(int maxProxies = 16384,btlOverlappingPairCache* pairCache=0);


	btlSapBroadphaseArray&	getBroadphaseArray()
	{
		return m_sapBroadphases;
	}

	const btlSapBroadphaseArray&	getBroadphaseArray() const
	{
		return m_sapBroadphases;
	}

	virtual ~btlMultiSapBroadphase();

	virtual btlBroadphaseProxy*	createProxy(  const btlVector3& aabbMin,  const btlVector3& aabbMax,int shapeType,void* userPtr, short int collisionFilterGroup,short int collisionFilterMask, btlDispatcher* dispatcher,void* multiSapProxy);
	virtual void	destroyProxy(btlBroadphaseProxy* proxy,btlDispatcher* dispatcher);
	virtual void	setAabb(btlBroadphaseProxy* proxy,const btlVector3& aabbMin,const btlVector3& aabbMax, btlDispatcher* dispatcher);
	virtual void	getAabb(btlBroadphaseProxy* proxy,btlVector3& aabbMin, btlVector3& aabbMax ) const;

	virtual void	rayTest(const btlVector3& rayFrom,const btlVector3& rayTo, btlBroadphaseRayCallback& rayCallback,const btlVector3& aabbMin=btlVector3(0,0,0),const btlVector3& aabbMax=btlVector3(0,0,0));

	void	addToChildBroadphase(btlMultiSapProxy* parentMultiSapProxy, btlBroadphaseProxy* childProxy, btlBroadphaseInterface*	childBroadphase);

	///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
	virtual void	calculateOverlappingPairs(btlDispatcher* dispatcher);

	bool	testAabbOverlap(btlBroadphaseProxy* proxy0,btlBroadphaseProxy* proxy1);

	virtual	btlOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_overlappingPairs;
	}
	virtual	const btlOverlappingPairCache*	getOverlappingPairCache() const
	{
		return m_overlappingPairs;
	}

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(btlVector3& aabbMin,btlVector3& aabbMax) const
	{
		aabbMin.setValue(-BT_LARGE_FLOAT,-BT_LARGE_FLOAT,-BT_LARGE_FLOAT);
		aabbMax.setValue(BT_LARGE_FLOAT,BT_LARGE_FLOAT,BT_LARGE_FLOAT);
	}

	void	buildTree(const btlVector3& bvhAabbMin,const btlVector3& bvhAabbMax);

	virtual void	printStats();

	void quicksort (btlBroadphasePairArray& a, int lo, int hi);

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(btlDispatcher* dispatcher);

};

#endif //BT_MULTI_SAP_BROADPHASE
