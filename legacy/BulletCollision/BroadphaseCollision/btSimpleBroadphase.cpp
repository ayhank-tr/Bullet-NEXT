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

#include "btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btAabbUtil2.h"

#include <new>

extern int btl_gOverlappingPairs;

void	btlSimpleBroadphase::validate()
{
	for (int i=0;i<m_numHandles;i++)
	{
		for (int j=i+1;j<m_numHandles;j++)
		{
			btlAssert(&m_pHandles[i] != &m_pHandles[j]);
		}
	}
	
}

btlSimpleBroadphase::btlSimpleBroadphase(int maxProxies, btlOverlappingPairCache* overlappingPairCache)
	:m_pairCache(overlappingPairCache),
	m_ownsPairCache(false),
	m_invalidPair(0)
{

	if (!overlappingPairCache)
	{
		void* mem = btlAlignedAlloc(sizeof(btlHashedOverlappingPairCache),16);
		m_pairCache = new (mem)btlHashedOverlappingPairCache();
		m_ownsPairCache = true;
	}

	// allocate handles buffer and put all handles on free list
	m_pHandlesRawPtr = btlAlignedAlloc(sizeof(btlSimpleBroadphaseProxy)*maxProxies,16);
	m_pHandles = new(m_pHandlesRawPtr) btlSimpleBroadphaseProxy[maxProxies];
	m_maxHandles = maxProxies;
	m_numHandles = 0;
	m_firstFreeHandle = 0;
	m_LastHandleIndex = -1;
	

	{
		for (int i = m_firstFreeHandle; i < maxProxies; i++)
		{
			m_pHandles[i].SetNextFree(i + 1);
			m_pHandles[i].m_uniqueId = i+2;//any UID will do, we just avoid too trivial values (0,1) for debugging purposes
		}
		m_pHandles[maxProxies - 1].SetNextFree(0);
	
	}

}

btlSimpleBroadphase::~btlSimpleBroadphase()
{
	btlAlignedFree(m_pHandlesRawPtr);

	if (m_ownsPairCache)
	{
		m_pairCache->~btlOverlappingPairCache();
		btlAlignedFree(m_pairCache);
	}
}


btlBroadphaseProxy*	btlSimpleBroadphase::createProxy(  const btlVector3& aabbMin,  const btlVector3& aabbMax,int shapeType,void* userPtr ,short int collisionFilterGroup,short int collisionFilterMask, btlDispatcher* /*dispatcher*/,void* multiSapProxy)
{
	if (m_numHandles >= m_maxHandles)
	{
		btlAssert(0);
		return 0; //should never happen, but don't let the game crash ;-)
	}
	btlAssert(aabbMin[0]<= aabbMax[0] && aabbMin[1]<= aabbMax[1] && aabbMin[2]<= aabbMax[2]);

	int newHandleIndex = allocHandle();
	btlSimpleBroadphaseProxy* proxy = new (&m_pHandles[newHandleIndex])btlSimpleBroadphaseProxy(aabbMin,aabbMax,shapeType,userPtr,collisionFilterGroup,collisionFilterMask,multiSapProxy);

	return proxy;
}

class	RemovingOverlapCallback : public btlOverlapCallback
{
protected:
	virtual bool	processOverlap(btlBroadphasePair& pair)
	{
		(void)pair;
		btlAssert(0);
		return false;
	}
};

class RemovePairContainingProxy
{

	btlBroadphaseProxy*	m_targetProxy;
	public:
	virtual ~RemovePairContainingProxy()
	{
	}
protected:
	virtual bool processOverlap(btlBroadphasePair& pair)
	{
		btlSimpleBroadphaseProxy* proxy0 = static_cast<btlSimpleBroadphaseProxy*>(pair.m_pProxy0);
		btlSimpleBroadphaseProxy* proxy1 = static_cast<btlSimpleBroadphaseProxy*>(pair.m_pProxy1);

		return ((m_targetProxy == proxy0 || m_targetProxy == proxy1));
	};
};

void	btlSimpleBroadphase::destroyProxy(btlBroadphaseProxy* proxyOrg,btlDispatcher* dispatcher)
{
		
		btlSimpleBroadphaseProxy* proxy0 = static_cast<btlSimpleBroadphaseProxy*>(proxyOrg);
		freeHandle(proxy0);

		m_pairCache->removeOverlappingPairsContainingProxy(proxyOrg,dispatcher);

		//validate();
		
}

void	btlSimpleBroadphase::getAabb(btlBroadphaseProxy* proxy,btlVector3& aabbMin, btlVector3& aabbMax ) const
{
	const btlSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
	aabbMin = sbp->m_aabbMin;
	aabbMax = sbp->m_aabbMax;
}

void	btlSimpleBroadphase::setAabb(btlBroadphaseProxy* proxy,const btlVector3& aabbMin,const btlVector3& aabbMax, btlDispatcher* /*dispatcher*/)
{
	btlSimpleBroadphaseProxy* sbp = getSimpleProxyFromProxy(proxy);
	sbp->m_aabbMin = aabbMin;
	sbp->m_aabbMax = aabbMax;
}

void	btlSimpleBroadphase::rayTest(const btlVector3& rayFrom,const btlVector3& rayTo, btlBroadphaseRayCallback& rayCallback, const btlVector3& aabbMin,const btlVector3& aabbMax)
{
	for (int i=0; i <= m_LastHandleIndex; i++)
	{
		btlSimpleBroadphaseProxy* proxy = &m_pHandles[i];
		if(!proxy->m_clientObject)
		{
			continue;
		}
		rayCallback.process(proxy);
	}
}


void	btlSimpleBroadphase::aabbTest(const btlVector3& aabbMin, const btlVector3& aabbMax, btlBroadphaseAabbCallback& callback)
{
	for (int i=0; i <= m_LastHandleIndex; i++)
	{
		btlSimpleBroadphaseProxy* proxy = &m_pHandles[i];
		if(!proxy->m_clientObject)
		{
			continue;
		}
		if (TestAabbAgainstAabb2(aabbMin,aabbMax,proxy->m_aabbMin,proxy->m_aabbMax))
		{
			callback.process(proxy);
		}
	}
}



	



bool	btlSimpleBroadphase::aabbOverlap(btlSimpleBroadphaseProxy* proxy0,btlSimpleBroadphaseProxy* proxy1)
{
	return proxy0->m_aabbMin[0] <= proxy1->m_aabbMax[0] && proxy1->m_aabbMin[0] <= proxy0->m_aabbMax[0] && 
		   proxy0->m_aabbMin[1] <= proxy1->m_aabbMax[1] && proxy1->m_aabbMin[1] <= proxy0->m_aabbMax[1] &&
		   proxy0->m_aabbMin[2] <= proxy1->m_aabbMax[2] && proxy1->m_aabbMin[2] <= proxy0->m_aabbMax[2];

}



//then remove non-overlapping ones
class CheckOverlapCallback : public btlOverlapCallback
{
public:
	virtual bool processOverlap(btlBroadphasePair& pair)
	{
		return (!btlSimpleBroadphase::aabbOverlap(static_cast<btlSimpleBroadphaseProxy*>(pair.m_pProxy0),static_cast<btlSimpleBroadphaseProxy*>(pair.m_pProxy1)));
	}
};

void	btlSimpleBroadphase::calculateOverlappingPairs(btlDispatcher* dispatcher)
{
	//first check for new overlapping pairs
	int i,j;
	if (m_numHandles >= 0)
	{
		int new_largest_index = -1;
		for (i=0; i <= m_LastHandleIndex; i++)
		{
			btlSimpleBroadphaseProxy* proxy0 = &m_pHandles[i];
			if(!proxy0->m_clientObject)
			{
				continue;
			}
			new_largest_index = i;
			for (j=i+1; j <= m_LastHandleIndex; j++)
			{
				btlSimpleBroadphaseProxy* proxy1 = &m_pHandles[j];
				btlAssert(proxy0 != proxy1);
				if(!proxy1->m_clientObject)
				{
					continue;
				}

				btlSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
				btlSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);

				if (aabbOverlap(p0,p1))
				{
					if ( !m_pairCache->findPair(proxy0,proxy1))
					{
						m_pairCache->addOverlappingPair(proxy0,proxy1);
					}
				} else
				{
					if (!m_pairCache->hasDeferredRemoval())
					{
						if ( m_pairCache->findPair(proxy0,proxy1))
						{
							m_pairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
						}
					}
				}
			}
		}

		m_LastHandleIndex = new_largest_index;

		if (m_ownsPairCache && m_pairCache->hasDeferredRemoval())
		{

			btlBroadphasePairArray&	overlappingPairArray = m_pairCache->getOverlappingPairArray();

			//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
			overlappingPairArray.quickSort(btlBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;


			btlBroadphasePair previousPair;
			previousPair.m_pProxy0 = 0;
			previousPair.m_pProxy1 = 0;
			previousPair.m_algorithm = 0;


			for (i=0;i<overlappingPairArray.size();i++)
			{

				btlBroadphasePair& pair = overlappingPairArray[i];

				bool isDuplicate = (pair == previousPair);

				previousPair = pair;

				bool needsRemoval = false;

				if (!isDuplicate)
				{
					bool hasOverlap = testAabbOverlap(pair.m_pProxy0,pair.m_pProxy1);

					if (hasOverlap)
					{
						needsRemoval = false;//callback->processOverlap(pair);
					} else
					{
						needsRemoval = true;
					}
				} else
				{
					//remove duplicate
					needsRemoval = true;
					//should have no algorithm
					btlAssert(!pair.m_algorithm);
				}

				if (needsRemoval)
				{
					m_pairCache->cleanOverlappingPair(pair,dispatcher);

					//		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
					//		m_overlappingPairArray.pop_back();
					pair.m_pProxy0 = 0;
					pair.m_pProxy1 = 0;
					m_invalidPair++;
					btl_gOverlappingPairs--;
				} 

			}

			///if you don't like to skip the invalid pairs in the array, execute following code:
#define CLEAN_INVALID_PAIRS 1
#ifdef CLEAN_INVALID_PAIRS

			//perform a sort, to sort 'invalid' pairs to the end
			overlappingPairArray.quickSort(btlBroadphasePairSortPredicate());

			overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
			m_invalidPair = 0;
#endif//CLEAN_INVALID_PAIRS

		}
	}
}


bool btlSimpleBroadphase::testAabbOverlap(btlBroadphaseProxy* proxy0,btlBroadphaseProxy* proxy1)
{
	btlSimpleBroadphaseProxy* p0 = getSimpleProxyFromProxy(proxy0);
	btlSimpleBroadphaseProxy* p1 = getSimpleProxyFromProxy(proxy1);
	return aabbOverlap(p0,p1);
}

void	btlSimpleBroadphase::resetPool(btlDispatcher* dispatcher)
{
	//not yet
}
