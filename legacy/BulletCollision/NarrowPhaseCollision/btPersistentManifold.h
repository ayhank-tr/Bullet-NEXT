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

#ifndef BT_PERSISTENT_MANIFOLD_H
#define BT_PERSISTENT_MANIFOLD_H


#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "btManifoldPoint.h"
class btlCollisionObject;
#include "LinearMath/btAlignedAllocator.h"

struct btlCollisionResult;

///maximum contact breaking and merging threshold
extern btlScalar btl_gContactBreakingThreshold;

typedef bool (*ContactDestroyedCallback)(void* userPersistentData);
typedef bool (*ContactProcessedCallback)(btlManifoldPoint& cp,void* body0,void* body1);
extern ContactDestroyedCallback	btl_gContactDestroyedCallback;
extern ContactProcessedCallback btl_gContactProcessedCallback;

//the enum starts at 1024 to avoid type conflicts with btTypedConstraint
enum btlContactManifoldTypes
{
	MIN_CONTACT_MANIFOLD_TYPE = 1024,
	BT_PERSISTENT_MANIFOLD_TYPE
};

#define MANIFOLD_CACHE_SIZE 4

///btPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
///Those contact points are created by the collision narrow phase.
///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
///reduces the cache to 4 points, when more then 4 points are added, using following rules:
///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
///note that some pairs of objects might have more then one contact manifold.


ATTRIBUTE_ALIGNED128( class) btlPersistentManifold : public btlTypedObject
//ATTRIBUTE_ALIGNED16( class) btPersistentManifold : public btTypedObject
{

	btlManifoldPoint m_pointCache[MANIFOLD_CACHE_SIZE];

	/// this two body pointers can point to the physics rigidbody class.
	const btlCollisionObject* m_body0;
	const btlCollisionObject* m_body1;

	int	m_cachedPoints;

	btlScalar	m_contactBreakingThreshold;
	btlScalar	m_contactProcessingThreshold;

	
	/// sort cached points so most isolated points come first
	int	sortCachedPoints(const btlManifoldPoint& pt);

	int		findContactPoint(const btlManifoldPoint* unUsed, int numUnused,const btlManifoldPoint& pt);

public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	int	m_companionIdA;
	int	m_companionIdB;

	int m_index1a;

	btlPersistentManifold();

	btlPersistentManifold(const btlCollisionObject* body0,const btlCollisionObject* body1,int , btlScalar contactBreakingThreshold,btlScalar contactProcessingThreshold)
		: btlTypedObject(BT_PERSISTENT_MANIFOLD_TYPE),
	m_body0(body0),m_body1(body1),m_cachedPoints(0),
		m_contactBreakingThreshold(contactBreakingThreshold),
		m_contactProcessingThreshold(contactProcessingThreshold)
	{
	}

	SIMD_FORCE_INLINE const btlCollisionObject* getBody0() const { return m_body0;}
	SIMD_FORCE_INLINE const btlCollisionObject* getBody1() const { return m_body1;}

	void	setBodies(const btlCollisionObject* body0,const btlCollisionObject* body1)
	{
		m_body0 = body0;
		m_body1 = body1;
	}

	void clearUserCache(btlManifoldPoint& pt);

#ifdef DEBUG_PERSISTENCY
	void	DebugPersistency();
#endif //
	
	SIMD_FORCE_INLINE int	getNumContacts() const { return m_cachedPoints;}
	/// the setNumContacts API is usually not used, except when you gather/fill all contacts manually
	void setNumContacts(int cachedPoints)
	{
		m_cachedPoints = cachedPoints;
	}


	SIMD_FORCE_INLINE const btlManifoldPoint& getContactPoint(int index) const
	{
		btlAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	SIMD_FORCE_INLINE btlManifoldPoint& getContactPoint(int index)
	{
		btlAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	///@todo: get this margin from the current physics / collision environment
	btlScalar	getContactBreakingThreshold() const;

	btlScalar	getContactProcessingThreshold() const
	{
		return m_contactProcessingThreshold;
	}
	
	void setContactBreakingThreshold(btlScalar contactBreakingThreshold)
	{
		m_contactBreakingThreshold = contactBreakingThreshold;
	}

	void setContactProcessingThreshold(btlScalar	contactProcessingThreshold)
	{
		m_contactProcessingThreshold = contactProcessingThreshold;
	}
	
	


	int getCacheEntry(const btlManifoldPoint& newPoint) const;

	int addManifoldPoint( const btlManifoldPoint& newPoint, bool isPredictive=false);

	void removeContactPoint (int index)
	{
		clearUserCache(m_pointCache[index]);

		int lastUsedIndex = getNumContacts() - 1;
//		m_pointCache[index] = m_pointCache[lastUsedIndex];
		if(index != lastUsedIndex) 
		{
			m_pointCache[index] = m_pointCache[lastUsedIndex]; 
			//get rid of duplicated userPersistentData pointer
			m_pointCache[lastUsedIndex].m_userPersistentData = 0;
			m_pointCache[lastUsedIndex].m_appliedImpulse = 0.f;
			m_pointCache[lastUsedIndex].m_lateralFrictionInitialized = false;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral1 = 0.f;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral2 = 0.f;
			m_pointCache[lastUsedIndex].m_lifeTime = 0;
		}

		btlAssert(m_pointCache[lastUsedIndex].m_userPersistentData==0);
		m_cachedPoints--;
	}
	void replaceContactPoint(const btlManifoldPoint& newPoint,int insertIndex)
	{
		btlAssert(validContactDistance(newPoint));

#define MAINTAIN_PERSISTENCY 1
#ifdef MAINTAIN_PERSISTENCY
		int	lifeTime = m_pointCache[insertIndex].getLifeTime();
		btlScalar	appliedImpulse = m_pointCache[insertIndex].m_appliedImpulse;
		btlScalar	appliedLateralImpulse1 = m_pointCache[insertIndex].m_appliedImpulseLateral1;
		btlScalar	appliedLateralImpulse2 = m_pointCache[insertIndex].m_appliedImpulseLateral2;
//		bool isLateralFrictionInitialized = m_pointCache[insertIndex].m_lateralFrictionInitialized;
		
		
			
		btlAssert(lifeTime>=0);
		void* cache = m_pointCache[insertIndex].m_userPersistentData;
		
		m_pointCache[insertIndex] = newPoint;

		m_pointCache[insertIndex].m_userPersistentData = cache;
		m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
		m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
		m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;
		

		m_pointCache[insertIndex].m_lifeTime = lifeTime;
#else
		clearUserCache(m_pointCache[insertIndex]);
		m_pointCache[insertIndex] = newPoint;
	
#endif
	}

	
	bool validContactDistance(const btlManifoldPoint& pt) const
	{
		return pt.m_distance1 <= getContactBreakingThreshold();
	}
	/// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
	void	refreshContactPoints(  const btlTransform& trA,const btlTransform& trB);

	
	SIMD_FORCE_INLINE	void	clearManifold()
	{
		int i;
		for (i=0;i<m_cachedPoints;i++)
		{
			clearUserCache(m_pointCache[i]);
		}
		m_cachedPoints = 0;
	}



}
;





#endif //BT_PERSISTENT_MANIFOLD_H
