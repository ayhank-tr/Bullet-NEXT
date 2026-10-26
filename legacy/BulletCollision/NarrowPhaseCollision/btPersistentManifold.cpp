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


#include "btPersistentManifold.h"
#include "LinearMath/btTransform.h"


btlScalar					btl_gContactBreakingThreshold = btlScalar(0.02);
ContactDestroyedCallback	btl_gContactDestroyedCallback = 0;
ContactProcessedCallback	btl_gContactProcessedCallback = 0;
///gContactCalcArea3Points will approximate the convex hull area using 3 points
///when setting it to false, it will use 4 points to compute the area: it is more accurate but slower
bool						btl_gContactCalcArea3Points = true;


btlPersistentManifold::btlPersistentManifold()
:btlTypedObject(BT_PERSISTENT_MANIFOLD_TYPE),
m_body0(0),
m_body1(0),
m_cachedPoints (0),
m_index1a(0)
{
}




#ifdef DEBUG_PERSISTENCY
#include <stdio.h>
void	btlPersistentManifold::DebugPersistency()
{
	int i;
	printf("DebugPersistency : numPoints %d\n",m_cachedPoints);
	for (i=0;i<m_cachedPoints;i++)
	{
		printf("m_pointCache[%d].m_userPersistentData = %x\n",i,m_pointCache[i].m_userPersistentData);
	}
}
#endif //DEBUG_PERSISTENCY

void btlPersistentManifold::clearUserCache(btlManifoldPoint& pt)
{

	void* oldPtr = pt.m_userPersistentData;
	if (oldPtr)
	{
#ifdef DEBUG_PERSISTENCY
		int i;
		int occurance = 0;
		for (i=0;i<m_cachedPoints;i++)
		{
			if (m_pointCache[i].m_userPersistentData == oldPtr)
			{
				occurance++;
				if (occurance>1)
					printf("error in clearUserCache\n");
			}
		}
		btlAssert(occurance<=0);
#endif //DEBUG_PERSISTENCY

		if (pt.m_userPersistentData && btl_gContactDestroyedCallback)
		{
			(*btl_gContactDestroyedCallback)(pt.m_userPersistentData);
			pt.m_userPersistentData = 0;
		}
		
#ifdef DEBUG_PERSISTENCY
		DebugPersistency();
#endif
	}

	
}

static inline btlScalar calcArea4Points(const btlVector3 &p0,const btlVector3 &p1,const btlVector3 &p2,const btlVector3 &p3)
{
	// It calculates possible 3 area constructed from random 4 points and returns the biggest one.

	btlVector3 a[3],b[3];
	a[0] = p0 - p1;
	a[1] = p0 - p2;
	a[2] = p0 - p3;
	b[0] = p2 - p3;
	b[1] = p1 - p3;
	b[2] = p1 - p2;

	//todo: Following 3 cross production can be easily optimized by SIMD.
	btlVector3 tmp0 = a[0].cross(b[0]);
	btlVector3 tmp1 = a[1].cross(b[1]);
	btlVector3 tmp2 = a[2].cross(b[2]);

	return btlMax(btlMax(tmp0.length2(),tmp1.length2()),tmp2.length2());
}

int btlPersistentManifold::sortCachedPoints(const btlManifoldPoint& pt) 
{
		//calculate 4 possible cases areas, and take biggest area
		//also need to keep 'deepest'
		
		int maxPenetrationIndex = -1;
#define KEEP_DEEPEST_POINT 1
#ifdef KEEP_DEEPEST_POINT
		btlScalar maxPenetration = pt.getDistance();
		for (int i=0;i<4;i++)
		{
			if (m_pointCache[i].getDistance() < maxPenetration)
			{
				maxPenetrationIndex = i;
				maxPenetration = m_pointCache[i].getDistance();
			}
		}
#endif //KEEP_DEEPEST_POINT
		
		btlScalar res0(btlScalar(0.)),res1(btlScalar(0.)),res2(btlScalar(0.)),res3(btlScalar(0.));

	if (btl_gContactCalcArea3Points)
	{
		if (maxPenetrationIndex != 0)
		{
			btlVector3 a0 = pt.m_localPointA-m_pointCache[1].m_localPointA;
			btlVector3 b0 = m_pointCache[3].m_localPointA-m_pointCache[2].m_localPointA;
			btlVector3 cross = a0.cross(b0);
			res0 = cross.length2();
		}
		if (maxPenetrationIndex != 1)
		{
			btlVector3 a1 = pt.m_localPointA-m_pointCache[0].m_localPointA;
			btlVector3 b1 = m_pointCache[3].m_localPointA-m_pointCache[2].m_localPointA;
			btlVector3 cross = a1.cross(b1);
			res1 = cross.length2();
		}

		if (maxPenetrationIndex != 2)
		{
			btlVector3 a2 = pt.m_localPointA-m_pointCache[0].m_localPointA;
			btlVector3 b2 = m_pointCache[3].m_localPointA-m_pointCache[1].m_localPointA;
			btlVector3 cross = a2.cross(b2);
			res2 = cross.length2();
		}

		if (maxPenetrationIndex != 3)
		{
			btlVector3 a3 = pt.m_localPointA-m_pointCache[0].m_localPointA;
			btlVector3 b3 = m_pointCache[2].m_localPointA-m_pointCache[1].m_localPointA;
			btlVector3 cross = a3.cross(b3);
			res3 = cross.length2();
		}
	} 
	else
	{
		if(maxPenetrationIndex != 0) {
			res0 = calcArea4Points(pt.m_localPointA,m_pointCache[1].m_localPointA,m_pointCache[2].m_localPointA,m_pointCache[3].m_localPointA);
		}

		if(maxPenetrationIndex != 1) {
			res1 = calcArea4Points(pt.m_localPointA,m_pointCache[0].m_localPointA,m_pointCache[2].m_localPointA,m_pointCache[3].m_localPointA);
		}

		if(maxPenetrationIndex != 2) {
			res2 = calcArea4Points(pt.m_localPointA,m_pointCache[0].m_localPointA,m_pointCache[1].m_localPointA,m_pointCache[3].m_localPointA);
		}

		if(maxPenetrationIndex != 3) {
			res3 = calcArea4Points(pt.m_localPointA,m_pointCache[0].m_localPointA,m_pointCache[1].m_localPointA,m_pointCache[2].m_localPointA);
		}
	}
	btlVector4 maxvec(res0,res1,res2,res3);
	int biggestarea = maxvec.closestAxis4();
	return biggestarea;
	
}


int btlPersistentManifold::getCacheEntry(const btlManifoldPoint& newPoint) const
{
	btlScalar shortestDist =  getContactBreakingThreshold() * getContactBreakingThreshold();
	int size = getNumContacts();
	int nearestPoint = -1;
	for( int i = 0; i < size; i++ )
	{
		const btlManifoldPoint &mp = m_pointCache[i];

		btlVector3 diffA =  mp.m_localPointA- newPoint.m_localPointA;
		const btlScalar distToManiPoint = diffA.dot(diffA);
		if( distToManiPoint < shortestDist )
		{
			shortestDist = distToManiPoint;
			nearestPoint = i;
		}
	}
	return nearestPoint;
}

int btlPersistentManifold::addManifoldPoint(const btlManifoldPoint& newPoint, bool isPredictive)
{
	if (!isPredictive)
	{
		btlAssert(validContactDistance(newPoint));
	}
	
	int insertIndex = getNumContacts();
	if (insertIndex == MANIFOLD_CACHE_SIZE)
	{
#if MANIFOLD_CACHE_SIZE >= 4
		//sort cache so best points come first, based on area
		insertIndex = sortCachedPoints(newPoint);
#else
		insertIndex = 0;
#endif
		clearUserCache(m_pointCache[insertIndex]);
		
	} else
	{
		m_cachedPoints++;

		
	}
	if (insertIndex<0)
		insertIndex=0;

	btlAssert(m_pointCache[insertIndex].m_userPersistentData==0);
	m_pointCache[insertIndex] = newPoint;
	return insertIndex;
}

btlScalar	btlPersistentManifold::getContactBreakingThreshold() const
{
	return m_contactBreakingThreshold;
}



void btlPersistentManifold::refreshContactPoints(const btlTransform& trA,const btlTransform& trB)
{
	int i;
#ifdef DEBUG_PERSISTENCY
	printf("refreshContactPoints posA = (%f,%f,%f) posB = (%f,%f,%f)\n",
		trA.getOrigin().getX(),
		trA.getOrigin().getY(),
		trA.getOrigin().getZ(),
		trB.getOrigin().getX(),
		trB.getOrigin().getY(),
		trB.getOrigin().getZ());
#endif //DEBUG_PERSISTENCY
	/// first refresh worldspace positions and distance
	for (i=getNumContacts()-1;i>=0;i--)
	{
		btlManifoldPoint &manifoldPoint = m_pointCache[i];
		manifoldPoint.m_positionWorldOnA = trA( manifoldPoint.m_localPointA );
		manifoldPoint.m_positionWorldOnB = trB( manifoldPoint.m_localPointB );
		manifoldPoint.m_distance1 = (manifoldPoint.m_positionWorldOnA -  manifoldPoint.m_positionWorldOnB).dot(manifoldPoint.m_normalWorldOnB);
		manifoldPoint.m_lifeTime++;
	}

	/// then 
	btlScalar distance2d;
	btlVector3 projectedDifference,projectedPoint;
	for (i=getNumContacts()-1;i>=0;i--)
	{
		
		btlManifoldPoint &manifoldPoint = m_pointCache[i];
		//contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
		if (!validContactDistance(manifoldPoint))
		{
			removeContactPoint(i);
		} else
		{
			//contact also becomes invalid when relative movement orthogonal to normal exceeds margin
			projectedPoint = manifoldPoint.m_positionWorldOnA - manifoldPoint.m_normalWorldOnB * manifoldPoint.m_distance1;
			projectedDifference = manifoldPoint.m_positionWorldOnB - projectedPoint;
			distance2d = projectedDifference.dot(projectedDifference);
			if (distance2d  > getContactBreakingThreshold()*getContactBreakingThreshold() )
			{
				removeContactPoint(i);
			} else
			{
				//contact point processed callback
				if (btl_gContactProcessedCallback)
					(*btl_gContactProcessedCallback)(manifoldPoint,(void*)m_body0,(void*)m_body1);
			}
		}
	}
#ifdef DEBUG_PERSISTENCY
	DebugPersistency();
#endif //
}





