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


#ifndef BT_MANIFOLD_RESULT_H
#define BT_MANIFOLD_RESULT_H

class btlCollisionObject;
struct btlCollisionObjectWrapper;

#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btlManifoldPoint;

#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"

#include "LinearMath/btTransform.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

typedef bool (*ContactAddedCallback)(btlManifoldPoint& cp,	const btlCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btlCollisionObjectWrapper* colObj1Wrap,int partId1,int index1);
extern ContactAddedCallback		btl_gContactAddedCallback;

//#define DEBUG_PART_INDEX 1


///btManifoldResult is a helper class to manage  contact results.
class btlManifoldResult : public btlDiscreteCollisionDetectorInterface::Result
{
protected:

	btlPersistentManifold* m_manifoldPtr;

	const btlCollisionObjectWrapper* m_body0Wrap;
	const btlCollisionObjectWrapper* m_body1Wrap;
	int	m_partId0;
	int m_partId1;
	int m_index0;
	int m_index1;
	

public:

	btlManifoldResult()
#ifdef DEBUG_PART_INDEX
		:
	m_partId0(-1),
	m_partId1(-1),
	m_index0(-1),
	m_index1(-1)
#endif //DEBUG_PART_INDEX
	{
	}

	btlManifoldResult(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap);

	virtual ~btlManifoldResult() {};

	void	setPersistentManifold(btlPersistentManifold* manifoldPtr)
	{
		m_manifoldPtr = manifoldPtr;
	}

	const btlPersistentManifold*	getPersistentManifold() const
	{
		return m_manifoldPtr;
	}
	btlPersistentManifold*	getPersistentManifold()
	{
		return m_manifoldPtr;
	}

	virtual void setShapeIdentifiersA(int partId0,int index0)
	{
		m_partId0=partId0;
		m_index0=index0;
	}

	virtual void setShapeIdentifiersB(	int partId1,int index1)
	{
		m_partId1=partId1;
		m_index1=index1;
	}


	virtual void addContactPoint(const btlVector3& normalOnBInWorld,const btlVector3& pointInWorld,btlScalar depth);

	SIMD_FORCE_INLINE	void refreshContactPoints()
	{
		btlAssert(m_manifoldPtr);
		if (!m_manifoldPtr->getNumContacts())
			return;

		bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();

		if (isSwapped)
		{
			m_manifoldPtr->refreshContactPoints(m_body1Wrap->getCollisionObject()->getWorldTransform(),m_body0Wrap->getCollisionObject()->getWorldTransform());
		} else
		{
			m_manifoldPtr->refreshContactPoints(m_body0Wrap->getCollisionObject()->getWorldTransform(),m_body1Wrap->getCollisionObject()->getWorldTransform());
		}
	}

	const btlCollisionObjectWrapper* getBody0Wrap() const
	{
		return m_body0Wrap;
	}
	const btlCollisionObjectWrapper* getBody1Wrap() const
	{
		return m_body1Wrap;
	}

	void setBody0Wrap(const btlCollisionObjectWrapper* obj0Wrap)
	{
		m_body0Wrap = obj0Wrap;
	}

	void setBody1Wrap(const btlCollisionObjectWrapper* obj1Wrap)
	{
		m_body1Wrap = obj1Wrap;
	}

	const btlCollisionObject* getBody0Internal() const
	{
		return m_body0Wrap->getCollisionObject();
	}

	const btlCollisionObject* getBody1Internal() const
	{
		return m_body1Wrap->getCollisionObject();
	}

	/// in the future we can let the user override the methods to combine restitution and friction
	static btlScalar	calculateCombinedRestitution(const btlCollisionObject* body0,const btlCollisionObject* body1);
	static btlScalar	calculateCombinedFriction(const btlCollisionObject* body0,const btlCollisionObject* body1);
};

#endif //BT_MANIFOLD_RESULT_H
