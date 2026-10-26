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

#ifndef BT_COMPOUND_COLLISION_ALGORITHM_H
#define BT_COMPOUND_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"

#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btlDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"
class btlDispatcher;
class btlCollisionObject;

class btlCollisionShape;
typedef bool (*btlShapePairCallback)(const btlCollisionShape* pShape0, const btlCollisionShape* pShape1);
extern btlShapePairCallback btl_gCompoundChildShapePairCallback;

/// btCompoundCollisionAlgorithm  supports collision between CompoundCollisionShapes and other collision shapes
class btlCompoundCollisionAlgorithm  : public btlActivatingCollisionAlgorithm
{
protected:
	btlAlignedObjectArray<btlCollisionAlgorithm*> m_childCollisionAlgorithms;
	bool m_isSwapped;

	class btlPersistentManifold*	m_sharedManifold;
	bool					m_ownsManifold;


	int	m_compoundShapeRevision;//to keep track of changes, so that childAlgorithm array can be updated
	
	void	removeChildAlgorithms();
	
	void	preallocateChildAlgorithms(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap);

public:

	btlCompoundCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped);

	virtual ~btlCompoundCollisionAlgorithm();

	btlCollisionAlgorithm* getChildAlgorithm (int n) const
	{
		return m_childCollisionAlgorithms[n];
	}


	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	btlScalar	calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
		int i;
		for (i=0;i<m_childCollisionAlgorithms.size();i++)
		{
			if (m_childCollisionAlgorithms[i])
				m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
		}
	}

	
	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlCompoundCollisionAlgorithm));
			return new(mem) btlCompoundCollisionAlgorithm(ci,body0Wrap,body1Wrap,false);
		}
	};

	struct SwappedCreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlCompoundCollisionAlgorithm));
			return new(mem) btlCompoundCollisionAlgorithm(ci,body0Wrap,body1Wrap,true);
		}
	};

};

#endif //BT_COMPOUND_COLLISION_ALGORITHM_H
