/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#ifndef BT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H
#define BT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H

#include "btCompoundCollisionAlgorithm.h"

#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"

#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btlDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionDispatch/btHashedSimplePairCache.h"
class btlDispatcher;
class btlCollisionObject;

class btlCollisionShape;
typedef bool (*btlShapePairCallback)(const btlCollisionShape* pShape0, const btlCollisionShape* pShape1);
extern btlShapePairCallback btl_gCompoundCompoundChildShapePairCallback;

/// btCompoundCompoundCollisionAlgorithm  supports collision between two btCompoundCollisionShape shapes
class btlCompoundCompoundCollisionAlgorithm  : public btlCompoundCollisionAlgorithm
{

	class btlHashedSimplePairCache*	m_childCollisionAlgorithmCache;
	btlSimplePairArray m_removePairs;


	int	m_compoundShapeRevision0;//to keep track of changes, so that childAlgorithm array can be updated
	int	m_compoundShapeRevision1;
	
	void	removeChildAlgorithms();
	
//	void	preallocateChildAlgorithms(const btCollisionObjectWrapper* body0Wrap,const btCollisionObjectWrapper* body1Wrap);

public:

	btlCompoundCompoundCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped);

	virtual ~btlCompoundCompoundCollisionAlgorithm();

	

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	btlScalar	calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray);
	
	
	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlCompoundCompoundCollisionAlgorithm));
			return new(mem) btlCompoundCompoundCollisionAlgorithm(ci,body0Wrap,body1Wrap,false);
		}
	};

	struct SwappedCreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlCompoundCompoundCollisionAlgorithm));
			return new(mem) btlCompoundCompoundCollisionAlgorithm(ci,body0Wrap,body1Wrap,true);
		}
	};

};

#endif //BT_COMPOUND_COMPOUND_COLLISION_ALGORITHM_H
