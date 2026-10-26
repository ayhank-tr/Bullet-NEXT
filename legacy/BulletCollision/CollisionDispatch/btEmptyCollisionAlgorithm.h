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

#ifndef BT_EMPTY_ALGORITH
#define BT_EMPTY_ALGORITH
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "btCollisionCreateFunc.h"
#include "btCollisionDispatcher.h"

#define ATTRIBUTE_ALIGNED(a)

///EmptyAlgorithm is a stub for unsupported collision pairs.
///The dispatcher can dispatch a persistent btEmptyAlgorithm to avoid a search every frame.
class btlEmptyAlgorithm : public btlCollisionAlgorithm
{

public:
	
	btlEmptyAlgorithm(const btlCollisionAlgorithmConstructionInfo& ci);

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual btlScalar calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
	}

	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
        virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
        {
			(void)body0Wrap;
			(void)body1Wrap;
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlEmptyAlgorithm));
			return new(mem) btlEmptyAlgorithm(ci);
		}
	};

} ATTRIBUTE_ALIGNED(16);

#endif //BT_EMPTY_ALGORITH
