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

#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAabbUtil2.h"
#include "btManifoldResult.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

btlShapePairCallback btl_gCompoundChildShapePairCallback = 0;

btlCompoundCollisionAlgorithm::btlCompoundCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped)
:btlActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_isSwapped(isSwapped),
m_sharedManifold(ci.m_manifold)
{
	m_ownsManifold = false;

	const btlCollisionObjectWrapper* colObjWrap = m_isSwapped? body1Wrap : body0Wrap;
	btlAssert (colObjWrap->getCollisionShape()->isCompound());
	
	const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(colObjWrap->getCollisionShape());
	m_compoundShapeRevision = compoundShape->getUpdateRevision();
	
	
	preallocateChildAlgorithms(body0Wrap,body1Wrap);
}

void	btlCompoundCollisionAlgorithm::preallocateChildAlgorithms(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
{
	const btlCollisionObjectWrapper* colObjWrap = m_isSwapped? body1Wrap : body0Wrap;
	const btlCollisionObjectWrapper* otherObjWrap = m_isSwapped? body0Wrap : body1Wrap;
	btlAssert (colObjWrap->getCollisionShape()->isCompound());
	
	const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(colObjWrap->getCollisionShape());

	int numChildren = compoundShape->getNumChildShapes();
	int i;
	
	m_childCollisionAlgorithms.resize(numChildren);
	for (i=0;i<numChildren;i++)
	{
		if (compoundShape->getDynamicAabbTree())
		{
			m_childCollisionAlgorithms[i] = 0;
		} else
		{
			
			const btlCollisionShape* childShape = compoundShape->getChildShape(i);

			btlCollisionObjectWrapper childWrap(colObjWrap,childShape,colObjWrap->getCollisionObject(),colObjWrap->getWorldTransform(),-1,i);//wrong child trans, but unused (hopefully)
			m_childCollisionAlgorithms[i] = m_dispatcher->findAlgorithm(&childWrap,otherObjWrap,m_sharedManifold);
		}
	}
}

void	btlCompoundCollisionAlgorithm::removeChildAlgorithms()
{
	int numChildren = m_childCollisionAlgorithms.size();
	int i;
	for (i=0;i<numChildren;i++)
	{
		if (m_childCollisionAlgorithms[i])
		{
			m_childCollisionAlgorithms[i]->~btlCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
		}
	}
}

btlCompoundCollisionAlgorithm::~btlCompoundCollisionAlgorithm()
{
	removeChildAlgorithms();
}




struct	btlCompoundLeafCallback : btlDbvt::ICollide
{

public:

	const btlCollisionObjectWrapper* m_compoundColObjWrap;
	const btlCollisionObjectWrapper* m_otherObjWrap;
	btlDispatcher* m_dispatcher;
	const btlDispatcherInfo& m_dispatchInfo;
	btlManifoldResult*	m_resultOut;
	btlCollisionAlgorithm**	m_childCollisionAlgorithms;
	btlPersistentManifold*	m_sharedManifold;
	
	btlCompoundLeafCallback (const btlCollisionObjectWrapper* compoundObjWrap,const btlCollisionObjectWrapper* otherObjWrap,btlDispatcher* dispatcher,const btlDispatcherInfo& dispatchInfo,btlManifoldResult*	resultOut,btlCollisionAlgorithm**	childCollisionAlgorithms,btlPersistentManifold*	sharedManifold)
		:m_compoundColObjWrap(compoundObjWrap),m_otherObjWrap(otherObjWrap),m_dispatcher(dispatcher),m_dispatchInfo(dispatchInfo),m_resultOut(resultOut),
		m_childCollisionAlgorithms(childCollisionAlgorithms),
		m_sharedManifold(sharedManifold)
	{

	}


	void	ProcessChildShape(const btlCollisionShape* childShape,int index)
	{
		btlAssert(index>=0);
		const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(m_compoundColObjWrap->getCollisionShape());
		btlAssert(index<compoundShape->getNumChildShapes());


		//backup
		btlTransform	orgTrans = m_compoundColObjWrap->getWorldTransform();
		
		const btlTransform& childTrans = compoundShape->getChildTransform(index);
		btlTransform	newChildWorldTrans = orgTrans*childTrans ;

		//perform an AABB check first
		btlVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;
		childShape->getAabb(newChildWorldTrans,aabbMin0,aabbMax0);
		m_otherObjWrap->getCollisionShape()->getAabb(m_otherObjWrap->getWorldTransform(),aabbMin1,aabbMax1);

		if (btl_gCompoundChildShapePairCallback)
		{
			if (!btl_gCompoundChildShapePairCallback(m_otherObjWrap->getCollisionShape(), childShape))
				return;
		}

		if (TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
		{

			btlCollisionObjectWrapper compoundWrap(this->m_compoundColObjWrap,childShape,m_compoundColObjWrap->getCollisionObject(),newChildWorldTrans,-1,index);


			//the contactpoint is still projected back using the original inverted worldtrans
			if (!m_childCollisionAlgorithms[index])
				m_childCollisionAlgorithms[index] = m_dispatcher->findAlgorithm(&compoundWrap,m_otherObjWrap,m_sharedManifold);

			
			const btlCollisionObjectWrapper* tmpWrap = 0;

			///detect swapping case
			if (m_resultOut->getBody0Internal() == m_compoundColObjWrap->getCollisionObject())
			{
				tmpWrap = m_resultOut->getBody0Wrap();
				m_resultOut->setBody0Wrap(&compoundWrap);
				m_resultOut->setShapeIdentifiersA(-1,index);
			} else
			{
				tmpWrap = m_resultOut->getBody1Wrap();
				m_resultOut->setBody1Wrap(&compoundWrap);
				m_resultOut->setShapeIdentifiersB(-1,index);
			}


			m_childCollisionAlgorithms[index]->processCollision(&compoundWrap,m_otherObjWrap,m_dispatchInfo,m_resultOut);

#if 0
			if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw->getDebugMode() & btlIDebugDraw::DBG_DrawAabb))
			{
				btlVector3 worldAabbMin,worldAabbMax;
				m_dispatchInfo.m_debugDraw->drawAabb(aabbMin0,aabbMax0,btlVector3(1,1,1));
				m_dispatchInfo.m_debugDraw->drawAabb(aabbMin1,aabbMax1,btlVector3(1,1,1));
			}
#endif

			if (m_resultOut->getBody0Internal() == m_compoundColObjWrap->getCollisionObject())
			{
				m_resultOut->setBody0Wrap(tmpWrap);
			} else
			{
				m_resultOut->setBody1Wrap(tmpWrap);
			}
			
		}
	}
	void		Process(const btlDbvtNode* leaf)
	{
		int index = leaf->dataAsInt;

		const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(m_compoundColObjWrap->getCollisionShape());
		const btlCollisionShape* childShape = compoundShape->getChildShape(index);

#if 0
		if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw->getDebugMode() & btlIDebugDraw::DBG_DrawAabb))
		{
			btlVector3 worldAabbMin,worldAabbMax;
			btlTransform	orgTrans = m_compoundColObjWrap->getWorldTransform();
			btlTransformAabb(leaf->volume.Mins(),leaf->volume.Maxs(),0.,orgTrans,worldAabbMin,worldAabbMax);
			m_dispatchInfo.m_debugDraw->drawAabb(worldAabbMin,worldAabbMax,btlVector3(1,0,0));
		}
#endif

		ProcessChildShape(childShape,index);

	}
};






void btlCompoundCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	const btlCollisionObjectWrapper* colObjWrap = m_isSwapped? body1Wrap : body0Wrap;
	const btlCollisionObjectWrapper* otherObjWrap = m_isSwapped? body0Wrap : body1Wrap;

	btlAssert (colObjWrap->getCollisionShape()->isCompound());
	const btlCompoundShape* compoundShape = static_cast<const btlCompoundShape*>(colObjWrap->getCollisionShape());

	///btCompoundShape might have changed:
	////make sure the internal child collision algorithm caches are still valid
	if (compoundShape->getUpdateRevision() != m_compoundShapeRevision)
	{
		///clear and update all
		removeChildAlgorithms();
		
		preallocateChildAlgorithms(body0Wrap,body1Wrap);
		m_compoundShapeRevision = compoundShape->getUpdateRevision();
	}

	if (m_childCollisionAlgorithms.size()==0)
		return;
    
	const btlDbvt* tree = compoundShape->getDynamicAabbTree();
	//use a dynamic aabb tree to cull potential child-overlaps
	btlCompoundLeafCallback  callback(colObjWrap,otherObjWrap,m_dispatcher,dispatchInfo,resultOut,&m_childCollisionAlgorithms[0],m_sharedManifold);

	///we need to refresh all contact manifolds
	///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
	///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
	{
		int i;
		btlManifoldArray manifoldArray;
		for (i=0;i<m_childCollisionAlgorithms.size();i++)
		{
			if (m_childCollisionAlgorithms[i])
			{
				m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
				for (int m=0;m<manifoldArray.size();m++)
				{
					if (manifoldArray[m]->getNumContacts())
					{
						resultOut->setPersistentManifold(manifoldArray[m]);
						resultOut->refreshContactPoints();
						resultOut->setPersistentManifold(0);//??necessary?
					}
				}
				manifoldArray.resize(0);
			}
		}
	}

	if (tree)
	{

		btlVector3 localAabbMin,localAabbMax;
		btlTransform otherInCompoundSpace;
		otherInCompoundSpace = colObjWrap->getWorldTransform().inverse() * otherObjWrap->getWorldTransform();
		otherObjWrap->getCollisionShape()->getAabb(otherInCompoundSpace,localAabbMin,localAabbMax);

		const ATTRIBUTE_ALIGNED16(btlDbvtVolume)	bounds=btlDbvtVolume::FromMM(localAabbMin,localAabbMax);
		//process all children, that overlap with  the given AABB bounds
		tree->collideTV(tree->m_root,bounds,callback);

	} else
	{
		//iterate over all children, perform an AABB check inside ProcessChildShape
		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		for (i=0;i<numChildren;i++)
		{
			callback.ProcessChildShape(compoundShape->getChildShape(i),i);
		}
	}

	{
				//iterate over all children, perform an AABB check inside ProcessChildShape
		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		btlManifoldArray	manifoldArray;
        const btlCollisionShape* childShape = 0;
        btlTransform	orgTrans;
        
        btlTransform	newChildWorldTrans;
        btlVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;        
        
		for (i=0;i<numChildren;i++)
		{
			if (m_childCollisionAlgorithms[i])
			{
				childShape = compoundShape->getChildShape(i);
			//if not longer overlapping, remove the algorithm
				orgTrans = colObjWrap->getWorldTransform();
                
				const btlTransform& childTrans = compoundShape->getChildTransform(i);
                newChildWorldTrans = orgTrans*childTrans ;

				//perform an AABB check first
				childShape->getAabb(newChildWorldTrans,aabbMin0,aabbMax0);
				otherObjWrap->getCollisionShape()->getAabb(otherObjWrap->getWorldTransform(),aabbMin1,aabbMax1);

				if (!TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
				{
					m_childCollisionAlgorithms[i]->~btlCollisionAlgorithm();
					m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
					m_childCollisionAlgorithms[i] = 0;
				}
			}
		}
	}
}

btlScalar	btlCompoundCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	btlAssert(0);
	//needs to be fixed, using btCollisionObjectWrapper and NOT modifying internal data structures
	btlCollisionObject* colObj = m_isSwapped? body1 : body0;
	btlCollisionObject* otherObj = m_isSwapped? body0 : body1;

	btlAssert (colObj->getCollisionShape()->isCompound());
	
	btlCompoundShape* compoundShape = static_cast<btlCompoundShape*>(colObj->getCollisionShape());

	//We will use the OptimizedBVH, AABB tree to cull potential child-overlaps
	//If both proxies are Compound, we will deal with that directly, by performing sequential/parallel tree traversals
	//given Proxy0 and Proxy1, if both have a tree, Tree0 and Tree1, this means:
	//determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
	//then use each overlapping node AABB against Tree0
	//and vise versa.

	btlScalar hitFraction = btlScalar(1.);

	int numChildren = m_childCollisionAlgorithms.size();
	int i;
    btlTransform	orgTrans;
    btlScalar frac;
	for (i=0;i<numChildren;i++)
	{
		//btCollisionShape* childShape = compoundShape->getChildShape(i);

		//backup
        orgTrans = colObj->getWorldTransform();
	
		const btlTransform& childTrans = compoundShape->getChildTransform(i);
		//btTransform	newChildWorldTrans = orgTrans*childTrans ;
		colObj->setWorldTransform( orgTrans*childTrans );

		//btCollisionShape* tmpShape = colObj->getCollisionShape();
		//colObj->internalSetTemporaryCollisionShape( childShape );
        frac = m_childCollisionAlgorithms[i]->calculateTimeOfImpact(colObj,otherObj,dispatchInfo,resultOut);
		if (frac<hitFraction)
		{
			hitFraction = frac;
		}
		//revert back
		//colObj->internalSetTemporaryCollisionShape( tmpShape);
		colObj->setWorldTransform( orgTrans);
	}
	return hitFraction;

}



