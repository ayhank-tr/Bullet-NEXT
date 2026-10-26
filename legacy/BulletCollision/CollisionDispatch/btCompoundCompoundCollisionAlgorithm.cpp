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

#include "btCompoundCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAabbUtil2.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"


btlShapePairCallback btl_gCompoundCompoundChildShapePairCallback = 0;

btlCompoundCompoundCollisionAlgorithm::btlCompoundCompoundCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped)
:btlCompoundCollisionAlgorithm(ci,body0Wrap,body1Wrap,isSwapped)
{

	void* ptr = btlAlignedAlloc(sizeof(btlHashedSimplePairCache),16);
	m_childCollisionAlgorithmCache= new(ptr) btlHashedSimplePairCache();

	const btlCollisionObjectWrapper* col0ObjWrap = body0Wrap;
	btlAssert (col0ObjWrap->getCollisionShape()->isCompound());

	const btlCollisionObjectWrapper* col1ObjWrap = body1Wrap;
	btlAssert (col1ObjWrap->getCollisionShape()->isCompound());
	
	const btlCompoundShape* compoundShape0 = static_cast<const btlCompoundShape*>(col0ObjWrap->getCollisionShape());
	m_compoundShapeRevision0 = compoundShape0->getUpdateRevision();

	const btlCompoundShape* compoundShape1 = static_cast<const btlCompoundShape*>(col1ObjWrap->getCollisionShape());
	m_compoundShapeRevision1 = compoundShape1->getUpdateRevision();
	
	
}


btlCompoundCompoundCollisionAlgorithm::~btlCompoundCompoundCollisionAlgorithm()
{
	removeChildAlgorithms();
	m_childCollisionAlgorithmCache->~btlHashedSimplePairCache();
	btlAlignedFree(m_childCollisionAlgorithmCache);
}

void	btlCompoundCompoundCollisionAlgorithm::getAllContactManifolds(btlManifoldArray&	manifoldArray)
{
	int i;
	btlSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();
	for (i=0;i<pairs.size();i++)
	{
		if (pairs[i].m_userPointer)
		{
			
			((btlCollisionAlgorithm*)pairs[i].m_userPointer)->getAllContactManifolds(manifoldArray);
		}
	}
}


void	btlCompoundCompoundCollisionAlgorithm::removeChildAlgorithms()
{
	btlSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();

	int numChildren = pairs.size();
	int i;
	for (i=0;i<numChildren;i++)
	{
		if (pairs[i].m_userPointer)
		{
			btlCollisionAlgorithm* algo = (btlCollisionAlgorithm*) pairs[i].m_userPointer;
			algo->~btlCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm(algo);
		}
	}
	m_childCollisionAlgorithmCache->removeAllPairs();
}

struct	btlCompoundCompoundLeafCallback : btlDbvt::ICollide
{
	int m_numOverlapPairs;


	const btlCollisionObjectWrapper* m_compound0ColObjWrap;
	const btlCollisionObjectWrapper* m_compound1ColObjWrap;
	btlDispatcher* m_dispatcher;
	const btlDispatcherInfo& m_dispatchInfo;
	btlManifoldResult*	m_resultOut;
	
	
	class btlHashedSimplePairCache*	m_childCollisionAlgorithmCache;
	
	btlPersistentManifold*	m_sharedManifold;
	
	btlCompoundCompoundLeafCallback (const btlCollisionObjectWrapper* compound1ObjWrap,
									const btlCollisionObjectWrapper* compound0ObjWrap,
									btlDispatcher* dispatcher,
									const btlDispatcherInfo& dispatchInfo,
									btlManifoldResult*	resultOut,
									btlHashedSimplePairCache* childAlgorithmsCache,
									btlPersistentManifold*	sharedManifold)
		:m_numOverlapPairs(0),m_compound0ColObjWrap(compound1ObjWrap),m_compound1ColObjWrap(compound0ObjWrap),m_dispatcher(dispatcher),m_dispatchInfo(dispatchInfo),m_resultOut(resultOut),
		m_childCollisionAlgorithmCache(childAlgorithmsCache),
		m_sharedManifold(sharedManifold)
	{

	}



	
	void		Process(const btlDbvtNode* leaf0,const btlDbvtNode* leaf1)
	{
		m_numOverlapPairs++;


		int childIndex0 = leaf0->dataAsInt;
		int childIndex1 = leaf1->dataAsInt;
		

		btlAssert(childIndex0>=0);
		btlAssert(childIndex1>=0);


		const btlCompoundShape* compoundShape0 = static_cast<const btlCompoundShape*>(m_compound0ColObjWrap->getCollisionShape());
		btlAssert(childIndex0<compoundShape0->getNumChildShapes());

		const btlCompoundShape* compoundShape1 = static_cast<const btlCompoundShape*>(m_compound1ColObjWrap->getCollisionShape());
		btlAssert(childIndex1<compoundShape1->getNumChildShapes());

		const btlCollisionShape* childShape0 = compoundShape0->getChildShape(childIndex0);
		const btlCollisionShape* childShape1 = compoundShape1->getChildShape(childIndex1);

		//backup
		btlTransform	orgTrans0 = m_compound0ColObjWrap->getWorldTransform();
		const btlTransform& childTrans0 = compoundShape0->getChildTransform(childIndex0);
		btlTransform	newChildWorldTrans0 = orgTrans0*childTrans0 ;
		
		btlTransform	orgTrans1 = m_compound1ColObjWrap->getWorldTransform();
		const btlTransform& childTrans1 = compoundShape1->getChildTransform(childIndex1);
		btlTransform	newChildWorldTrans1 = orgTrans1*childTrans1 ;
		

		//perform an AABB check first
		btlVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;
		childShape0->getAabb(newChildWorldTrans0,aabbMin0,aabbMax0);
		childShape1->getAabb(newChildWorldTrans1,aabbMin1,aabbMax1);
		
		if (btl_gCompoundCompoundChildShapePairCallback)
		{
			if (!btl_gCompoundCompoundChildShapePairCallback(childShape0,childShape1))
				return;
		}

		if (TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
		{
			btlCollisionObjectWrapper compoundWrap0(this->m_compound0ColObjWrap,childShape0, m_compound0ColObjWrap->getCollisionObject(),newChildWorldTrans0,-1,childIndex0);
			btlCollisionObjectWrapper compoundWrap1(this->m_compound1ColObjWrap,childShape1,m_compound1ColObjWrap->getCollisionObject(),newChildWorldTrans1,-1,childIndex1);
			

			btlSimplePair* pair = m_childCollisionAlgorithmCache->findPair(childIndex0,childIndex1);

			btlCollisionAlgorithm* colAlgo = 0;

			if (pair)
			{
				colAlgo = (btlCollisionAlgorithm*)pair->m_userPointer;
				
			} else
			{
				colAlgo = m_dispatcher->findAlgorithm(&compoundWrap0,&compoundWrap1,m_sharedManifold);
				pair = m_childCollisionAlgorithmCache->addOverlappingPair(childIndex0,childIndex1);
				btlAssert(pair);
				pair->m_userPointer = colAlgo;
			}

			btlAssert(colAlgo);
						
			const btlCollisionObjectWrapper* tmpWrap0 = 0;
			const btlCollisionObjectWrapper* tmpWrap1 = 0;

			tmpWrap0 = m_resultOut->getBody0Wrap();
			tmpWrap1 = m_resultOut->getBody1Wrap();

			m_resultOut->setBody0Wrap(&compoundWrap0);
			m_resultOut->setBody1Wrap(&compoundWrap1);

			m_resultOut->setShapeIdentifiersA(-1,childIndex0);
			m_resultOut->setShapeIdentifiersB(-1,childIndex1);


			colAlgo->processCollision(&compoundWrap0,&compoundWrap1,m_dispatchInfo,m_resultOut);
			
			m_resultOut->setBody0Wrap(tmpWrap0);
			m_resultOut->setBody1Wrap(tmpWrap1);
			


		}
	}
};


static DBVT_INLINE bool		MyIntersect(	const btlDbvtAabbMm& a,
								  const btlDbvtAabbMm& b, const btlTransform& xform)
{
	btlVector3 newmin,newmax;
	btlTransformAabb(b.Mins(),b.Maxs(),0.f,xform,newmin,newmax);
	btlDbvtAabbMm newb = btlDbvtAabbMm::FromMM(newmin,newmax);
	return Intersect(a,newb);
}


static inline void		MycollideTT(	const btlDbvtNode* root0,
								  const btlDbvtNode* root1,
								  const btlTransform& xform,
								  btlCompoundCompoundLeafCallback* callback)
{

		if(root0&&root1)
		{
			int								depth=1;
			int								treshold=btlDbvt::DOUBLE_STACKSIZE-4;
			btlAlignedObjectArray<btlDbvt::sStkNN>	stkStack;
			stkStack.resize(btlDbvt::DOUBLE_STACKSIZE);
			stkStack[0]=btlDbvt::sStkNN(root0,root1);
			do	{
				btlDbvt::sStkNN	p=stkStack[--depth];
				if(MyIntersect(p.a->volume,p.b->volume,xform))
				{
					if(depth>treshold)
					{
						stkStack.resize(stkStack.size()*2);
						treshold=stkStack.size()-4;
					}
					if(p.a->isinternal())
					{
						if(p.b->isinternal())
						{					
							stkStack[depth++]=btlDbvt::sStkNN(p.a->childs[0],p.b->childs[0]);
							stkStack[depth++]=btlDbvt::sStkNN(p.a->childs[1],p.b->childs[0]);
							stkStack[depth++]=btlDbvt::sStkNN(p.a->childs[0],p.b->childs[1]);
							stkStack[depth++]=btlDbvt::sStkNN(p.a->childs[1],p.b->childs[1]);
						}
						else
						{
							stkStack[depth++]=btlDbvt::sStkNN(p.a->childs[0],p.b);
							stkStack[depth++]=btlDbvt::sStkNN(p.a->childs[1],p.b);
						}
					}
					else
					{
						if(p.b->isinternal())
						{
							stkStack[depth++]=btlDbvt::sStkNN(p.a,p.b->childs[0]);
							stkStack[depth++]=btlDbvt::sStkNN(p.a,p.b->childs[1]);
						}
						else
						{
							callback->Process(p.a,p.b);
						}
					}
				}
			} while(depth);
		}
}

void btlCompoundCompoundCollisionAlgorithm::processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{

	const btlCollisionObjectWrapper* col0ObjWrap = body0Wrap;
	const btlCollisionObjectWrapper* col1ObjWrap= body1Wrap;

	btlAssert (col0ObjWrap->getCollisionShape()->isCompound());
	btlAssert (col1ObjWrap->getCollisionShape()->isCompound());
	const btlCompoundShape* compoundShape0 = static_cast<const btlCompoundShape*>(col0ObjWrap->getCollisionShape());
	const btlCompoundShape* compoundShape1 = static_cast<const btlCompoundShape*>(col1ObjWrap->getCollisionShape());

	const btlDbvt* tree0 = compoundShape0->getDynamicAabbTree();
	const btlDbvt* tree1 = compoundShape1->getDynamicAabbTree();
	if (!tree0 || !tree1)
	{
		return btlCompoundCollisionAlgorithm::processCollision(body0Wrap,body1Wrap,dispatchInfo,resultOut);
	}
	///btCompoundShape might have changed:
	////make sure the internal child collision algorithm caches are still valid
	if ((compoundShape0->getUpdateRevision() != m_compoundShapeRevision0) || (compoundShape1->getUpdateRevision() != m_compoundShapeRevision1))
	{
		///clear all
		removeChildAlgorithms();
		m_compoundShapeRevision0 = compoundShape0->getUpdateRevision();
		m_compoundShapeRevision1 = compoundShape1->getUpdateRevision();

	}


	///we need to refresh all contact manifolds
	///note that we should actually recursively traverse all children, btCompoundShape can nested more then 1 level deep
	///so we should add a 'refreshManifolds' in the btCollisionAlgorithm
	{
		int i;
		btlManifoldArray manifoldArray;
		btlSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();
		for (i=0;i<pairs.size();i++)
		{
			if (pairs[i].m_userPointer)
			{
				btlCollisionAlgorithm* algo = (btlCollisionAlgorithm*) pairs[i].m_userPointer;
				algo->getAllContactManifolds(manifoldArray);
				for (int m=0;m<manifoldArray.size();m++)
				{
					if (manifoldArray[m]->getNumContacts())
					{
						resultOut->setPersistentManifold(manifoldArray[m]);
						resultOut->refreshContactPoints();
						resultOut->setPersistentManifold(0);
					}
				}
				manifoldArray.resize(0);
			}
		}
	}


	

	btlCompoundCompoundLeafCallback callback(col0ObjWrap,col1ObjWrap,this->m_dispatcher,dispatchInfo,resultOut,this->m_childCollisionAlgorithmCache,m_sharedManifold);


	const btlTransform	xform=col0ObjWrap->getWorldTransform().inverse()*col1ObjWrap->getWorldTransform();
	MycollideTT(tree0->m_root,tree1->m_root,xform,&callback);

	//printf("#compound-compound child/leaf overlap =%d                      \r",callback.m_numOverlapPairs);

	//remove non-overlapping child pairs

	{
		btlAssert(m_removePairs.size()==0);

		//iterate over all children, perform an AABB check inside ProcessChildShape
		btlSimplePairArray& pairs = m_childCollisionAlgorithmCache->getOverlappingPairArray();
		
		int i;
		btlManifoldArray	manifoldArray;
        
		

        
        
        btlVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;        
        
		for (i=0;i<pairs.size();i++)
		{
			if (pairs[i].m_userPointer)
			{
				btlCollisionAlgorithm* algo = (btlCollisionAlgorithm*)pairs[i].m_userPointer;

				{
					btlTransform	orgTrans0;
					const btlCollisionShape* childShape0 = 0;
					
					btlTransform	newChildWorldTrans0;
					btlTransform	orgInterpolationTrans0;
					childShape0 = compoundShape0->getChildShape(pairs[i].m_indexA);
					orgTrans0 = col0ObjWrap->getWorldTransform();
					orgInterpolationTrans0 = col0ObjWrap->getWorldTransform();
					const btlTransform& childTrans0 = compoundShape0->getChildTransform(pairs[i].m_indexA);
					newChildWorldTrans0 = orgTrans0*childTrans0 ;
					childShape0->getAabb(newChildWorldTrans0,aabbMin0,aabbMax0);
				}

				{
					btlTransform	orgInterpolationTrans1;
					const btlCollisionShape* childShape1 = 0;
					btlTransform	orgTrans1;
					btlTransform	newChildWorldTrans1;

					childShape1 = compoundShape1->getChildShape(pairs[i].m_indexB);
					orgTrans1 = col1ObjWrap->getWorldTransform();
					orgInterpolationTrans1 = col1ObjWrap->getWorldTransform();
					const btlTransform& childTrans1 = compoundShape1->getChildTransform(pairs[i].m_indexB);
					newChildWorldTrans1 = orgTrans1*childTrans1 ;
					childShape1->getAabb(newChildWorldTrans1,aabbMin1,aabbMax1);
				}
				
				

				if (!TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
				{
					algo->~btlCollisionAlgorithm();
					m_dispatcher->freeCollisionAlgorithm(algo);
					m_removePairs.push_back(btlSimplePair(pairs[i].m_indexA,pairs[i].m_indexB));
				}
			}
		}
		for (int i=0;i<m_removePairs.size();i++)
		{
			m_childCollisionAlgorithmCache->removeOverlappingPair(m_removePairs[i].m_indexA,m_removePairs[i].m_indexB);
		}
		m_removePairs.clear();
	}

}

btlScalar	btlCompoundCompoundCollisionAlgorithm::calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	btlAssert(0);
	return 0.f;

}



