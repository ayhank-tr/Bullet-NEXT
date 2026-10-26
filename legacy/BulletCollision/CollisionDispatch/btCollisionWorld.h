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


/**
 * @page Bullet Documentation
 *
 * @section intro_sec Introduction
 * Bullet is a Collision Detection and Rigid Body Dynamics Library. The Library is Open Source and free for commercial use, under the ZLib license ( http://opensource.org/licenses/zlib-license.php ).
 *
 * The main documentation is Bullet_User_Manual.pdf, included in the source code distribution.
 * There is the Physics Forum for feedback and general Collision Detection and Physics discussions.
 * Please visit http://www.bulletphysics.org
 *
 * @section install_sec Installation
 *
 * @subsection step1 Step 1: Download
 * You can download the Bullet Physics Library from the github repository: https://github.com/bulletphysics/bullet3/releases 
 *
 * @subsection step2 Step 2: Building
 * Bullet has multiple build systems, including premake, cmake and autotools. Premake and cmake support all platforms.
 * Premake is included in the Bullet/build folder for Windows, Mac OSX and Linux. 
 * Under Windows you can click on Bullet/build/vs2010.bat to create Microsoft Visual Studio projects. 
 * On Mac OSX and Linux you can open a terminal and generate Makefile, codeblocks or Xcode4 projects:
 * cd Bullet/build
 * ./premake4_osx gmake or ./premake4_linux gmake or ./premake4_linux64 gmake or (for Mac) ./premake4_osx xcode4
 * cd Bullet/build/gmake
 * make
 * 
 * An alternative to premake is cmake. You can download cmake from http://www.cmake.org
 * cmake can autogenerate projectfiles for Microsoft Visual Studio, Apple Xcode, KDevelop and Unix Makefiles.
 * The easiest is to run the CMake cmake-gui graphical user interface and choose the options and generate projectfiles.
 * You can also use cmake in the command-line. Here are some examples for various platforms:
 * cmake . -G "Visual Studio 9 2008"
 * cmake . -G Xcode
 * cmake . -G "Unix Makefiles"
 * Although cmake is recommended, you can also use autotools for UNIX: ./autogen.sh ./configure to create a Makefile and then run make.
 * 
 * @subsection step3 Step 3: Testing demos
 * Try to run and experiment with BasicDemo executable as a starting point.
 * Bullet can be used in several ways, as Full Rigid Body simulation, as Collision Detector Library or Low Level / Snippets like the GJK Closest Point calculation.
 * The Dependencies can be seen in this documentation under Directories
 * 
 * @subsection step4 Step 4: Integrating in your application, full Rigid Body and Soft Body simulation
 * Check out BasicDemo how to create a btDynamicsWorld, btRigidBody and btCollisionShape, Stepping the simulation and synchronizing your graphics object transform.
 * Check out SoftDemo how to use soft body dynamics, using btSoftRigidDynamicsWorld.
 * @subsection step5 Step 5 : Integrate the Collision Detection Library (without Dynamics and other Extras)
 * Bullet Collision Detection can also be used without the Dynamics/Extras.
 * Check out btCollisionWorld and btCollisionObject, and the CollisionInterfaceDemo.
 * @subsection step6 Step 6 : Use Snippets like the GJK Closest Point calculation.
 * Bullet has been designed in a modular way keeping dependencies to a minimum. The ConvexHullDistance demo demonstrates direct use of btGjkPairDetector.
 *
 * @section copyright Copyright
 * For up-to-data information and copyright and contributors list check out the Bullet_User_Manual.pdf
 * 
 */
 
 

#ifndef BT_COLLISION_WORLD_H
#define BT_COLLISION_WORLD_H

class btlCollisionShape;
class btlConvexShape;
class btlBroadphaseInterface;
class btlSerializer;

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "btCollisionObject.h"
#include "btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "LinearMath/btAlignedObjectArray.h"

///CollisionWorld is interface and container for the collision detection
class btlCollisionWorld
{

	
protected:

	btlAlignedObjectArray<btlCollisionObject*>	m_collisionObjects;
	
	btlDispatcher*	m_dispatcher1;

	btlDispatcherInfo	m_dispatchInfo;

	btlBroadphaseInterface*	m_broadphasePairCache;

	btlIDebugDraw*	m_debugDrawer;

	///m_forceUpdateAllAabbs can be set to false as an optimization to only update active object AABBs
	///it is true by default, because it is error-prone (setting the position of static objects wouldn't update their AABB)
	bool m_forceUpdateAllAabbs;

	void	serializeCollisionObjects(btlSerializer* serializer);

public:

	//this constructor doesn't own the dispatcher and paircache/broadphase
	btlCollisionWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* broadphasePairCache, btlCollisionConfiguration* collisionConfiguration);

	virtual ~btlCollisionWorld();

	void	setBroadphase(btlBroadphaseInterface*	pairCache)
	{
		m_broadphasePairCache = pairCache;
	}

	const btlBroadphaseInterface*	getBroadphase() const
	{
		return m_broadphasePairCache;
	}

	btlBroadphaseInterface*	getBroadphase()
	{
		return m_broadphasePairCache;
	}

	btlOverlappingPairCache*	getPairCache()
	{
		return m_broadphasePairCache->getOverlappingPairCache();
	}


	btlDispatcher*	getDispatcher()
	{
		return m_dispatcher1;
	}

	const btlDispatcher*	getDispatcher() const
	{
		return m_dispatcher1;
	}

	void	updateSingleAabb(btlCollisionObject* colObj);

	virtual void	updateAabbs();

	///the computeOverlappingPairs is usually already called by performDiscreteCollisionDetection (or stepSimulation)
	///it can be useful to use if you perform ray tests without collision detection/simulation
	virtual void	computeOverlappingPairs();

	
	virtual void	setDebugDrawer(btlIDebugDraw*	debugDrawer)
	{
			m_debugDrawer = debugDrawer;
	}

	virtual btlIDebugDraw*	getDebugDrawer()
	{
		return m_debugDrawer;
	}

	virtual void	debugDrawWorld();

	virtual void debugDrawObject(const btlTransform& worldTransform, const btlCollisionShape* shape, const btlVector3& color);


	///LocalShapeInfo gives extra information for complex shapes
	///Currently, only btTriangleMeshShape is available, so it just contains triangleIndex and subpart
	struct	LocalShapeInfo
	{
		int	m_shapePart;
		int	m_triangleIndex;
		
		//const btCollisionShape*	m_shapeTemp;
		//const btTransform*	m_shapeLocalTransform;
	};

	struct	LocalRayResult
	{
		LocalRayResult(const btlCollisionObject*	collisionObject, 
			LocalShapeInfo*	localShapeInfo,
			const btlVector3&		hitNormalLocal,
			btlScalar hitFraction)
		:m_collisionObject(collisionObject),
		m_localShapeInfo(localShapeInfo),
		m_hitNormalLocal(hitNormalLocal),
		m_hitFraction(hitFraction)
		{
		}

		const btlCollisionObject*		m_collisionObject;
		LocalShapeInfo*			m_localShapeInfo;
		btlVector3				m_hitNormalLocal;
		btlScalar				m_hitFraction;

	};

	///RayResultCallback is used to report new raycast results
	struct	RayResultCallback
	{
		btlScalar	m_closestHitFraction;
		const btlCollisionObject*		m_collisionObject;
		short int	m_collisionFilterGroup;
		short int	m_collisionFilterMask;
		//@BP Mod - Custom flags, currently used to enable backface culling on tri-meshes, see btRaycastCallback.h. Apply any of the EFlags defined there on m_flags here to invoke.
		unsigned int m_flags;

		virtual ~RayResultCallback()
		{
		}
		bool	hasHit() const
		{
			return (m_collisionObject != 0);
		}

		RayResultCallback()
			:m_closestHitFraction(btlScalar(1.)),
			m_collisionObject(0),
			m_collisionFilterGroup(btlBroadphaseProxy::DefaultFilter),
			m_collisionFilterMask(btlBroadphaseProxy::AllFilter),
			//@BP Mod
			m_flags(0)
		{
		}

		virtual bool needsCollision(btlBroadphaseProxy* proxy0) const
		{
			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}


		virtual	btlScalar	addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace) = 0;
	};

	struct	ClosestRayResultCallback : public RayResultCallback
	{
		ClosestRayResultCallback(const btlVector3&	rayFromWorld,const btlVector3&	rayToWorld)
		:m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld)
		{
		}

		btlVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
		btlVector3	m_rayToWorld;

		btlVector3	m_hitNormalWorld;
		btlVector3	m_hitPointWorld;
			
		virtual	btlScalar	addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace)
		{
			//caller already does the filter on the m_closestHitFraction
			btlAssert(rayResult.m_hitFraction <= m_closestHitFraction);
			
			m_closestHitFraction = rayResult.m_hitFraction;
			m_collisionObject = rayResult.m_collisionObject;
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = rayResult.m_hitNormalLocal;
			} else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
			}
			m_hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
			return rayResult.m_hitFraction;
		}
	};

	struct	AllHitsRayResultCallback : public RayResultCallback
	{
		AllHitsRayResultCallback(const btlVector3&	rayFromWorld,const btlVector3&	rayToWorld)
		:m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld)
		{
		}

		btlAlignedObjectArray<const btlCollisionObject*>		m_collisionObjects;

		btlVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
		btlVector3	m_rayToWorld;

		btlAlignedObjectArray<btlVector3>	m_hitNormalWorld;
		btlAlignedObjectArray<btlVector3>	m_hitPointWorld;
		btlAlignedObjectArray<btlScalar> m_hitFractions;
			
		virtual	btlScalar	addSingleResult(LocalRayResult& rayResult,bool normalInWorldSpace)
		{
			m_collisionObject = rayResult.m_collisionObject;
			m_collisionObjects.push_back(rayResult.m_collisionObject);
			btlVector3 hitNormalWorld;
			if (normalInWorldSpace)
			{
				hitNormalWorld = rayResult.m_hitNormalLocal;
			} else
			{
				///need to transform normal into worldspace
				hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
			}
			m_hitNormalWorld.push_back(hitNormalWorld);
			btlVector3 hitPointWorld;
			hitPointWorld.setInterpolate3(m_rayFromWorld,m_rayToWorld,rayResult.m_hitFraction);
			m_hitPointWorld.push_back(hitPointWorld);
			m_hitFractions.push_back(rayResult.m_hitFraction);
			return m_closestHitFraction;
		}
	};


	struct LocalConvexResult
	{
		LocalConvexResult(const btlCollisionObject*	hitCollisionObject, 
			LocalShapeInfo*	localShapeInfo,
			const btlVector3&		hitNormalLocal,
			const btlVector3&		hitPointLocal,
			btlScalar hitFraction
			)
		:m_hitCollisionObject(hitCollisionObject),
		m_localShapeInfo(localShapeInfo),
		m_hitNormalLocal(hitNormalLocal),
		m_hitPointLocal(hitPointLocal),
		m_hitFraction(hitFraction)
		{
		}

		const btlCollisionObject*		m_hitCollisionObject;
		LocalShapeInfo*			m_localShapeInfo;
		btlVector3				m_hitNormalLocal;
		btlVector3				m_hitPointLocal;
		btlScalar				m_hitFraction;
	};

	///RayResultCallback is used to report new raycast results
	struct	ConvexResultCallback
	{
		btlScalar	m_closestHitFraction;
		short int	m_collisionFilterGroup;
		short int	m_collisionFilterMask;
		
		ConvexResultCallback()
			:m_closestHitFraction(btlScalar(1.)),
			m_collisionFilterGroup(btlBroadphaseProxy::DefaultFilter),
			m_collisionFilterMask(btlBroadphaseProxy::AllFilter)
		{
		}

		virtual ~ConvexResultCallback()
		{
		}
		
		bool	hasHit() const
		{
			return (m_closestHitFraction < btlScalar(1.));
		}

		

		virtual bool needsCollision(btlBroadphaseProxy* proxy0) const
		{
			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}

		virtual	btlScalar	addSingleResult(LocalConvexResult& convexResult,bool normalInWorldSpace) = 0;
	};

	struct	ClosestConvexResultCallback : public ConvexResultCallback
	{
		ClosestConvexResultCallback(const btlVector3&	convexFromWorld,const btlVector3&	convexToWorld)
		:m_convexFromWorld(convexFromWorld),
		m_convexToWorld(convexToWorld),
		m_hitCollisionObject(0)
		{
		}

		btlVector3	m_convexFromWorld;//used to calculate hitPointWorld from hitFraction
		btlVector3	m_convexToWorld;

		btlVector3	m_hitNormalWorld;
		btlVector3	m_hitPointWorld;
		const btlCollisionObject*	m_hitCollisionObject;
		
		virtual	btlScalar	addSingleResult(LocalConvexResult& convexResult,bool normalInWorldSpace)
		{
//caller already does the filter on the m_closestHitFraction
			btlAssert(convexResult.m_hitFraction <= m_closestHitFraction);
						
			m_closestHitFraction = convexResult.m_hitFraction;
			m_hitCollisionObject = convexResult.m_hitCollisionObject;
			if (normalInWorldSpace)
			{
				m_hitNormalWorld = convexResult.m_hitNormalLocal;
			} else
			{
				///need to transform normal into worldspace
				m_hitNormalWorld = m_hitCollisionObject->getWorldTransform().getBasis()*convexResult.m_hitNormalLocal;
			}
			m_hitPointWorld = convexResult.m_hitPointLocal;
			return convexResult.m_hitFraction;
		}
	};

	///ContactResultCallback is used to report contact points
	struct	ContactResultCallback
	{
		short int	m_collisionFilterGroup;
		short int	m_collisionFilterMask;
		
		ContactResultCallback()
			:m_collisionFilterGroup(btlBroadphaseProxy::DefaultFilter),
			m_collisionFilterMask(btlBroadphaseProxy::AllFilter)
		{
		}

		virtual ~ContactResultCallback()
		{
		}
		
		virtual bool needsCollision(btlBroadphaseProxy* proxy0) const
		{
			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
			return collides;
		}

		virtual	btlScalar	addSingleResult(btlManifoldPoint& cp,	const btlCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btlCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) = 0;
	};



	int	getNumCollisionObjects() const
	{
		return int(m_collisionObjects.size());
	}

	/// rayTest performs a raycast on all objects in the btCollisionWorld, and calls the resultCallback
	/// This allows for several queries: first hit, all hits, any hit, dependent on the value returned by the callback.
	virtual void rayTest(const btlVector3& rayFromWorld, const btlVector3& rayToWorld, RayResultCallback& resultCallback) const; 

	/// convexTest performs a swept convex cast on all objects in the btCollisionWorld, and calls the resultCallback
	/// This allows for several queries: first hit, all hits, any hit, dependent on the value return by the callback.
	void    convexSweepTest (const btlConvexShape* castShape, const btlTransform& from, const btlTransform& to, ConvexResultCallback& resultCallback,  btlScalar allowedCcdPenetration = btlScalar(0.)) const;

	///contactTest performs a discrete collision test between colObj against all objects in the btCollisionWorld, and calls the resultCallback.
	///it reports one or more contact points for every overlapping object (including the one with deepest penetration)
	void	contactTest(btlCollisionObject* colObj, ContactResultCallback& resultCallback);

	///contactTest performs a discrete collision test between two collision objects and calls the resultCallback if overlap if detected.
	///it reports one or more contact points (including the one with deepest penetration)
	void	contactPairTest(btlCollisionObject* colObjA, btlCollisionObject* colObjB, ContactResultCallback& resultCallback);


	/// rayTestSingle performs a raycast call and calls the resultCallback. It is used internally by rayTest.
	/// In a future implementation, we consider moving the ray test as a virtual method in btCollisionShape.
	/// This allows more customization.
	static void	rayTestSingle(const btlTransform& rayFromTrans,const btlTransform& rayToTrans,
					  btlCollisionObject* collisionObject,
					  const btlCollisionShape* collisionShape,
					  const btlTransform& colObjWorldTransform,
					  RayResultCallback& resultCallback);

	static void	rayTestSingleInternal(const btlTransform& rayFromTrans,const btlTransform& rayToTrans,
					  const btlCollisionObjectWrapper* collisionObjectWrap,
					  RayResultCallback& resultCallback);

	/// objectQuerySingle performs a collision detection query and calls the resultCallback. It is used internally by rayTest.
	static void	objectQuerySingle(const btlConvexShape* castShape, const btlTransform& rayFromTrans,const btlTransform& rayToTrans,
					  btlCollisionObject* collisionObject,
					  const btlCollisionShape* collisionShape,
					  const btlTransform& colObjWorldTransform,
					  ConvexResultCallback& resultCallback, btlScalar	allowedPenetration);

	static void	objectQuerySingleInternal(const btlConvexShape* castShape,const btlTransform& convexFromTrans,const btlTransform& convexToTrans,
											const btlCollisionObjectWrapper* colObjWrap,
											ConvexResultCallback& resultCallback, btlScalar allowedPenetration);

	virtual void	addCollisionObject(btlCollisionObject* collisionObject,short int collisionFilterGroup=btlBroadphaseProxy::DefaultFilter,short int collisionFilterMask=btlBroadphaseProxy::AllFilter);

	btlCollisionObjectArray& getCollisionObjectArray()
	{
		return m_collisionObjects;
	}

	const btlCollisionObjectArray& getCollisionObjectArray() const
	{
		return m_collisionObjects;
	}


	virtual void	removeCollisionObject(btlCollisionObject* collisionObject);

	virtual void	performDiscreteCollisionDetection();

	btlDispatcherInfo& getDispatchInfo()
	{
		return m_dispatchInfo;
	}

	const btlDispatcherInfo& getDispatchInfo() const
	{
		return m_dispatchInfo;
	}
	
	bool	getForceUpdateAllAabbs() const
	{
		return m_forceUpdateAllAabbs;
	}
	void setForceUpdateAllAabbs( bool forceUpdateAllAabbs)
	{
		m_forceUpdateAllAabbs = forceUpdateAllAabbs;
	}

	///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (Bullet/Demos/SerializeDemo)
	virtual	void	serialize(btlSerializer* serializer);

};


#endif //BT_COLLISION_WORLD_H
