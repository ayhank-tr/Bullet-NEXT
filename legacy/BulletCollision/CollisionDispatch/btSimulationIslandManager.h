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

#ifndef BT_SIMULATION_ISLAND_MANAGER_H
#define BT_SIMULATION_ISLAND_MANAGER_H

#include "BulletCollision/CollisionDispatch/btUnionFind.h"
#include "btCollisionCreateFunc.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btCollisionObject.h"

class btlCollisionObject;
class btlCollisionWorld;
class btlDispatcher;
class btlPersistentManifold;


///SimulationIslandManager creates and handles simulation islands, using btUnionFind
class btlSimulationIslandManager
{
	btlUnionFind m_unionFind;

	btlAlignedObjectArray<btlPersistentManifold*>  m_islandmanifold;
	btlAlignedObjectArray<btlCollisionObject* >  m_islandBodies;
	
	bool m_splitIslands;
	
public:
	btlSimulationIslandManager();
	virtual ~btlSimulationIslandManager();


	void initUnionFind(int n);	
	
		
	btlUnionFind& getUnionFind() { return m_unionFind;}

	virtual	void	updateActivationState(btlCollisionWorld* colWorld,btlDispatcher* dispatcher);
	virtual	void	storeIslandActivationState(btlCollisionWorld* world);


	void	findUnions(btlDispatcher* dispatcher,btlCollisionWorld* colWorld);

	

	struct	IslandCallback
	{
		virtual ~IslandCallback() {};

		virtual	void	processIsland(btlCollisionObject** bodies,int numBodies,class btlPersistentManifold**	manifolds,int numManifolds, int islandId) = 0;
	};

	void	buildAndProcessIslands(btlDispatcher* dispatcher,btlCollisionWorld* collisionWorld, IslandCallback* callback);

	void buildIslands(btlDispatcher* dispatcher,btlCollisionWorld* colWorld);

	bool getSplitIslands()
	{
		return m_splitIslands;
	}
	void setSplitIslands(bool doSplitIslands)
	{
		m_splitIslands = doSplitIslands;
	}

};

#endif //BT_SIMULATION_ISLAND_MANAGER_H

