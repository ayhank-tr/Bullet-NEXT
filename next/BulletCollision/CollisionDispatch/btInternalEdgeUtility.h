
#ifndef BT_INTERNAL_EDGE_UTILITY_H
#define BT_INTERNAL_EDGE_UTILITY_H

#include "LinearMath/btHashMap.h"
#include "LinearMath/btVector3.h"

#include "BulletCollision/CollisionShapes/btTriangleInfoMap.h"

///The btInternalEdgeUtility helps to avoid or reduce artifacts due to wrong collision normals caused by internal edges.
///See also http://code.google.com/p/bullet/issues/detail?id=27

class btBvhTriangleMeshShape;
class btCollisionObject;
struct btCollisionObjectWrapper;
class btManifoldPoint;
class btIDebugDraw;
class btHeightfieldTerrainShape;

enum btInternalEdgeAdjustFlags
{
	BT_TRIANGLE_CONVEX_BACKFACE_MODE = 1,
	BT_TRIANGLE_CONCAVE_DOUBLE_SIDED = 2,  //double sided options are experimental, single sided is recommended
	BT_TRIANGLE_CONVEX_DOUBLE_SIDED = 4
};

///Call btGenerateInternalEdgeInfo to create triangle info, store in the shape 'userInfo'
void btGenerateInternalEdgeInfo(btBvhTriangleMeshShape* trimeshShape, btTriangleInfoMap* triangleInfoMap);

void btGenerateInternalEdgeInfo(btHeightfieldTerrainShape* trimeshShape, btTriangleInfoMap* triangleInfoMap);

/// Bullet NEXT (Stage 1): the edge data of one triangle (vertices in the shape's local space, as
/// the collision callbacks give them). From the map, or worked out from the heightfield's heights
/// when the map is a heightfield's computed one; then the result is written to scratch. Returns
/// 0 when the triangle has no data (no neighbour shares an edge), as a baked map would.
const btTriangleInfo* btNextFindTriangleInfo(const btTriangleInfoMap* map, const btVector3* triangleVertices,
											 int partId, int triangleIndex, btTriangleInfo& scratch);

/// Bullet NEXT (Stage 1): makes the map a heightfield's computed one (no bake) and gives it to the
/// heightfield. The heightfield must not have a map yet.
void btNextUseComputedEdgeInfo(btHeightfieldTerrainShape* heightfieldShape, btTriangleInfoMap* triangleInfoMap);

///Call the btFixMeshNormal to adjust the collision normal, using the triangle info map (generated using btGenerateInternalEdgeInfo)
///If this info map is missing, or the triangle is not store in this map, nothing will be done
void btAdjustInternalEdgeContacts(btManifoldPoint& cp, const btCollisionObjectWrapper* trimeshColObj0Wrap, const btCollisionObjectWrapper* otherColObj1Wrap, int partId0, int index0, int normalAdjustFlags = 0);

///Enable the BT_INTERNAL_EDGE_DEBUG_DRAW define and call btSetDebugDrawer, to get visual info to see if the internal edge utility works properly.
///If the utility doesn't work properly, you might have to adjust the threshold values in btTriangleInfoMap
//#define BT_INTERNAL_EDGE_DEBUG_DRAW

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
void btSetDebugDrawer(btIDebugDraw* debugDrawer);
#endif  //BT_INTERNAL_EDGE_DEBUG_DRAW

#endif  //BT_INTERNAL_EDGE_UTILITY_H
