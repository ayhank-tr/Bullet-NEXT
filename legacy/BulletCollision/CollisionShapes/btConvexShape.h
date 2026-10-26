/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONVEX_SHAPE_INTERFACE1
#define BT_CONVEX_SHAPE_INTERFACE1

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"
#include "LinearMath/btAlignedAllocator.h"

#define MAX_PREFERRED_PENETRATION_DIRECTIONS 10

/// The btConvexShape is an abstract shape interface, implemented by all convex shapes such as btBoxShape, btConvexHullShape etc.
/// It describes general convex shapes using the localGetSupportingVertex interface, used by collision detectors such as btGjkPairDetector.
ATTRIBUTE_ALIGNED16(class) btlConvexShape : public btlCollisionShape
{


public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlConvexShape ();

	virtual ~btlConvexShape();

	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec)const = 0;

	////////
	#ifndef __SPU__
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec) const=0;
	#endif //#ifndef __SPU__

	btlVector3 localGetSupportVertexWithoutMarginNonVirtual (const btlVector3& vec) const;
	btlVector3 localGetSupportVertexNonVirtual (const btlVector3& vec) const;
	btlScalar getMarginNonVirtual () const;
	void getAabbNonVirtual (const btlTransform& t, btlVector3& aabbMin, btlVector3& aabbMax) const;


	virtual void project(const btlTransform& trans, const btlVector3& dir, btlScalar& minProj, btlScalar& maxProj, btlVector3& witnesPtMin,btlVector3& witnesPtMax) const;

	
	//notice that the vectors should be unit length
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const= 0;

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const =0;

	virtual void getAabbSlow(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const =0;

	virtual void	setLocalScaling(const btlVector3& scaling) =0;
	virtual const btlVector3& getLocalScaling() const =0;

	virtual void	setMargin(btlScalar margin)=0;

	virtual btlScalar	getMargin() const=0;

	virtual int		getNumPreferredPenetrationDirections() const=0;
	
	virtual void	getPreferredPenetrationDirection(int index, btlVector3& penetrationVector) const=0;


	
	
};



#endif //BT_CONVEX_SHAPE_INTERFACE1
