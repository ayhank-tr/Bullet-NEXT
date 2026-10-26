/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///This file was written by Erwin Coumans


#ifndef BT_POLYHEDRAL_CONTACT_CLIPPING_H
#define BT_POLYHEDRAL_CONTACT_CLIPPING_H


#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "btDiscreteCollisionDetectorInterface.h"

class btlConvexPolyhedron;

typedef btlAlignedObjectArray<btlVector3> btlVertexArray;

// Clips a face to the back of a plane
struct btlPolyhedralContactClipping
{
	static void clipHullAgainstHull(const btlVector3& separatingNormal, const btlConvexPolyhedron& hullA, const btlConvexPolyhedron& hullB, const btlTransform& transA,const btlTransform& transB, const btlScalar minDist, btlScalar maxDist, btlDiscreteCollisionDetectorInterface::Result& resultOut);
	static void	clipFaceAgainstHull(const btlVector3& separatingNormal, const btlConvexPolyhedron& hullA,  const btlTransform& transA, btlVertexArray& worldVertsB1, const btlScalar minDist, btlScalar maxDist,btlDiscreteCollisionDetectorInterface::Result& resultOut);

	static bool findSeparatingAxis(	const btlConvexPolyhedron& hullA, const btlConvexPolyhedron& hullB, const btlTransform& transA,const btlTransform& transB, btlVector3& sep, btlDiscreteCollisionDetectorInterface::Result& resultOut);

	///the clipFace method is used internally
	static void clipFace(const btlVertexArray& pVtxIn, btlVertexArray& ppVtxOut, const btlVector3& planeNormalWS,btlScalar planeEqWS);

};

#endif // BT_POLYHEDRAL_CONTACT_CLIPPING_H

