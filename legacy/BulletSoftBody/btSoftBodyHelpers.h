/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SOFT_BODY_HELPERS_H
#define BT_SOFT_BODY_HELPERS_H

#include "btSoftBody.h"

//
// Helpers
//

/* fDrawFlags															*/ 
struct	fDrawFlags { enum _ {
	Nodes		=	0x0001,
	Links		=	0x0002,
	Faces		=	0x0004,
	Tetras		=	0x0008,
	Normals		=	0x0010,
	Contacts	=	0x0020,
	Anchors		=	0x0040,
	Notes		=	0x0080,
	Clusters	=	0x0100,
	NodeTree	=	0x0200,
	FaceTree	=	0x0400,
	ClusterTree	=	0x0800,
	Joints		=	0x1000,
	/* presets	*/ 
	Std			=	Links+Faces+Tetras+Anchors+Notes+Joints,
	StdTetra	=	Std-Faces+Tetras
};};

struct	btlSoftBodyHelpers
{
	/* Draw body															*/ 
	static void				Draw(		btlSoftBody* psb,
		btlIDebugDraw* idraw,
		int drawflags=fDrawFlags::Std);
	/* Draw body infos														*/ 
	static	void			DrawInfos(	btlSoftBody* psb,
		btlIDebugDraw* idraw,
		bool masses,
		bool areas,
		bool stress);
	/* Draw node tree														*/ 
	static void				DrawNodeTree(	btlSoftBody* psb,
		btlIDebugDraw* idraw,
		int mindepth=0,
		int maxdepth=-1);
	/* Draw face tree														*/ 
	static void				DrawFaceTree(	btlSoftBody* psb,
		btlIDebugDraw* idraw,
		int mindepth=0,
		int maxdepth=-1);
	/* Draw cluster tree													*/ 
	static void				DrawClusterTree(btlSoftBody* psb,
		btlIDebugDraw* idraw,
		int mindepth=0,
		int maxdepth=-1);
	/* Draw rigid frame														*/ 
	static	void			DrawFrame(		btlSoftBody* psb,
		btlIDebugDraw* idraw);
	/* Create a rope														*/ 
	static	btlSoftBody*		CreateRope( btlSoftBodyWorldInfo& worldInfo,
		const btlVector3& from,
		const btlVector3& to,
		int res,
		int fixeds);
	/* Create a patch														*/ 
	static	btlSoftBody*		CreatePatch(btlSoftBodyWorldInfo& worldInfo,
		const btlVector3& corner00,
		const btlVector3& corner10,
		const btlVector3& corner01,
		const btlVector3& corner11,
		int resx,
		int resy,
		int fixeds,
		bool gendiags);
	/* Create a patch with UV Texture Coordinates	*/ 
	static	btlSoftBody*		CreatePatchUV(btlSoftBodyWorldInfo& worldInfo,
		const btlVector3& corner00,
		const btlVector3& corner10,
		const btlVector3& corner01,
		const btlVector3& corner11,
		int resx,
		int resy,
		int fixeds,
		bool gendiags,
		float* tex_coords=0);
	static	float	CalculateUV(int resx,int resy,int ix,int iy,int id);
	/* Create an ellipsoid													*/ 
	static	btlSoftBody*		CreateEllipsoid(btlSoftBodyWorldInfo& worldInfo,
		const btlVector3& center,
		const btlVector3& radius,
		int res);	
	/* Create from trimesh													*/ 
	static	btlSoftBody*		CreateFromTriMesh(	btlSoftBodyWorldInfo& worldInfo,
		const btlScalar*	vertices,
		const int* triangles,
		int ntriangles,
		bool randomizeConstraints = true);
	/* Create from convex-hull												*/ 
	static	btlSoftBody*		CreateFromConvexHull(	btlSoftBodyWorldInfo& worldInfo,
		const btlVector3* vertices,
		int nvertices,
		bool randomizeConstraints = true);


	/* Export TetGen compatible .smesh file									*/ 
//	static void				ExportAsSMeshFile(	btSoftBody* psb,
//												const char* filename);	
	/* Create from TetGen .ele, .face, .node files							*/ 
//	static btSoftBody*		CreateFromTetGenFile(	btSoftBodyWorldInfo& worldInfo,
//													const char* ele,
//													const char* face,
//													const char* node,
//													bool bfacelinks,
//													bool btetralinks,
//													bool bfacesfromtetras);
	/* Create from TetGen .ele, .face, .node data							*/ 
	static btlSoftBody*		CreateFromTetGenData(	btlSoftBodyWorldInfo& worldInfo,
													const char* ele,
													const char* face,
													const char* node,
													bool bfacelinks,
													bool btetralinks,
													bool bfacesfromtetras);

	/// Sort the list of links to move link calculations that are dependent upon earlier
	/// ones as far as possible away from the calculation of those values
	/// This tends to make adjacent loop iterations not dependent upon one another,
	/// so out-of-order processors can execute instructions from multiple iterations at once
	static void ReoptimizeLinkOrder(btlSoftBody *psb );
};

#endif //BT_SOFT_BODY_HELPERS_H
