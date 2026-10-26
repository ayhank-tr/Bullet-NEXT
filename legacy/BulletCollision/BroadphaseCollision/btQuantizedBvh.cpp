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

#include "btQuantizedBvh.h"

#include "LinearMath/btAabbUtil2.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btSerializer.h"

#define RAYAABB2

btlQuantizedBvh::btlQuantizedBvh() : 
					m_bulletVersion(BT_BULLET_VERSION),
					m_useQuantization(false), 
					//m_traversalMode(TRAVERSAL_STACKLESS_CACHE_FRIENDLY)
					m_traversalMode(TRAVERSAL_STACKLESS)
					//m_traversalMode(TRAVERSAL_RECURSIVE)
					,m_subtreeHeaderCount(0) //PCK: add this line
{
	m_bvhAabbMin.setValue(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY);
	m_bvhAabbMax.setValue(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY);
}





void btlQuantizedBvh::buildInternal()
{
	///assumes that caller filled in the m_quantizedLeafNodes
	m_useQuantization = true;
	int numLeafNodes = 0;
	
	if (m_useQuantization)
	{
		//now we have an array of leafnodes in m_leafNodes
		numLeafNodes = m_quantizedLeafNodes.size();

		m_quantizedContiguousNodes.resize(2*numLeafNodes);

	}

	m_curNodeIndex = 0;

	buildTree(0,numLeafNodes);

	///if the entire tree is small then subtree size, we need to create a header info for the tree
	if(m_useQuantization && !m_SubtreeHeaders.size())
	{
		btlBvhSubtreeInfo& subtree = m_SubtreeHeaders.expand();
		subtree.setAabbFromQuantizeNode(m_quantizedContiguousNodes[0]);
		subtree.m_rootNodeIndex = 0;
		subtree.m_subtreeSize = m_quantizedContiguousNodes[0].isLeafNode() ? 1 : m_quantizedContiguousNodes[0].getEscapeIndex();
	}

	//PCK: update the copy of the size
	m_subtreeHeaderCount = m_SubtreeHeaders.size();

	//PCK: clear m_quantizedLeafNodes and m_leafNodes, they are temporary
	m_quantizedLeafNodes.clear();
	m_leafNodes.clear();
}



///just for debugging, to visualize the individual patches/subtrees
#ifdef DEBUG_PATCH_COLORS
btlVector3 color[4]=
{
	btlVector3(1,0,0),
	btlVector3(0,1,0),
	btlVector3(0,0,1),
	btlVector3(0,1,1)
};
#endif //DEBUG_PATCH_COLORS



void	btlQuantizedBvh::setQuantizationValues(const btlVector3& bvhAabbMin,const btlVector3& bvhAabbMax,btlScalar quantizationMargin)
{
	//enlarge the AABB to avoid division by zero when initializing the quantization values
	btlVector3 clampValue(quantizationMargin,quantizationMargin,quantizationMargin);
	m_bvhAabbMin = bvhAabbMin - clampValue;
	m_bvhAabbMax = bvhAabbMax + clampValue;
	btlVector3 aabbSize = m_bvhAabbMax - m_bvhAabbMin;
	m_bvhQuantization = btlVector3(btlScalar(65533.0),btlScalar(65533.0),btlScalar(65533.0)) / aabbSize;

	m_useQuantization = true;

	{
		unsigned short vecIn[3];
		btlVector3 v;
		{
			quantize(vecIn,m_bvhAabbMin,false);
			v = unQuantize(vecIn);
			m_bvhAabbMin.setMin(v-clampValue);
		}
		{
			quantize(vecIn,m_bvhAabbMax,true);
			v = unQuantize(vecIn);
			m_bvhAabbMax.setMax(v+clampValue);
		}
		aabbSize = m_bvhAabbMax - m_bvhAabbMin;
		m_bvhQuantization = btlVector3(btlScalar(65533.0),btlScalar(65533.0),btlScalar(65533.0)) / aabbSize;
	}
}




btlQuantizedBvh::~btlQuantizedBvh()
{
}

#ifdef DEBUG_TREE_BUILDING
int gStackDepth = 0;
int gMaxStackDepth = 0;
#endif //DEBUG_TREE_BUILDING

void	btlQuantizedBvh::buildTree	(int startIndex,int endIndex)
{
#ifdef DEBUG_TREE_BUILDING
	gStackDepth++;
	if (gStackDepth > gMaxStackDepth)
		gMaxStackDepth = gStackDepth;
#endif //DEBUG_TREE_BUILDING


	int splitAxis, splitIndex, i;
	int numIndices =endIndex-startIndex;
	int curIndex = m_curNodeIndex;

	btlAssert(numIndices>0);

	if (numIndices==1)
	{
#ifdef DEBUG_TREE_BUILDING
		gStackDepth--;
#endif //DEBUG_TREE_BUILDING
		
		assignInternalNodeFromLeafNode(m_curNodeIndex,startIndex);

		m_curNodeIndex++;
		return;	
	}
	//calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.
	
	splitAxis = calcSplittingAxis(startIndex,endIndex);

	splitIndex = sortAndCalcSplittingIndex(startIndex,endIndex,splitAxis);

	int internalNodeIndex = m_curNodeIndex;
	
	//set the min aabb to 'inf' or a max value, and set the max aabb to a -inf/minimum value.
	//the aabb will be expanded during buildTree/mergeInternalNodeAabb with actual node values
	setInternalNodeAabbMin(m_curNodeIndex,m_bvhAabbMax);//can't use btVector3(SIMD_INFINITY,SIMD_INFINITY,SIMD_INFINITY)) because of quantization
	setInternalNodeAabbMax(m_curNodeIndex,m_bvhAabbMin);//can't use btVector3(-SIMD_INFINITY,-SIMD_INFINITY,-SIMD_INFINITY)) because of quantization
	
	
	for (i=startIndex;i<endIndex;i++)
	{
		mergeInternalNodeAabb(m_curNodeIndex,getAabbMin(i),getAabbMax(i));
	}

	m_curNodeIndex++;
	

	//internalNode->m_escapeIndex;
	
	int leftChildNodexIndex = m_curNodeIndex;

	//build left child tree
	buildTree(startIndex,splitIndex);

	int rightChildNodexIndex = m_curNodeIndex;
	//build right child tree
	buildTree(splitIndex,endIndex);

#ifdef DEBUG_TREE_BUILDING
	gStackDepth--;
#endif //DEBUG_TREE_BUILDING

	int escapeIndex = m_curNodeIndex - curIndex;

	if (m_useQuantization)
	{
		//escapeIndex is the number of nodes of this subtree
		const int sizeQuantizedNode =sizeof(btlQuantizedBvhNode);
		const int treeSizeInBytes = escapeIndex * sizeQuantizedNode;
		if (treeSizeInBytes > MAX_SUBTREE_SIZE_IN_BYTES)
		{
			updateSubtreeHeaders(leftChildNodexIndex,rightChildNodexIndex);
		}
	} else
	{

	}

	setInternalNodeEscapeIndex(internalNodeIndex,escapeIndex);

}

void	btlQuantizedBvh::updateSubtreeHeaders(int leftChildNodexIndex,int rightChildNodexIndex)
{
	btlAssert(m_useQuantization);

	btlQuantizedBvhNode& leftChildNode = m_quantizedContiguousNodes[leftChildNodexIndex];
	int leftSubTreeSize = leftChildNode.isLeafNode() ? 1 : leftChildNode.getEscapeIndex();
	int leftSubTreeSizeInBytes =  leftSubTreeSize * static_cast<int>(sizeof(btlQuantizedBvhNode));
	
	btlQuantizedBvhNode& rightChildNode = m_quantizedContiguousNodes[rightChildNodexIndex];
	int rightSubTreeSize = rightChildNode.isLeafNode() ? 1 : rightChildNode.getEscapeIndex();
	int rightSubTreeSizeInBytes =  rightSubTreeSize *  static_cast<int>(sizeof(btlQuantizedBvhNode));

	if(leftSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES)
	{
		btlBvhSubtreeInfo& subtree = m_SubtreeHeaders.expand();
		subtree.setAabbFromQuantizeNode(leftChildNode);
		subtree.m_rootNodeIndex = leftChildNodexIndex;
		subtree.m_subtreeSize = leftSubTreeSize;
	}

	if(rightSubTreeSizeInBytes <= MAX_SUBTREE_SIZE_IN_BYTES)
	{
		btlBvhSubtreeInfo& subtree = m_SubtreeHeaders.expand();
		subtree.setAabbFromQuantizeNode(rightChildNode);
		subtree.m_rootNodeIndex = rightChildNodexIndex;
		subtree.m_subtreeSize = rightSubTreeSize;
	}

	//PCK: update the copy of the size
	m_subtreeHeaderCount = m_SubtreeHeaders.size();
}


int	btlQuantizedBvh::sortAndCalcSplittingIndex(int startIndex,int endIndex,int splitAxis)
{
	int i;
	int splitIndex =startIndex;
	int numIndices = endIndex - startIndex;
	btlScalar splitValue;

	btlVector3 means(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	for (i=startIndex;i<endIndex;i++)
	{
		btlVector3 center = btlScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		means+=center;
	}
	means *= (btlScalar(1.)/(btlScalar)numIndices);
	
	splitValue = means[splitAxis];
	
	//sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
	for (i=startIndex;i<endIndex;i++)
	{
		btlVector3 center = btlScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		if (center[splitAxis] > splitValue)
		{
			//swap
			swapLeafNodes(i,splitIndex);
			splitIndex++;
		}
	}

	//if the splitIndex causes unbalanced trees, fix this by using the center in between startIndex and endIndex
	//otherwise the tree-building might fail due to stack-overflows in certain cases.
	//unbalanced1 is unsafe: it can cause stack overflows
	//bool unbalanced1 = ((splitIndex==startIndex) || (splitIndex == (endIndex-1)));

	//unbalanced2 should work too: always use center (perfect balanced trees)	
	//bool unbalanced2 = true;

	//this should be safe too:
	int rangeBalancedIndices = numIndices/3;
	bool unbalanced = ((splitIndex<=(startIndex+rangeBalancedIndices)) || (splitIndex >=(endIndex-1-rangeBalancedIndices)));
	
	if (unbalanced)
	{
		splitIndex = startIndex+ (numIndices>>1);
	}

	bool unbal = (splitIndex==startIndex) || (splitIndex == (endIndex));
	(void)unbal;
	btlAssert(!unbal);

	return splitIndex;
}


int	btlQuantizedBvh::calcSplittingAxis(int startIndex,int endIndex)
{
	int i;

	btlVector3 means(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	btlVector3 variance(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	int numIndices = endIndex-startIndex;

	for (i=startIndex;i<endIndex;i++)
	{
		btlVector3 center = btlScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		means+=center;
	}
	means *= (btlScalar(1.)/(btlScalar)numIndices);
		
	for (i=startIndex;i<endIndex;i++)
	{
		btlVector3 center = btlScalar(0.5)*(getAabbMax(i)+getAabbMin(i));
		btlVector3 diff2 = center-means;
		diff2 = diff2 * diff2;
		variance += diff2;
	}
	variance *= (btlScalar(1.)/	((btlScalar)numIndices-1)	);
	
	return variance.maxAxis();
}



void	btlQuantizedBvh::reportAabbOverlappingNodex(btlNodeOverlapCallback* nodeCallback,const btlVector3& aabbMin,const btlVector3& aabbMax) const
{
	//either choose recursive traversal (walkTree) or stackless (walkStacklessTree)

	if (m_useQuantization)
	{
		///quantize query AABB
		unsigned short int quantizedQueryAabbMin[3];
		unsigned short int quantizedQueryAabbMax[3];
		quantizeWithClamp(quantizedQueryAabbMin,aabbMin,0);
		quantizeWithClamp(quantizedQueryAabbMax,aabbMax,1);

		switch (m_traversalMode)
		{
		case TRAVERSAL_STACKLESS:
				walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,0,m_curNodeIndex);
			break;
		case TRAVERSAL_STACKLESS_CACHE_FRIENDLY:
				walkStacklessQuantizedTreeCacheFriendly(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
			break;
		case TRAVERSAL_RECURSIVE:
			{
				const btlQuantizedBvhNode* rootNode = &m_quantizedContiguousNodes[0];
				walkRecursiveQuantizedTreeAgainstQueryAabb(rootNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
			}
			break;
		default:
			//unsupported
			btlAssert(0);
		}
	} else
	{
		walkStacklessTree(nodeCallback,aabbMin,aabbMax);
	}
}


int btl_maxIterations = 0;


void	btlQuantizedBvh::walkStacklessTree(btlNodeOverlapCallback* nodeCallback,const btlVector3& aabbMin,const btlVector3& aabbMax) const
{
	btlAssert(!m_useQuantization);

	const btlOptimizedBvhNode* rootNode = &m_contiguousNodes[0];
	int escapeIndex, curIndex = 0;
	int walkIterations = 0;
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap;

	while (curIndex < m_curNodeIndex)
	{
		//catch bugs in tree data
		btlAssert (walkIterations < m_curNodeIndex);

		walkIterations++;
		aabbOverlap = TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode->m_aabbMinOrg,rootNode->m_aabbMaxOrg);
		isLeafNode = rootNode->m_escapeIndex == -1;
		
		//PCK: unsigned instead of bool
		if (isLeafNode && (aabbOverlap != 0))
		{
			nodeCallback->processNode(rootNode->m_subPart,rootNode->m_triangleIndex);
		} 
		
		//PCK: unsigned instead of bool
		if ((aabbOverlap != 0) || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->m_escapeIndex;
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}
	if (btl_maxIterations < walkIterations)
		btl_maxIterations = walkIterations;

}

/*
///this was the original recursive traversal, before we optimized towards stackless traversal
void	btQuantizedBvh::walkTree(btOptimizedBvhNode* rootNode,btNodeOverlapCallback* nodeCallback,const btVector3& aabbMin,const btVector3& aabbMax) const
{
	bool isLeafNode, aabbOverlap = TestAabbAgainstAabb2(aabbMin,aabbMax,rootNode->m_aabbMin,rootNode->m_aabbMax);
	if (aabbOverlap)
	{
		isLeafNode = (!rootNode->m_leftChild && !rootNode->m_rightChild);
		if (isLeafNode)
		{
			nodeCallback->processNode(rootNode);
		} else
		{
			walkTree(rootNode->m_leftChild,nodeCallback,aabbMin,aabbMax);
			walkTree(rootNode->m_rightChild,nodeCallback,aabbMin,aabbMax);
		}
	}

}
*/

void btlQuantizedBvh::walkRecursiveQuantizedTreeAgainstQueryAabb(const btlQuantizedBvhNode* currentNode,btlNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const
{
	btlAssert(m_useQuantization);
	
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap;

	//PCK: unsigned instead of bool
	aabbOverlap = btl_testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,currentNode->m_quantizedAabbMin,currentNode->m_quantizedAabbMax);
	isLeafNode = currentNode->isLeafNode();
		
	//PCK: unsigned instead of bool
	if (aabbOverlap != 0)
	{
		if (isLeafNode)
		{
			nodeCallback->processNode(currentNode->getPartId(),currentNode->getTriangleIndex());
		} else
		{
			//process left and right children
			const btlQuantizedBvhNode* leftChildNode = currentNode+1;
			walkRecursiveQuantizedTreeAgainstQueryAabb(leftChildNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);

			const btlQuantizedBvhNode* rightChildNode = leftChildNode->isLeafNode() ? leftChildNode+1:leftChildNode+leftChildNode->getEscapeIndex();
			walkRecursiveQuantizedTreeAgainstQueryAabb(rightChildNode,nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax);
		}
	}		
}



void	btlQuantizedBvh::walkStacklessTreeAgainstRay(btlNodeOverlapCallback* nodeCallback, const btlVector3& raySource, const btlVector3& rayTarget, const btlVector3& aabbMin, const btlVector3& aabbMax, int startNodeIndex,int endNodeIndex) const
{
	btlAssert(!m_useQuantization);

	const btlOptimizedBvhNode* rootNode = &m_contiguousNodes[0];
	int escapeIndex, curIndex = 0;
	int walkIterations = 0;
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap=0;
	unsigned rayBoxOverlap=0;
	btlScalar lambda_max = 1.0;
	
		/* Quick pruning by quantized box */
	btlVector3 rayAabbMin = raySource;
	btlVector3 rayAabbMax = raySource;
	rayAabbMin.setMin(rayTarget);
	rayAabbMax.setMax(rayTarget);

	/* Add box cast extents to bounding box */
	rayAabbMin += aabbMin;
	rayAabbMax += aabbMax;

#ifdef RAYAABB2
	btlVector3 rayDir = (rayTarget-raySource);
	rayDir.normalize ();
	lambda_max = rayDir.dot(rayTarget-raySource);
	///what about division by zero? --> just set rayDirection[i] to 1.0
	btlVector3 rayDirectionInverse;
	rayDirectionInverse[0] = rayDir[0] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[0];
	rayDirectionInverse[1] = rayDir[1] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[1];
	rayDirectionInverse[2] = rayDir[2] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDir[2];
	unsigned int sign[3] = { rayDirectionInverse[0] < 0.0, rayDirectionInverse[1] < 0.0, rayDirectionInverse[2] < 0.0};
#endif

	btlVector3 bounds[2];

	while (curIndex < m_curNodeIndex)
	{
		btlScalar param = 1.0;
		//catch bugs in tree data
		btlAssert (walkIterations < m_curNodeIndex);

		walkIterations++;

		bounds[0] = rootNode->m_aabbMinOrg;
		bounds[1] = rootNode->m_aabbMaxOrg;
		/* Add box cast extents */
		bounds[0] -= aabbMax;
		bounds[1] -= aabbMin;

		aabbOverlap = TestAabbAgainstAabb2(rayAabbMin,rayAabbMax,rootNode->m_aabbMinOrg,rootNode->m_aabbMaxOrg);
		//perhaps profile if it is worth doing the aabbOverlap test first

#ifdef RAYAABB2
			///careful with this check: need to check division by zero (above) and fix the unQuantize method
			///thanks Joerg/hiker for the reproduction case!
			///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858
		rayBoxOverlap = aabbOverlap ? btlRayAabb2 (raySource, rayDirectionInverse, sign, bounds, param, 0.0f, lambda_max) : false;

#else
		btlVector3 normal;
		rayBoxOverlap = btlRayAabb(raySource, rayTarget,bounds[0],bounds[1],param, normal);
#endif

		isLeafNode = rootNode->m_escapeIndex == -1;
		
		//PCK: unsigned instead of bool
		if (isLeafNode && (rayBoxOverlap != 0))
		{
			nodeCallback->processNode(rootNode->m_subPart,rootNode->m_triangleIndex);
		} 
		
		//PCK: unsigned instead of bool
		if ((rayBoxOverlap != 0) || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->m_escapeIndex;
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}
	if (btl_maxIterations < walkIterations)
		btl_maxIterations = walkIterations;

}



void	btlQuantizedBvh::walkStacklessQuantizedTreeAgainstRay(btlNodeOverlapCallback* nodeCallback, const btlVector3& raySource, const btlVector3& rayTarget, const btlVector3& aabbMin, const btlVector3& aabbMax, int startNodeIndex,int endNodeIndex) const
{
	btlAssert(m_useQuantization);
	
	int curIndex = startNodeIndex;
	int walkIterations = 0;
	int subTreeSize = endNodeIndex - startNodeIndex;
	(void)subTreeSize;

	const btlQuantizedBvhNode* rootNode = &m_quantizedContiguousNodes[startNodeIndex];
	int escapeIndex;
	
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned boxBoxOverlap = 0;
	unsigned rayBoxOverlap = 0;

	btlScalar lambda_max = 1.0;

#ifdef RAYAABB2
	btlVector3 rayDirection = (rayTarget-raySource);
	rayDirection.normalize ();
	lambda_max = rayDirection.dot(rayTarget-raySource);
	///what about division by zero? --> just set rayDirection[i] to 1.0
	rayDirection[0] = rayDirection[0] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDirection[0];
	rayDirection[1] = rayDirection[1] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDirection[1];
	rayDirection[2] = rayDirection[2] == btlScalar(0.0) ? btlScalar(BT_LARGE_FLOAT) : btlScalar(1.0) / rayDirection[2];
	unsigned int sign[3] = { rayDirection[0] < 0.0, rayDirection[1] < 0.0, rayDirection[2] < 0.0};
#endif

	/* Quick pruning by quantized box */
	btlVector3 rayAabbMin = raySource;
	btlVector3 rayAabbMax = raySource;
	rayAabbMin.setMin(rayTarget);
	rayAabbMax.setMax(rayTarget);

	/* Add box cast extents to bounding box */
	rayAabbMin += aabbMin;
	rayAabbMax += aabbMax;

	unsigned short int quantizedQueryAabbMin[3];
	unsigned short int quantizedQueryAabbMax[3];
	quantizeWithClamp(quantizedQueryAabbMin,rayAabbMin,0);
	quantizeWithClamp(quantizedQueryAabbMax,rayAabbMax,1);

	while (curIndex < endNodeIndex)
	{

//#define VISUALLY_ANALYZE_BVH 1
#ifdef VISUALLY_ANALYZE_BVH
		//some code snippet to debugDraw aabb, to visually analyze bvh structure
		static int drawPatch = 0;
		//need some global access to a debugDrawer
		extern btlIDebugDraw* debugDrawerPtr;
		if (curIndex==drawPatch)
		{
			btlVector3 aabbMin,aabbMax;
			aabbMin = unQuantize(rootNode->m_quantizedAabbMin);
			aabbMax = unQuantize(rootNode->m_quantizedAabbMax);
			btlVector3	color(1,0,0);
			debugDrawerPtr->drawAabb(aabbMin,aabbMax,color);
		}
#endif//VISUALLY_ANALYZE_BVH

		//catch bugs in tree data
		btlAssert (walkIterations < subTreeSize);

		walkIterations++;
		//PCK: unsigned instead of bool
		// only interested if this is closer than any previous hit
		btlScalar param = 1.0;
		rayBoxOverlap = 0;
		boxBoxOverlap = btl_testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode->m_quantizedAabbMin,rootNode->m_quantizedAabbMax);
		isLeafNode = rootNode->isLeafNode();
		if (boxBoxOverlap)
		{
			btlVector3 bounds[2];
			bounds[0] = unQuantize(rootNode->m_quantizedAabbMin);
			bounds[1] = unQuantize(rootNode->m_quantizedAabbMax);
			/* Add box cast extents */
			bounds[0] -= aabbMax;
			bounds[1] -= aabbMin;
			btlVector3 normal;
#if 0
			bool ra2 = btlRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0, lambda_max);
			bool ra = btlRayAabb (raySource, rayTarget, bounds[0], bounds[1], param, normal);
			if (ra2 != ra)
			{
				printf("functions don't match\n");
			}
#endif
#ifdef RAYAABB2
			///careful with this check: need to check division by zero (above) and fix the unQuantize method
			///thanks Joerg/hiker for the reproduction case!
			///http://www.bulletphysics.com/Bullet/phpBB3/viewtopic.php?f=9&t=1858

			//BT_PROFILE("btRayAabb2");
			rayBoxOverlap = btlRayAabb2 (raySource, rayDirection, sign, bounds, param, 0.0f, lambda_max);
			
#else
			rayBoxOverlap = true;//btRayAabb(raySource, rayTarget, bounds[0], bounds[1], param, normal);
#endif
		}
		
		if (isLeafNode && rayBoxOverlap)
		{
			nodeCallback->processNode(rootNode->getPartId(),rootNode->getTriangleIndex());
		}
		
		//PCK: unsigned instead of bool
		if ((rayBoxOverlap != 0) || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->getEscapeIndex();
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}
	if (btl_maxIterations < walkIterations)
		btl_maxIterations = walkIterations;

}

void	btlQuantizedBvh::walkStacklessQuantizedTree(btlNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax,int startNodeIndex,int endNodeIndex) const
{
	btlAssert(m_useQuantization);
	
	int curIndex = startNodeIndex;
	int walkIterations = 0;
	int subTreeSize = endNodeIndex - startNodeIndex;
	(void)subTreeSize;

	const btlQuantizedBvhNode* rootNode = &m_quantizedContiguousNodes[startNodeIndex];
	int escapeIndex;
	
	bool isLeafNode;
	//PCK: unsigned instead of bool
	unsigned aabbOverlap;

	while (curIndex < endNodeIndex)
	{

//#define VISUALLY_ANALYZE_BVH 1
#ifdef VISUALLY_ANALYZE_BVH
		//some code snippet to debugDraw aabb, to visually analyze bvh structure
		static int drawPatch = 0;
		//need some global access to a debugDrawer
		extern btlIDebugDraw* debugDrawerPtr;
		if (curIndex==drawPatch)
		{
			btlVector3 aabbMin,aabbMax;
			aabbMin = unQuantize(rootNode->m_quantizedAabbMin);
			aabbMax = unQuantize(rootNode->m_quantizedAabbMax);
			btlVector3	color(1,0,0);
			debugDrawerPtr->drawAabb(aabbMin,aabbMax,color);
		}
#endif//VISUALLY_ANALYZE_BVH

		//catch bugs in tree data
		btlAssert (walkIterations < subTreeSize);

		walkIterations++;
		//PCK: unsigned instead of bool
		aabbOverlap = btl_testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,rootNode->m_quantizedAabbMin,rootNode->m_quantizedAabbMax);
		isLeafNode = rootNode->isLeafNode();
		
		if (isLeafNode && aabbOverlap)
		{
			nodeCallback->processNode(rootNode->getPartId(),rootNode->getTriangleIndex());
		} 
		
		//PCK: unsigned instead of bool
		if ((aabbOverlap != 0) || isLeafNode)
		{
			rootNode++;
			curIndex++;
		} else
		{
			escapeIndex = rootNode->getEscapeIndex();
			rootNode += escapeIndex;
			curIndex += escapeIndex;
		}
	}
	if (btl_maxIterations < walkIterations)
		btl_maxIterations = walkIterations;

}

//This traversal can be called from Playstation 3 SPU
void	btlQuantizedBvh::walkStacklessQuantizedTreeCacheFriendly(btlNodeOverlapCallback* nodeCallback,unsigned short int* quantizedQueryAabbMin,unsigned short int* quantizedQueryAabbMax) const
{
	btlAssert(m_useQuantization);

	int i;


	for (i=0;i<this->m_SubtreeHeaders.size();i++)
	{
		const btlBvhSubtreeInfo& subtree = m_SubtreeHeaders[i];

		//PCK: unsigned instead of bool
		unsigned overlap = btl_testQuantizedAabbAgainstQuantizedAabb(quantizedQueryAabbMin,quantizedQueryAabbMax,subtree.m_quantizedAabbMin,subtree.m_quantizedAabbMax);
		if (overlap != 0)
		{
			walkStacklessQuantizedTree(nodeCallback,quantizedQueryAabbMin,quantizedQueryAabbMax,
				subtree.m_rootNodeIndex,
				subtree.m_rootNodeIndex+subtree.m_subtreeSize);
		}
	}
}


void	btlQuantizedBvh::reportRayOverlappingNodex (btlNodeOverlapCallback* nodeCallback, const btlVector3& raySource, const btlVector3& rayTarget) const
{
	reportBoxCastOverlappingNodex(nodeCallback,raySource,rayTarget,btlVector3(0,0,0),btlVector3(0,0,0));
}


void	btlQuantizedBvh::reportBoxCastOverlappingNodex(btlNodeOverlapCallback* nodeCallback, const btlVector3& raySource, const btlVector3& rayTarget, const btlVector3& aabbMin,const btlVector3& aabbMax) const
{
	//always use stackless

	if (m_useQuantization)
	{
		walkStacklessQuantizedTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex);
	}
	else
	{
		walkStacklessTreeAgainstRay(nodeCallback, raySource, rayTarget, aabbMin, aabbMax, 0, m_curNodeIndex);
	}
	/*
	{
		//recursive traversal
		btVector3 qaabbMin = raySource;
		btVector3 qaabbMax = raySource;
		qaabbMin.setMin(rayTarget);
		qaabbMax.setMax(rayTarget);
		qaabbMin += aabbMin;
		qaabbMax += aabbMax;
		reportAabbOverlappingNodex(nodeCallback,qaabbMin,qaabbMax);
	}
	*/

}


void	btlQuantizedBvh::swapLeafNodes(int i,int splitIndex)
{
	if (m_useQuantization)
	{
			btlQuantizedBvhNode tmp = m_quantizedLeafNodes[i];
			m_quantizedLeafNodes[i] = m_quantizedLeafNodes[splitIndex];
			m_quantizedLeafNodes[splitIndex] = tmp;
	} else
	{
			btlOptimizedBvhNode tmp = m_leafNodes[i];
			m_leafNodes[i] = m_leafNodes[splitIndex];
			m_leafNodes[splitIndex] = tmp;
	}
}

void	btlQuantizedBvh::assignInternalNodeFromLeafNode(int internalNode,int leafNodeIndex)
{
	if (m_useQuantization)
	{
		m_quantizedContiguousNodes[internalNode] = m_quantizedLeafNodes[leafNodeIndex];
	} else
	{
		m_contiguousNodes[internalNode] = m_leafNodes[leafNodeIndex];
	}
}

//PCK: include
#include <new>

#if 0
//PCK: consts
static const unsigned BVH_ALIGNMENT = 16;
static const unsigned BVH_ALIGNMENT_MASK = BVH_ALIGNMENT-1;

static const unsigned BVH_ALIGNMENT_BLOCKS = 2;
#endif


unsigned int btlQuantizedBvh::getAlignmentSerializationPadding()
{
	// I changed this to 0 since the extra padding is not needed or used.
	return 0;//BVH_ALIGNMENT_BLOCKS * BVH_ALIGNMENT;
}

unsigned btlQuantizedBvh::calculateSerializeBufferSize() const
{
	unsigned baseSize = sizeof(btlQuantizedBvh) + getAlignmentSerializationPadding();
	baseSize += sizeof(btlBvhSubtreeInfo) * m_subtreeHeaderCount;
	if (m_useQuantization)
	{
		return baseSize + m_curNodeIndex * sizeof(btlQuantizedBvhNode);
	}
	return baseSize + m_curNodeIndex * sizeof(btlOptimizedBvhNode);
}

bool btlQuantizedBvh::serialize(void *o_alignedDataBuffer, unsigned /*i_dataBufferSize */, bool i_swapEndian) const
{
	btlAssert(m_subtreeHeaderCount == m_SubtreeHeaders.size());
	m_subtreeHeaderCount = m_SubtreeHeaders.size();

/*	if (i_dataBufferSize < calculateSerializeBufferSize() || o_alignedDataBuffer == NULL || (((unsigned)o_alignedDataBuffer & BVH_ALIGNMENT_MASK) != 0))
	{
		///check alignedment for buffer?
		btAssert(0);
		return false;
	}
*/

	btlQuantizedBvh *targetBvh = (btlQuantizedBvh *)o_alignedDataBuffer;

	// construct the class so the virtual function table, etc will be set up
	// Also, m_leafNodes and m_quantizedLeafNodes will be initialized to default values by the constructor
	new (targetBvh) btlQuantizedBvh;

	if (i_swapEndian)
	{
		targetBvh->m_curNodeIndex = static_cast<int>(btlSwapEndian(m_curNodeIndex));


		btlSwapVector3Endian(m_bvhAabbMin,targetBvh->m_bvhAabbMin);
		btlSwapVector3Endian(m_bvhAabbMax,targetBvh->m_bvhAabbMax);
		btlSwapVector3Endian(m_bvhQuantization,targetBvh->m_bvhQuantization);

		targetBvh->m_traversalMode = (btlTraversalMode)btlSwapEndian(m_traversalMode);
		targetBvh->m_subtreeHeaderCount = static_cast<int>(btlSwapEndian(m_subtreeHeaderCount));
	}
	else
	{
		targetBvh->m_curNodeIndex = m_curNodeIndex;
		targetBvh->m_bvhAabbMin = m_bvhAabbMin;
		targetBvh->m_bvhAabbMax = m_bvhAabbMax;
		targetBvh->m_bvhQuantization = m_bvhQuantization;
		targetBvh->m_traversalMode = m_traversalMode;
		targetBvh->m_subtreeHeaderCount = m_subtreeHeaderCount;
	}

	targetBvh->m_useQuantization = m_useQuantization;

	unsigned char *nodeData = (unsigned char *)targetBvh;
	nodeData += sizeof(btlQuantizedBvh);
	
	unsigned sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;
	
	int nodeCount = m_curNodeIndex;

	if (m_useQuantization)
	{
		targetBvh->m_quantizedContiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = btlSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = btlSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = btlSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2]);

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = btlSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = btlSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1]);
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = btlSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2]);

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = static_cast<int>(btlSwapEndian(m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex));
			}
		}
		else
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
	
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2];

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1];
				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2];

				targetBvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex;


			}
		}
		nodeData += sizeof(btlQuantizedBvhNode) * nodeCount;

		// this clears the pointer in the member variable it doesn't really do anything to the data
		// it does call the destructor on the contained objects, but they are all classes with no destructor defined
		// so the memory (which is not freed) is left alone
		targetBvh->m_quantizedContiguousNodes.initializeFromBuffer(NULL, 0, 0);
	}
	else
	{
		targetBvh->m_contiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				btlSwapVector3Endian(m_contiguousNodes[nodeIndex].m_aabbMinOrg, targetBvh->m_contiguousNodes[nodeIndex].m_aabbMinOrg);
				btlSwapVector3Endian(m_contiguousNodes[nodeIndex].m_aabbMaxOrg, targetBvh->m_contiguousNodes[nodeIndex].m_aabbMaxOrg);

				targetBvh->m_contiguousNodes[nodeIndex].m_escapeIndex = static_cast<int>(btlSwapEndian(m_contiguousNodes[nodeIndex].m_escapeIndex));
				targetBvh->m_contiguousNodes[nodeIndex].m_subPart = static_cast<int>(btlSwapEndian(m_contiguousNodes[nodeIndex].m_subPart));
				targetBvh->m_contiguousNodes[nodeIndex].m_triangleIndex = static_cast<int>(btlSwapEndian(m_contiguousNodes[nodeIndex].m_triangleIndex));
			}
		}
		else
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				targetBvh->m_contiguousNodes[nodeIndex].m_aabbMinOrg = m_contiguousNodes[nodeIndex].m_aabbMinOrg;
				targetBvh->m_contiguousNodes[nodeIndex].m_aabbMaxOrg = m_contiguousNodes[nodeIndex].m_aabbMaxOrg;

				targetBvh->m_contiguousNodes[nodeIndex].m_escapeIndex = m_contiguousNodes[nodeIndex].m_escapeIndex;
				targetBvh->m_contiguousNodes[nodeIndex].m_subPart = m_contiguousNodes[nodeIndex].m_subPart;
				targetBvh->m_contiguousNodes[nodeIndex].m_triangleIndex = m_contiguousNodes[nodeIndex].m_triangleIndex;
			}
		}
		nodeData += sizeof(btlOptimizedBvhNode) * nodeCount;

		// this clears the pointer in the member variable it doesn't really do anything to the data
		// it does call the destructor on the contained objects, but they are all classes with no destructor defined
		// so the memory (which is not freed) is left alone
		targetBvh->m_contiguousNodes.initializeFromBuffer(NULL, 0, 0);
	}

	sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	// Now serialize the subtree headers
	targetBvh->m_SubtreeHeaders.initializeFromBuffer(nodeData, m_subtreeHeaderCount, m_subtreeHeaderCount);
	if (i_swapEndian)
	{
		for (int i = 0; i < m_subtreeHeaderCount; i++)
		{
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0] = btlSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMin[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1] = btlSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMin[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2] = btlSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMin[2]);

			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0] = btlSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMax[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1] = btlSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMax[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2] = btlSwapEndian(m_SubtreeHeaders[i].m_quantizedAabbMax[2]);

			targetBvh->m_SubtreeHeaders[i].m_rootNodeIndex = static_cast<int>(btlSwapEndian(m_SubtreeHeaders[i].m_rootNodeIndex));
			targetBvh->m_SubtreeHeaders[i].m_subtreeSize = static_cast<int>(btlSwapEndian(m_SubtreeHeaders[i].m_subtreeSize));
		}
	}
	else
	{
		for (int i = 0; i < m_subtreeHeaderCount; i++)
		{
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0] = (m_SubtreeHeaders[i].m_quantizedAabbMin[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1] = (m_SubtreeHeaders[i].m_quantizedAabbMin[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2] = (m_SubtreeHeaders[i].m_quantizedAabbMin[2]);

			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0] = (m_SubtreeHeaders[i].m_quantizedAabbMax[0]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1] = (m_SubtreeHeaders[i].m_quantizedAabbMax[1]);
			targetBvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2] = (m_SubtreeHeaders[i].m_quantizedAabbMax[2]);

			targetBvh->m_SubtreeHeaders[i].m_rootNodeIndex = (m_SubtreeHeaders[i].m_rootNodeIndex);
			targetBvh->m_SubtreeHeaders[i].m_subtreeSize = (m_SubtreeHeaders[i].m_subtreeSize);

			// need to clear padding in destination buffer
			targetBvh->m_SubtreeHeaders[i].m_padding[0] = 0;
			targetBvh->m_SubtreeHeaders[i].m_padding[1] = 0;
			targetBvh->m_SubtreeHeaders[i].m_padding[2] = 0;
		}
	}
	nodeData += sizeof(btlBvhSubtreeInfo) * m_subtreeHeaderCount;

	// this clears the pointer in the member variable it doesn't really do anything to the data
	// it does call the destructor on the contained objects, but they are all classes with no destructor defined
	// so the memory (which is not freed) is left alone
	targetBvh->m_SubtreeHeaders.initializeFromBuffer(NULL, 0, 0);

	// this wipes the virtual function table pointer at the start of the buffer for the class
	*((void**)o_alignedDataBuffer) = NULL;

	return true;
}

btlQuantizedBvh *btlQuantizedBvh::deSerializeInPlace(void *i_alignedDataBuffer, unsigned int i_dataBufferSize, bool i_swapEndian)
{

	if (i_alignedDataBuffer == NULL)// || (((unsigned)i_alignedDataBuffer & BVH_ALIGNMENT_MASK) != 0))
	{
		return NULL;
	}
	btlQuantizedBvh *bvh = (btlQuantizedBvh *)i_alignedDataBuffer;

	if (i_swapEndian)
	{
		bvh->m_curNodeIndex = static_cast<int>(btlSwapEndian(bvh->m_curNodeIndex));

		btlUnSwapVector3Endian(bvh->m_bvhAabbMin);
		btlUnSwapVector3Endian(bvh->m_bvhAabbMax);
		btlUnSwapVector3Endian(bvh->m_bvhQuantization);

		bvh->m_traversalMode = (btlTraversalMode)btlSwapEndian(bvh->m_traversalMode);
		bvh->m_subtreeHeaderCount = static_cast<int>(btlSwapEndian(bvh->m_subtreeHeaderCount));
	}

	unsigned int calculatedBufSize = bvh->calculateSerializeBufferSize();
	btlAssert(calculatedBufSize <= i_dataBufferSize);

	if (calculatedBufSize > i_dataBufferSize)
	{
		return NULL;
	}

	unsigned char *nodeData = (unsigned char *)bvh;
	nodeData += sizeof(btlQuantizedBvh);
	
	unsigned sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;
	
	int nodeCount = bvh->m_curNodeIndex;

	// Must call placement new to fill in virtual function table, etc, but we don't want to overwrite most data, so call a special version of the constructor
	// Also, m_leafNodes and m_quantizedLeafNodes will be initialized to default values by the constructor
	new (bvh) btlQuantizedBvh(*bvh, false);

	if (bvh->m_useQuantization)
	{
		bvh->m_quantizedContiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0] = btlSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[0]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1] = btlSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[1]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2] = btlSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMin[2]);

				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0] = btlSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[0]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1] = btlSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[1]);
				bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2] = btlSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_quantizedAabbMax[2]);

				bvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex = static_cast<int>(btlSwapEndian(bvh->m_quantizedContiguousNodes[nodeIndex].m_escapeIndexOrTriangleIndex));
			}
		}
		nodeData += sizeof(btlQuantizedBvhNode) * nodeCount;
	}
	else
	{
		bvh->m_contiguousNodes.initializeFromBuffer(nodeData, nodeCount, nodeCount);

		if (i_swapEndian)
		{
			for (int nodeIndex = 0; nodeIndex < nodeCount; nodeIndex++)
			{
				btlUnSwapVector3Endian(bvh->m_contiguousNodes[nodeIndex].m_aabbMinOrg);
				btlUnSwapVector3Endian(bvh->m_contiguousNodes[nodeIndex].m_aabbMaxOrg);
				
				bvh->m_contiguousNodes[nodeIndex].m_escapeIndex = static_cast<int>(btlSwapEndian(bvh->m_contiguousNodes[nodeIndex].m_escapeIndex));
				bvh->m_contiguousNodes[nodeIndex].m_subPart = static_cast<int>(btlSwapEndian(bvh->m_contiguousNodes[nodeIndex].m_subPart));
				bvh->m_contiguousNodes[nodeIndex].m_triangleIndex = static_cast<int>(btlSwapEndian(bvh->m_contiguousNodes[nodeIndex].m_triangleIndex));
			}
		}
		nodeData += sizeof(btlOptimizedBvhNode) * nodeCount;
	}

	sizeToAdd = 0;//(BVH_ALIGNMENT-((unsigned)nodeData & BVH_ALIGNMENT_MASK))&BVH_ALIGNMENT_MASK;
	nodeData += sizeToAdd;

	// Now serialize the subtree headers
	bvh->m_SubtreeHeaders.initializeFromBuffer(nodeData, bvh->m_subtreeHeaderCount, bvh->m_subtreeHeaderCount);
	if (i_swapEndian)
	{
		for (int i = 0; i < bvh->m_subtreeHeaderCount; i++)
		{
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0] = btlSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[0]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1] = btlSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[1]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2] = btlSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMin[2]);

			bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0] = btlSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[0]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1] = btlSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[1]);
			bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2] = btlSwapEndian(bvh->m_SubtreeHeaders[i].m_quantizedAabbMax[2]);

			bvh->m_SubtreeHeaders[i].m_rootNodeIndex = static_cast<int>(btlSwapEndian(bvh->m_SubtreeHeaders[i].m_rootNodeIndex));
			bvh->m_SubtreeHeaders[i].m_subtreeSize = static_cast<int>(btlSwapEndian(bvh->m_SubtreeHeaders[i].m_subtreeSize));
		}
	}

	return bvh;
}

// Constructor that prevents btVector3's default constructor from being called
btlQuantizedBvh::btlQuantizedBvh(btlQuantizedBvh &self, bool /* ownsMemory */) :
m_bvhAabbMin(self.m_bvhAabbMin),
m_bvhAabbMax(self.m_bvhAabbMax),
m_bvhQuantization(self.m_bvhQuantization),
m_bulletVersion(BT_BULLET_VERSION)
{

}

void btlQuantizedBvh::deSerializeFloat(struct btlQuantizedBvhFloatData& quantizedBvhFloatData)
{
	m_bvhAabbMax.deSerializeFloat(quantizedBvhFloatData.m_bvhAabbMax);
	m_bvhAabbMin.deSerializeFloat(quantizedBvhFloatData.m_bvhAabbMin);
	m_bvhQuantization.deSerializeFloat(quantizedBvhFloatData.m_bvhQuantization);

	m_curNodeIndex = quantizedBvhFloatData.m_curNodeIndex;
	m_useQuantization = quantizedBvhFloatData.m_useQuantization!=0;
	
	{
		int numElem = quantizedBvhFloatData.m_numContiguousLeafNodes;
		m_contiguousNodes.resize(numElem);

		if (numElem)
		{
			btlOptimizedBvhNodeFloatData* memPtr = quantizedBvhFloatData.m_contiguousNodesPtr;

			for (int i=0;i<numElem;i++,memPtr++)
			{
				m_contiguousNodes[i].m_aabbMaxOrg.deSerializeFloat(memPtr->m_aabbMaxOrg);
				m_contiguousNodes[i].m_aabbMinOrg.deSerializeFloat(memPtr->m_aabbMinOrg);
				m_contiguousNodes[i].m_escapeIndex = memPtr->m_escapeIndex;
				m_contiguousNodes[i].m_subPart = memPtr->m_subPart;
				m_contiguousNodes[i].m_triangleIndex = memPtr->m_triangleIndex;
			}
		}
	}

	{
		int numElem = quantizedBvhFloatData.m_numQuantizedContiguousNodes;
		m_quantizedContiguousNodes.resize(numElem);
		
		if (numElem)
		{
			btlQuantizedBvhNodeData* memPtr = quantizedBvhFloatData.m_quantizedContiguousNodesPtr;
			for (int i=0;i<numElem;i++,memPtr++)
			{
				m_quantizedContiguousNodes[i].m_escapeIndexOrTriangleIndex = memPtr->m_escapeIndexOrTriangleIndex;
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[0] = memPtr->m_quantizedAabbMax[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[1] = memPtr->m_quantizedAabbMax[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[2] = memPtr->m_quantizedAabbMax[2];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[0] = memPtr->m_quantizedAabbMin[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[1] = memPtr->m_quantizedAabbMin[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[2] = memPtr->m_quantizedAabbMin[2];
			}
		}
	}

	m_traversalMode = btlTraversalMode(quantizedBvhFloatData.m_traversalMode);
	
	{
		int numElem = quantizedBvhFloatData.m_numSubtreeHeaders;
		m_SubtreeHeaders.resize(numElem);
		if (numElem)
		{
			btlBvhSubtreeInfoData* memPtr = quantizedBvhFloatData.m_subTreeInfoPtr;
			for (int i=0;i<numElem;i++,memPtr++)
			{
				m_SubtreeHeaders[i].m_quantizedAabbMax[0] = memPtr->m_quantizedAabbMax[0] ;
				m_SubtreeHeaders[i].m_quantizedAabbMax[1] = memPtr->m_quantizedAabbMax[1];
				m_SubtreeHeaders[i].m_quantizedAabbMax[2] = memPtr->m_quantizedAabbMax[2];
				m_SubtreeHeaders[i].m_quantizedAabbMin[0] = memPtr->m_quantizedAabbMin[0];
				m_SubtreeHeaders[i].m_quantizedAabbMin[1] = memPtr->m_quantizedAabbMin[1];
				m_SubtreeHeaders[i].m_quantizedAabbMin[2] = memPtr->m_quantizedAabbMin[2];
				m_SubtreeHeaders[i].m_rootNodeIndex = memPtr->m_rootNodeIndex;
				m_SubtreeHeaders[i].m_subtreeSize = memPtr->m_subtreeSize;
			}
		}
	}
}

void btlQuantizedBvh::deSerializeDouble(struct btlQuantizedBvhDoubleData& quantizedBvhDoubleData)
{
	m_bvhAabbMax.deSerializeDouble(quantizedBvhDoubleData.m_bvhAabbMax);
	m_bvhAabbMin.deSerializeDouble(quantizedBvhDoubleData.m_bvhAabbMin);
	m_bvhQuantization.deSerializeDouble(quantizedBvhDoubleData.m_bvhQuantization);

	m_curNodeIndex = quantizedBvhDoubleData.m_curNodeIndex;
	m_useQuantization = quantizedBvhDoubleData.m_useQuantization!=0;
	
	{
		int numElem = quantizedBvhDoubleData.m_numContiguousLeafNodes;
		m_contiguousNodes.resize(numElem);

		if (numElem)
		{
			btlOptimizedBvhNodeDoubleData* memPtr = quantizedBvhDoubleData.m_contiguousNodesPtr;

			for (int i=0;i<numElem;i++,memPtr++)
			{
				m_contiguousNodes[i].m_aabbMaxOrg.deSerializeDouble(memPtr->m_aabbMaxOrg);
				m_contiguousNodes[i].m_aabbMinOrg.deSerializeDouble(memPtr->m_aabbMinOrg);
				m_contiguousNodes[i].m_escapeIndex = memPtr->m_escapeIndex;
				m_contiguousNodes[i].m_subPart = memPtr->m_subPart;
				m_contiguousNodes[i].m_triangleIndex = memPtr->m_triangleIndex;
			}
		}
	}

	{
		int numElem = quantizedBvhDoubleData.m_numQuantizedContiguousNodes;
		m_quantizedContiguousNodes.resize(numElem);
		
		if (numElem)
		{
			btlQuantizedBvhNodeData* memPtr = quantizedBvhDoubleData.m_quantizedContiguousNodesPtr;
			for (int i=0;i<numElem;i++,memPtr++)
			{
				m_quantizedContiguousNodes[i].m_escapeIndexOrTriangleIndex = memPtr->m_escapeIndexOrTriangleIndex;
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[0] = memPtr->m_quantizedAabbMax[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[1] = memPtr->m_quantizedAabbMax[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMax[2] = memPtr->m_quantizedAabbMax[2];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[0] = memPtr->m_quantizedAabbMin[0];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[1] = memPtr->m_quantizedAabbMin[1];
				m_quantizedContiguousNodes[i].m_quantizedAabbMin[2] = memPtr->m_quantizedAabbMin[2];
			}
		}
	}

	m_traversalMode = btlTraversalMode(quantizedBvhDoubleData.m_traversalMode);
	
	{
		int numElem = quantizedBvhDoubleData.m_numSubtreeHeaders;
		m_SubtreeHeaders.resize(numElem);
		if (numElem)
		{
			btlBvhSubtreeInfoData* memPtr = quantizedBvhDoubleData.m_subTreeInfoPtr;
			for (int i=0;i<numElem;i++,memPtr++)
			{
				m_SubtreeHeaders[i].m_quantizedAabbMax[0] = memPtr->m_quantizedAabbMax[0] ;
				m_SubtreeHeaders[i].m_quantizedAabbMax[1] = memPtr->m_quantizedAabbMax[1];
				m_SubtreeHeaders[i].m_quantizedAabbMax[2] = memPtr->m_quantizedAabbMax[2];
				m_SubtreeHeaders[i].m_quantizedAabbMin[0] = memPtr->m_quantizedAabbMin[0];
				m_SubtreeHeaders[i].m_quantizedAabbMin[1] = memPtr->m_quantizedAabbMin[1];
				m_SubtreeHeaders[i].m_quantizedAabbMin[2] = memPtr->m_quantizedAabbMin[2];
				m_SubtreeHeaders[i].m_rootNodeIndex = memPtr->m_rootNodeIndex;
				m_SubtreeHeaders[i].m_subtreeSize = memPtr->m_subtreeSize;
			}
		}
	}

}



///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlQuantizedBvh::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlQuantizedBvhData* quantizedData = (btlQuantizedBvhData*)dataBuffer;
	
	m_bvhAabbMax.serialize(quantizedData->m_bvhAabbMax);
	m_bvhAabbMin.serialize(quantizedData->m_bvhAabbMin);
	m_bvhQuantization.serialize(quantizedData->m_bvhQuantization);

	quantizedData->m_curNodeIndex = m_curNodeIndex;
	quantizedData->m_useQuantization = m_useQuantization;
	
	quantizedData->m_numContiguousLeafNodes = m_contiguousNodes.size();
	quantizedData->m_contiguousNodesPtr = (btlOptimizedBvhNodeData*) (m_contiguousNodes.size() ? serializer->getUniquePointer((void*)&m_contiguousNodes[0]) : 0);
	if (quantizedData->m_contiguousNodesPtr)
	{
		int sz = sizeof(btlOptimizedBvhNodeData);
		int numElem = m_contiguousNodes.size();
		btlChunk* chunk = serializer->allocate(sz,numElem);
		btlOptimizedBvhNodeData* memPtr = (btlOptimizedBvhNodeData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			m_contiguousNodes[i].m_aabbMaxOrg.serialize(memPtr->m_aabbMaxOrg);
			m_contiguousNodes[i].m_aabbMinOrg.serialize(memPtr->m_aabbMinOrg);
			memPtr->m_escapeIndex = m_contiguousNodes[i].m_escapeIndex;
			memPtr->m_subPart = m_contiguousNodes[i].m_subPart;
			memPtr->m_triangleIndex = m_contiguousNodes[i].m_triangleIndex;
		}
		serializer->finalizeChunk(chunk,"btOptimizedBvhNodeData",BT_ARRAY_CODE,(void*)&m_contiguousNodes[0]);
	}

	quantizedData->m_numQuantizedContiguousNodes = m_quantizedContiguousNodes.size();
//	printf("quantizedData->m_numQuantizedContiguousNodes=%d\n",quantizedData->m_numQuantizedContiguousNodes);
	quantizedData->m_quantizedContiguousNodesPtr =(btlQuantizedBvhNodeData*) (m_quantizedContiguousNodes.size() ? serializer->getUniquePointer((void*)&m_quantizedContiguousNodes[0]) : 0);
	if (quantizedData->m_quantizedContiguousNodesPtr)
	{
		int sz = sizeof(btlQuantizedBvhNodeData);
		int numElem = m_quantizedContiguousNodes.size();
		btlChunk* chunk = serializer->allocate(sz,numElem);
		btlQuantizedBvhNodeData* memPtr = (btlQuantizedBvhNodeData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			memPtr->m_escapeIndexOrTriangleIndex = m_quantizedContiguousNodes[i].m_escapeIndexOrTriangleIndex;
			memPtr->m_quantizedAabbMax[0] = m_quantizedContiguousNodes[i].m_quantizedAabbMax[0];
			memPtr->m_quantizedAabbMax[1] = m_quantizedContiguousNodes[i].m_quantizedAabbMax[1];
			memPtr->m_quantizedAabbMax[2] = m_quantizedContiguousNodes[i].m_quantizedAabbMax[2];
			memPtr->m_quantizedAabbMin[0] = m_quantizedContiguousNodes[i].m_quantizedAabbMin[0];
			memPtr->m_quantizedAabbMin[1] = m_quantizedContiguousNodes[i].m_quantizedAabbMin[1];
			memPtr->m_quantizedAabbMin[2] = m_quantizedContiguousNodes[i].m_quantizedAabbMin[2];
		}
		serializer->finalizeChunk(chunk,"btQuantizedBvhNodeData",BT_ARRAY_CODE,(void*)&m_quantizedContiguousNodes[0]);
	}

	quantizedData->m_traversalMode = int(m_traversalMode);
	quantizedData->m_numSubtreeHeaders = m_SubtreeHeaders.size();

	quantizedData->m_subTreeInfoPtr = (btlBvhSubtreeInfoData*) (m_SubtreeHeaders.size() ? serializer->getUniquePointer((void*)&m_SubtreeHeaders[0]) : 0);
	if (quantizedData->m_subTreeInfoPtr)
	{
		int sz = sizeof(btlBvhSubtreeInfoData);
		int numElem = m_SubtreeHeaders.size();
		btlChunk* chunk = serializer->allocate(sz,numElem);
		btlBvhSubtreeInfoData* memPtr = (btlBvhSubtreeInfoData*)chunk->m_oldPtr;
		for (int i=0;i<numElem;i++,memPtr++)
		{
			memPtr->m_quantizedAabbMax[0] = m_SubtreeHeaders[i].m_quantizedAabbMax[0];
			memPtr->m_quantizedAabbMax[1] = m_SubtreeHeaders[i].m_quantizedAabbMax[1];
			memPtr->m_quantizedAabbMax[2] = m_SubtreeHeaders[i].m_quantizedAabbMax[2];
			memPtr->m_quantizedAabbMin[0] = m_SubtreeHeaders[i].m_quantizedAabbMin[0];
			memPtr->m_quantizedAabbMin[1] = m_SubtreeHeaders[i].m_quantizedAabbMin[1];
			memPtr->m_quantizedAabbMin[2] = m_SubtreeHeaders[i].m_quantizedAabbMin[2];

			memPtr->m_rootNodeIndex = m_SubtreeHeaders[i].m_rootNodeIndex;
			memPtr->m_subtreeSize = m_SubtreeHeaders[i].m_subtreeSize;
		}
		serializer->finalizeChunk(chunk,"btBvhSubtreeInfoData",BT_ARRAY_CODE,(void*)&m_SubtreeHeaders[0]);
	}
	return btlQuantizedBvhDataName;
}





