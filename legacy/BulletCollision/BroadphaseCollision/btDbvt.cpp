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
///btDbvt implementation by Nathanael Presson

#include "btDbvt.h"

//
typedef btlAlignedObjectArray<btlDbvtNode*>			tNodeArray;
typedef btlAlignedObjectArray<const btlDbvtNode*>	tConstNodeArray;

//
struct btlDbvtNodeEnumerator : btlDbvt::ICollide
{
	tConstNodeArray	nodes;
	void Process(const btlDbvtNode* n) { nodes.push_back(n); }
};

//
static DBVT_INLINE int			indexof(const btlDbvtNode* node)
{
	return(node->parent->childs[1]==node);
}

//
static DBVT_INLINE btlDbvtVolume	merge(	const btlDbvtVolume& a,
									  const btlDbvtVolume& b)
{
#if (DBVT_MERGE_IMPL==DBVT_IMPL_SSE)
	ATTRIBUTE_ALIGNED16( char locals[sizeof(btlDbvtAabbMm)]);
	btlDbvtVolume* ptr = (btlDbvtVolume*) locals;
	btlDbvtVolume&	res=*ptr;
#else
		btlDbvtVolume	res;
#endif
	Merge(a,b,res);
	return(res);
}

// volume+edge lengths
static DBVT_INLINE btlScalar		size(const btlDbvtVolume& a)
{
	const btlVector3	edges=a.Lengths();
	return(	edges.x()*edges.y()*edges.z()+
		edges.x()+edges.y()+edges.z());
}

//
static void						getmaxdepth(const btlDbvtNode* node,int depth,int& maxdepth)
{
	if(node->isinternal())
	{
		getmaxdepth(node->childs[0],depth+1,maxdepth);
		getmaxdepth(node->childs[1],depth+1,maxdepth);
	} else maxdepth=btlMax(maxdepth,depth);
}

//
static DBVT_INLINE void			deletenode(	btlDbvt* pdbvt,
										   btlDbvtNode* node)
{
	btlAlignedFree(pdbvt->m_free);
	pdbvt->m_free=node;
}

//
static void						recursedeletenode(	btlDbvt* pdbvt,
												  btlDbvtNode* node)
{
	if(!node->isleaf())
	{
		recursedeletenode(pdbvt,node->childs[0]);
		recursedeletenode(pdbvt,node->childs[1]);
	}
	if(node==pdbvt->m_root) pdbvt->m_root=0;
	deletenode(pdbvt,node);
}

//
static DBVT_INLINE btlDbvtNode*	createnode(	btlDbvt* pdbvt,
										   btlDbvtNode* parent,
										   void* data)
{
	btlDbvtNode*	node;
	if(pdbvt->m_free)
	{ node=pdbvt->m_free;pdbvt->m_free=0; }
	else
	{ node=new(btlAlignedAlloc(sizeof(btlDbvtNode),16)) btlDbvtNode(); }
	node->parent	=	parent;
	node->data		=	data;
	node->childs[1]	=	0;
	return(node);
}

//
static DBVT_INLINE btlDbvtNode*	createnode(	btlDbvt* pdbvt,
										   btlDbvtNode* parent,
										   const btlDbvtVolume& volume,
										   void* data)
{
	btlDbvtNode*	node=createnode(pdbvt,parent,data);
	node->volume=volume;
	return(node);
}

//
static DBVT_INLINE btlDbvtNode*	createnode(	btlDbvt* pdbvt,
										   btlDbvtNode* parent,
										   const btlDbvtVolume& volume0,
										   const btlDbvtVolume& volume1,
										   void* data)
{
	btlDbvtNode*	node=createnode(pdbvt,parent,data);
	Merge(volume0,volume1,node->volume);
	return(node);
}

//
static void						insertleaf(	btlDbvt* pdbvt,
										   btlDbvtNode* root,
										   btlDbvtNode* leaf)
{
	if(!pdbvt->m_root)
	{
		pdbvt->m_root	=	leaf;
		leaf->parent	=	0;
	}
	else
	{
		if(!root->isleaf())
		{
			do	{
				root=root->childs[Select(	leaf->volume,
					root->childs[0]->volume,
					root->childs[1]->volume)];
			} while(!root->isleaf());
		}
		btlDbvtNode*	prev=root->parent;
		btlDbvtNode*	node=createnode(pdbvt,prev,leaf->volume,root->volume,0);
		if(prev)
		{
			prev->childs[indexof(root)]	=	node;
			node->childs[0]				=	root;root->parent=node;
			node->childs[1]				=	leaf;leaf->parent=node;
			do	{
				if(!prev->volume.Contain(node->volume))
					Merge(prev->childs[0]->volume,prev->childs[1]->volume,prev->volume);
				else
					break;
				node=prev;
			} while(0!=(prev=node->parent));
		}
		else
		{
			node->childs[0]	=	root;root->parent=node;
			node->childs[1]	=	leaf;leaf->parent=node;
			pdbvt->m_root	=	node;
		}
	}
}

//
static btlDbvtNode*				removeleaf(	btlDbvt* pdbvt,
										   btlDbvtNode* leaf)
{
	if(leaf==pdbvt->m_root)
	{
		pdbvt->m_root=0;
		return(0);
	}
	else
	{
		btlDbvtNode*	parent=leaf->parent;
		btlDbvtNode*	prev=parent->parent;
		btlDbvtNode*	sibling=parent->childs[1-indexof(leaf)];			
		if(prev)
		{
			prev->childs[indexof(parent)]=sibling;
			sibling->parent=prev;
			deletenode(pdbvt,parent);
			while(prev)
			{
				const btlDbvtVolume	pb=prev->volume;
				Merge(prev->childs[0]->volume,prev->childs[1]->volume,prev->volume);
				if(NotEqual(pb,prev->volume))
				{
					prev=prev->parent;
				} else break;
			}
			return(prev?prev:pdbvt->m_root);
		}
		else
		{								
			pdbvt->m_root=sibling;
			sibling->parent=0;
			deletenode(pdbvt,parent);
			return(pdbvt->m_root);
		}			
	}
}

//
static void						fetchleaves(btlDbvt* pdbvt,
											btlDbvtNode* root,
											tNodeArray& leaves,
											int depth=-1)
{
	if(root->isinternal()&&depth)
	{
		fetchleaves(pdbvt,root->childs[0],leaves,depth-1);
		fetchleaves(pdbvt,root->childs[1],leaves,depth-1);
		deletenode(pdbvt,root);
	}
	else
	{
		leaves.push_back(root);
	}
}

//
static void						split(	const tNodeArray& leaves,
									  tNodeArray& left,
									  tNodeArray& right,
									  const btlVector3& org,
									  const btlVector3& axis)
{
	left.resize(0);
	right.resize(0);
	for(int i=0,ni=leaves.size();i<ni;++i)
	{
		if(btlDot(axis,leaves[i]->volume.Center()-org)<0)
			left.push_back(leaves[i]);
		else
			right.push_back(leaves[i]);
	}
}

//
static btlDbvtVolume				bounds(	const tNodeArray& leaves)
{
#if DBVT_MERGE_IMPL==DBVT_IMPL_SSE
	ATTRIBUTE_ALIGNED16(char	locals[sizeof(btlDbvtVolume)]);
	btlDbvtVolume* ptr = (btlDbvtVolume*) locals;
	btlDbvtVolume&	volume=*ptr;
	volume=leaves[0]->volume;
#else
	btlDbvtVolume volume=leaves[0]->volume;
#endif
	for(int i=1,ni=leaves.size();i<ni;++i)
	{
		Merge(volume,leaves[i]->volume,volume);
	}
	return(volume);
}

//
static void						bottomup(	btlDbvt* pdbvt,
										 tNodeArray& leaves)
{
	while(leaves.size()>1)
	{
		btlScalar	minsize=SIMD_INFINITY;
		int			minidx[2]={-1,-1};
		for(int i=0;i<leaves.size();++i)
		{
			for(int j=i+1;j<leaves.size();++j)
			{
				const btlScalar	sz=size(merge(leaves[i]->volume,leaves[j]->volume));
				if(sz<minsize)
				{
					minsize		=	sz;
					minidx[0]	=	i;
					minidx[1]	=	j;
				}
			}
		}
		btlDbvtNode*	n[]	=	{leaves[minidx[0]],leaves[minidx[1]]};
		btlDbvtNode*	p	=	createnode(pdbvt,0,n[0]->volume,n[1]->volume,0);
		p->childs[0]		=	n[0];
		p->childs[1]		=	n[1];
		n[0]->parent		=	p;
		n[1]->parent		=	p;
		leaves[minidx[0]]	=	p;
		leaves.swap(minidx[1],leaves.size()-1);
		leaves.pop_back();
	}
}

//
static btlDbvtNode*			topdown(btlDbvt* pdbvt,
									tNodeArray& leaves,
									int bu_treshold)
{
	static const btlVector3	axis[]={btlVector3(1,0,0),
		btlVector3(0,1,0),
		btlVector3(0,0,1)};
	if(leaves.size()>1)
	{
		if(leaves.size()>bu_treshold)
		{
			const btlDbvtVolume	vol=bounds(leaves);
			const btlVector3			org=vol.Center();
			tNodeArray				sets[2];
			int						bestaxis=-1;
			int						bestmidp=leaves.size();
			int						splitcount[3][2]={{0,0},{0,0},{0,0}};
			int i;
			for( i=0;i<leaves.size();++i)
			{
				const btlVector3	x=leaves[i]->volume.Center()-org;
				for(int j=0;j<3;++j)
				{
					++splitcount[j][btlDot(x,axis[j])>0?1:0];
				}
			}
			for( i=0;i<3;++i)
			{
				if((splitcount[i][0]>0)&&(splitcount[i][1]>0))
				{
					const int	midp=(int)btlFabs(btlScalar(splitcount[i][0]-splitcount[i][1]));
					if(midp<bestmidp)
					{
						bestaxis=i;
						bestmidp=midp;
					}
				}
			}
			if(bestaxis>=0)
			{
				sets[0].reserve(splitcount[bestaxis][0]);
				sets[1].reserve(splitcount[bestaxis][1]);
				split(leaves,sets[0],sets[1],org,axis[bestaxis]);
			}
			else
			{
				sets[0].reserve(leaves.size()/2+1);
				sets[1].reserve(leaves.size()/2);
				for(int i=0,ni=leaves.size();i<ni;++i)
				{
					sets[i&1].push_back(leaves[i]);
				}
			}
			btlDbvtNode*	node=createnode(pdbvt,0,vol,0);
			node->childs[0]=topdown(pdbvt,sets[0],bu_treshold);
			node->childs[1]=topdown(pdbvt,sets[1],bu_treshold);
			node->childs[0]->parent=node;
			node->childs[1]->parent=node;
			return(node);
		}
		else
		{
			bottomup(pdbvt,leaves);
			return(leaves[0]);
		}
	}
	return(leaves[0]);
}

//
static DBVT_INLINE btlDbvtNode*	sort(btlDbvtNode* n,btlDbvtNode*& r)
{
	btlDbvtNode*	p=n->parent;
	btlAssert(n->isinternal());
	if(p>n)
	{
		const int		i=indexof(n);
		const int		j=1-i;
		btlDbvtNode*	s=p->childs[j];
		btlDbvtNode*	q=p->parent;
		btlAssert(n==p->childs[i]);
		if(q) q->childs[indexof(p)]=n; else r=n;
		s->parent=n;
		p->parent=n;
		n->parent=q;
		p->childs[0]=n->childs[0];
		p->childs[1]=n->childs[1];
		n->childs[0]->parent=p;
		n->childs[1]->parent=p;
		n->childs[i]=p;
		n->childs[j]=s;
		btlSwap(p->volume,n->volume);
		return(p);
	}
	return(n);
}

#if 0
static DBVT_INLINE btlDbvtNode*	walkup(btlDbvtNode* n,int count)
{
	while(n&&(count--)) n=n->parent;
	return(n);
}
#endif

//
// Api
//

//
btlDbvt::btlDbvt()
{
	m_root		=	0;
	m_free		=	0;
	m_lkhd		=	-1;
	m_leaves	=	0;
	m_opath		=	0;
}

//
btlDbvt::~btlDbvt()
{
	clear();
}

//
void			btlDbvt::clear()
{
	if(m_root)	
		recursedeletenode(this,m_root);
	btlAlignedFree(m_free);
	m_free=0;
	m_lkhd		=	-1;
	m_stkStack.clear();
	m_opath		=	0;
	
}

//
void			btlDbvt::optimizeBottomUp()
{
	if(m_root)
	{
		tNodeArray leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this,m_root,leaves);
		bottomup(this,leaves);
		m_root=leaves[0];
	}
}

//
void			btlDbvt::optimizeTopDown(int bu_treshold)
{
	if(m_root)
	{
		tNodeArray	leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this,m_root,leaves);
		m_root=topdown(this,leaves,bu_treshold);
	}
}

//
void			btlDbvt::optimizeIncremental(int passes)
{
	if(passes<0) passes=m_leaves;
	if(m_root&&(passes>0))
	{
		do	{
			btlDbvtNode*		node=m_root;
			unsigned	bit=0;
			while(node->isinternal())
			{
				node=sort(node,m_root)->childs[(m_opath>>bit)&1];
				bit=(bit+1)&(sizeof(unsigned)*8-1);
			}
			update(node);
			++m_opath;
		} while(--passes);
	}
}

//
btlDbvtNode*	btlDbvt::insert(const btlDbvtVolume& volume,void* data)
{
	btlDbvtNode*	leaf=createnode(this,0,volume,data);
	insertleaf(this,m_root,leaf);
	++m_leaves;
	return(leaf);
}

//
void			btlDbvt::update(btlDbvtNode* leaf,int lookahead)
{
	btlDbvtNode*	root=removeleaf(this,leaf);
	if(root)
	{
		if(lookahead>=0)
		{
			for(int i=0;(i<lookahead)&&root->parent;++i)
			{
				root=root->parent;
			}
		} else root=m_root;
	}
	insertleaf(this,root,leaf);
}

//
void			btlDbvt::update(btlDbvtNode* leaf,btlDbvtVolume& volume)
{
	btlDbvtNode*	root=removeleaf(this,leaf);
	if(root)
	{
		if(m_lkhd>=0)
		{
			for(int i=0;(i<m_lkhd)&&root->parent;++i)
			{
				root=root->parent;
			}
		} else root=m_root;
	}
	leaf->volume=volume;
	insertleaf(this,root,leaf);
}

//
bool			btlDbvt::update(btlDbvtNode* leaf,btlDbvtVolume& volume,const btlVector3& velocity,btlScalar margin)
{
	if(leaf->volume.Contain(volume)) return(false);
	volume.Expand(btlVector3(margin,margin,margin));
	volume.SignedExpand(velocity);
	update(leaf,volume);
	return(true);
}

//
bool			btlDbvt::update(btlDbvtNode* leaf,btlDbvtVolume& volume,const btlVector3& velocity)
{
	if(leaf->volume.Contain(volume)) return(false);
	volume.SignedExpand(velocity);
	update(leaf,volume);
	return(true);
}

//
bool			btlDbvt::update(btlDbvtNode* leaf,btlDbvtVolume& volume,btlScalar margin)
{
	if(leaf->volume.Contain(volume)) return(false);
	volume.Expand(btlVector3(margin,margin,margin));
	update(leaf,volume);
	return(true);
}

//
void			btlDbvt::remove(btlDbvtNode* leaf)
{
	removeleaf(this,leaf);
	deletenode(this,leaf);
	--m_leaves;
}

//
void			btlDbvt::write(IWriter* iwriter) const
{
	btlDbvtNodeEnumerator	nodes;
	nodes.nodes.reserve(m_leaves*2);
	enumNodes(m_root,nodes);
	iwriter->Prepare(m_root,nodes.nodes.size());
	for(int i=0;i<nodes.nodes.size();++i)
	{
		const btlDbvtNode* n=nodes.nodes[i];
		int			p=-1;
		if(n->parent) p=nodes.nodes.findLinearSearch(n->parent);
		if(n->isinternal())
		{
			const int	c0=nodes.nodes.findLinearSearch(n->childs[0]);
			const int	c1=nodes.nodes.findLinearSearch(n->childs[1]);
			iwriter->WriteNode(n,i,p,c0,c1);
		}
		else
		{
			iwriter->WriteLeaf(n,i,p);
		}	
	}
}

//
void			btlDbvt::clone(btlDbvt& dest,IClone* iclone) const
{
	dest.clear();
	if(m_root!=0)
	{	
		btlAlignedObjectArray<sStkCLN>	stack;
		stack.reserve(m_leaves);
		stack.push_back(sStkCLN(m_root,0));
		do	{
			const int		i=stack.size()-1;
			const sStkCLN	e=stack[i];
			btlDbvtNode*			n=createnode(&dest,e.parent,e.node->volume,e.node->data);
			stack.pop_back();
			if(e.parent!=0)
				e.parent->childs[i&1]=n;
			else
				dest.m_root=n;
			if(e.node->isinternal())
			{
				stack.push_back(sStkCLN(e.node->childs[0],n));
				stack.push_back(sStkCLN(e.node->childs[1],n));
			}
			else
			{
				iclone->CloneLeaf(n);
			}
		} while(stack.size()>0);
	}
}

//
int				btlDbvt::maxdepth(const btlDbvtNode* node)
{
	int	depth=0;
	if(node) getmaxdepth(node,1,depth);
	return(depth);
}

//
int				btlDbvt::countLeaves(const btlDbvtNode* node)
{
	if(node->isinternal())
		return(countLeaves(node->childs[0])+countLeaves(node->childs[1]));
	else
		return(1);
}

//
void			btlDbvt::extractLeaves(const btlDbvtNode* node,btlAlignedObjectArray<const btlDbvtNode*>& leaves)
{
	if(node->isinternal())
	{
		extractLeaves(node->childs[0],leaves);
		extractLeaves(node->childs[1],leaves);
	}
	else
	{
		leaves.push_back(node);
	}	
}

//
#if DBVT_ENABLE_BENCHMARK

#include <stdio.h>
#include <stdlib.h>
#include "LinearMath/btQuickProf.h"

/*
q6600,2.4ghz

/Ox /Ob2 /Oi /Ot /I "." /I "..\.." /I "..\..\src" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"
/GF /FD /MT /GS- /Gy /arch:SSE2 /Zc:wchar_t- /Fp"..\..\out\release8\build\libbulletcollision\libbulletcollision.pch"
/Fo"..\..\out\release8\build\libbulletcollision\\"
/Fd"..\..\out\release8\build\libbulletcollision\bulletcollision.pdb"
/W3 /nologo /c /Wp64 /Zi /errorReport:prompt

Benchmarking dbvt...
World scale: 100.000000
Extents base: 1.000000
Extents range: 4.000000
Leaves: 8192
sizeof(btDbvtVolume): 32 bytes
sizeof(btDbvtNode):   44 bytes
[1] btDbvtVolume intersections: 3499 ms (-1%)
[2] btDbvtVolume merges: 1934 ms (0%)
[3] btDbvt::collideTT: 5485 ms (-21%)
[4] btDbvt::collideTT self: 2814 ms (-20%)
[5] btDbvt::collideTT xform: 7379 ms (-1%)
[6] btDbvt::collideTT xform,self: 7270 ms (-2%)
[7] btDbvt::rayTest: 6314 ms (0%),(332143 r/s)
[8] insert/remove: 2093 ms (0%),(1001983 ir/s)
[9] updates (teleport): 1879 ms (-3%),(1116100 u/s)
[10] updates (jitter): 1244 ms (-4%),(1685813 u/s)
[11] optimize (incremental): 2514 ms (0%),(1668000 o/s)
[12] btDbvtVolume notequal: 3659 ms (0%)
[13] culling(OCL+fullsort): 2218 ms (0%),(461 t/s)
[14] culling(OCL+qsort): 3688 ms (5%),(2221 t/s)
[15] culling(KDOP+qsort): 1139 ms (-1%),(7192 t/s)
[16] insert/remove batch(256): 5092 ms (0%),(823704 bir/s)
[17] btDbvtVolume select: 3419 ms (0%)
*/

struct btlDbvtBenchmark
{
	struct NilPolicy : btlDbvt::ICollide
	{
		NilPolicy() : m_pcount(0),m_depth(-SIMD_INFINITY),m_checksort(true)		{}
		void	Process(const btlDbvtNode*,const btlDbvtNode*)				{ ++m_pcount; }
		void	Process(const btlDbvtNode*)									{ ++m_pcount; }
		void	Process(const btlDbvtNode*,btlScalar depth)
		{
			++m_pcount;
			if(m_checksort)
			{ if(depth>=m_depth) m_depth=depth; else printf("wrong depth: %f (should be >= %f)\r\n",depth,m_depth); }
		}
		int			m_pcount;
		btlScalar	m_depth;
		bool		m_checksort;
	};
	struct P14 : btlDbvt::ICollide
	{
		struct Node
		{
			const btlDbvtNode*	leaf;
			btlScalar			depth;
		};
		void Process(const btlDbvtNode* leaf,btlScalar depth)
		{
			Node	n;
			n.leaf	=	leaf;
			n.depth	=	depth;
		}
		static int sortfnc(const Node& a,const Node& b)
		{
			if(a.depth<b.depth) return(+1);
			if(a.depth>b.depth) return(-1);
			return(0);
		}
		btlAlignedObjectArray<Node>		m_nodes;
	};
	struct P15 : btlDbvt::ICollide
	{
		struct Node
		{
			const btlDbvtNode*	leaf;
			btlScalar			depth;
		};
		void Process(const btlDbvtNode* leaf)
		{
			Node	n;
			n.leaf	=	leaf;
			n.depth	=	dot(leaf->volume.Center(),m_axis);
		}
		static int sortfnc(const Node& a,const Node& b)
		{
			if(a.depth<b.depth) return(+1);
			if(a.depth>b.depth) return(-1);
			return(0);
		}
		btlAlignedObjectArray<Node>		m_nodes;
		btlVector3						m_axis;
	};
	static btlScalar			RandUnit()
	{
		return(rand()/(btlScalar)RAND_MAX);
	}
	static btlVector3		RandVector3()
	{
		return(btlVector3(RandUnit(),RandUnit(),RandUnit()));
	}
	static btlVector3		RandVector3(btlScalar cs)
	{
		return(RandVector3()*cs-btlVector3(cs,cs,cs)/2);
	}
	static btlDbvtVolume	RandVolume(btlScalar cs,btlScalar eb,btlScalar es)
	{
		return(btlDbvtVolume::FromCE(RandVector3(cs),btlVector3(eb,eb,eb)+RandVector3()*es));
	}
	static btlTransform		RandTransform(btlScalar cs)
	{
		btlTransform	t;
		t.setOrigin(RandVector3(cs));
		t.setRotation(btlQuaternion(RandUnit()*SIMD_PI*2,RandUnit()*SIMD_PI*2,RandUnit()*SIMD_PI*2).normalized());
		return(t);
	}
	static void				RandTree(btlScalar cs,btlScalar eb,btlScalar es,int leaves,btlDbvt& dbvt)
	{
		dbvt.clear();
		for(int i=0;i<leaves;++i)
		{
			dbvt.insert(RandVolume(cs,eb,es),0);
		}
	}
};

void			btlDbvt::benchmark()
{
	static const btlScalar	cfgVolumeCenterScale		=	100;
	static const btlScalar	cfgVolumeExentsBase			=	1;
	static const btlScalar	cfgVolumeExentsScale		=	4;
	static const int		cfgLeaves					=	8192;
	static const bool		cfgEnable					=	true;

	//[1] btDbvtVolume intersections
	bool					cfgBenchmark1_Enable		=	cfgEnable;
	static const int		cfgBenchmark1_Iterations	=	8;
	static const int		cfgBenchmark1_Reference		=	3499;
	//[2] btDbvtVolume merges
	bool					cfgBenchmark2_Enable		=	cfgEnable;
	static const int		cfgBenchmark2_Iterations	=	4;
	static const int		cfgBenchmark2_Reference		=	1945;
	//[3] btDbvt::collideTT
	bool					cfgBenchmark3_Enable		=	cfgEnable;
	static const int		cfgBenchmark3_Iterations	=	512;
	static const int		cfgBenchmark3_Reference		=	5485;
	//[4] btDbvt::collideTT self
	bool					cfgBenchmark4_Enable		=	cfgEnable;
	static const int		cfgBenchmark4_Iterations	=	512;
	static const int		cfgBenchmark4_Reference		=	2814;
	//[5] btDbvt::collideTT xform
	bool					cfgBenchmark5_Enable		=	cfgEnable;
	static const int		cfgBenchmark5_Iterations	=	512;
	static const btlScalar	cfgBenchmark5_OffsetScale	=	2;
	static const int		cfgBenchmark5_Reference		=	7379;
	//[6] btDbvt::collideTT xform,self
	bool					cfgBenchmark6_Enable		=	cfgEnable;
	static const int		cfgBenchmark6_Iterations	=	512;
	static const btlScalar	cfgBenchmark6_OffsetScale	=	2;
	static const int		cfgBenchmark6_Reference		=	7270;
	//[7] btDbvt::rayTest
	bool					cfgBenchmark7_Enable		=	cfgEnable;
	static const int		cfgBenchmark7_Passes		=	32;
	static const int		cfgBenchmark7_Iterations	=	65536;
	static const int		cfgBenchmark7_Reference		=	6307;
	//[8] insert/remove
	bool					cfgBenchmark8_Enable		=	cfgEnable;
	static const int		cfgBenchmark8_Passes		=	32;
	static const int		cfgBenchmark8_Iterations	=	65536;
	static const int		cfgBenchmark8_Reference		=	2105;
	//[9] updates (teleport)
	bool					cfgBenchmark9_Enable		=	cfgEnable;
	static const int		cfgBenchmark9_Passes		=	32;
	static const int		cfgBenchmark9_Iterations	=	65536;
	static const int		cfgBenchmark9_Reference		=	1879;
	//[10] updates (jitter)
	bool					cfgBenchmark10_Enable		=	cfgEnable;
	static const btlScalar	cfgBenchmark10_Scale		=	cfgVolumeCenterScale/10000;
	static const int		cfgBenchmark10_Passes		=	32;
	static const int		cfgBenchmark10_Iterations	=	65536;
	static const int		cfgBenchmark10_Reference	=	1244;
	//[11] optimize (incremental)
	bool					cfgBenchmark11_Enable		=	cfgEnable;
	static const int		cfgBenchmark11_Passes		=	64;
	static const int		cfgBenchmark11_Iterations	=	65536;
	static const int		cfgBenchmark11_Reference	=	2510;
	//[12] btDbvtVolume notequal
	bool					cfgBenchmark12_Enable		=	cfgEnable;
	static const int		cfgBenchmark12_Iterations	=	32;
	static const int		cfgBenchmark12_Reference	=	3677;
	//[13] culling(OCL+fullsort)
	bool					cfgBenchmark13_Enable		=	cfgEnable;
	static const int		cfgBenchmark13_Iterations	=	1024;
	static const int		cfgBenchmark13_Reference	=	2231;
	//[14] culling(OCL+qsort)
	bool					cfgBenchmark14_Enable		=	cfgEnable;
	static const int		cfgBenchmark14_Iterations	=	8192;
	static const int		cfgBenchmark14_Reference	=	3500;
	//[15] culling(KDOP+qsort)
	bool					cfgBenchmark15_Enable		=	cfgEnable;
	static const int		cfgBenchmark15_Iterations	=	8192;
	static const int		cfgBenchmark15_Reference	=	1151;
	//[16] insert/remove batch
	bool					cfgBenchmark16_Enable		=	cfgEnable;
	static const int		cfgBenchmark16_BatchCount	=	256;
	static const int		cfgBenchmark16_Passes		=	16384;
	static const int		cfgBenchmark16_Reference	=	5138;
	//[17] select
	bool					cfgBenchmark17_Enable		=	cfgEnable;
	static const int		cfgBenchmark17_Iterations	=	4;
	static const int		cfgBenchmark17_Reference	=	3390;

	btlClock					wallclock;
	printf("Benchmarking dbvt...\r\n");
	printf("\tWorld scale: %f\r\n",cfgVolumeCenterScale);
	printf("\tExtents base: %f\r\n",cfgVolumeExentsBase);
	printf("\tExtents range: %f\r\n",cfgVolumeExentsScale);
	printf("\tLeaves: %u\r\n",cfgLeaves);
	printf("\tsizeof(btDbvtVolume): %u bytes\r\n",sizeof(btlDbvtVolume));
	printf("\tsizeof(btDbvtNode):   %u bytes\r\n",sizeof(btlDbvtNode));
	if(cfgBenchmark1_Enable)
	{// Benchmark 1	
		srand(380843);
		btlAlignedObjectArray<btlDbvtVolume>	volumes;
		btlAlignedObjectArray<bool>			results;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			volumes[i]=btlDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		printf("[1] btDbvtVolume intersections: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark1_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					results[k]=Intersect(volumes[j],volumes[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark1_Reference)*100/time);
	}
	if(cfgBenchmark2_Enable)
	{// Benchmark 2	
		srand(380843);
		btlAlignedObjectArray<btlDbvtVolume>	volumes;
		btlAlignedObjectArray<btlDbvtVolume>	results;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			volumes[i]=btlDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		printf("[2] btDbvtVolume merges: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark2_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					Merge(volumes[j],volumes[k],results[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark2_Reference)*100/time);
	}
	if(cfgBenchmark3_Enable)
	{// Benchmark 3	
		srand(380843);
		btlDbvt						dbvt[2];
		btlDbvtBenchmark::NilPolicy	policy;
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
		dbvt[0].optimizeTopDown();
		dbvt[1].optimizeTopDown();
		printf("[3] btDbvt::collideTT: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark3_Iterations;++i)
		{
			btlDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,policy);
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark3_Reference)*100/time);
	}
	if(cfgBenchmark4_Enable)
	{// Benchmark 4
		srand(380843);
		btlDbvt						dbvt;
		btlDbvtBenchmark::NilPolicy	policy;
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[4] btDbvt::collideTT self: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark4_Iterations;++i)
		{
			btlDbvt::collideTT(dbvt.m_root,dbvt.m_root,policy);
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark4_Reference)*100/time);
	}
	if(cfgBenchmark5_Enable)
	{// Benchmark 5	
		srand(380843);
		btlDbvt								dbvt[2];
		btlAlignedObjectArray<btlTransform>	transforms;
		btlDbvtBenchmark::NilPolicy			policy;
		transforms.resize(cfgBenchmark5_Iterations);
		for(int i=0;i<transforms.size();++i)
		{
			transforms[i]=btlDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark5_OffsetScale);
		}
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
		dbvt[0].optimizeTopDown();
		dbvt[1].optimizeTopDown();
		printf("[5] btDbvt::collideTT xform: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark5_Iterations;++i)
		{
			btlDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,transforms[i],policy);
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark5_Reference)*100/time);
	}
	if(cfgBenchmark6_Enable)
	{// Benchmark 6	
		srand(380843);
		btlDbvt								dbvt;
		btlAlignedObjectArray<btlTransform>	transforms;
		btlDbvtBenchmark::NilPolicy			policy;
		transforms.resize(cfgBenchmark6_Iterations);
		for(int i=0;i<transforms.size();++i)
		{
			transforms[i]=btlDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark6_OffsetScale);
		}
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[6] btDbvt::collideTT xform,self: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark6_Iterations;++i)
		{
			btlDbvt::collideTT(dbvt.m_root,dbvt.m_root,transforms[i],policy);		
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark6_Reference)*100/time);
	}
	if(cfgBenchmark7_Enable)
	{// Benchmark 7	
		srand(380843);
		btlDbvt								dbvt;
		btlAlignedObjectArray<btlVector3>		rayorg;
		btlAlignedObjectArray<btlVector3>		raydir;
		btlDbvtBenchmark::NilPolicy			policy;
		rayorg.resize(cfgBenchmark7_Iterations);
		raydir.resize(cfgBenchmark7_Iterations);
		for(int i=0;i<rayorg.size();++i)
		{
			rayorg[i]=btlDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
			raydir[i]=btlDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
		}
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[7] btDbvt::rayTest: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark7_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark7_Iterations;++j)
			{
				btlDbvt::rayTest(dbvt.m_root,rayorg[j],rayorg[j]+raydir[j],policy);
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		unsigned	rays=cfgBenchmark7_Passes*cfgBenchmark7_Iterations;
		printf("%u ms (%i%%),(%u r/s)\r\n",time,(time-cfgBenchmark7_Reference)*100/time,(rays*1000)/time);
	}
	if(cfgBenchmark8_Enable)
	{// Benchmark 8	
		srand(380843);
		btlDbvt								dbvt;
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[8] insert/remove: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark8_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark8_Iterations;++j)
			{
				dbvt.remove(dbvt.insert(btlDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale),0));
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	ir=cfgBenchmark8_Passes*cfgBenchmark8_Iterations;
		printf("%u ms (%i%%),(%u ir/s)\r\n",time,(time-cfgBenchmark8_Reference)*100/time,ir*1000/time);
	}
	if(cfgBenchmark9_Enable)
	{// Benchmark 9	
		srand(380843);
		btlDbvt										dbvt;
		btlAlignedObjectArray<const btlDbvtNode*>	leaves;
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		dbvt.extractLeaves(dbvt.m_root,leaves);
		printf("[9] updates (teleport): ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark9_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark9_Iterations;++j)
			{
				dbvt.update(const_cast<btlDbvtNode*>(leaves[rand()%cfgLeaves]),
					btlDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale));
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	up=cfgBenchmark9_Passes*cfgBenchmark9_Iterations;
		printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark9_Reference)*100/time,up*1000/time);
	}
	if(cfgBenchmark10_Enable)
	{// Benchmark 10	
		srand(380843);
		btlDbvt										dbvt;
		btlAlignedObjectArray<const btlDbvtNode*>	leaves;
		btlAlignedObjectArray<btlVector3>				vectors;
		vectors.resize(cfgBenchmark10_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(btlDbvtBenchmark::RandVector3()*2-btlVector3(1,1,1))*cfgBenchmark10_Scale;
		}
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		dbvt.extractLeaves(dbvt.m_root,leaves);
		printf("[10] updates (jitter): ");
		wallclock.reset();

		for(int i=0;i<cfgBenchmark10_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark10_Iterations;++j)
			{			
				const btlVector3&	d=vectors[j];
				btlDbvtNode*		l=const_cast<btlDbvtNode*>(leaves[rand()%cfgLeaves]);
				btlDbvtVolume		v=btlDbvtVolume::FromMM(l->volume.Mins()+d,l->volume.Maxs()+d);
				dbvt.update(l,v);
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	up=cfgBenchmark10_Passes*cfgBenchmark10_Iterations;
		printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark10_Reference)*100/time,up*1000/time);
	}
	if(cfgBenchmark11_Enable)
	{// Benchmark 11	
		srand(380843);
		btlDbvt										dbvt;
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[11] optimize (incremental): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark11_Passes;++i)
		{
			dbvt.optimizeIncremental(cfgBenchmark11_Iterations);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	op=cfgBenchmark11_Passes*cfgBenchmark11_Iterations;
		printf("%u ms (%i%%),(%u o/s)\r\n",time,(time-cfgBenchmark11_Reference)*100/time,op/time*1000);
	}
	if(cfgBenchmark12_Enable)
	{// Benchmark 12	
		srand(380843);
		btlAlignedObjectArray<btlDbvtVolume>	volumes;
		btlAlignedObjectArray<bool>				results;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			volumes[i]=btlDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		printf("[12] btDbvtVolume notequal: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark12_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					results[k]=NotEqual(volumes[j],volumes[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark12_Reference)*100/time);
	}
	if(cfgBenchmark13_Enable)
	{// Benchmark 13	
		srand(380843);
		btlDbvt								dbvt;
		btlAlignedObjectArray<btlVector3>		vectors;
		btlDbvtBenchmark::NilPolicy			policy;
		vectors.resize(cfgBenchmark13_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(btlDbvtBenchmark::RandVector3()*2-btlVector3(1,1,1)).normalized();
		}
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[13] culling(OCL+fullsort): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark13_Iterations;++i)
		{
			static const btlScalar	offset=0;
			policy.m_depth=-SIMD_INFINITY;
			dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	t=cfgBenchmark13_Iterations;
		printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark13_Reference)*100/time,(t*1000)/time);
	}
	if(cfgBenchmark14_Enable)
	{// Benchmark 14	
		srand(380843);
		btlDbvt								dbvt;
		btlAlignedObjectArray<btlVector3>		vectors;
		btlDbvtBenchmark::P14				policy;
		vectors.resize(cfgBenchmark14_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(btlDbvtBenchmark::RandVector3()*2-btlVector3(1,1,1)).normalized();
		}
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		policy.m_nodes.reserve(cfgLeaves);
		printf("[14] culling(OCL+qsort): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark14_Iterations;++i)
		{
			static const btlScalar	offset=0;
			policy.m_nodes.resize(0);
			dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy,false);
			policy.m_nodes.quickSort(btlDbvtBenchmark::P14::sortfnc);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	t=cfgBenchmark14_Iterations;
		printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark14_Reference)*100/time,(t*1000)/time);
	}
	if(cfgBenchmark15_Enable)
	{// Benchmark 15	
		srand(380843);
		btlDbvt								dbvt;
		btlAlignedObjectArray<btlVector3>		vectors;
		btlDbvtBenchmark::P15				policy;
		vectors.resize(cfgBenchmark15_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(btlDbvtBenchmark::RandVector3()*2-btlVector3(1,1,1)).normalized();
		}
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		policy.m_nodes.reserve(cfgLeaves);
		printf("[15] culling(KDOP+qsort): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark15_Iterations;++i)
		{
			static const btlScalar	offset=0;
			policy.m_nodes.resize(0);
			policy.m_axis=vectors[i];
			dbvt.collideKDOP(dbvt.m_root,&vectors[i],&offset,1,policy);
			policy.m_nodes.quickSort(btlDbvtBenchmark::P15::sortfnc);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	t=cfgBenchmark15_Iterations;
		printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark15_Reference)*100/time,(t*1000)/time);
	}
	if(cfgBenchmark16_Enable)
	{// Benchmark 16	
		srand(380843);
		btlDbvt								dbvt;
		btlAlignedObjectArray<btlDbvtNode*>	batch;
		btlDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		batch.reserve(cfgBenchmark16_BatchCount);
		printf("[16] insert/remove batch(%u): ",cfgBenchmark16_BatchCount);
		wallclock.reset();
		for(int i=0;i<cfgBenchmark16_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark16_BatchCount;++j)
			{
				batch.push_back(dbvt.insert(btlDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale),0));
			}
			for(int j=0;j<cfgBenchmark16_BatchCount;++j)
			{
				dbvt.remove(batch[j]);
			}
			batch.resize(0);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	ir=cfgBenchmark16_Passes*cfgBenchmark16_BatchCount;
		printf("%u ms (%i%%),(%u bir/s)\r\n",time,(time-cfgBenchmark16_Reference)*100/time,int(ir*1000.0/time));
	}
	if(cfgBenchmark17_Enable)
	{// Benchmark 17
		srand(380843);
		btlAlignedObjectArray<btlDbvtVolume>	volumes;
		btlAlignedObjectArray<int>			results;
		btlAlignedObjectArray<int>			indices;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		indices.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			indices[i]=i;
			volumes[i]=btlDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		for(int i=0;i<cfgLeaves;++i)
		{
			btlSwap(indices[i],indices[rand()%cfgLeaves]);
		}
		printf("[17] btDbvtVolume select: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark17_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					const int idx=indices[k];
					results[idx]=Select(volumes[idx],volumes[j],volumes[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark17_Reference)*100/time);
	}
	printf("\r\n\r\n");
}
#endif
