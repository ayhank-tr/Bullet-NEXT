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

#include "btAlignedAllocator.h"

int btl_gNumAlignedAllocs = 0;
int btl_gNumAlignedFree = 0;
int btl_gTotalBytesAlignedAllocs = 0;//detect memory leaks

static void *btlAllocDefault(size_t size)
{
	return malloc(size);
}

static void btlFreeDefault(void *ptr)
{
	free(ptr);
}

static btlAllocFunc *sAllocFunc = btlAllocDefault;
static btlFreeFunc *sFreeFunc = btlFreeDefault;



#if defined (BT_HAS_ALIGNED_ALLOCATOR)
#include <malloc.h>
static void *btlAlignedAllocDefault(size_t size, int alignment)
{
	return _aligned_malloc(size, (size_t)alignment);
}

static void btlAlignedFreeDefault(void *ptr)
{
	_aligned_free(ptr);
}
#elif defined(__CELLOS_LV2__)
#include <stdlib.h>

static inline void *btlAlignedAllocDefault(size_t size, int alignment)
{
	return memalign(alignment, size);
}

static inline void btlAlignedFreeDefault(void *ptr)
{
	free(ptr);
}
#else





static inline void *btlAlignedAllocDefault(size_t size, int alignment)
{
  void *ret;
  char *real;
  real = (char *)sAllocFunc(size + sizeof(void *) + (alignment-1));
  if (real) {
	ret = btlAlignPointer(real + sizeof(void *),alignment);
    *((void **)(ret)-1) = (void *)(real);
  } else {
    ret = (void *)(real);
  }
  return (ret);
}

static inline void btlAlignedFreeDefault(void *ptr)
{
  void* real;

  if (ptr) {
    real = *((void **)(ptr)-1);
    sFreeFunc(real);
  }
}
#endif


static btlAlignedAllocFunc *sAlignedAllocFunc = btlAlignedAllocDefault;
static btlAlignedFreeFunc *sAlignedFreeFunc = btlAlignedFreeDefault;

void btlAlignedAllocSetCustomAligned(btlAlignedAllocFunc *allocFunc, btlAlignedFreeFunc *freeFunc)
{
  sAlignedAllocFunc = allocFunc ? allocFunc : btlAlignedAllocDefault;
  sAlignedFreeFunc = freeFunc ? freeFunc : btlAlignedFreeDefault;
}

void btlAlignedAllocSetCustom(btlAllocFunc *allocFunc, btlFreeFunc *freeFunc)
{
  sAllocFunc = allocFunc ? allocFunc : btlAllocDefault;
  sFreeFunc = freeFunc ? freeFunc : btlFreeDefault;
}

#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
//this generic allocator provides the total allocated number of bytes
#include <stdio.h>

void*   btlAlignedAllocInternal  (size_t size, int alignment,int line,char* filename)
{
 void *ret;
 char *real;

 btl_gTotalBytesAlignedAllocs += size;
 btl_gNumAlignedAllocs++;

 
 real = (char *)sAllocFunc(size + 2*sizeof(void *) + (alignment-1));
 if (real) {
   ret = (void*) btlAlignPointer(real + 2*sizeof(void *), alignment);
   *((void **)(ret)-1) = (void *)(real);
       *((int*)(ret)-2) = size;

 } else {
   ret = (void *)(real);//??
 }

 printf("allocation#%d at address %x, from %s,line %d, size %d\n",btl_gNumAlignedAllocs,real, filename,line,size);

 int* ptr = (int*)ret;
 *ptr = 12;
 return (ret);
}

void    btlAlignedFreeInternal   (void* ptr,int line,char* filename)
{

 void* real;
 btl_gNumAlignedFree++;

 if (ptr) {
   real = *((void **)(ptr)-1);
       int size = *((int*)(ptr)-2);
       btl_gTotalBytesAlignedAllocs -= size;

	   printf("free #%d at address %x, from %s,line %d, size %d\n",btl_gNumAlignedFree,real, filename,line,size);

   sFreeFunc(real);
 } else
 {
	 printf("NULL ptr\n");
 }
}

#else //BT_DEBUG_MEMORY_ALLOCATIONS

void*	btlAlignedAllocInternal	(size_t size, int alignment)
{
	btl_gNumAlignedAllocs++;
	void* ptr;
	ptr = sAlignedAllocFunc(size, alignment);
//	printf("btAlignedAllocInternal %d, %x\n",size,ptr);
	return ptr;
}

void	btlAlignedFreeInternal	(void* ptr)
{
	if (!ptr)
	{
		return;
	}

	btl_gNumAlignedFree++;
//	printf("btAlignedFreeInternal %x\n",ptr);
	sAlignedFreeFunc(ptr);
}

#endif //BT_DEBUG_MEMORY_ALLOCATIONS

