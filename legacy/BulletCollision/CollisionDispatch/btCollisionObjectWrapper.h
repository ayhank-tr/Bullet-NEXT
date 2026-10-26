#ifndef BT_COLLISION_OBJECT_WRAPPER_H
#define BT_COLLISION_OBJECT_WRAPPER_H

///btCollisionObjectWrapperis an internal data structure. 
///Most users can ignore this and use btCollisionObject and btCollisionShape instead
class btlCollisionShape;
class btlCollisionObject;
class btlTransform;
#include "LinearMath/btScalar.h" // for SIMD_FORCE_INLINE definition

#define BT_DECLARE_STACK_ONLY_OBJECT \
	private: \
		void* operator new(size_t size); \
		void operator delete(void*);

struct btlCollisionObjectWrapper;
struct btlCollisionObjectWrapper
{
BT_DECLARE_STACK_ONLY_OBJECT

private:
	btlCollisionObjectWrapper(const btlCollisionObjectWrapper&); // not implemented. Not allowed.
	btlCollisionObjectWrapper* operator=(const btlCollisionObjectWrapper&);

public:
	const btlCollisionObjectWrapper* m_parent;
	const btlCollisionShape* m_shape;
	const btlCollisionObject* m_collisionObject;
	const btlTransform& m_worldTransform;
	int		m_partId;
	int		m_index;

	btlCollisionObjectWrapper(const btlCollisionObjectWrapper* parent, const btlCollisionShape* shape, const btlCollisionObject* collisionObject, const btlTransform& worldTransform, int partId, int index)
	: m_parent(parent), m_shape(shape), m_collisionObject(collisionObject), m_worldTransform(worldTransform),
	m_partId(partId), m_index(index)
	{}

	SIMD_FORCE_INLINE const btlTransform& getWorldTransform() const { return m_worldTransform; }
	SIMD_FORCE_INLINE const btlCollisionObject* getCollisionObject() const { return m_collisionObject; }
	SIMD_FORCE_INLINE const btlCollisionShape* getCollisionShape() const { return m_shape; }
};

#endif //BT_COLLISION_OBJECT_WRAPPER_H
