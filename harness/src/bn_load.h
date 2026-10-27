// Bullet NEXT Lab - builds a harness World from a scene file (FL, CL.5).
#ifndef BN_LOAD_H
#define BN_LOAD_H

#include "bn_world.h"
#include "bnl_scene.h"

BN_NAMESPACE_BEGIN

Settings settingsOf(const bnl::SettingsDesc& s);
Settings settingsOf(const bnl::SceneDesc& sc);  // with the scene's host profile
btCollisionShape* makeShape(const bnl::ShapeDesc& s);

// Creates every object of the scene in file order, then applies initial velocities.
void build(World& w, const bnl::SceneDesc& s);

// Applies the scene's scripted actions for one frame (called before World::step).
void act(World& w, const bnl::SceneDesc& s, int frame);

// A frame as Blender builds one (setIdentity, setOrigin, setRotation): RB_body_new, make_constraint_transforms.
inline btTransform toBtRb(const bnl::Xform& x) {
  btTransform t;
  t.setIdentity();
  t.setOrigin(btVector3(x.p.x, x.p.y, x.p.z));
  t.setRotation(btQuaternion(x.q.x, x.q.y, x.q.z, x.q.w));
  return t;
}

inline btVector3 toBt(const bnl::Vec3& v) { return btVector3(v.x, v.y, v.z); }
inline btTransform toBt(const bnl::Xform& x) {
  if (x.q.x == 0 && x.q.y == 0 && x.q.z == 0 && x.q.w == 1) {  // exactly what the F0 scenes built
    btTransform t = btTransform::getIdentity();
    t.setOrigin(toBt(x.p));
    return t;
  }
  return btTransform(btQuaternion(x.q.x, x.q.y, x.q.z, x.q.w), toBt(x.p));
}

BN_NAMESPACE_END  // namespace bn

#endif
