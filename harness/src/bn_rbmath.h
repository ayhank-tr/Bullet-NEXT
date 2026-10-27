// Bullet NEXT Lab - Blender's interpolation of keyframed rigid bodies between substeps (FL, CL.7).
// Shared by the Lab's Blender rigid body profile (bn_world.cpp) and the reference driver over
// Blender's own rb_bullet_api.cpp (lab/reference/bnl_rbref.cpp). Floats only, as in Blender.
#ifndef BN_RBMATH_H
#define BN_RBMATH_H

#include <cmath>

// With Bullet NEXT's portable math (btScalar.h), the interpolation uses it too, so the Blender
// profile gives the same results on every system and in the browser.
#ifdef BN_PORTABLE_MATH
extern "C" {
float bn_sinf(float);
float bn_acosf(float);
}
#define BN_RB_SIN bn_sinf
#define BN_RB_ACOS bn_acosf
#else
#define BN_RB_SIN std::sin
#define BN_RB_ACOS std::acos
#endif

// static: each Lab engine compiles its own copy (with or without the portable math).
namespace bnrb {

// interp_v3_v3v3 (blenlib math_vector)
static inline void interpV3(float r[3], const float a[3], const float b[3], float t) {
  const float s = 1.0f - t;
  r[0] = s * a[0] + t * b[0];
  r[1] = s * a[1] + t * b[1];
  r[2] = s * a[2] + t * b[2];
}

// interp_qt_qtqt (blenlib math_rotation), quaternions as w, x, y, z
static inline void interpQt(float q[4], const float a[4], const float b[4], float t) {
  float quat[4];
  float cosom = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (cosom < 0.0f) {
    cosom = -cosom;
    for (int i = 0; i < 4; ++i) quat[i] = -a[i];
  } else {
    for (int i = 0; i < 4; ++i) quat[i] = a[i];
  }
  float sc1, sc2;
  if ((1.0f - cosom) > 0.0001f) {
    const float omega = BN_RB_ACOS(cosom), sinom = BN_RB_SIN(omega);
    sc1 = BN_RB_SIN((1.0f - t) * omega) / sinom;
    sc2 = BN_RB_SIN(t * omega) / sinom;
  } else {
    sc1 = 1.0f - t;
    sc2 = t;
  }
  for (int i = 0; i < 4; ++i) q[i] = sc1 * quat[i] + sc2 * b[i];
}

}  // namespace bnrb

#endif
