/* Bullet NEXT (FL, CL.10): stand-in for musl's internal libm.h, so that musl's float math
 * functions build anywhere (GCC, Clang, MSVC, Emscripten) as private copies named bn_*.
 *
 * Why: sinf, cosf, atan2f... are not the same on every system. glibc, the Windows C runtime and
 * the browser (musl) return different last bits, and glibc even picks a different routine when
 * the CPU has FMA, so the same Bullet scene gave different results on different machines and in
 * the browser. Bullet NEXT calls these copies instead (BN_PORTABLE_MATH, btScalar.h), and they
 * are compiled with the strict floating point rules of the rest of Bullet NEXT (no FMA
 * contraction, no fast-math), so they give the same bits everywhere.
 *
 * Only the helpers the vendored files use are here. The files themselves are musl 1.2.4 as
 * shipped in Emscripten 3.1.47, unchanged except for the <features.h> include in the *_data.h
 * headers (see README.md). MIT licence: COPYRIGHT. */
#ifndef BN_MUSL_LIBM_H
#define BN_MUSL_LIBM_H

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif
#include <float.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif
#ifndef FLT_EVAL_METHOD
#define FLT_EVAL_METHOD 0
#endif

/* Private names: never clash with the system's libm (or with musl itself in the browser). */
#define sinf bn_sinf
#define cosf bn_cosf
#define tanf bn_tanf
#define asinf bn_asinf
#define acosf bn_acosf
#define atanf bn_atanf
#define atan2f bn_atan2f
#define expf bn_expf
#define logf bn_logf
#define powf bn_powf
#define __sindf bn_musl_sindf
#define __cosdf bn_musl_cosdf
#define __tandf bn_musl_tandf
#define __rem_pio2f bn_musl_rem_pio2f
#define __rem_pio2_large bn_musl_rem_pio2_large
#define __exp2f_data bn_musl_exp2f_data
#define __logf_data bn_musl_logf_data
#define __powf_log2_data bn_musl_powf_log2_data

#define hidden
#define WANT_ROUNDING 1
#define WANT_SNAN 0
#define TOINT_INTRINSICS 0
#define issignalingf_inline(x) 0
#define predict_true(x) (x)
#define predict_false(x) (x)

#ifdef _MSC_VER
#define BN_MUSL_INLINE static __inline
#else
#define BN_MUSL_INLINE static inline
#endif

BN_MUSL_INLINE uint32_t bn_musl_asuint(float f) { union { float f; uint32_t i; } u; u.f = f; return u.i; }
BN_MUSL_INLINE float bn_musl_asfloat(uint32_t i) { union { uint32_t i; float f; } u; u.i = i; return u.f; }
BN_MUSL_INLINE uint64_t bn_musl_asuint64(double f) { union { double f; uint64_t i; } u; u.f = f; return u.i; }
BN_MUSL_INLINE double bn_musl_asdouble(uint64_t i) { union { uint64_t i; double f; } u; u.i = i; return u.f; }
#define asuint(f) bn_musl_asuint(f)
#define asfloat(i) bn_musl_asfloat(i)
#define asuint64(f) bn_musl_asuint64(f)
#define asdouble(i) bn_musl_asdouble(i)
#define GET_FLOAT_WORD(w, d) do { (w) = asuint(d); } while (0)
#define SET_FLOAT_WORD(d, w) do { (d) = asfloat(w); } while (0)

BN_MUSL_INLINE float eval_as_float(float x) { return x; }
BN_MUSL_INLINE double eval_as_double(double x) { return x; }
BN_MUSL_INLINE float fp_barrierf(float x) { volatile float y = x; return y; }
/* Evaluates x for its floating point exception flags only. */
#define FORCE_EVAL(x) do { volatile double bn_musl_force = (double)(x); (void)bn_musl_force; } while (0)

/* musl's error helpers (src/math/__math_*.c): the same values, without errno. */
BN_MUSL_INLINE float bn_musl_xflowf(uint32_t sign, float y) { return fp_barrierf(sign ? -y : y) * y; }
BN_MUSL_INLINE float __math_oflowf(uint32_t sign) { return bn_musl_xflowf(sign, 1.58456325028528675187087900672E29f); }
BN_MUSL_INLINE float __math_uflowf(uint32_t sign) { return bn_musl_xflowf(sign, 2.524354896707237777317531408904915934954260592348873615264892578125E-29f); }
BN_MUSL_INLINE float __math_invalidf(float x) { return (x - x) / (x - x); }
BN_MUSL_INLINE float __math_divzerof(uint32_t sign) { return fp_barrierf(sign ? -1.0f : 1.0f) / 0.0f; }

/* The renamed functions call each other (atan2f calls atanf...), so declare them all. */
float sinf(float);
float cosf(float);
float tanf(float);
float asinf(float);
float acosf(float);
float atanf(float);
float atan2f(float, float);
float expf(float);
float logf(float);
float powf(float, float);
int __rem_pio2f(float, double *);
int __rem_pio2_large(double *, double *, int, int, int);
float __sindf(double);
float __cosdf(double);
float __tandf(double, int);

#endif
