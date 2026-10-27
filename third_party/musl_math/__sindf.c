/* origin: FreeBSD /usr/src/lib/msun/src/k_sinf.c */
/*
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 * Optimized by Bruce D. Evans.
 */
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#include "libm.h"

/* |sin(x)/x - s(x)| < 2**-37.5 (~[-4.89e-12, 4.824e-12]). */
static const double
S1 = -1.666666664162652355951621530039119534194469451904296875E-1, /* -0.166666666416265235595 */
S2 =  8.3333293858894631755962478791843750514090061187744140625E-3, /*  0.0083333293858894631756 */
S3 = -1.98393348360966317346643084107427057460881769657135009765625E-4, /* -0.000198393348360966317347 */
S4 =  2.71831149398982190639665994236207069434385630302131175994873046875E-6; /*  0.0000027183114939898219064 */

float __sindf(double x)
{
	double_t r, s, w, z;

	/* Try to optimize for parallel evaluation as in __tandf.c. */
	z = x*x;
	w = z*z;
	r = S3 + z*S4;
	s = z*x;
	return (x + s*(S1 + z*S2)) + s*w*r;
}
