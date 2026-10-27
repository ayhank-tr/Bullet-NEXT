/* origin: FreeBSD /usr/src/lib/msun/src/k_cosf.c */
/*
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 * Debugged and optimized by Bruce D. Evans.
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

/* |cos(x) - c(x)| < 2**-34.1 (~[-5.37e-11, 5.295e-11]). */
static const double
C0  = -4.99999997251031003120402829154045321047306060791015625E-1, /* -0.499999997251031003120 */
C1  =  4.166662332373906318938594495193683542311191558837890625E-2, /*  0.0416666233237390631894 */
C2  = -1.38867637746099294691892733766280798590742051601409912109375E-3, /* -0.00138867637746099294692 */
C3  =  2.439044879627740906543643950588062807582900859415531158447265625E-5; /*  0.0000243904487962774090654 */

float __cosdf(double x)
{
	double_t r, w, z;

	/* Try to optimize for parallel evaluation as in __tandf.c. */
	z = x*x;
	w = z*z;
	r = C2+z*C3;
	return ((1.0+z*C0) + w*C1) + (w*z)*r;
}
