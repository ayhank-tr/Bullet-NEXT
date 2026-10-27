/* origin: FreeBSD /usr/src/lib/msun/src/k_tanf.c */
/*
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 * Optimized by Bruce D. Evans.
 */
/*
 * ====================================================
 * Copyright 2004 Sun Microsystems, Inc.  All Rights Reserved.
 *
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#include "libm.h"

/* |tan(x)/x - t(x)| < 2**-25.5 (~[-2e-08, 2e-08]). */
static const double T[] = {
  3.33331395030791399758385296081542037427425384521484375E-1, /* 0.333331395030791399758 */
  1.33392002712976742717643219293677248060703277587890625E-1, /* 0.133392002712976742718 */
  5.338123784456703935230592605876154266297817230224609375E-2, /* 0.0533812378445670393523 */
  2.4528318116654727887304687783398549072444438934326171875E-2, /* 0.0245283181166547278873 */
  2.974357433599673049273715008666840731166303157806396484375E-3, /* 0.00297435743359967304927 */
  9.46564784943673166728306256345604197122156620025634765625E-3, /* 0.00946564784943673166728 */
};

float __tandf(double x, int odd)
{
	double_t z,r,w,s,t,u;

	z = x*x;
	/*
	 * Split up the polynomial into small independent terms to give
	 * opportunities for parallel evaluation.  The chosen splitting is
	 * micro-optimized for Athlons (XP, X64).  It costs 2 multiplications
	 * relative to Horner's method on sequential machines.
	 *
	 * We add the small terms from lowest degree up for efficiency on
	 * non-sequential machines (the lowest degree terms tend to be ready
	 * earlier).  Apart from this, we don't care about order of
	 * operations, and don't need to to care since we have precision to
	 * spare.  However, the chosen splitting is good for accuracy too,
	 * and would give results as accurate as Horner's method if the
	 * small terms were added from highest degree down.
	 */
	r = T[4] + z*T[5];
	t = T[2] + z*T[3];
	w = z*z;
	s = z*x;
	u = T[0] + z*T[1];
	r = (x + s*u) + (s*w)*(t + w*r);
	return odd ? -1.0/r : r;
}
