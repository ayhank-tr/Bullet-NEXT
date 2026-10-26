#include "btPolarDecomposition.h"
#include "btMinMax.h"

namespace
{
  btlScalar abs_column_sum(const btlMatrix3x3& a, int i)
  {
    return btlFabs(a[0][i]) + btlFabs(a[1][i]) + btlFabs(a[2][i]);
  }

  btlScalar abs_row_sum(const btlMatrix3x3& a, int i)
  {
    return btlFabs(a[i][0]) + btlFabs(a[i][1]) + btlFabs(a[i][2]);
  }

  btlScalar p1_norm(const btlMatrix3x3& a)
  {
    const btlScalar sum0 = abs_column_sum(a,0);
    const btlScalar sum1 = abs_column_sum(a,1);
    const btlScalar sum2 = abs_column_sum(a,2);
    return btlMax(btlMax(sum0, sum1), sum2);
  }

  btlScalar pinf_norm(const btlMatrix3x3& a)
  {
    const btlScalar sum0 = abs_row_sum(a,0);
    const btlScalar sum1 = abs_row_sum(a,1);
    const btlScalar sum2 = abs_row_sum(a,2);
    return btlMax(btlMax(sum0, sum1), sum2);
  }
}

const btlScalar btlPolarDecomposition::DEFAULT_TOLERANCE = btlScalar(0.0001);
const unsigned int btlPolarDecomposition::DEFAULT_MAX_ITERATIONS = 16;

btlPolarDecomposition::btlPolarDecomposition(btlScalar tolerance, unsigned int btl_maxIterations)
: m_tolerance(tolerance)
, m_maxIterations(btl_maxIterations)
{
}

unsigned int btlPolarDecomposition::decompose(const btlMatrix3x3& a, btlMatrix3x3& u, btlMatrix3x3& h) const
{
  // Use the 'u' and 'h' matrices for intermediate calculations
  u = a;
  h = a.inverse();

  for (unsigned int i = 0; i < m_maxIterations; ++i)
  {
    const btlScalar h_1 = p1_norm(h);
    const btlScalar h_inf = pinf_norm(h);
    const btlScalar u_1 = p1_norm(u);
    const btlScalar u_inf = pinf_norm(u);

    const btlScalar h_norm = h_1 * h_inf;
    const btlScalar u_norm = u_1 * u_inf;

    // The matrix is effectively singular so we cannot invert it
    if (btlFuzzyZero(h_norm) || btlFuzzyZero(u_norm))
      break;

    const btlScalar gamma = btlPow(h_norm / u_norm, 0.25f);
    const btlScalar inv_gamma = btlScalar(1.0) / gamma;

    // Determine the delta to 'u'
    const btlMatrix3x3 delta = (u * (gamma - btlScalar(2.0)) + h.transpose() * inv_gamma) * btlScalar(0.5);

    // Update the matrices
    u += delta;
    h = u.inverse();

    // Check for convergence
    if (p1_norm(delta) <= m_tolerance * u_1)
    {
      h = u.transpose() * a;
      h = (h + h.transpose()) * 0.5;
      return i;
    }
  }

  // The algorithm has failed to converge to the specified tolerance, but we
  // want to make sure that the matrices returned are in the right form.
  h = u.transpose() * a;
  h = (h + h.transpose()) * 0.5;

  return m_maxIterations;
}

unsigned int btlPolarDecomposition::btl_maxIterations() const
{
  return m_maxIterations;
}

unsigned int btl_polarDecompose(const btlMatrix3x3& a, btlMatrix3x3& u, btlMatrix3x3& h)
{
  static btlPolarDecomposition polar;
  return polar.decompose(a, u, h);
}

