// T0.2 symbol isolation: legacy and next Bullet in one program.
#include <cmath>
#include <cstdio>

float bn_symcheck_legacy_world();
float bn_symcheck_next_world();

int main() {
  const float a = bn_symcheck_legacy_world();
  const float b = bn_symcheck_next_world();
  std::printf("legacy ball z = %.6f, next ball z = %.6f\n", a, b);
  // Both worlds must simulate a ball resting on the ground (z close to 0.5).
  const bool ok = std::fabs(a - 0.5f) < 0.1f && std::fabs(b - 0.5f) < 0.1f;
  std::printf("symbol isolation %s\n", ok ? "OK" : "FAILED");
  return ok ? 0 : 1;
}
