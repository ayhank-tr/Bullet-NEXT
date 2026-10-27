$input a_position, a_color0
$output v_color0

// Bullet NEXT Lab viewer: grid and contact lines (world space).
#include <bgfx_shader.sh>

void main()
{
	v_color0 = a_color0;
	gl_Position = mul(u_viewProj, vec4(a_position, 1.0));
}
