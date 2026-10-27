$input a_position, a_normal
$output v_normal, v_pos

// Bullet NEXT Lab viewer: flat-shaded bodies.
#include <bgfx_shader.sh>

void main()
{
	vec4 world = mul(u_model[0], vec4(a_position, 1.0));
	v_pos = world.xyz;
	v_normal = mul(u_model[0], vec4(a_normal.xyz, 0.0)).xyz;
	gl_Position = mul(u_viewProj, world);
}
