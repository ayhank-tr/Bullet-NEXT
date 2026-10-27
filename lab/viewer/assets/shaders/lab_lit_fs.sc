$input v_normal, v_pos

// Bullet NEXT Lab viewer: one directional light, a sky/ground ambient and a little rim.
#include <bgfx_shader.sh>

uniform vec4 uColor;
uniform vec4 uLightDir;

void main()
{
	vec3 n = normalize(v_normal);
	vec3 l = -normalize(uLightDir.xyz);
	float diffuse = max(dot(n, l), 0.0);
	float hemi = 0.5 + 0.5 * n.y;
	vec3 ambient = mix(vec3(0.16, 0.15, 0.14), vec3(0.32, 0.35, 0.40), hemi);
	vec3 rgb = uColor.rgb * (ambient + 0.8 * diffuse);
	gl_FragColor = vec4(pow(rgb, vec3_splat(1.0 / 2.2)), uColor.a);
}
