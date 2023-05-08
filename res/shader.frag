#version 120
uniform float time;
const float PI = 3.14159265;

void main()
{
    vec3 color = vec3(0.5) + 0.5 * vec3(sin(time), cos(time), sin(2.0 * time + PI/2.0));
    gl_FragColor = vec4(color, 1.0);
}