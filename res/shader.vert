#version 120

uniform mat4 uMVPMatrix;

attribute vec3 aPosition;
attribute vec3 aColor;

varying vec3 vColor;

void main()
{
    vColor = aColor;
    gl_Position = uMVPMatrix * vec4(aPosition, 1.0);
}
