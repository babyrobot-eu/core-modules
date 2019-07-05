/**
 * From the OpenGL Programming wikibook: http://en.wikibooks.org/wiki/OpenGL_Programming
 * This file is in the public domain.
 * Contributors: Martin Kraus, Sylvain Beucler
 */
attribute vec4 v_coord;
attribute vec3 v_normal;
attribute vec3 v_colour;
varying vec4 position;  // position of the vertex (and fragment) in world space
varying vec3 varyingNormalDirection;  // surface normal vector in world space
uniform mat4 m, v, p;
uniform mat3 m_3x3_inv_transp;
varying vec3 f_colour;

void main()
{
  position = m * v_coord;
  varyingNormalDirection = normalize(m_3x3_inv_transp * v_normal);

  mat4 mvp = p*v*m;
  gl_Position = mvp * v_coord;
  f_colour = v_colour;
}
