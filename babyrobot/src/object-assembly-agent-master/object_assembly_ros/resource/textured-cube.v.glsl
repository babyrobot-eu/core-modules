attribute vec4 v_coord;
attribute vec2 texcoord;
varying vec2 f_texcoord;
varying vec4 position;
uniform mat4 m, v, p;
uniform mat3 m_3x3_inv_transp;

void main(void) {
  position = m * v_coord;
  mat4 mvp = p*v*m;
  f_texcoord = texcoord;
  gl_Position = mvp * v_coord;

}
