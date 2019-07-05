#ifndef MESH_HPP
#define MESH_HPP

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <GL/glew.h>
#include <GL/glut.h>
/* Using SDL2 for the base window and OpenGL context init */
//#include "SDL.h"
/* Using SDL2_image to load PNG & JPG in memory */
#include "SDL_image.h"


//TODO: remove globals
    GLint attribute_v_coord = -1;
    GLint attribute_v_normal = -1;
    GLint attribute_v_colour = -1;
    GLint uniform_m = -1, uniform_v = -1, uniform_p = -1, uniform_mvp = -1;
    GLint uniform_m_3x3_inv_transp = -1, uniform_v_inv = -1;

    GLint attribute_texcoord = -1;
    GLint uniform_mytexture = -1;
    bool use_png_texture = false;

class Mesh {
private:
    GLuint vbo_vertices, vbo_normals, ibo_elements, vbo_colours, vbo_cube_texcoords;

public:
    GLuint texture_id;
    std::vector<glm::vec4> vertices;
    std::vector<glm::vec3> normals;
    std::vector<GLushort> elements;
    glm::mat4 object2world;
    std::vector<glm::vec3> colours;
    GLfloat cube_texcoords[2*4*6] = {
	    // front
	    0.0, 0.0,
	    1.0, 0.0,
	    1.0, 1.0,
	    0.0, 1.0,
    };
    char *img_name=NULL;


    Mesh() : vbo_vertices(0), vbo_normals(0), ibo_elements(0), vbo_colours(0), object2world(glm::mat4(1)), texture_id(0), vbo_cube_texcoords(0) {}
    ~Mesh() {
        if (vbo_vertices != 0)
          glDeleteBuffers(1, &vbo_vertices);
        if (vbo_normals != 0)
          glDeleteBuffers(1, &vbo_normals);
        if (ibo_elements != 0)
          glDeleteBuffers(1, &ibo_elements);
        if (vbo_colours != 0)
          glDeleteBuffers(1, &vbo_colours);
        if (use_png_texture)
        {
          if (vbo_cube_texcoords != 0)
          	glDeleteBuffers(1, &vbo_cube_texcoords);
          if (texture_id != 0)
            glDeleteTextures(1, &texture_id);
        }
        if (img_name != NULL)
          delete [] img_name;
    }

  /**
   * Store object vertices, normals and/or elements in graphic card
   * buffers
   */
    void upload()
    {
        if (this->vertices.size() > 0)
        {
          glGenBuffers(1, &this->vbo_vertices);
          glBindBuffer(GL_ARRAY_BUFFER, this->vbo_vertices);
          glBufferData(GL_ARRAY_BUFFER, this->vertices.size() * sizeof(this->vertices[0]),
		       this->vertices.data(), GL_STATIC_DRAW);
        }
        
        if (this->normals.size() > 0)
        {
          glGenBuffers(1, &this->vbo_normals);
          glBindBuffer(GL_ARRAY_BUFFER, this->vbo_normals);
          glBufferData(GL_ARRAY_BUFFER, this->normals.size() * sizeof(this->normals[0]),
		       this->normals.data(), GL_STATIC_DRAW);
        }
        
        if (this->colours.size() > 0)
        {
          glGenBuffers(1, &this->vbo_colours);
          glBindBuffer(GL_ARRAY_BUFFER, this->vbo_colours);
          glBufferData(GL_ARRAY_BUFFER, this->colours.size() * sizeof(this->colours[0]),
		       this->colours.data(), GL_STATIC_DRAW);
        }

        if (this->elements.size() > 0)
        {
          glGenBuffers(1, &this->ibo_elements);
          glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->ibo_elements);
          glBufferData(GL_ELEMENT_ARRAY_BUFFER, this->elements.size() * sizeof(this->elements[0]),
		       this->elements.data(), GL_STATIC_DRAW);
        }

        if (use_png_texture)
        {
            for (int i = 1; i < 6; i++)
                memcpy(&cube_texcoords[i*4*2], &cube_texcoords[0], 2*4*sizeof(GLfloat));
            glGenBuffers(1, &vbo_cube_texcoords);
            glBindBuffer(GL_ARRAY_BUFFER, vbo_cube_texcoords);
            glBufferData(GL_ARRAY_BUFFER, sizeof(cube_texcoords), cube_texcoords, GL_STATIC_DRAW);
	        SDL_Surface* res_texture = IMG_Load(img_name);
	        if (res_texture == NULL) {
		        std::cerr << "IMG_Load: " << SDL_GetError() << std::endl;
		        exit(-1);
	        }
	        glGenTextures(1, &texture_id);
	        glBindTexture(GL_TEXTURE_2D, texture_id);
	        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	        glTexImage2D(GL_TEXTURE_2D, // target
		        0,  // level, 0 = base, no minimap,
		        GL_RGBA, // internalformat
		        res_texture->w,  // width
		        res_texture->h,  // height
		        0,  // border, always 0 in OpenGL ES
		        GL_RGB,  // format
		        GL_UNSIGNED_BYTE, // type
		        res_texture->pixels);
	        SDL_FreeSurface(res_texture);
        }
    }

  /**
   * Draw the object
   */
  void draw() {

    if (this->vbo_vertices != 0) {
      glEnableVertexAttribArray(attribute_v_coord);
      glBindBuffer(GL_ARRAY_BUFFER, this->vbo_vertices);
      glVertexAttribPointer(
        attribute_v_coord,  // attribute
        4,                  // number of elements per vertex, here (x,y,z,w)
        GL_FLOAT,           // the type of each element
        GL_FALSE,           // take our values as-is
        0,                  // no extra data between each position
        0                   // offset of first element
      );
    }

    if (this->vbo_normals != 0) {
      glEnableVertexAttribArray(attribute_v_normal);
      glBindBuffer(GL_ARRAY_BUFFER, this->vbo_normals);
      glVertexAttribPointer(
        attribute_v_normal, // attribute
        3,                  // number of elements per vertex, here (x,y,z)
        GL_FLOAT,           // the type of each element
        GL_FALSE,           // take our values as-is
        0,                  // no extra data between each position
        0                   // offset of first element
      );
    }
    
    if (this->vbo_colours != 0) {
      glEnableVertexAttribArray(attribute_v_colour);
      glBindBuffer(GL_ARRAY_BUFFER, this->vbo_colours);
      glVertexAttribPointer(
        attribute_v_colour, // attribute
        3,                  // number of elements per vertex, here (R,G,B)
        GL_FLOAT,           // the type of each element
        GL_FALSE,           // take our values as-is
        0,                  // no extra data between each position
        0                   // offset of first element
      );
    }

    if (use_png_texture)
    {
        glEnableVertexAttribArray(attribute_texcoord);
	    glBindBuffer(GL_ARRAY_BUFFER, vbo_cube_texcoords);
	    glVertexAttribPointer(
		    attribute_texcoord, // attribute
		    2,                  // number of elements per vertex, here (x,y)
		    GL_FLOAT,           // the type of each element
		    GL_FALSE,           // take our values as-is
		    0,                  // no extra data between each position
		    0                   // offset of first element
	    );
    }
    if (use_png_texture)
    {
        glActiveTexture(GL_TEXTURE0);
	    glUniform1i(uniform_mytexture, /*GL_TEXTURE*/0);
	    glBindTexture(GL_TEXTURE_2D, texture_id);
    }
    /* Apply object's transformation matrix */
    glUniformMatrix4fv(uniform_m, 1, GL_FALSE, glm::value_ptr(this->object2world));
    /* Transform normal vectors with transpose of inverse of upper left
       3x3 model matrix (ex-gl_NormalMatrix): */
    glm::mat3 m_3x3_inv_transp = glm::transpose(glm::inverse(glm::mat3(this->object2world)));
    glUniformMatrix3fv(uniform_m_3x3_inv_transp, 1, GL_FALSE, glm::value_ptr(m_3x3_inv_transp));
    
    /* Push each element in buffer_vertices to the vertex shader */
    if (this->ibo_elements != 0) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->ibo_elements);
      int size;  glGetBufferParameteriv(GL_ELEMENT_ARRAY_BUFFER, GL_BUFFER_SIZE, &size);
      glDrawElements(GL_TRIANGLES, size/sizeof(GLushort), GL_UNSIGNED_SHORT, 0);
    } else {
      glDrawArrays(GL_TRIANGLES, 0, this->vertices.size());
    }

    if (this->vbo_normals != 0)
      glDisableVertexAttribArray(attribute_v_normal);
    if (this->vbo_vertices != 0)
      glDisableVertexAttribArray(attribute_v_coord);
    if (this->vbo_colours != 0)
      glDisableVertexAttribArray(attribute_v_colour);
    if (use_png_texture)
        glDisableVertexAttribArray(attribute_texcoord);
  }

  /**
   * Draw object bounding box
   */
    void draw_bbox() {
        if (this->vertices.size() == 0)
            return;

        // Cube 1x1x1, centered on origin
        GLfloat vertices[] = {
              -0.5, -0.5, -0.5, 1.0,
               0.5, -0.5, -0.5, 1.0,
               0.5,  0.5, -0.5, 1.0,
              -0.5,  0.5, -0.5, 1.0,
              -0.5, -0.5,  0.5, 1.0,
               0.5, -0.5,  0.5, 1.0,
               0.5,  0.5,  0.5, 1.0,
              -0.5,  0.5,  0.5, 1.0,
        };
        GLuint vbo_vertices;
        glGenBuffers(1, &vbo_vertices);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        GLushort elements[] = {
              0, 1, 2, 3,
              4, 5, 6, 7,
              0, 4, 1, 5, 2, 6, 3, 7
        };
        GLuint ibo_elements;
        glGenBuffers(1, &ibo_elements);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(elements), elements, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

        GLfloat
              min_x, max_x,
              min_y, max_y,
              min_z, max_z;
        min_x = max_x = this->vertices[0].x;
        min_y = max_y = this->vertices[0].y;
        min_z = max_z = this->vertices[0].z;
        for (unsigned int i = 0; i < this->vertices.size(); i++) {
              if (this->vertices[i].x < min_x) min_x = this->vertices[i].x;
              if (this->vertices[i].x > max_x) max_x = this->vertices[i].x;
              if (this->vertices[i].y < min_y) min_y = this->vertices[i].y;
              if (this->vertices[i].y > max_y) max_y = this->vertices[i].y;
              if (this->vertices[i].z < min_z) min_z = this->vertices[i].z;
              if (this->vertices[i].z > max_z) max_z = this->vertices[i].z;
        }
        glm::vec3 size = glm::vec3(max_x-min_x, max_y-min_y, max_z-min_z);
        glm::vec3 center = glm::vec3((min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2);
        glm::mat4 transform = glm::scale(glm::mat4(1), size) * glm::translate(glm::mat4(1), center);

        /* Apply object's transformation matrix */
        glm::mat4 m = this->object2world * transform;
        glUniformMatrix4fv(uniform_m, 1, GL_FALSE, glm::value_ptr(m));

        glBindBuffer(GL_ARRAY_BUFFER, vbo_vertices);
        glEnableVertexAttribArray(attribute_v_coord);
        glVertexAttribPointer(
              attribute_v_coord,  // attribute
              4,                  // number of elements per vertex, here (x,y,z,w)
              GL_FLOAT,           // the type of each element
              GL_FALSE,           // take our values as-is
              0,                  // no extra data between each position
              0                   // offset of first element
        );

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_elements);
            glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, 0);
            glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_SHORT, (GLvoid*)(4*sizeof(GLushort)));
            glDrawElements(GL_LINES, 8, GL_UNSIGNED_SHORT, (GLvoid*)(8*sizeof(GLushort)));
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            
            glDisableVertexAttribArray(attribute_v_coord);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
            
            glDeleteBuffers(1, &vbo_vertices);
            glDeleteBuffers(1, &ibo_elements);
    }

    void set_uniform_colour(glm::vec3 colour)
    {
//glDeleteBuffers(1, &vbo_colours);
        colours.clear();

        for (int j = 0; j < vertices.size(); j++)
            colours.push_back(colour);
  
        if (this->colours.size() > 0 && this->vbo_colours != 0) {
            glDeleteBuffers(1, &vbo_colours);
            glGenBuffers(1, &this->vbo_colours);
            glBindBuffer(GL_ARRAY_BUFFER, this->vbo_colours);
            glBufferData(GL_ARRAY_BUFFER, this->colours.size() * sizeof(this->colours[0]),
		    this->colours.data(), GL_STATIC_DRAW);
        }
    }

    void load_new_texture()
    {
std::cout << "loading"<< img_name <<"\n";
        glDeleteTextures(1, &texture_id);

        if (vbo_cube_texcoords != 0)
          	glDeleteBuffers(1, &vbo_cube_texcoords);
        glGenBuffers(1, &vbo_cube_texcoords);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_cube_texcoords);
        glBufferData(GL_ARRAY_BUFFER, sizeof(cube_texcoords), cube_texcoords, GL_STATIC_DRAW);

        SDL_Surface* res_texture = IMG_Load(img_name);
        if (res_texture == NULL) {
	        std::cerr << "IMG_Load: " << SDL_GetError() << std::endl;
	        exit(-1);
        }

        glGenTextures(1, &texture_id);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, // target
	        0,  // level, 0 = base, no minimap,
	        GL_RGBA, // internalformat
	        res_texture->w,  // width
	        res_texture->h,  // height
	        0,  // border, always 0 in OpenGL ES
	        GL_RGB,  // format
	        GL_UNSIGNED_BYTE, // type
	        res_texture->pixels);
        SDL_FreeSurface(res_texture);
    }
};

#endif //MESH_HPP
