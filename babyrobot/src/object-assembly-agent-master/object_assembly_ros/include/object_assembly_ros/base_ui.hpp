#ifndef BASE_UI_HPP
#define BASE_UI_HPP

#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <mutex>
#include <GL/glew.h>
#include <GL/glut.h>
#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <mesh.hpp>
#include "shader_utils.h"


class BaseUI
{
public:
    std::vector<Mesh> meshes_;

    BaseUI(std::vector<std::string> filenames);
    int gui_main(int argc, char* argv[]);
    static int gui_main_static(int argc, char* argv[]);
    void update_poses(std::vector<tf::Transform> new_poses);
    void reload_textures();
    //void set_filenames(std::vector<std::string> filenames);
    void load_mesh(const char* filename, Mesh* mesh);
    int init_resources(char* vshader_filename, char* fshader_filename);
    void init_view();
    void onReshape(int width, int height);
    static void onReshape_static(int width, int height);
    static void onDisplay_static();
    static void onIdle_static();
    void free_resources();

protected:
    std::vector<std::string> filenames_;

    Mesh ground, light_bbox;
    char mode_name_[32];

    //stuff taken from obj-viewer.cpp. //TODO: clean up
    int screen_width=800, screen_height=600;
    GLuint program;

    bool compute_arcball;
    int last_mx = 0, last_my = 0, cur_mx = 0, cur_my = 0;
    int arcball_on = false;
    enum MODES { MODE_OBJECT, MODE_CAMERA, MODE_LIGHT, MODE_LAST } view_mode;
    int rotY_direction = 0, rotX_direction = 0, transZ_direction = 0, strife = 0;
    float speed_factor = 1;
    glm::mat4 transforms[MODE_LAST];
    int last_ticks = 0;
    unsigned int fps_start = glutGet(GLUT_ELAPSED_TIME);
    unsigned int fps_frames = 0;
    std::mutex mutex_;

};

BaseUI::BaseUI(std::vector<std::string> filenames) : filenames_(filenames)
{
    for (int i = 0; i < filenames_.size(); i++)
    {
        Mesh new_mesh;
        load_mesh(filenames_[i].c_str(), &new_mesh);
        meshes_.push_back(new_mesh);
    }
}

void BaseUI::update_poses(std::vector<tf::Transform> new_poses)
{
    std::lock_guard<std::mutex> lock_mutex(mutex_);
    for (int i = 0; i < new_poses.size(); i++)
    {
        double m[16];
        new_poses[i].getOpenGLMatrix(m);
        meshes_[i].object2world = glm::make_mat4(m);
    }
}

void BaseUI::reload_textures()
{
    std::lock_guard<std::mutex> lock_mutex(mutex_);
    for (int i = 0; i < meshes_.size(); i++)
    {
        meshes_[i].load_new_texture();
    }
}

void BaseUI::load_mesh(const char* filename, Mesh* mesh) {
  std::ifstream in(filename, std::ios::in);
  if (!in) { std::cerr << "Cannot open " << filename << std::endl; exit(1); }
  std::vector<int> nb_seen;

  std::string line;
  while (std::getline(in, line)) {
    if (line.substr(0,2) == "v ") {
      std::istringstream s(line.substr(2));
      glm::vec4 v; s >> v.x; s >> v.y; s >> v.z; v.w = 1.0;
      mesh->vertices.push_back(v);
    }  else if (line.substr(0,2) == "f ") {
      std::istringstream s(line.substr(2));
      GLushort a,b,c;
      s >> a; s >> b; s >> c;
      a--; b--; c--;
      mesh->elements.push_back(a); mesh->elements.push_back(b); mesh->elements.push_back(c);
    }
    else if (line[0] == '#') { /* ignoring this line */ }
    else { /* ignoring this line */ }
  }

  mesh->normals.resize(mesh->vertices.size(), glm::vec3(0.0, 0.0, 0.0));
  nb_seen.resize(mesh->vertices.size(), 0);
  for (unsigned int i = 0; i < mesh->elements.size(); i+=3) {
    GLushort ia = mesh->elements[i];
    GLushort ib = mesh->elements[i+1];
    GLushort ic = mesh->elements[i+2];
    glm::vec3 normal = glm::normalize(glm::cross(
      glm::vec3(mesh->vertices[ib]) - glm::vec3(mesh->vertices[ia]),
      glm::vec3(mesh->vertices[ic]) - glm::vec3(mesh->vertices[ia])));

    int v[3];  v[0] = ia;  v[1] = ib;  v[2] = ic;
    for (int j = 0; j < 3; j++) {
      GLushort cur_v = v[j];
      nb_seen[cur_v]++;
      if (nb_seen[cur_v] == 1) {
	mesh->normals[cur_v] = normal;
      } else {
	// average
	mesh->normals[cur_v].x = mesh->normals[cur_v].x * (1.0 - 1.0/nb_seen[cur_v]) + normal.x * 1.0/nb_seen[cur_v];
	mesh->normals[cur_v].y = mesh->normals[cur_v].y * (1.0 - 1.0/nb_seen[cur_v]) + normal.y * 1.0/nb_seen[cur_v];
	mesh->normals[cur_v].z = mesh->normals[cur_v].z * (1.0 - 1.0/nb_seen[cur_v]) + normal.z * 1.0/nb_seen[cur_v];
	mesh->normals[cur_v] = glm::normalize(mesh->normals[cur_v]);
      }
    }
  }
}

int BaseUI::init_resources(char* vshader_filename, char* fshader_filename)
{
  // mesh position initialized in init_view()

    for (int i = -10; i < 10; i++) {
        for (int j = -10; j < 10; j++) {
            ground.vertices.push_back(glm::vec4(i,   0.0,  j+1, 1.0));
            ground.vertices.push_back(glm::vec4(i+1, 0.0,  j+1, 1.0));
            ground.vertices.push_back(glm::vec4(i,   0.0,  j,   1.0));
            ground.vertices.push_back(glm::vec4(i,   0.0,  j,   1.0));
            ground.vertices.push_back(glm::vec4(i+1, 0.0,  j+1, 1.0));
            ground.vertices.push_back(glm::vec4(i+1, 0.0,  j,   1.0));
            for (unsigned int k = 0; k < 6; k++)
                ground.normals.push_back(glm::vec3(0.0, 1.0, 0.0));
        }
    }

    glm::vec3 light_position = glm::vec3(0.0,  1.0,  2.0);
    light_bbox.vertices.push_back(glm::vec4(-0.1, -0.1, -0.1, 0.0));
    light_bbox.vertices.push_back(glm::vec4( 0.1, -0.1, -0.1, 0.0));
    light_bbox.vertices.push_back(glm::vec4( 0.1,  0.1, -0.1, 0.0));
    light_bbox.vertices.push_back(glm::vec4(-0.1,  0.1, -0.1, 0.0));
    light_bbox.vertices.push_back(glm::vec4(-0.1, -0.1,  0.1, 0.0));
    light_bbox.vertices.push_back(glm::vec4( 0.1, -0.1,  0.1, 0.0));
    light_bbox.vertices.push_back(glm::vec4( 0.1,  0.1,  0.1, 0.0));
    light_bbox.vertices.push_back(glm::vec4(-0.1,  0.1,  0.1, 0.0));
    light_bbox.object2world = glm::translate(glm::mat4(1), light_position);

    for (int i = 0; i < meshes_.size(); i++)
        meshes_[i].upload();
    if (!use_png_texture)
        ground.upload();
    //light_bbox.upload();


    /* Compile and link shaders */
    GLint link_ok = GL_FALSE;
    GLint validate_ok = GL_FALSE;

    GLuint vs, fs;
    if ((vs = create_shader(vshader_filename, GL_VERTEX_SHADER))   == 0) return 0;
    if ((fs = create_shader(fshader_filename, GL_FRAGMENT_SHADER)) == 0) return 0;

    program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &link_ok);
    if (!link_ok) {
        fprintf(stderr, "glLinkProgram:");
        print_log(program);
        return 0;
    }
    glValidateProgram(program);
    glGetProgramiv(program, GL_VALIDATE_STATUS, &validate_ok);
    if (!validate_ok) {
        fprintf(stderr, "glValidateProgram:");
        print_log(program);
    }

    const char* attribute_name;
    attribute_name = "v_coord";
    attribute_v_coord = glGetAttribLocation(program, attribute_name);
    if (attribute_v_coord == -1) {
        fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
        return 0;
    }
    if (!use_png_texture)
    {
        attribute_name = "v_normal";
        attribute_v_normal = glGetAttribLocation(program, attribute_name);
        if (attribute_v_normal == -1) {
            fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
            return 0;
        }
        attribute_name = "v_colour";
        attribute_v_colour = glGetAttribLocation(program, attribute_name);
        if (attribute_v_colour == -1) {
            fprintf(stderr, "Could not bind attribute %s\n", attribute_name);
            return 0;
        }
    }
    if (use_png_texture)
    {
        attribute_name = "texcoord";
	    attribute_texcoord = glGetAttribLocation(program, attribute_name);
	    if (attribute_texcoord == -1) {
		    std::cerr << "Could not bind attribute " << attribute_name << std::endl;
		    return false;
	    }
    }

    const char* uniform_name;

        uniform_name = "m";
        uniform_m = glGetUniformLocation(program, uniform_name);
        if (uniform_m == -1) {
            fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
            return 0;
        }
        uniform_name = "v";
        uniform_v = glGetUniformLocation(program, uniform_name);
        if (uniform_v == -1) {
            fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
            return 0;
        }
        uniform_name = "p";
        uniform_p = glGetUniformLocation(program, uniform_name);
        if (uniform_p == -1) {
            fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
            return 0;
        }
    if (!use_png_texture)
    {
        uniform_name = "m_3x3_inv_transp";
        uniform_m_3x3_inv_transp = glGetUniformLocation(program, uniform_name);
        if (uniform_m_3x3_inv_transp == -1) {
            fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
            return 0;
        }
    
        uniform_name = "v_inv";
        uniform_v_inv = glGetUniformLocation(program, uniform_name);
        if (uniform_v_inv == -1) {
            fprintf(stderr, "Could not bind uniform %s\n", uniform_name);
            return 0;
        }
    }
    if (use_png_texture)
    {
        uniform_name = "mytexture";
	    uniform_mytexture = glGetUniformLocation(program, uniform_name);
	    if (uniform_mytexture == -1) {
		    std::cerr << "Could not bind uniform " << uniform_name << std::endl;
		    return false;
	    }
    }
    fps_start = glutGet(GLUT_ELAPSED_TIME);

    return 1;
}

void BaseUI::onReshape(int width, int height) {
    screen_width = width;
    screen_height = height;
    glViewport(0, 0, screen_width, screen_height);
}

void BaseUI::init_view() {
  //main_object.object2world = glm::mat4(1);
    transforms[MODE_CAMERA] = glm::lookAt(
        glm::vec3(0.0, -1.0,    0.4),   // eye
        glm::vec3(0.0, -0.2929, 0.5071),   // direction
        glm::vec3(0.0, -0.7071, 0.7071));  // up
}

void BaseUI::free_resources()
{
    glDeleteProgram(program);
}

int BaseUI::gui_main(int argc, char* argv[]) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA|GLUT_ALPHA|GLUT_DOUBLE|GLUT_DEPTH);
    glutInitWindowSize(screen_width, screen_height);
    glutCreateWindow(mode_name_); //TODO: check if set

    GLenum glew_status = glewInit();
    if (glew_status != GLEW_OK) {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(glew_status));
        return 1;
    }

    if (!GLEW_VERSION_2_0) {
        fprintf(stderr, "Error: your graphic card does not support OpenGL 2.0\n");
        return 1;
    }
    char v_shader_filename[128];
    char f_shader_filename[128];
    if (!use_png_texture)
    {
        std::strcpy(v_shader_filename, "/home/jack/code_test2/catkinws2/src/object_assembly_ros/resource/phong-shading.v.glsl");
        std::strcpy(f_shader_filename, "/home/jack/code_test2/catkinws2/src/object_assembly_ros/resource/phong-shading.f.glsl");
    }
    else
    {
        std::strcpy(v_shader_filename, "/home/jack/code_test2/catkinws2/src/object_assembly_ros/resource/textured-cube.v.glsl");
        std::strcpy(f_shader_filename, "/home/jack/code_test2/catkinws2/src/object_assembly_ros/resource/textured-cube.f.glsl");
    }

    if (init_resources(v_shader_filename, f_shader_filename)) {
        init_view();
        glutDisplayFunc(onDisplay_static);
        //glutSpecialFunc(onSpecial);
        //glutSpecialUpFunc(onSpecialUp);
        //glutMouseFunc(onMouse);
        //glutMotionFunc(onMotion);
        glutReshapeFunc(onReshape_static);
        glutIdleFunc(onIdle_static);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_TEXTURE_2D);
        //glDepthFunc(GL_LEQUAL);
        //glDepthRange(1, 0);
        last_ticks = glutGet(GLUT_ELAPSED_TIME);
        glutMainLoop();
    }

    free_resources();
    return 0;
}

#endif //BASE_UI_HPP 
