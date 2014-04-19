/*
 * Mesh.h
 *
 *  Created on: 27 Feb 2013
 *  Author: Alexandros Neophytou
 *
 */

#ifndef MESH_H_
#define MESH_H_
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <fstream>

#include <Windows.h>
#include <GL/gl.h>
#include <GL/glut.h>

using namespace std;
struct Vector2f{
    float x, y;
};
struct Vector3f{
    float x, y, z;
};

struct Face{
	int position_idx[3];
	int normal_idx[3];
	int texture_idx[3];
};

struct Mesh{
    std::vector<Face> faces;
    std::vector<Vector3f> vertices;
    std::vector<Vector3f> vertex_normals;
    std::vector<Vector2f> tex_coords;
};

//Function that returns the centroid of the mesh
Vector3f getCentroid(Mesh& mesh);

// Return max and min x, y and z co-ordinates
Vector3f getMax(Mesh&mesh);
Vector3f getMin(Mesh&mesh);

//Loads a Mesh given a path to an obj
void loadMesh(Mesh& mesh,std::string filename);

//Draws the mesh using OpenGL 
void drawMesh(Mesh mesh);

#endif /* MESH_H_ */
