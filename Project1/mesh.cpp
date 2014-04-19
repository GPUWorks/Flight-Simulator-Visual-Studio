/*
 * Mesh.cpp
 *
 *  Created on: 27 Feb 2013
 *  Author: Alexandros Neophytou
 */

#include "mesh.h"

void loadMesh(Mesh& myMesh,std::string filename)
{

	/**
	 * OBJ file format:
	 * '#'  = comments
	 * 'v'  = vertex coordinates: 3 floats x-y-z
	 * 'vt' = vertex texture coordinates: 2 floats u-v
	 * 'vn' = vertex normals: 3 floats x-y-z
	 * 'f'  = faces are represented by a set of id numbers separated by a "/" and space :vertex_id/texture_id/normal_id
	 *  For example: f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
	 */

	std::ifstream filestream;
	filestream.open(filename.c_str());

	int fcount = 0;
	int vcount = 0;
	int ncount = 0;
	int tcount = 0;

	std::string line_stream;
	while(std::getline(filestream, line_stream)){
		std::stringstream str_stream(line_stream);
		std::string type_str;
		str_stream >> type_str;
		if(type_str == "v"){
			Vector3f position;
			str_stream >> position.x >> position.y >> position.z;
			myMesh.vertices.push_back(position);
			vcount++;
		}else if(type_str == "vt"){
			Vector2f texture;
			str_stream >> texture.x >> texture.y;
			myMesh.tex_coords.push_back(texture);
			tcount++;
		}else if(type_str == "vn"){
			Vector3f normal;
			str_stream >> normal.x >> normal.y >> normal.z;
			myMesh.vertex_normals.push_back(normal);
			ncount++;
		}else if(type_str == "f"){
			char temp;
			Face face;
			for(int i = 0; i < 3; ++i){
				str_stream >> face.position_idx[i] >> temp
				>> face.texture_idx[i]  >> temp
				>> face.normal_idx[i];

				face.position_idx[i]-=1;
				face.normal_idx[i]-=1;
				face.texture_idx[i]-=1;
			}
			fcount++;
			myMesh.faces.push_back(face);
		}
	}
	 
	printf("Loaded Mesh %s consisting of %d faces, %d positions, %d normals and %d texture coordinates\n" , filename.c_str(), fcount, vcount, ncount, tcount);
  
	// Explicit closing of the file
	filestream.close();
}
void drawMesh(Mesh mesh)
{
	// Begin drawing of triangles.
	glBegin(GL_TRIANGLES);

	// Iterate over each face.
	for(unsigned int i= 0; i<mesh.faces.size(); i++)
	{
		Face face = mesh.faces[i];
		for(int j= 0; j<3; j++)
		{
			Vector3f vpos = mesh.vertices[face.position_idx[j]];
			Vector3f vnorm = mesh.vertex_normals[face.normal_idx[j]];
			Vector2f vtex = mesh.tex_coords[face.texture_idx[j]];

			glNormal3f(vnorm.x, vnorm.y, vnorm.z); 
			glTexCoord2f(vtex.x, vtex.y);//comment this line if not textures are being used
			glVertex3f(vpos.x, vpos.y, vpos.z);
		}
	}
	// End drawing of triangles.
	glEnd();
}

Vector3f getCentroid(Mesh& mesh)
{
	Vector3f mean;
	mean.x=0;
	mean.y=0;
	mean.z=0;
	int n=mesh.vertices.size();
	for(int i=0;i<n;i++)
	{
		mean.x+=mesh.vertices[i].x;
		mean.y+=mesh.vertices[i].y;
		mean.z+=mesh.vertices[i].z;
	}
	mean.x/=n;
	mean.y/=n;
	mean.z/=n;
	return mean;
}

Vector3f getMax(Mesh&mesh)
{
	Vector3f max;
	max.x=0;
	max.y=0;
	max.z=0;
	int n=mesh.vertices.size();
	for(int i=0;i<n;i++)
	{
		if(max.x < mesh.vertices[i].x)
			max.x = mesh.vertices[i].x;

		if(max.y < mesh.vertices[i].y)
			max.y = mesh.vertices[i].y;

		if(max.z < mesh.vertices[i].z)
			max.z = mesh.vertices[i].z;
	}

	return max;
}

Vector3f getMin(Mesh&mesh)
{
	Vector3f min;
	min.x=0;
	min.y=0;
	min.z=0;
	int n=mesh.vertices.size();
	for(int i=0;i<n;i++)
	{
		if(min.x > mesh.vertices[i].x)
			min.x = mesh.vertices[i].x;

		if(min.y > mesh.vertices[i].y)
			min.y = mesh.vertices[i].y;

		if(min.z > mesh.vertices[i].z)
			min.z = mesh.vertices[i].z;
	}

	return min;
}