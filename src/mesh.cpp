#include "mesh.h"

#include <igl/readOBJ.h>


void Mesh::load(const std::string _filename)
{
    igl::readOBJ(_filename,m_vertices,m_faces);
}
