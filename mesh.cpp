#include "glCanvas.h"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <windows.h>
#include <algorithm>
#include <stdio.h>
#include <math.h>
#define PI 3.14159265

#include "mesh.h"
#include "edge.h"
#include "vertex.h"
#include "triangle.h"


int Triangle::next_triangle_id = 0;

// helper for VBOs
#define BUFFER_OFFSET(i) ((char *)NULL + (i))


// =======================================================================
// MESH DESTRUCTOR 
// =======================================================================

Mesh::~Mesh() {
  cleanupVBOs();

  // delete all the triangles
  std::vector<Triangle*> todo;
  for (triangleshashtype::iterator iter = triangles.begin();
       iter != triangles.end(); iter++) {
    Triangle *t = iter->second;
    todo.push_back(t);
  }
  int num_triangles = todo.size();
  for (int i = 0; i < num_triangles; i++) {
    removeTriangle(todo[i]);
  }
  // delete all the vertices
  int num_vertices = numVertices();
  for (int i = 0; i < num_vertices; i++) {
    delete vertices[i];
  }
}

// =======================================================================
// MODIFIERS:   ADD & REMOVE
// =======================================================================

Vertex* Mesh::addVertex(const Vec3f &position) {
  int index = numVertices();
  Vertex *v = new Vertex(index, position);
  vertices.push_back(v);
  if (numVertices() == 1)
    bbox = BoundingBox(position,position);
  else 
    bbox.Extend(position);
  return v;
}


void Mesh::addTriangle(Vertex *a, Vertex *b, Vertex *c) {
  // create the triangle
  Triangle *t = new Triangle();
  // create the edges
  Edge *ea = new Edge(a,b,t);
  Edge *eb = new Edge(b,c,t);
  Edge *ec = new Edge(c,a,t);
  // point the triangle to one of its edges
  t->setEdge(ea);
  // connect the edges to each other
  ea->setNext(eb);
  eb->setNext(ec);
  ec->setNext(ea);
  // verify these edges aren't already in the mesh 
  // (which would be a bug, or a non-manifold mesh)
  assert (edges.find(std::make_pair(a,b)) == edges.end());
  assert (edges.find(std::make_pair(b,c)) == edges.end());
  assert (edges.find(std::make_pair(c,a)) == edges.end());
  // add the edges to the master list
  edges[std::make_pair(a,b)] = ea;
  edges[std::make_pair(b,c)] = eb;
  edges[std::make_pair(c,a)] = ec;
  // connect up with opposite edges (if they exist)
  edgeshashtype::iterator ea_op = edges.find(std::make_pair(b,a)); 
  edgeshashtype::iterator eb_op = edges.find(std::make_pair(c,b)); 
  edgeshashtype::iterator ec_op = edges.find(std::make_pair(a,c)); 
  if (ea_op != edges.end()) { ea_op->second->setOpposite(ea); }
  if (eb_op != edges.end()) { eb_op->second->setOpposite(eb); }
  if (ec_op != edges.end()) { ec_op->second->setOpposite(ec); }
  // add the triangle to the master list
  assert (triangles.find(t->getID()) == triangles.end());
  triangles[t->getID()] = t;
}


void Mesh::removeTriangle(Triangle *t) {
  Edge *ea = t->getEdge();
  Edge *eb = ea->getNext();
  Edge *ec = eb->getNext();
  Vertex *a = ea->getStartVertex();
  Vertex *b = eb->getStartVertex();
  Vertex *c = ec->getStartVertex();
  // remove these elements from master lists
  edges.erase(std::make_pair(a,b)); 
  edges.erase(std::make_pair(b,c)); 
  edges.erase(std::make_pair(c,a)); 
  triangles.erase(t->getID());
  // clean up memory
  delete ea;
  delete eb;
  delete ec;
  delete t;
}


// =======================================================================
// Helper functions for accessing data in the hash table
// =======================================================================

Edge* Mesh::getMeshEdge(Vertex *a, Vertex *b) const {
  edgeshashtype::const_iterator iter = edges.find(std::make_pair(a,b));
  if (iter == edges.end()) return NULL;
  return iter->second;
}

Vertex* Mesh::getChildVertex(Vertex *p1, Vertex *p2) const {
  vphashtype::const_iterator iter = vertex_parents.find(std::make_pair(p1,p2)); 
  if (iter == vertex_parents.end()) return NULL;
  return iter->second; 
}

void Mesh::setParentsChild(Vertex *p1, Vertex *p2, Vertex *child) {
  assert (vertex_parents.find(std::make_pair(p1,p2)) == vertex_parents.end());
  vertex_parents[std::make_pair(p1,p2)] = child; 
}


// =======================================================================
// the load function parses very simple .obj files
// the basic format has been extended to allow the specification 
// of crease weights on the edges.
// =======================================================================

#define MAX_CHAR_PER_LINE 200

void Mesh::Load(const std::string &input_file) {

  std::ifstream istr(input_file.c_str());
  if (!istr) {
    std::cout << "ERROR! CANNOT OPEN: " << input_file << std::endl;
    return;
  }

  char line[MAX_CHAR_PER_LINE];
  std::string token, token2;
  float x,y,z;
  int a,b,c;
  int index = 0;
  int vert_count = 0;
  int vert_index = 1;

  // read in each line of the file
  while (istr.getline(line,MAX_CHAR_PER_LINE)) { 
    // put the line into a stringstream for parsing
    std::stringstream ss;
    ss << line;

    // check for blank line
    token = "";   
    ss >> token;
    if (token == "") continue;

    if (token == std::string("usemtl") ||
	token == std::string("g")) {
      vert_index = 1; 
      index++;
    } else if (token == std::string("v")) {
      vert_count++;
      ss >> x >> y >> z;
      addVertex(Vec3f(x,y,z));
    } else if (token == std::string("f")) {
      a = b = c = -1;
      ss >> a >> b >> c;
      a -= vert_index;
      b -= vert_index;
      c -= vert_index;
      assert (a >= 0 && a < numVertices());
      assert (b >= 0 && b < numVertices());
      assert (c >= 0 && c < numVertices());
      addTriangle(getVertex(a),getVertex(b),getVertex(c));
    } else if (token == std::string("e")) {
      a = b = -1;
      ss >> a >> b >> token2;
      // whoops: inconsistent file format, don't subtract 1
      assert (a >= 0 && a <= numVertices());
      assert (b >= 0 && b <= numVertices());
      if (token2 == std::string("inf")) x = 1000000; // this is close to infinity...
      x = atof(token2.c_str());
      Vertex *va = getVertex(a);
      Vertex *vb = getVertex(b);
      Edge *ab = getMeshEdge(va,vb);
      Edge *ba = getMeshEdge(vb,va);
      assert (ab != NULL);
      assert (ba != NULL);
      ab->setCrease(x);
      ba->setCrease(x);
    } else if (token == std::string("vt")) {
    } else if (token == std::string("vn")) {
    } else if (token[0] == '#') {
    } else {
      printf ("LINE: '%s'",line);
    }
  }
}


// =======================================================================
// DRAWING
// =======================================================================

Vec3f ComputeNormal(const Vec3f &p1, const Vec3f &p2, const Vec3f &p3) {
  Vec3f v12 = p2;
  v12 -= p1;
  Vec3f v23 = p3;
  v23 -= p2;
  Vec3f normal;
  Vec3f::Cross3(normal,v12,v23);
  normal.Normalize();
  return normal;
}


void Mesh::initializeVBOs() {
  // create a pointer for the vertex & index VBOs
  glGenBuffers(1, &mesh_tri_verts_VBO);
  glGenBuffers(1, &mesh_tri_indices_VBO);
  glGenBuffers(1, &mesh_verts_VBO);
  glGenBuffers(1, &mesh_boundary_edge_indices_VBO);
  glGenBuffers(1, &mesh_crease_edge_indices_VBO);
  glGenBuffers(1, &mesh_other_edge_indices_VBO);
  setupVBOs();
}

void Mesh::setupVBOs() {
  HandleGLError("in setup mesh VBOs");
  setupTriVBOs();
  setupEdgeVBOs();
  HandleGLError("leaving setup mesh");
}


void Mesh::setupTriVBOs() {

  VBOTriVert* mesh_tri_verts;
  VBOTri* mesh_tri_indices;
  unsigned int num_tris = triangles.size();

  // allocate space for the data
  mesh_tri_verts = new VBOTriVert[num_tris*3];
  mesh_tri_indices = new VBOTri[num_tris];

  // write the vertex & triangle data
  unsigned int i = 0;
  triangleshashtype::iterator iter = triangles.begin();
  for (; iter != triangles.end(); iter++,i++) {
    Triangle *t = iter->second;
    Vec3f a = (*t)[0]->getPos();
    Vec3f b = (*t)[1]->getPos();
    Vec3f c = (*t)[2]->getPos();
    
    if (args->gouraud) {


      // =====================================
      // ASSIGNMENT: reimplement 
      Vec3f normal = ComputeNormal(a,b,c);
      mesh_tri_verts[i*3]   = VBOTriVert(a,normal);
      mesh_tri_verts[i*3+1] = VBOTriVert(b,normal);
      mesh_tri_verts[i*3+2] = VBOTriVert(c,normal);
      // =====================================


    } else {
      Vec3f normal = ComputeNormal(a,b,c);
      mesh_tri_verts[i*3]   = VBOTriVert(a,normal);
      mesh_tri_verts[i*3+1] = VBOTriVert(b,normal);
      mesh_tri_verts[i*3+2] = VBOTriVert(c,normal);
    }
    mesh_tri_indices[i] = VBOTri(i*3,i*3+1,i*3+2);
  }

  // cleanup old buffer data (if any)
  glDeleteBuffers(1, &mesh_tri_verts_VBO);
  glDeleteBuffers(1, &mesh_tri_indices_VBO);

  // copy the data to each VBO
  glBindBuffer(GL_ARRAY_BUFFER,mesh_tri_verts_VBO); 
  glBufferData(GL_ARRAY_BUFFER,
	       sizeof(VBOTriVert) * num_tris * 3,
	       mesh_tri_verts,
	       GL_STATIC_DRAW); 
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,mesh_tri_indices_VBO); 
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
	       sizeof(VBOTri) * num_tris,
	       mesh_tri_indices, GL_STATIC_DRAW);

  delete [] mesh_tri_verts;
  delete [] mesh_tri_indices;

}


void Mesh::setupEdgeVBOs() {

  VBOVert* mesh_verts;
  VBOEdge* mesh_boundary_edge_indices;
  VBOEdge* mesh_crease_edge_indices;
  VBOEdge* mesh_other_edge_indices;

  mesh_boundary_edge_indices = NULL;
  mesh_crease_edge_indices = NULL;
  mesh_other_edge_indices = NULL;

  unsigned int num_verts = vertices.size();

  // first count the edges of each type
  num_boundary_edges = 0;
  num_crease_edges = 0;
  num_other_edges = 0;
  for (edgeshashtype::iterator iter = edges.begin();
       iter != edges.end(); iter++) {
    Edge *e = iter->second;
    int a = e->getStartVertex()->getIndex();
    int b = e->getEndVertex()->getIndex();
    if (e->getOpposite() == NULL) {
      num_boundary_edges++;
    } else {
      if (a < b) continue; // don't double count edges!
      if (e->getCrease() > 0) num_crease_edges++;
      else num_other_edges++;
    }
  }

  // allocate space for the data
  mesh_verts = new VBOVert[num_verts];
  if (num_boundary_edges > 0)
    mesh_boundary_edge_indices = new VBOEdge[num_boundary_edges];
  if (num_crease_edges > 0)
    mesh_crease_edge_indices = new VBOEdge[num_crease_edges];
  if (num_other_edges > 0)
    mesh_other_edge_indices = new VBOEdge[num_other_edges];

  // write the vertex data
  for (unsigned int i = 0; i < num_verts; i++) {
    mesh_verts[i] = VBOVert(vertices[i]->getPos());
  }

  // write the edge data
  int bi = 0;
  int ci = 0;
  int oi = 0; 
  for (edgeshashtype::iterator iter = edges.begin();
       iter != edges.end(); iter++) {
    Edge *e = iter->second;
    int a = e->getStartVertex()->getIndex();
    int b = e->getEndVertex()->getIndex();
    if (e->getOpposite() == NULL) {
      mesh_boundary_edge_indices[bi++] = VBOEdge(a,b);
    } else {
      if (a < b) continue; // don't double count edges!
      if (e->getCrease() > 0) 
	mesh_crease_edge_indices[ci++] = VBOEdge(a,b);
      else 
	mesh_other_edge_indices[oi++] = VBOEdge(a,b);
    }
  }
  assert (bi == num_boundary_edges);
  assert (ci == num_crease_edges);
  assert (oi == num_other_edges);

  // cleanup old buffer data (if any)
  glDeleteBuffers(1, &mesh_verts_VBO);
  glDeleteBuffers(1, &mesh_boundary_edge_indices_VBO);
  glDeleteBuffers(1, &mesh_crease_edge_indices_VBO);
  glDeleteBuffers(1, &mesh_other_edge_indices_VBO);

  // copy the data to each VBO
  glBindBuffer(GL_ARRAY_BUFFER,mesh_verts_VBO); 
  glBufferData(GL_ARRAY_BUFFER,
	       sizeof(VBOVert) * num_verts,
	       mesh_verts,
	       GL_STATIC_DRAW); 

  if (num_boundary_edges > 0) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,mesh_boundary_edge_indices_VBO); 
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		 sizeof(VBOEdge) * num_boundary_edges,
		 mesh_boundary_edge_indices, GL_STATIC_DRAW);
  }
  if (num_crease_edges > 0) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,mesh_crease_edge_indices_VBO); 
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		 sizeof(VBOEdge) * num_crease_edges,
		 mesh_crease_edge_indices, GL_STATIC_DRAW);
  }
  if (num_other_edges > 0) {
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,mesh_other_edge_indices_VBO); 
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
		 sizeof(VBOEdge) * num_other_edges,
		 mesh_other_edge_indices, GL_STATIC_DRAW);
  }

  delete [] mesh_verts;
  delete [] mesh_boundary_edge_indices;
  delete [] mesh_crease_edge_indices;
  delete [] mesh_other_edge_indices;
}


void Mesh::cleanupVBOs() {
  glDeleteBuffers(1, &mesh_tri_verts_VBO);
  glDeleteBuffers(1, &mesh_tri_indices_VBO);
  glDeleteBuffers(1, &mesh_verts_VBO);
  glDeleteBuffers(1, &mesh_boundary_edge_indices_VBO);
  glDeleteBuffers(1, &mesh_crease_edge_indices_VBO);
  glDeleteBuffers(1, &mesh_other_edge_indices_VBO);
}


void Mesh::drawVBOs() {

  HandleGLError("in draw mesh");

  // scale it so it fits in the window
  Vec3f center; bbox.getCenter(center);
  float s = 1/bbox.maxDim();
  glScalef(s,s,s);
  glTranslatef(-center.x(),-center.y(),-center.z());

  // this offset prevents "z-fighting" bewteen the edges and faces
  // so the edges will always win
  if (args->wireframe) {
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.1,4.0); 
  } 

  // ======================
  // draw all the triangles
  unsigned int num_tris = triangles.size();
  glColor3f(1,1,1);

  // select the vertex buffer
  glBindBuffer(GL_ARRAY_BUFFER, mesh_tri_verts_VBO);
  // describe the layout of data in the vertex buffer
  glEnableClientState(GL_VERTEX_ARRAY);
  glVertexPointer(3, GL_FLOAT, sizeof(VBOTriVert), BUFFER_OFFSET(0));
  glEnableClientState(GL_NORMAL_ARRAY);
  glNormalPointer(GL_FLOAT, sizeof(VBOTriVert), BUFFER_OFFSET(12));

  // select the index buffer
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_tri_indices_VBO);
  // draw this data
  glDrawElements(GL_TRIANGLES, 
		 num_tris*3,
		 GL_UNSIGNED_INT,
		 BUFFER_OFFSET(0));

  glDisableClientState(GL_NORMAL_ARRAY);
  glDisableClientState(GL_VERTEX_ARRAY);

  if (args->wireframe) {
    glDisable(GL_POLYGON_OFFSET_FILL); 
  }

  // =================================
  // draw the different types of edges
  if (args->wireframe) {
    glDisable(GL_LIGHTING);

    // select the vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, mesh_verts_VBO);
    // describe the layout of data in the vertex buffer
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(3, GL_FLOAT, sizeof(VBOVert), BUFFER_OFFSET(0));

    // draw all the boundary edges
    glLineWidth(3);
    glColor3f(1,0,0);
    // select the index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_boundary_edge_indices_VBO);
    // draw this data
    glDrawElements(GL_LINES, num_boundary_edges*2, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

    // draw all the interior, crease edges
    glLineWidth(3);
    glColor3f(1,1,0);
    // select the index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_crease_edge_indices_VBO);
    // draw this data
    glDrawElements(GL_LINES, num_crease_edges*2, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

    // draw all the interior, non-crease edges
    glLineWidth(1);
    glColor3f(0,0,0);
    // select the index buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_other_edge_indices_VBO);
    // draw this data
    glDrawElements(GL_LINES, num_other_edges*2, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

    glDisableClientState(GL_VERTEX_ARRAY);
  }

  HandleGLError("leaving draw VBOs");
}

// =================================================================
// SUBDIVISION
// =================================================================

void Mesh::SimpleSubdivision() {
    std::vector<Edge*> ed;
    for (triangleshashtype::iterator iter = triangles.begin(); iter != triangles.end(); iter++) {
        ed.push_back(iter->second->getEdge());
    }
    for (int i = 0; i < ed.size(); i++) {
        Vertex* v1 = ed[i]->getStartVertex();
        ed[i] = ed[i]->getNext();
        Vertex* v2 = ed[i]->getStartVertex();
        ed[i] = ed[i]->getNext();
        Vertex* v3 = ed[i]->getStartVertex();
        Vec3f vc12 = Vec3f((v1->x() + v2->x()) / 2, (v1->y() + v2->y()) / 2, (v1->z() + v2->z()) / 2);
        Vec3f vc23 = Vec3f((v2->x() + v3->x()) / 2, (v2->y() + v3->y()) / 2, (v2->z() + v3->z()) / 2);
        Vec3f vc31 = Vec3f((v3->x() + v1->x()) / 2, (v3->y() + v1->y()) / 2, (v3->z() + v1->z()) / 2);
        Vertex* v12;
        Vertex* v23;
        Vertex* v31;
        bool flag1 = TRUE;
        bool flag2 = TRUE;
        bool flag3 = TRUE;


        for (int j = 0; j < vertices.size(); j++) {
            if (vertices[j]->x() == vc12.x() & vertices[j]->y() == vc12.y() & vertices[j]->z() == vc12.z()) {
                v12 = vertices[j];
                flag1 = FALSE;
            }

            if (vertices[j]->x() == vc23.x() & vertices[j]->y() == vc23.y() & vertices[j]->z() == vc23.z()) {
                v23 = vertices[j];
                flag2 = FALSE;
            }

            if (vertices[j]->x() == vc31.x() & vertices[j]->y() == vc31.y() & vertices[j]->z() == vc31.z()) {
                v31 = vertices[j];
                flag3 = FALSE;
            }

        }
        if (flag1) {
            v12 = addVertex(vc12);


        }
        if (flag2) {
            v23 = addVertex(vc23);


        }
        if (flag3) {
            v31 = addVertex(vc31);
        }
        removeTriangle(ed[i]->getTriangle());
        addTriangle(v1, v12, v31);
        addTriangle(v12, v2, v23);
        addTriangle(v12, v23, v31);
        addTriangle(v31, v23, v3);




    }


    printf("Subdivide the mesh!\n");

    // =====================================
    // ASSIGNMENT: complete this functionality
    // =====================================

}








void Mesh::LoopSubdivision() {
    std::vector<Triangle*> tria;
    std::vector<Edge*> ed;
    for (triangleshashtype::iterator iter = triangles.begin(); iter != triangles.end(); iter++) {
        ed.push_back(iter->second->getEdge());
    }
    for (int i = 0; i < ed.size(); i++) {
        bool j1 = TRUE;
        bool j2 = TRUE;
        bool j3 = TRUE;
        Vertex* vl12;
        Vertex* vl23;
        Vertex* vl31;
        Vec3f vc12;
        Vec3f vc23;
        Vec3f vc31;
        std::vector<Vertex*> thr;

        Vertex* v1 = ed[i]->getStartVertex();
        Edge* fv1 = ed[i]->getOpposite();
        if (fv1 != NULL) {
            fv1 = fv1->getNext();
            vl12 = fv1->getEndVertex();
            j1 = FALSE;
        }

        ed[i] = ed[i]->getNext();
        Vertex* v2 = ed[i]->getStartVertex();
        fv1 = ed[i]->getOpposite();
        if (fv1 != NULL) {
            fv1 = fv1->getNext();
            vl23 = fv1->getEndVertex();
            j2 = FALSE;
        }

        ed[i] = ed[i]->getNext();
        Vertex* v3 = ed[i]->getStartVertex();
        fv1 = ed[i]->getOpposite();
        if (fv1 != NULL) {
            fv1 = fv1->getNext();
            vl31 = fv1->getEndVertex();
            j3 = FALSE;
        }
        thr.push_back(v1);
        thr.push_back(v2);
        thr.push_back(v3);
        if (j1) {
            vc12 = Vec3f((v1->x() + v2->x()) / 2, (v1->y() + v2->y()) / 2, (v1->z() + v2->z()) / 2);
        }
        else {
            vc12 = Vec3f((v1->x() + v2->x()) * 3.0 / 8.0 + (v3->x() + vl12->x()) * 1.0 / 8.0, (v1->y() + v2->y()) * 3.0 / 8.0 + (v3->y() + vl12->y()) * 1.0 / 8.0, (v1->z() + v2->z()) * 3.0 / 8.0 + (v3->z() + vl12->z()) * 1.0 / 8.0);

        }

        if (j2) {
            vc23 = Vec3f((v2->x() + v3->x()) / 2, (v2->y() + v3->y()) / 2, (v2->z() + v3->z()) / 2);
        }
        else {
            vc23 = Vec3f((v2->x() + v3->x()) * 3.0 / 8.0 + (v1->x() + vl23->x()) * 1.0 / 8.0, (v2->y() + v3->y()) * 3.0 / 8.0 + (v1->y() + vl23->y()) * 1.0 / 8.0, (v2->z() + v3->z()) * 3.0 / 8.0 + (v1->z() + vl23->z()) * 1.0 / 8.0);

        }

        if (j3) {
            vc31 = Vec3f((v3->x() + v1->x()) / 2, (v3->y() + v1->y()) / 2, (v3->z() + v1->z()) / 2);
        }
        else {
            vc31 = Vec3f((v3->x() + v1->x()) * 3.0 / 8.0 + (v2->x() + vl31->x()) * 1.0 / 8.0, (v3->y() + v1->y()) * 3.0 / 8.0 + (v2->y() + vl31->y()) * 1.0 / 8.0, (v3->z() + v1->z()) * 3.0 / 8.0 + (v2->z() + vl31->z()) * 1.0 / 8.0);

        }

        Vertex* v12;
        Vertex* v23;
        Vertex* v31;
        bool flag1 = TRUE;
        bool flag2 = TRUE;
        bool flag3 = TRUE;


        for (int j = 0; j < vertices.size(); j++) {
            if (vertices[j]->x() == vc12.x() & vertices[j]->y() == vc12.y() & vertices[j]->z() == vc12.z()) {
                v12 = vertices[j];
                flag1 = FALSE;
            }

            if (vertices[j]->x() == vc23.x() & vertices[j]->y() == vc23.y() & vertices[j]->z() == vc23.z()) {
                v23 = vertices[j];
                flag2 = FALSE;
            }

            if (vertices[j]->x() == vc31.x() & vertices[j]->y() == vc31.y() & vertices[j]->z() == vc31.z()) {
                v31 = vertices[j];
                flag3 = FALSE;
            }

        }
        if (flag1) {
            v12 = addVertex(vc12);


        }
        if (flag2) {
            v23 = addVertex(vc23);


        }
        if (flag3) {
            v31 = addVertex(vc31);
        }

        std:vector<Vertex*> th;

        for (int ijk = 0; ijk < 3; ijk++) {
            ed[i] = ed[i]->getNext();
            Vertex* v0 = ed[i]->getStartVertex();
            std::vector<Vertex*> ve;
            Edge* temp = ed[i];
            bool edb = FALSE;
            while (TRUE) {
                if (temp->getOpposite() == NULL) {
                    edb = TRUE;
                    break;
                }
                temp = temp->getOpposite();
                ve.push_back(temp->getStartVertex());
                temp = temp->getNext();
                if (temp == ed[i]) {
                    break;
                }
            }
            if (edb) {
                ve.clear();
                ve.push_back(temp->getEndVertex());
                temp = temp->getNext();
                temp = temp->getNext();

                while (temp->getOpposite() != NULL) {
                    
                    ve.push_back(temp->getStartVertex());
                    temp = temp->getOpposite();
                    temp = temp->getNext();
                    temp = temp->getNext();

                }
                std::vector<Vertex*> temm;
                temm.push_back(ve[0]);
                temm.push_back(ve[ve.size() - 1]);
                ve.clear();
                ve.push_back(temm[0]);
                ve.push_back(temm[1]);
            }

            if (edb) {
                Vec3f v1c;

                v1c.setx(thr[ijk]->x()*3.0/4.0+ve[0]->x()*1.0/8.0+ve[1]->x()*1.0/8.0);
                v1c.sety(thr[ijk]->y() * 3.0/ 4.0 + ve[0]->y() * 1.0 / 8.0 + ve[1]->y() * 1.0 / 8.0);
                v1c.setz(thr[ijk]->z() * 3.0 / 4.0 + ve[0]->z() * 1.0 / 8.0 + ve[1]->z() * 1.0 / 8.0);
                Vertex* v1cd;
                bool flagg = TRUE;
                for (int j = 0; j < vertices.size(); j++) {
                    if (vertices[j]->x() == v1c.x() & vertices[j]->y() == v1c.y() & vertices[j]->z() == v1c.z()) {
                        v1cd = vertices[j];
                        flagg = FALSE;
                    }

                }
                if (flagg) {
                    v1cd = addVertex(v1c);
                }

                th.push_back(v1cd);
            }
            else {
                double beta = (1.0 / (double)(ve.size())) * (5.0 / 8.0 - (3.0 / 8.0 + 1.0 / 4.0 * std::cos((2.0 * PI) / (double)(ve.size()))) * (3.0 / 8.0 + 1.0 / 4.0 * std::cos((2.0 * PI) / (double)(ve.size()))));
                double v1x = 0;
                double v1y = 0;
                double v1z = 0;
                for (int y = 0; y < ve.size(); y++) {
                    v1x += ve[y]->x();
                    v1y += ve[y]->y();
                    v1z += ve[y]->z();

                }

                Vec3f v1c;

                v1c.setx((1 - ve.size() * beta) * v0->x() + beta * v1x);
                v1c.sety((1 - ve.size() * beta) * v0->y() + beta * v1y);
                v1c.setz((1 - ve.size() * beta) * v0->z() + beta * v1z);
                Vertex* v1cd;
                bool flagg = TRUE;
                for (int j = 0; j < vertices.size(); j++) {
                    if (vertices[j]->x() == v1c.x() & vertices[j]->y() == v1c.y() & vertices[j]->z() == v1c.z()) {
                        v1cd = vertices[j];
                        flagg = FALSE;
                    }

                }
                if (flagg) {
                    v1cd = addVertex(v1c);
                }

                th.push_back(v1cd);
            }


            
        }

        

        tria.push_back(ed[i]->getTriangle());
        addTriangle(th[0], v12, v31);
        addTriangle(v12, th[1], v23);
        addTriangle(v12, v23, v31);
        addTriangle(v31, v23, th[2]);
        th.clear();



    }

    for (int lll = 0; lll < tria.size(); lll++) {
        removeTriangle(tria[lll]);
    }


    printf("Subdivide the mesh!\n");

    // =====================================
    // ASSIGNMENT: complete this functionality
    // =====================================

}



void Mesh::ButterflySubdivision() {
    std::vector<Triangle*> tria;
    std::vector<Edge*> ed;

    for (triangleshashtype::iterator iter = triangles.begin(); iter != triangles.end(); iter++) {
        ed.push_back(iter->second->getEdge());
    }
    for (int i = 0; i < ed.size(); i++) {
        Vertex* v1 = ed[i]->getStartVertex();
        ed[i] = ed[i]->getNext();
        Vertex* v2 = ed[i]->getStartVertex();
        ed[i] = ed[i]->getNext();
        Vertex* v3 = ed[i]->getStartVertex();

        std::vector<Vec3f> vec3;

        for (int ikj = 0; ikj < 3; ikj++) {
            ed[i] = ed[i]->getNext();

            if (ed[i]->getOpposite() == NULL) {
                Edge* temp = ed[i];
                std::vector<Vertex*> ve;
                Vertex* vb1 = temp->getStartVertex();
                Vertex *vb2 = temp->getEndVertex();
                
                temp = temp->getNext();
                Vertex* vb3 = temp->getEndVertex();
                ve.push_back(vb3);
                ve.push_back(vb3);
                ve.push_back(vb2);
                ve.push_back(vb1);

 
                Vec3f v12 = Vec3f(ve[ve.size() - 1]->x() * 9.0 / 16.0 + ve[ve.size() - 2]->x() * 9.0 / 16.0 - ve[0]->x() * 1.0 / 16.0 - ve[ve.size() - 3]->x() * 1.0 / 16.0, ve[ve.size() - 1]->y() * 9.0 / 16.0 + ve[ve.size() - 2]->y() * 9.0 / 16.0 - ve[0]->y() * 1.0 / 16.0 - ve[ve.size() - 3]->y() * 1.0 / 16.0, ve[ve.size() - 1]->z() * 9.0 / 16.0 + ve[ve.size() - 2]->z() * 9.0 / 16.0 - ve[0]->z() * 1.0 / 16.0 - ve[ve.size() - 3]->z() * 1.0 / 16.0);
                vec3.push_back(v12);
            }
            else {
                Vertex* v0 = ed[i]->getStartVertex();
                std::vector<Vertex*> ve;
                Edge* temp = ed[i];
                bool edb = FALSE;
                while (TRUE) {
                    if (temp->getOpposite() == NULL) {
                        edb = TRUE;
                        break;
                    }
                    temp = temp->getOpposite();
                    ve.push_back(temp->getStartVertex());
                    temp = temp->getNext();
                    if (temp == ed[i]) {
                        break;
                    }
                }
                if (edb) {
                    ve.push_back(temp->getEndVertex());
                    temp = temp->getNext();
                    temp = temp->getNext();

                    while (temp->getOpposite() != NULL) {
                        
                        temp = temp->getOpposite();
                        temp = temp->getNext();
                        temp = temp->getNext();

                    }
                    ve.push_back(temp->getStartVertex());
                    temp = temp->getNext();
                    while (temp != ed[i]) {

                        temp = temp->getOpposite();
                        ve.push_back(temp->getStartVertex());
                        temp = temp->getNext();

                    }

                }

                Edge* temp1 = ed[i]->getOpposite();

                Vertex* v01 = temp1->getStartVertex();
                std::vector<Vertex*> ve1;
                temp = temp1;
                edb = FALSE;
                while (TRUE) {
                    if (temp->getOpposite() == NULL) {
                        edb = TRUE;
                        break;
                    }
                    temp = temp->getOpposite();
                    ve1.push_back(temp->getStartVertex());
                    temp = temp->getNext();
                    if (temp == temp1) {
                        break;
                    }
                }
                if (edb) {
                    ve1.push_back(temp->getEndVertex());
                    temp = temp->getNext();
                    temp = temp->getNext();

                    while (temp->getOpposite() != NULL) {

                        temp = temp->getOpposite();
                        temp = temp->getNext();
                        temp = temp->getNext();

                    }
                    ve1.push_back(temp->getStartVertex());
                    temp = temp->getNext();
                    while (temp != temp1) {

                        temp = temp->getOpposite();
                        ve1.push_back(temp->getStartVertex());
                        temp = temp->getNext();

                    }

                }

                Vec3f Vc12;

                if (ve.size() == 6 & ve1.size() == 6) {
                    Vc12 = Vec3f(v0->x() * 1.0 / 2.0 + v01->x() * 1.0 / 2.0 + ve[1]->x() * 1.0 / 8.0 + ve[5]->x() * 1.0 / 8.0 - ve[2]->x() * 1.0 / 16.0 - ve[4]->x() * 1.0 / 16.0 - ve1[2]->x() * 1.0 / 16.0 - ve1[4]->x() * 1.0 / 16.0, v0->x() * 1.0 / 2.0 + v01->x() * 1.0 / 2.0 + ve[1]->x() * 1.0 / 8.0 + ve[5]->x() * 1.0 / 8.0 - ve[2]->x() * 1.0 / 16.0 - ve[4]->x() * 1.0 / 16.0 - ve1[2]->x() * 1.0 / 16.0 - ve1[4]->x() * 1.0 / 16.0, v0->z() * 1.0 / 2.0 + v01->z() * 1.0 / 2.0 + ve[1]->z() * 1.0 / 8.0 + ve[5]->z() * 1.0 / 8.0 - ve[2]->z() * 1.0 / 16.0 - ve[4]->z() * 1.0 / 16.0 - ve1[2]->z() * 1.0 / 16.0 - ve1[4]->z() * 1.0 / 16.0);
                }
                else if (ve.size() != 6 & ve1.size() == 6) {
                    if (ve.size() == 3) {
                        Vc12 = Vec3f(v0->x() * 3.0 / 4.0 + ve[0]->x() * 5.0 / 12.0 - (ve[1]->x() + ve[2]->x()) * 1.0 / 12.0, v0->y() * 3.0 / 4.0 + ve[0]->y() * 5.0 / 12.0 - (ve[1]->y() + ve[2]->y()) * 1.0 / 12.0, v0->z() * 3.0 / 4.0 + ve[0]->z() * 5.0 / 12.0 - (ve[1]->z() + ve[2]->z()) * 1.0 / 12.0);


                    }
                    else if (ve.size() == 4) {
                        Vc12 = Vec3f(v0->x() * 3.0 / 4.0 + ve[0]->x() * 3.0 / 8.0 - ve[2]->x() * 1.0 / 8.0, v0->y() * 3.0 / 4.0 + ve[0]->y() * 3.0 / 8.0 - ve[2]->y() * 1.0 / 8.0, v0->z() * 3.0 / 4.0 + ve[0]->z() * 3.0 / 8.0 - ve[2]->z() * 1.0 / 8.0);
                    }
                    else {
                        double x = 0;
                        double y = 0;
                        double z = 0;
                        for (int j = 0; j < ve.size(); j++) {
                            x += (1.0 / 4.0 + cos(PI * 2.0 * j / ve.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve.size())) / ve.size() * ve[j]->x();
                            y += (1.0 / 4.0 + cos(PI * 2.0 * j / ve.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve.size())) / ve.size() * ve[j]->y();
                            z += (1.0 / 4.0 + cos(PI * 2.0 * j / ve.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve.size())) / ve.size() * ve[j]->z();


                        }
                        Vc12 = Vec3f(v0->x() * 3.0 / 4.0 + x, v0->y() * 3.0 / 4.0 + y, v0->z() * 3.0 / 4.0 + z);
                    }
                }
                else if (ve.size() == 6 & ve1.size() != 6) {
                    if (ve1.size() == 3) {
                        Vc12 = Vec3f(v01->x() * 3.0 / 4.0 + ve1[0]->x() * 5.0 / 12.0 - (ve1[1]->x() + ve1[2]->x()) * 1.0 / 12.0, v01->y() * 3.0 / 4.0 + ve1[0]->y() * 5.0 / 12.0 - (ve1[1]->y() + ve1[2]->y()) * 1.0 / 12.0, v01->z() * 3.0 / 4.0 + ve1[0]->z() * 5.0 / 12.0 - (ve1[1]->z() + ve1[2]->z()) * 1.0 / 12.0);


                    }
                    else if (ve1.size() == 4) {
                        Vc12 = Vec3f(v01->x() * 3.0 / 4.0 + ve1[0]->x() * 3.0 / 8.0 - ve1[2]->x() * 1.0 / 8.0, v01->y() * 3.0 / 4.0 + ve1[0]->y() * 3.0 / 8.0 - ve1[2]->y() * 1.0 / 8.0, v01->z() * 3.0 / 4.0 + ve1[0]->z() * 3.0 / 8.0 - ve1[2]->z() * 1.0 / 8.0);
                    }
                    else {
                        double x = 0;
                        double y = 0;
                        double z = 0;
                        for (int j = 0; j < ve1.size(); j++) {
                            x += (1.0 / 4.0 + cos(PI * 2.0 * j / ve1.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve1.size())) / ve1.size() * ve1[j]->x();
                            y += (1.0 / 4.0 + cos(PI * 2.0 * j / ve1.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve1.size())) / ve1.size() * ve1[j]->y();
                            z += (1.0 / 4.0 + cos(PI * 2.0 * j / ve1.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve1.size())) / ve1.size() * ve1[j]->z();


                        }
                        Vc12 = Vec3f(v01->x() * 3.0 / 4.0 + x, v01->y() * 3.0 / 4.0 + y, v01->z() * 3.0 / 4.0 + z);
                    }

                }
                else {
                    Vec3f vc121;
                    Vec3f vc122;
                    if (ve1.size() == 3) {
                        vc121 = Vec3f(v01->x() * 3.0 / 4.0 + ve1[0]->x() * 5.0 / 12.0 - (ve1[1]->x() + ve1[2]->x()) * 1.0 / 12.0, v01->y() * 3.0 / 4.0 + ve1[0]->y() * 5.0 / 12.0 - (ve1[1]->y() + ve1[2]->y()) * 1.0 / 12.0, v01->z() * 3.0 / 4.0 + ve1[0]->z() * 5.0 / 12.0 - (ve1[1]->z() + ve1[2]->z()) * 1.0 / 12.0);


                    }
                    else if (ve1.size() == 4) {
                        vc121 = Vec3f(v01->x() * 3.0 / 4.0 + ve1[0]->x() * 3.0 / 8.0 - ve1[2]->x() * 1.0 / 8.0, v01->y() * 3.0 / 4.0 + ve1[0]->y() * 3.0 / 8.0 - ve1[2]->y() * 1.0 / 8.0, v01->z() * 3.0 / 4.0 + ve1[0]->z() * 3.0 / 8.0 - ve1[2]->z() * 1.0 / 8.0);
                    }
                    else {
                        double x = 0;
                        double y = 0;
                        double z = 0;
                        for (int j = 0; j < ve1.size(); j++) {
                            x += (1.0 / 4.0 + cos(PI * 2.0 * j / ve1.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve1.size())) / ve1.size() * ve1[j]->x();
                            y += (1.0 / 4.0 + cos(PI * 2.0 * j / ve1.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve1.size())) / ve1.size() * ve1[j]->y();
                            z += (1.0 / 4.0 + cos(PI * 2.0 * j / ve1.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve1.size())) / ve1.size() * ve1[j]->z();


                        }
                        vc121 = Vec3f(v01->x() * 3.0 / 4.0 + x, v01->y() * 3.0 / 4.0 + y, v01->z() * 3.0 / 4.0 + z);

                    }

                    if (ve.size() == 3) {
                        vc122 = Vec3f(v0->x() * 3.0 / 4.0 + ve[0]->x() * 5.0 / 12.0 - (ve[1]->x() + ve[2]->x()) * 1.0 / 12.0, v0->y() * 3.0 / 4.0 + ve[0]->y() * 5.0 / 12.0 - (ve[1]->y() + ve[2]->y()) * 1.0 / 12.0, v0->z() * 3.0 / 4.0 + ve[0]->z() * 5.0 / 12.0 - (ve[1]->z() + ve[2]->z()) * 1.0 / 12.0);


                    }
                    else if (ve.size() == 4) {
                        vc122 = Vec3f(v0->x() * 3.0 / 4.0 + ve[0]->x() * 3.0 / 8.0 - ve[2]->x() * 1.0 / 8.0, v0->y() * 3.0 / 4.0 + ve[0]->y() * 3.0 / 8.0 - ve[2]->y() * 1.0 / 8.0, v0->z() * 3.0 / 4.0 + ve[0]->z() * 3.0 / 8.0 - ve[2]->z() * 1.0 / 8.0);
                    }
                    else {
                        double x = 0;
                        double y = 0;
                        double z = 0;
                        for (int j = 0; j < ve.size(); j++) {
                            x += (1.0 / 4.0 + cos(PI * 2.0 * j / ve.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve.size())) / ve.size() * ve[j]->x();
                            y += (1.0 / 4.0 + cos(PI * 2.0 * j / ve.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve.size())) / ve.size() * ve[j]->y();
                            z += (1.0 / 4.0 + cos(PI * 2.0 * j / ve.size()) + 1.0 / 2.0 * cos(4.0 * PI * j / ve.size())) / ve.size() * ve[j]->z();


                        }
                        vc122 = Vec3f(v0->x() * 3.0 / 4.0 + x, v0->y() * 3.0 / 4.0 + y, v0->z() * 3.0 / 4.0 + z);
                    }

                    Vc12 = 1.0 / 2.0 * (vc121 + vc122);
                }

                vec3.push_back(Vc12);

            }
        }

        Vertex* v12;
        Vertex* v23;
        Vertex* v31;
        bool flag1 = TRUE;
        bool flag2 = TRUE;
        bool flag3 = TRUE;


        for (int j = 0; j < vertices.size(); j++) {
            if (vertices[j]->x() == vec3[0].x() & vertices[j]->y() == vec3[0].y() & vertices[j]->z() == vec3[0].z()) {
                v12 = vertices[j];
                flag1 = FALSE;
            }

            if (vertices[j]->x() == vec3[1].x() & vertices[j]->y() == vec3[1].y() & vertices[j]->z() == vec3[1].z()) {
                v23 = vertices[j];
                flag2 = FALSE;
            }

            if (vertices[j]->x() == vec3[2].x() & vertices[j]->y() == vec3[2].y() & vertices[j]->z() == vec3[2].z()) {
                v31 = vertices[j];
                flag3 = FALSE;
            }

        }
        if (flag1) {
            v12 = addVertex(vec3[0]);


        }
        if (flag2) {
            v23 = addVertex(vec3[1]);


        }
        if (flag3) {
            v31 = addVertex(vec3[2]);
        }
        tria.push_back(ed[i]->getTriangle());
        addTriangle(v1, v12, v31);
        addTriangle(v12, v2, v23);
        addTriangle(v12, v23, v31);
        addTriangle(v31, v23, v3);






    }

    for (int lll = 0; lll < tria.size(); lll++) {
        removeTriangle(tria[lll]);
    }



    printf("Subdivide the mesh!\n");

    // =====================================
    // ASSIGNMENT: complete this functionality
    // =====================================

}




// =================================================================
// SIMPLIFICATION
// =================================================================

void Mesh::Simplification(int target_tri_count) {
  // clear out any previous relationships between vertices
  vertex_parents.clear();
  for (int k = 0; k < (numTriangles() - target_tri_count) / 2; k++) {
      triangleshashtype::iterator iter = triangles.begin();
      for (int i = 0; i < int(rand() % (numTriangles() - 0)); i++){
          iter++;
      }
          
      std::vector<Triangle*> tr;
      std::vector<Vertex*> ve;
      Triangle* t = iter->second;
      tr.push_back(t);
      Edge *e1 = t->getEdge();
      Vertex* v0 = e1->getStartVertex();
      Edge *e1o = e1->getOpposite();
      Vertex *v1 = e1o->getStartVertex();
      Triangle* ttem = e1o->getTriangle();
      while (ttem!=t) { 
          tr.push_back(ttem);
          Edge* e2 = e1o->getNext();
          Edge* e2o = e2->getOpposite();
          Vertex* v2 = e2o->getStartVertex();
          ve.push_back(v2);
          ttem = e2o->getTriangle();
          e1o = e2o;

      }
       
      for (int i = 0; i < tr.size(); i++) {
          removeTriangle(tr[i]);
      }
      //delete vertices[v0->getIndex()]; 
      bool flag = TRUE;
      for (int i = 0; i < ve.size() - 1; i++) {
          if (edges.find(std::make_pair(v1, ve[i + 1])) != edges.end() || edges.find(std::make_pair(ve[i + 1], ve[i])) != edges.end() || edges.find(std::make_pair(ve[i], v1)) != edges.end()) {
              flag = FALSE;
          }
      }
      
      if (flag) {
          while (ve.size() > 1) {
              addTriangle(v1, ve[1], ve[0]);
              ve.erase(ve.begin());


          }
      }
      else {
          addTriangle(v0, ve[0], v1);
          addTriangle(v0, v1, ve[ve.size() - 1]);
          while (ve.size() > 1) {
              addTriangle(v0, ve[1], ve[0]);
              ve.erase(ve.begin());


          }

          continue;
      }
      
   

  }
  
  printf ("Simplify the mesh! %d -> %d\n", numTriangles(), target_tri_count);

  // =====================================
  // ASSIGNMENT: complete this functionality
  // =====================================
}


// =================================================================
