#pragma once
#include <assert.h>

#include <fstream>
#include <map>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "flow.h"
#include "vector3.h"

#define EPSILON 1e-12
#define INF_FLOAT 1e12
#define FUZZY_REGION 0.25  //[0.5 - fr, 0.5 + fr] is fuzzy region
#define FUZZY -2
#define REPA -3
#define REPB -1

typedef Vector3f Point;
typedef Vector3i Indices;
typedef std::pair<float, int> fipair;

struct DualEdge {
  int face;  // which face
  float weight;
  float angle;
  float angle_dist, geo_dist;
  DualEdge(int f, float a_dist, float g_dist, float ang)
      : face(f),
        angle_dist(a_dist),
        geo_dist(g_dist),
        angle(ang),
        weight(0.0f) {}
  friend std::ostream& operator<<(std::ostream& os, const DualEdge& de) {
    return os << de.face << ", " << de.weight << ", " << de.angle_dist << ", "
              << de.geo_dist;
  }
};

struct Face {
  Indices indices;  // index of the corresponding three vertices
  Vector3f normal;  // normal vector of the face
  Point center;
  std::vector<DualEdge> dedges;  // Neighbor faces
  Face(Indices index, const Point& px, const Point& py, const Point& pz) {
    indices = index;
    center = (px + py + pz) / 3;
    normal = normalize(crossProduct(py - px, pz - px));
  }
};

class Mesh {
 public:
  // initialize by passing the path of an .obj file
  Mesh(const std::string& path);
  bool loadFile(const std::string& path);
  std::vector<std::string> split(std::string str, const std::string& delim);
  ~Mesh();

  std::vector<Vector3i> meshes;
  std::vector<Point> vertices;
  std::vector<Face> faces;
  std::vector<std::vector<float>> distance;
  float avg_angle_dist;

  std::vector<float> prob;
  std::vector<int> partition;

  int num_faces;

  bool isNeighFace(const Face& a, const Face& b, std::vector<int>& common);
  double angleDist(const Face& a, const Face& b, float& angle);
  double geoDist(const Face& a, const Face& b, const std::vector<int>& common);
  void getDual();
  void compDist();
  void dijkstra(int src, std::vector<float>& d);
  void meshSeg(int depth, int id, std::vector<int>& vs,
               std::map<int, int>& part);
  void solve();
  void fuzzyCluster(std::vector<int>& vs, std::map<int, int>& part, int& fuzzy,
                    int repA, int repB);
  bool buildFlowGraph(std::vector<int>& vs, std::map<int, int>& part);
  void writeObj(const std::string& name);
};
