#include "mesh.h"

Mesh::Mesh(const std::string& path) {
  if (loadFile(path) == false) {
    std::cout << "Loading failed" << std::endl;
  };
}

bool Mesh::loadFile(const std::string& path) {
  if (path.substr(path.size() - 4, 4) != ".obj") {
    return false;
  }
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }
  meshes.clear();
  vertices.clear();
  std::string curline;
  const std::string delim(" ");
  while (std::getline(file, curline)) {
    if (char(curline[0]) == '#') continue;
    std::vector<std::string> line = split(curline, delim);
    if (line[0] == std::string("v")) {
      vertices.push_back(
          Vector3f(std::stof(line[1]), std::stof(line[2]), std::stof(line[3])));
    } else if (line[0] == std::string("f")) {
      int x = std::stoi(line[1]), y = std::stoi(line[2]),
          z = std::stoi(line[3]);
      Indices index(x - 1, y - 1, z - 1);
      Face face(index, vertices[x - 1], vertices[y - 1], vertices[z - 1]);
      faces.push_back(face);
      // meshes.push_back(
      //     Vector3i(std::stoi(line[1]), std::stoi(line[2]),
      //     std::stoi(line[3])));
    } else {
      std::cout << "Loader currently doesn't support lines other than v and f"
                << std::endl;
      return false;
    }
  }
  num_faces = faces.size();

  std::cout << "Loading finished, contains " << vertices.size()
            << " vertices and " << faces.size() << " meshes" << std::endl;
  //   for (int i = 0; i < 10; i++) {
  //     std::cout << meshes[i] << std::endl;
  //     std::cout << vertices[i] << std::endl;
  //   }
  return true;
}

std::vector<std::string> Mesh::split(std::string str,
                                     const std::string& delim) {
  std::vector<std::string> result;
  int cur;
  while ((cur = str.find_first_of(delim)) != str.npos) {
    if (cur > 0) {
      result.push_back(str.substr(0, cur));
    }
    str = str.substr(cur + 1);
  }
  if (str.length() > 0) {
    result.push_back(str);
  }
  return result;
}

Mesh::~Mesh() {
  meshes.clear();
  vertices.clear();
}

bool Mesh::isNeighFace(const Face& a, const Face& b, std::vector<int>& common) {
  common.clear();
  Indices ia = a.indices, ib = b.indices;
  int same = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (ia[i] == ib[j]) {
        common.push_back(ia[i]);
        same++;
        break;
      }
    }
  }
  assert(same != 3);
  if (same == 2) return true;
  if (same == 3) {
    std::cout << "Same face !!!" << std::endl;
  }
  return false;
}

double Mesh::angleDist(const Face& a, const Face& b, float& angle) {
  float dot = dotProduct(a.normal, b.normal);
  if (dot >= 1.0f)
    dot = 1.0f;
  else if (dot <= -1.0f)
    dot = -1.0f;
  angle = asin(dot);
  float angle_dist = 1 - cos(angle);
  if (dotProduct(b.normal - a.normal, a.center - b.center) < 1e-12) {
    angle_dist *= 0.2;
  }
  return angle_dist;
}

double Mesh::geoDist(const Face& a, const Face& b,
                     const std::vector<int>& common) {
  assert(common.size() == 2);
  Vector3f p0 = vertices[common[0]], p1 = vertices[common[1]];
  Vector3f ce = p1 - p0, va = a.center - p0, vb = b.center - p0;
  float l = vlen(ce), la = vlen(va), lb = vlen(vb);
  float angle =
      acos(dotProduct(va, ce) / (la * l)) + acos(dotProduct(vb, ce) / (lb * l));
  return la * la + lb * lb - 2 * la * lb * cos(angle);
}

void Mesh::getDual() {
  int num_neigh = 0;
  float tot_angle_dist = 0.0f, tot_geo_dist = 0.0f;
  for (int i = 0; i < num_faces; i++) {
    for (int j = i + 1; j < num_faces; j++) {
      std::vector<int> common;
      if (isNeighFace(faces[i], faces[j], common)) {
        num_neigh++;
        float angle;
        float angle_dist = angleDist(faces[i], faces[j], angle);
        float geo_dist = geoDist(faces[i], faces[j], common);
        tot_angle_dist += angle_dist;
        tot_geo_dist += geo_dist;
        // fout << faces[i].normal << " " << faces[j].normal << " " <<
        // dotProduct(faces[i].normal, faces[j].normal)
        // << " " << 1 - cos(asin(dotProduct(faces[i].normal, faces[j].normal)))
        // << " " << angle_dist << " " << tot_angle_dist << std::endl;

        faces[i].dedges.push_back(DualEdge(j, angle_dist, geo_dist, angle));
        faces[j].dedges.push_back(DualEdge(i, angle_dist, geo_dist, angle));
        // std::cout << i << " " << j << " " << angle_dist << " " << geo_dist <<
        // " " << std::endl;
      }
    }
  }
  avg_angle_dist = tot_angle_dist / num_neigh;
  float avg_geo_dist = tot_geo_dist / num_neigh;
  for (Face& f : faces) {
    for (DualEdge& de : f.dedges) {
      de.weight = 0.2 * de.angle_dist / avg_angle_dist +
                  0.8 * de.geo_dist / avg_geo_dist;
    }
  }
  // for(DualEdge& de: faces[3000].dedges) {
  //   std::cout << de << std::endl;
  // }
  std::cout << "Num neighbor faces " << num_neigh << std::endl;
  std::cout << "Avg Angle dist " << avg_angle_dist << std::endl;
  std::cout << "Avg Geo dist " << avg_geo_dist << std::endl;
}

void Mesh::compDist() {
  for (int i = 0; i < num_faces; i++) {
    std::vector<float> tmp_dist(num_faces, INF_FLOAT);
    dijkstra(i, tmp_dist);
    distance.push_back(tmp_dist);
  }
  // std::ofstream fout("dist");
  // for(int i = 0; i < num_faces; i++) {
  //   for(int j = 0; j < num_faces; j++) {
  //     if(i % 100 == 0 && j % 100 == 0)
  //       fout << distance[i][j] << " ";
  //   }
  //   fout << std::endl;
  // }
  // fout.close();
  std::cout << "Finish computing shortest path" << std::endl;
}

// Code from
// https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
void Mesh::dijkstra(int src, std::vector<float>& tmp_dist) {
  std::priority_queue<fipair, std::vector<fipair>, std::greater<fipair>> pq;
  pq.push(std::make_pair(0.0f, src));
  tmp_dist[src] = 0;
  while (!pq.empty()) {
    int u = pq.top().second;
    pq.pop();
    for (DualEdge& de : faces[u].dedges) {
      int v = de.face;
      float weight = de.weight;
      if (tmp_dist[v] > tmp_dist[u] + weight) {
        tmp_dist[v] = tmp_dist[u] + weight;
        pq.push(std::make_pair(tmp_dist[v], v));
      }
    }
  }
}

// pass a list of vertices to be used
void Mesh::fuzzyCluster(std::vector<int>& vs, std::map<int, int>& part,
                        int& fuzzy, int repA, int repB) {
  int nv = vs.size();
  prob.reserve(nv);
  std::cout << "Representives " << repA << " " << repB << std::endl;
  for (int iter = 0; iter < 10; iter++) {
    // compute probability
    for (int t = 0; t < nv; t++) {
      int v = vs[t];
      float a_fi = distance[v][repA], b_fi = distance[v][repB];
      // the probability of the t th vertice belongs to B
      prob[t] = a_fi / (a_fi + b_fi);
    }
    // find new representatives
    int lastA = repA, lastB = repB;
    float min_repB = INF_FLOAT, min_repA = INF_FLOAT;
    for (int i = 0; i < nv; i++) {
      int v1 = vs[i];
      int tmp_repB = 0.0f, tmp_repA = 0.0f;
      for (int j = 0; j < nv; j++) {
        int v2 = vs[j];
        tmp_repB += prob[j] * distance[v1][v2];
        tmp_repA += (1.0 - prob[j]) * distance[v1][v2];
      }
      if (tmp_repB < min_repB) {
        min_repB = tmp_repB;
        repB = v1;
      }
      if (tmp_repA < min_repA) {
        min_repA = tmp_repA;
        repA = v1;
      }
    }
    std::cout << repA << " " << repB << std::endl;
    if (lastA == repA && lastB == repB) break;
  }
  // assign partition
  // std::ofstream fout("log");
  // partition.resize(num_faces);
  for (int i = 0; i < nv; i++) {
    int vertice = vs[i];
    // assert(part.find(vertice) == part.end());
    if (prob[i] > 0.5 + FUZZY_REGION) {
      // partition[i] = REPB;
      part[vertice] = REPB;
    } else if (prob[i] < 0.5 - FUZZY_REGION) {
      // partition[i] = REPA;
      part[vertice] = REPA;
    } else {
      // partition[i] = FUZZY;
      part[vertice] = FUZZY;
      fuzzy++;
    }
    // fout << partition[i] << std::endl;
  }
  // fout.close();
  std::cout << "Finish fuzzy decomposition, found " << fuzzy
            << " fuzzy vertices" << std::endl;
}

void Mesh::solve() {
  partition.resize(num_faces);
  std::map<int, int> part;
  std::vector<int> vs;
  for (int i = 0; i < num_faces; i++) vs.push_back(i);
  meshSeg(0, 0, vs, part);
}

// label should be id*2+1, id*2+2
void Mesh::meshSeg(int depth, int id, std::vector<int>& vs,
                   std::map<int, int>& part) {
  // if(stop condition), return
  // if(vs.size() <= 300) {
  //   for(int v: vs) {
  //     partition[v] = id;
  //   }
  //   return;
  // }
  std::cout << "Depth and id: " << depth << " " << id << std::endl;
  int fuzzy = 0;
  int repA, repB;
  float max_dist = -1.0f;
  float max_angle = -100, min_angle = 100;
  std::set<int> cur_vs(vs.begin(), vs.end());
  for (int v : vs) {
    for (DualEdge& de : faces[v].dedges) {
      if (cur_vs.find(de.face) == cur_vs.end()) continue;
      if (de.angle > max_angle) max_angle = de.angle;
      if (de.angle < min_angle) min_angle = de.angle;
    }
  }
  for (int i = 0; i < vs.size(); i++) {
    for (int j = 0; j < vs.size(); j++) {
      int m = vs[i], n = vs[j];
      if (distance[m][n] > max_dist) {
        max_dist = distance[m][n];
        if (m < n) {
          repA = m;
          repB = n;
        } else {
          repA = n;
          repB = m;
        }
      }
    }
  }
  std::cout << "thresold " << distance[repA][repB] << " "
            << max_angle - min_angle << std::endl;
  // 40 3/ 20 1
  if (distance[repA][repB] < 30 || max_angle - min_angle < 1.1) {
    for (int v : vs) {
      partition[v] = id;
    }
    return;
  }
  fuzzyCluster(vs, part, fuzzy, repA, repB);
  if (fuzzy != 0)
    if (buildFlowGraph(vs, part) == false) {
      for (int v : vs) {
        partition[v] = id;
      }
      return;
    };
  if (depth >= 5) {
    // assign labels
    int pa = 0, pb = 0;
    for (int v : vs) {
      assert(part[v] != FUZZY);
      if (part[v] == REPA) {
        pa++;
        partition[v] = id * 2 + 1;
      } else {
        pb++;
        partition[v] = id * 2 + 2;
      }
    }
    std::cout << "Finish parition, pa and pb " << pa << " " << pb << std::endl;
    return;
  }
  // gather A and B
  std::vector<int> vs_a, vs_b;
  for (int v : vs) {
    assert(part[v] != FUZZY);
    if (part[v] == REPA) {
      vs_a.push_back(v);
    } else {
      vs_b.push_back(v);
    }
    part.erase(v);
  }
  // if(vs_a.size() >= 9 * vs_b.size() || vs_b.size() >= 9 * vs_a.size()) {
  //   for(int v: vs) {
  //     assert(part[v] != FUZZY);
  //     if(part[v] == REPA) {
  //       partition[v] = id*2 + 1;
  //     } else {
  //       partition[v] = id*2 + 2;
  //     }
  //   }
  //   return;
  // }
  std::cout << "VS, VS A, VS B " << vs.size() << " " << vs_a.size() << " "
            << vs_b.size() << std::endl;
  std::cout << "part size " << part.size() << std::endl;
  meshSeg(depth + 1, id * 2 + 1, vs_a, part);
  std::cout << "part size " << part.size() << std::endl;
  meshSeg(depth + 1, id * 2 + 2, vs_b, part);
}

// vs: all vertices used in this decomposition
// part: corresponding partition for vs
bool Mesh::buildFlowGraph(std::vector<int>& vs, std::map<int, int>& part) {
  FlowNet flowNet(num_faces);
  std::map<int, int> ori2flow;  // origin vertices to flow net vertices
  std::set<int> v_ca, v_cb;
  std::set<int> cur_vs(vs.begin(), vs.end());

  int src = 0, dst;
  int vcnt = 1, fuzzy = 0;
  for (int v : vs) {
    if (part[v] == FUZZY) {
      fuzzy++;
      if (ori2flow.find(v) == ori2flow.end()) {
        ori2flow[v] = vcnt++;
      }
      int m = 0;
      for (DualEdge& de : faces[v].dedges) {
        if (cur_vs.find(de.face) == cur_vs.end()) continue;
        int neibor = de.face, par = part[neibor];
        if (ori2flow.find(neibor) == ori2flow.end()) {
          ori2flow[neibor] = vcnt++;
        }
        bool add_edge = true;
        if (par == FUZZY) {
          assert(v != neibor);
          // make sure edges in C are only added once
          if (neibor < v) {
            add_edge = false;
          }
        } else if (par == REPA) {
          v_ca.insert(neibor);
        } else if (par == REPB) {
          v_cb.insert(neibor);
        }
        if (add_edge) {
          float cap = 1.0 / (1 + de.angle_dist / avg_angle_dist);
          int from = ori2flow[v], to = ori2flow[neibor];
          flowNet.addEdge(from, to, cap);
        }
      }
    }
  }
  dst = vcnt;
  std::cout << "Vertices all: " << vcnt + 1 << " fuzzy: " << fuzzy
            << " VCA: " << v_ca.size() << " VCB: " << v_cb.size() << std::endl;
  if (v_ca.size() == 0 || v_cb.size() == 0) return false;
  // add edges for src and dst
  for (int vca : v_ca) {
    flowNet.addEdge(0, ori2flow[vca], INF_FLOAT);
  }
  for (int vcb : v_cb) {
    flowNet.addEdge(dst, ori2flow[vcb], INF_FLOAT);
  }
  flowNet.num_v = vcnt + 1;
  flowNet.EK(src, dst);
  // assign flow net result
  for (std::map<int, int>::const_iterator it = ori2flow.begin();
       it != ori2flow.end(); it++) {
    if (flowNet.visit[it->second]) {
      partition[it->first] = REPA;
      part[it->first] = REPA;
    } else {
      partition[it->first] = REPB;
      part[it->first] = REPB;
    }
  }
  return true;
}

void Mesh::writeObj(const std::string& name) {
  std::cout << "file name is: " << name << std::endl;
  std::ofstream fout(name + ".ply");
  fout << "ply" << std::endl;
  fout << "format ascii 1.0" << std::endl;
  fout << "element vertex " << vertices.size() << std::endl;
  fout << "property float x" << std::endl;
  fout << "property float y" << std::endl;
  fout << "property float z" << std::endl;
  fout << "element face " << num_faces << std::endl;
  fout << "property list uchar int vertex_indices" << std::endl;
  fout << "property uint8 red" << std::endl;
  fout << "property uint8 green" << std::endl;
  fout << "property uint8 blue" << std::endl;
  fout << "end_header" << std::endl;
  for (Point v : vertices) {
    fout << v.x << " " << v.y << " " << v.z << std::endl;
  }
  for (int i = 0; i < num_faces; i++) {
    fout << "3 " << faces[i].indices.x << " " << faces[i].indices.y << " "
         << faces[i].indices.z << " ";
    fout << 50 * ((partition[i] + 3) % 5 + 1) << " "
         << 80 * ((partition[i] + 1) % 3 + 1) << " "
         << 60 * (partition[i] % 4 + 1) << std::endl;
    // if(partition[i] == 3) {
    //   fout << 255 << " " << 0 << " " << 255 << std::endl;
    // } else if(partition[i] == 4) {
    //   fout << 255 << " " << 255 << " " << 255 << std::endl;
    // } else if(partition[i] == 5) {
    //   fout << 255 << " " << 255 << " " << 0 << std::endl;
    // } else if(partition[i] == 6) {
    //   fout << 0 << " " << 255 << " " << 0 << std::endl;
    // }
  }
  fout.close();
}