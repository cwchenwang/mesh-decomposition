#pragma once
#include <iostream>
#include <queue>
#include <vector>

#define FLOW_EPSILON 1e-10
#define INF 1e12

struct Edge {
  int to, next;
  int from;
  float cap;
  friend std::ostream& operator<<(std::ostream& os, const Edge& e) {
    return os << e.from << ", " << e.to << ", " << e.cap;
  }
};

class FlowNet {
 public:
  int num_v, edges_num;
  std::vector<int> head, last;
  std::vector<Edge> edges;
  std::vector<bool> visit;
  std::vector<float> flow;

  FlowNet(int num_ver);
  void addEdge(int from, int to, float cap);

  bool bfs(int src, int dst);
  float EK(int src, int dst);
  void findCut(int src);
};