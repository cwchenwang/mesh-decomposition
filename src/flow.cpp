#include "flow.h"

#include <iostream>

FlowNet::FlowNet(int num_ver) : num_v(num_ver), edges_num(0) {
  head.resize(num_v);
  last.resize(num_v);
  flow.resize(num_v);
  visit.resize(num_v);
  for (int i = 0; i < num_v; i++) {
    head[i] = -1;
    last[i] = -1;
    visit[i] = false;
    flow[i] = 0.0f;
  }
  edges.resize(num_v * 2 + 10);
}

void FlowNet::addEdge(int from, int to, float cap) {
  edges[edges_num].from = from;
  edges[edges_num].to = to;
  edges[edges_num].cap = cap;
  edges[edges_num].next = head[from];
  head[from] = edges_num++;

  edges[edges_num].from = to;
  edges[edges_num].to = from;
  edges[edges_num].cap = cap;  // TODO
  edges[edges_num].next = head[to];
  head[to] = edges_num++;
  if (edges_num >= edges.size()) {
    std::cout << "Edges exceeded, resize" << std::endl;
    edges.resize(edges.size() * 2);
  }
}

bool FlowNet::bfs(int s, int t) {
  for (int i = 0; i < num_v; i++) {
    last[i] = -1;
  }
  std::queue<int> q;
  q.push(s);
  flow[s] = INF;
  while (!q.empty()) {
    int p = q.front();
    q.pop();
    if (p == t)  // reach destination, finish searching
      break;
    for (int eg = head[p]; eg != -1; eg = edges[eg].next) {
      int to = edges[eg].to;
      float vol = edges[eg].cap;
      // if not visited (last == -1) and have capacity
      if (vol > FLOW_EPSILON && last[to] == -1) {
        last[to] = eg;
        flow[to] = fmin(flow[p], vol);
        q.push(to);
      }
    }
  }
  return last[t] != -1;
}

float FlowNet::EK(int s, int t) {
  float maxflow = 0;
  while (bfs(s, t)) {
    maxflow += flow[t];
    // go back to source from destination and update capacity
    for (int i = t; i != s; i = edges[last[i] ^ 1].to) {
      edges[last[i]].cap -= flow[t];
      edges[last[i] ^ 1].cap += flow[t];
    }
  }

  //   for (int i = 0; i < edges_num; i++) {
  //     std::cout << edges[i] << std::endl;
  //   }
  findCut(0);
  return maxflow;
}

void FlowNet::findCut(int src) {
  //   std::cout << "visit " << src << std::endl;
  visit[src] = true;
  for (int i = head[src]; i != -1; i = edges[i].next) {
    int to = edges[i].to;
    if (edges[i].cap > FLOW_EPSILON && !visit[to]) {
      findCut(to);
    }
  }
}

// int main() {
//   FlowNet flowNet(6);
//   flowNet.addEdge(0, 1, 16.0);
//   flowNet.addEdge(0, 2, 13.0);
//   flowNet.addEdge(1, 2, 10.0);
//   flowNet.addEdge(2, 1, 4.0);
//   flowNet.addEdge(1, 3, 12.0);
//   flowNet.addEdge(3, 2, 9.0);
//   flowNet.addEdge(2, 4, 14.0);
//   flowNet.addEdge(4, 3, 7.0);
//   flowNet.addEdge(3, 5, 20.0);
//   flowNet.addEdge(4, 5, 4.0);
//   std::cout << flowNet.EK(0, 5) << std::endl;
//   return 0;
// }