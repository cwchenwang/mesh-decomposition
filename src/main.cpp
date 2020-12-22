#include "mesh.h"
#include "vector3.h"

int main(int argc, char** argv) {
  std::string file_path = std::string(argv[1]);
  std::unique_ptr<Mesh> mesh = std::make_unique<Mesh>("../obj/" + file_path);
  mesh->getDual();
  mesh->compDist();
  mesh->solve();
  mesh->writeObj(file_path.substr(0, file_path.size() - 4));
  return 0;
}