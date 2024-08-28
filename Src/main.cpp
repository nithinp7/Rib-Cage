#include "RibCage.h"
#include <Althea/Application.h>

#include <iostream>

namespace RibCage {
  Application* GApplication;
} // namespace RibCage

using namespace RibCage;
using namespace AltheaEngine;

int main() {
  GApplication = new Application("Rib Cage", "..", "../Extern/Althea");
  GApplication->createGame<::RibCage::RibCage>();

  try {
    GApplication->run();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    delete GApplication;
    return EXIT_FAILURE;
  }

  delete GApplication;

  return EXIT_SUCCESS;
}