#include "RibCage.h"
#include <Althea/Application.h>

#include <iostream>

using namespace AltheaEngine;

int main() {
  Application app("Rib Cage", "..", "../Extern/Althea");
  app.createGame<RibCage::RibCage>();

  try {
    app.run();
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}