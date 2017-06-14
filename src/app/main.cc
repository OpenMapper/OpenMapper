//============================================================================
// Name        : main.cc
// Author      : Carlos Gomes
// Copyright   : Copyright OpenMapper
// Description : Sample code to handle with the wrapper library
//============================================================================

#include <iostream>
#include <vector>

#include "wrapper.h"

int main(int argc, char** argv) {
  std::cout << "Hello World!" << std::endl;

  // Create instances of the classes providing the required methods.
  openmapper_wrapper::Wrapper wrapper;
  return 0;
}
