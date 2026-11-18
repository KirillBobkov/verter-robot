#include "slam_toolbox/motion_gate.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

int main()
{
  using slam_toolbox::hasMovedEnough;
  const double pi = std::acos(-1.0);
  int checks = 0;
  auto expect = [&](bool actual, bool expected, const char * name) {
      ++checks;
      if (actual != expected) {
        std::cerr << "FAIL " << name << std::endl;
        std::exit(1);
      }
    };
  expect(hasMovedEnough(0, 0, .0064, .0873), false, "stationary");
  expect(hasMovedEnough(.001, .02, .0064, .0873), false, "both below threshold");
  expect(hasMovedEnough(.0064, 0, .0064, .0873), true, "translation equality");
  expect(hasMovedEnough(.01, 0, .0064, .0873), true, "translation only");
  expect(hasMovedEnough(0, .0873, .0064, .0873), true, "heading equality");
  expect(hasMovedEnough(0, .09, .0064, .0873), true, "rotation only");
  expect(hasMovedEnough(0, -.09, .0064, .0873), true, "negative rotation");
  expect(hasMovedEnough(.01, .09, .0064, .0873), true, "both above threshold");
  expect(hasMovedEnough(0, -2 * pi + .02, .0064, .0873), false, "wrap small positive");
  expect(hasMovedEnough(0, 2 * pi - .02, .0064, .0873), false, "wrap small negative");
  expect(hasMovedEnough(0, -2 * pi + .1, .0064, .0873), true, "wrap sufficient turn");
  expect(hasMovedEnough(0, pi, .0064, .0873), true, "half turn");
  std::cout << checks << " motion gate checks passed" << std::endl;
}
