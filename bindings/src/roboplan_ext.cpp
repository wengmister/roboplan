#include <iostream>
#include <nanobind/nanobind.h>
#include <roboplan/utils.hpp>

namespace roboplan {

NB_MODULE(roboplan, m) {
    m.def("add", &add);
}

}  // namespace roboplan
