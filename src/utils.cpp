#include <roboplan/utils.hpp>

#include "pinocchio/multibody/model.hpp"

namespace roboplan {

int add(int a, int b) { 
    std::cout << "Adding " << a << " and " << b << ".\n";
    return a + b;
}

void createPinocchioModel() {
    pinocchio::Model model;
    std::cout << "Built a Pinocchio model with " << std::to_string(model.nq) << " DOFs\n";
    return;
}

} // namespace roboplan
