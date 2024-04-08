#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace object_manipulation {

struct ObjectParameters {
  /** Constructor */
  ObjectParameters() {  }

  void display() {
    std::cerr << "Object parameters: "
              << "\n";
    std::cerr << "Mass:   " << Mass_ << "\n";
    std::cerr << "Inertia:   " << Inertia_ << "\n";
    std::cerr << "rx: " << rx_ << "\n";
    std::cerr << "ry:    " << ry_ << "\n";
  }

  /** Loads the Object's parameters. */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << "\n #### Object Parameters:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, Mass_, fieldName + ".Mass", verbose);
    loadData::loadPtreeValue(pt, Inertia_, fieldName + ".Inertia", verbose);
    loadData::loadPtreeValue(pt, rx_, fieldName + ".rx", verbose);
    loadData::loadPtreeValue(pt, ry_, fieldName + ".ry", verbose);
    if (verbose) {
      std::cerr << " #### =============================================================================\n" << std::endl;
    }
  }

  scalar_t Mass_ = 1.0;       // [kg]
  scalar_t Inertia_ = 1.0;       // [kg*m^2]
  scalar_t rx_ = 1.0;     // [m]
  scalar_t ry_ = 6.0;       // [m]

 private:

};

}  // namespace object_manipulation
}  // namespace ocs2
