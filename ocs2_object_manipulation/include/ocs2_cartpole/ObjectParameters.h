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
  ObjectParameters() { computeInertiaTerms(); }

  void display() {
    std::cerr << "Cart-pole parameters: "
              << "\n";
    std::cerr << "cartMass:   " << cartMass_ << "\n";
    std::cerr << "poleMass:   " << poleMass_ << "\n";
    std::cerr << "poleLength: " << poleLength_ << "\n";
    std::cerr << "poleMoi:    " << poleMoi_ << "\n";
    std::cerr << "maxInput:   " << maxInput_ << "\n";
    std::cerr << "gravity:    " << gravity_ << "\n";
  }

  /** Loads the Cart-Pole's parameters. */
  void loadSettings(const std::string& filename, const std::string& fieldName, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);
    if (verbose) {
      std::cerr << "\n #### Cart-pole Parameters:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, cartMass_, fieldName + ".cartMass", verbose);
    loadData::loadPtreeValue(pt, poleMass_, fieldName + ".poleMass", verbose);
    loadData::loadPtreeValue(pt, poleLength_, fieldName + ".poleLength", verbose);
    loadData::loadPtreeValue(pt, maxInput_, fieldName + ".maxInput", verbose);
    loadData::loadPtreeValue(pt, gravity_, fieldName + ".gravity", verbose);
    computeInertiaTerms();
    if (verbose) {
      std::cerr << " #### =============================================================================\n" << std::endl;
    }
  }

  scalar_t cartMass_ = 1.0;       // [kg]
  scalar_t poleMass_ = 1.0;       // [kg]
  scalar_t poleLength_ = 1.0;     // [m]
  scalar_t maxInput_ = 6.0;       // [N]
  scalar_t gravity_ = 9.8;        // [m/s^2]
  scalar_t poleHalfLength_ = -1;  // [m]
  scalar_t poleMoi_ = -1;         // [kg*m^2]
  scalar_t poleSteinerMoi_ = -1;  // [kg*m^2]

 private:
  void computeInertiaTerms() {
    poleHalfLength_ = poleLength_ / 2.0;
    poleMoi_ = 1.0 / 12.0 * poleMass_ * (poleLength_ * poleLength_);
    poleSteinerMoi_ = poleMoi_ + poleMass_ * (poleHalfLength_ * poleHalfLength_);
  }
};

}  // namespace object_manipulation
}  // namespace ocs2
