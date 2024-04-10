#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_object_manipulation/ObjectParameters.h"
#include "ocs2_object_manipulation/definitions.h"

namespace ocs2 {
namespace object_manipulation {

class ObjectSytemDynamics : public SystemDynamicsBaseAD {
 public:
  ObjectSytemDynamics(const ObjectParameters& objectParameters, const std::string& libraryFolder, bool verbose)
      : param_(objectParameters) {
    initialize(STATE_DIM, INPUT_DIM, "object_dynamics", libraryFolder, true, verbose);
  }

  ~ObjectSytemDynamics() override = default;

  ObjectSytemDynamics(const ObjectSytemDynamics& rhs) = default;

  ObjectSytemDynamics* clone() const override { return new ObjectSytemDynamics(*this); }

  ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                            const ad_vector_t& parameters) const override {
    const ad_scalar_t cosTheta = cos(state(0));
    const ad_scalar_t sinTheta = sin(state(0));

    // Inertia tensor
    Eigen::Matrix<ad_scalar_t, 3, 3> I;
    I << static_cast<ad_scalar_t>(param_.Mass_), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), 
         static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(param_.Mass_), static_cast<ad_scalar_t>(0.0),
         static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(param_.Inertia_);

    // RHS
    Eigen::Matrix<ad_scalar_t, 3, 1> rhs(input(0), input(1), input(2));

    // dxdt
    ad_vector_t stateDerivative(STATE_DIM);
    stateDerivative << state.tail<3>(), I.inverse() * rhs;
    return stateDerivative;
  }

 private:
  ObjectParameters param_;
};

}  // namespace object_manipulation
}  // namespace ocs2
