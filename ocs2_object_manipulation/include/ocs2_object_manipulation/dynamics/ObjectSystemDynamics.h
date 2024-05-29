#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_object_manipulation/ObjectParameters.h"
#include "ocs2_object_manipulation/definitions.h"
#include "ocs2_object_manipulation/adaptiveControl.h"

namespace ocs2
{
  namespace object_manipulation
  {
    template <typename T>
    Eigen::Matrix<T, 2, 2> rotmat_2d(T yaw)
    {
      Eigen::Matrix<T, 2, 2> rotmat_2d;
      rotmat_2d << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
      return rotmat_2d;
    }

    class ObjectSytemDynamics : public SystemDynamicsBaseAD
    {
    public:
      ObjectSytemDynamics(const ObjectManipulationParameters &objectParameters, std::shared_ptr<AdaptiveControl> adaptiveControlPtr,
                          const std::string &libraryFolder, bool verbose)
          : param_(objectParameters), adaptiveControlPtr_(adaptiveControlPtr)
      {
        initialize(STATE_DIM, INPUT_DIM, "object_dynamics", libraryFolder, true, verbose);
      }

      ~ObjectSytemDynamics() override = default;

      ObjectSytemDynamics(const ObjectSytemDynamics &rhs) = default;

      ObjectSytemDynamics *clone() const override { return new ObjectSytemDynamics(*this); }

      vector_t getFlowMapParameters(scalar_t time, const PreComputation & /* preComputation */) const override { 
        return adaptiveControlPtr_->getAdaptiveLaw();}

      size_t getNumFlowMapParameters() const { return adaptiveControlPtr_->getAdaptiveLaw().size(); }  

      ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
                                const ad_vector_t &parameters) const override
      {

        ad_vector_t total_forces_b(2);
        for (int i = 0; i < AGENT_COUNT; i++)
        {
          total_forces_b += rotmat_2d(static_cast<ad_scalar_t>(param_.agents_init_yaw_[i])) * (ad_vector_t(2) << input(i), static_cast<ad_scalar_t>(0.0)).finished();
        }

        ad_vector_t total_forces_w = rotmat_2d(state(2)) * total_forces_b;

        // Inertia tensor
        Eigen::Matrix<ad_scalar_t, 3, 3> I;
        I << static_cast<ad_scalar_t>(param_.Mass_), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0),
            static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(param_.Mass_), static_cast<ad_scalar_t>(0.0),
            static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(0.0), static_cast<ad_scalar_t>(param_.Inertia_);

        // RHS
        Eigen::Matrix<ad_scalar_t, 3, 1> rhs(total_forces_w(0), total_forces_w(1), -input(0) * input(2) - input(1) * input(3));

        // dxdt
        ad_vector_t stateDerivative(STATE_DIM);
        stateDerivative << state.tail<3>(), I.inverse() * rhs;
        return stateDerivative;
      }

    private:
      ObjectManipulationParameters param_;
      std::shared_ptr<AdaptiveControl> adaptiveControlPtr_;
    };

  } // namespace object_manipulation
} // namespace ocs2
