#pragma once

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

#include "ocs2_object_manipulation/ObjectParameters.h"
#include "ocs2_object_manipulation/definitions.h"

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
      ObjectSytemDynamics(const ObjectParameters &objectParameters, const std::string &libraryFolder, bool verbose)
          : param_(objectParameters)
      {
        initialize(STATE_DIM, INPUT_DIM, "object_dynamics", libraryFolder, true, verbose);
      }

      ~ObjectSytemDynamics() override = default;

      ObjectSytemDynamics(const ObjectSytemDynamics &rhs) = default;

      ObjectSytemDynamics *clone() const override { return new ObjectSytemDynamics(*this); }

      ad_vector_t systemFlowMap(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
                                const ad_vector_t &parameters) const override
      {
        // Forces
        ad_vector_t total_forces_b(2);
        for (int i = 0; i < AGENT_NUM; i++)
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
        ad_scalar_t torque;
        for (int i = 0; i < AGENT_NUM; i++)
        {
          torque += -state(6 + i) * input(i);
        }
        Eigen::Matrix<ad_scalar_t, 3, 1> rhs(total_forces_w(0), total_forces_w(1), torque);

        // dxdt
        ad_vector_t aux_state_derivative(AGENT_NUM);
        for (int i = 0; i < AGENT_NUM; i++)
        {
          aux_state_derivative(i) = input(i + AGENT_NUM);
        }

        ad_vector_t stateDerivative(STATE_DIM);
        stateDerivative << state.tail<3>(), I.inverse() * rhs, aux_state_derivative;
        return stateDerivative;
      }

    private:
      ObjectParameters param_;
    };

  } // namespace object_manipulation
} // namespace ocs2
