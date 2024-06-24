
#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_object_manipulation/definitions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2
{
  namespace object_manipulation
  {

    class RobotCBFConstraint final : public StateInputConstraint
    {
    public:
      RobotCBFConstraint(std::vector<std::pair<scalar_t, scalar_t>> pos_array, scalar_array_t radius_array, scalar_t alpha, scalar_array_t init_yaw)
          : StateInputConstraint(ConstraintOrder::Linear), pos_array_(pos_array), radius_array_(radius_array), alpha_(alpha), init_yaw_(init_yaw)
      {
        assert(pos_array_.size() == radius_array_.size());
      };

      ~RobotCBFConstraint() override = default;
      RobotCBFConstraint *clone() const override { return new RobotCBFConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return AGENT_COUNT * pos_array_.size(); };

      vector_t getValue(scalar_t time, const vector_t &state, const vector_t &input, const PreComputation &preComp) const override
      {
        vector_t constraint(AGENT_COUNT * pos_array_.size());

        for (size_t j = 0; j < AGENT_COUNT; ++j)
        {
          Eigen::Matrix3d rotmat = getRotationMatrixFromZyxEulerAngles((Eigen::Vector3d() << state(2) + init_yaw_[j], 0.0, 0.0).finished()); // (yaw, pitch, roll)

          Eigen::Matrix<scalar_t, 3, 1> offset = rotmat * (Eigen::Matrix<scalar_t, 3, 1>() << offset_x,
                                                           input(AGENT_COUNT + j), 0.0)
                                                              .finished();
          vector_t robot_pose_world(2);
          robot_pose_world << state(0) + offset(0), state(1) + offset(1);

          for (size_t i = 0; i < pos_array_.size(); ++i)
          {
            scalar_t B = -(radius_array_[i] + 0.8) * (radius_array_[i] + 0.8) + (robot_pose_world(0) - pos_array_[i].first) * (robot_pose_world(0) - pos_array_[i].first) + (robot_pose_world(1) - pos_array_[i].second) * (robot_pose_world(1) - pos_array_[i].second);
            constraint(i + pos_array_.size() * j) = alpha_ * B;
          }
        }

        return constraint;
      };

      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const vector_t &input, const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;

        linearApproximation.f = getValue(time, state, input, preComp);

        matrix_t C(AGENT_COUNT * pos_array_.size(), state.size());
        C.setZero();

        matrix_t D(AGENT_COUNT * pos_array_.size(), input.size());
        D.setZero();

        for (size_t j = 0; j < AGENT_COUNT; ++j)
        {

          Eigen::Matrix3d rotmat = getRotationMatrixFromZyxEulerAngles((Eigen::Vector3d() << state(2) + init_yaw_[j], 0.0, 0.0).finished()); // (yaw, pitch, roll)

          Eigen::Matrix<scalar_t, 3, 1> offset = rotmat * (Eigen::Matrix<scalar_t, 3, 1>() << offset_x,
                                                           input(AGENT_COUNT + j), 0.0)
                                                              .finished();
          vector_t robot_pose_world(2);
          robot_pose_world << state(0) + offset(0), state(1) + offset(1);

          for (size_t i = 0; i < pos_array_.size(); ++i)
          {
            C.row(i)(0) = 2 * (robot_pose_world(0) - pos_array_[i].first);
            C.row(i)(1) = 2 * (robot_pose_world(1) - pos_array_[i].second);
            C.row(i)(2) = 2 * (robot_pose_world(0) - pos_array_[i].first) * (-offset_x * sin(state(2) + init_yaw_[j]) - input(AGENT_COUNT + j) * cos(state(2) + init_yaw_[j])) +
                          2 * (robot_pose_world(1) - pos_array_[i].second) * (offset_x * cos(state(2) + init_yaw_[j]) - input(AGENT_COUNT + j) * sin(state(2) + init_yaw_[j]));

            D.row(i)(AGENT_COUNT + j) = 2 * (robot_pose_world(0) - pos_array_[i].first) * -sin(state(2) + init_yaw_[j]) +
                                        2 * (robot_pose_world(1) - pos_array_[i].second) * cos(state(2) + init_yaw_[j]);
          }
        }

        linearApproximation.dfdx = C;
        linearApproximation.dfdu = D;
        return linearApproximation;
      };

      // VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t &state, const vector_t &input,
      //                                                                const PreComputation &preComp) const override
      // {
      //   VectorFunctionQuadraticApproximation quadraticApproximation;

      //   quadraticApproximation.f = getValue(time, state, input, preComp);

      //   quadraticApproximation.dfdx = getLinearApproximation(time, state, input, preComp).dfdx;

      //   matrix_t dC = matrix_t::Zero(state.size(), state.size());
      //   dC(0, 0) = 2;
      //   dC(3, 0) = 2;
      //   dC(1, 1) = 2;
      //   dC(4, 1) = 2;
      //   dC(0, 3) = 2;
      //   dC(1, 4) = 2;

      //   for (size_t i = 0; i < pos_array_.size(); ++i)
      //   {
      //     quadraticApproximation.dfdxx.emplace_back(dC);
      //   }

      //   return quadraticApproximation;
      // }

    private:
      RobotCBFConstraint(const RobotCBFConstraint &other) = default;
      const std::vector<std::pair<scalar_t, scalar_t>> pos_array_;
      const scalar_array_t radius_array_;
      scalar_t alpha_;
      scalar_array_t init_yaw_;
      scalar_t offset_x = -0.25 - 0.35;
    };

  } // namespace object_manipulation
} // namespace ocs2
