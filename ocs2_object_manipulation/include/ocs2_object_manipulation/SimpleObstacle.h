#pragma once

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
// #include <ocs2_object_manipulation_ros/StateEstimation.h>

namespace ocs2
{
  namespace object_manipulation
  {

    class ObstacleSimple : public SolverSynchronizedModule
    {
    public:
      ObstacleSimple(const vector_t &pos, scalar_t radius)
      {
        params_ = vector_t(5);
        params_.head(2) = pos;
        params_(2) = radius;
      };

      vector_t getParameters() const { return params_; }

      void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                        const ReferenceManagerInterface &referenceManager) override
      {
        params_(3) = initTime;
        // state = SE.object_data.state;
        params_(4) = (params_.segment<2>(0) - currentState.segment<2>(0)).norm() - params_(2) - 1;
      }

      void postSolverRun(const PrimalSolution& primalSolution) override{};

    private:
      vector_t params_;
      // StateEstimation SE;
      // vector_t state;
    };

    class CBF final : public StateInputConstraintCppAd
    {
    public:
      CBF(const ObstacleSimple &distance)
          : StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear), distance_(distance)
      {
        initialize(6, 3, 5, "CbfObstacle", "/tmp/ocs2", true, true);
      }

      vector_t getParameters(scalar_t time, const PreComputation & /* preComputation */) const override { return distance_.getParameters(); }

      CBF *clone() const override { return new CBF(*this); }

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; }

      ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t &state, const ad_vector_t &input,
                                     const ad_vector_t &parameters) const override
      {
        ad_vector_t constraint(2);

        constraint(0) = abs(state(3)) - ad_scalar_t(0.1);
        constraint(1) = abs(state(4)) - ad_scalar_t(0.1);

        return constraint;
      }

    private:
      CBF(const CBF &rhs) : StateInputConstraintCppAd(rhs), distance_(rhs.distance_) {}
      const ObstacleSimple &distance_;
    };

  } // namespace object_manipulation
} // namespace ocs2
