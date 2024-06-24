
#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2
{
  namespace object_manipulation
  {

    class ObjectCBFConstraint final : public StateConstraint
    {
    public:
      ObjectCBFConstraint(std::vector<std::pair<scalar_t, scalar_t>> pos_array, scalar_array_t radius_array, scalar_t alpha)
          : StateConstraint(ConstraintOrder::Quadratic), pos_array_(pos_array), radius_array_(radius_array), alpha_(alpha)
      {
        assert(pos_array_.size() == radius_array_.size());
      };

      ~ObjectCBFConstraint() override = default;
      ObjectCBFConstraint *clone() const override { return new ObjectCBFConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return pos_array_.size(); };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        vector_t constraint(pos_array_.size());

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          scalar_t B = -(radius_array_[i] + 0.75) * (radius_array_[i] + 0.75) + (state(0) - pos_array_[i].first) * (state(0) - pos_array_[i].first) + (state(1) - pos_array_[i].second) * (state(1) - pos_array_[i].second);
          constraint(i) = alpha_ * B + 2 * (state(0) - pos_array_[i].first) * state(3) + 2 * (state(1) - pos_array_[i].second) * state(4);
        }

        return constraint;
      };

      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;

        linearApproximation.f = getValue(time, state, preComp);

        matrix_t C(pos_array_.size(), state.size());

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          C.row(i) << 2 * (state(0) - pos_array_[i].first) + 2 * state(3), 2 * (state(1) - pos_array_[i].second) + 2 * state(4),
              0, 2 * (state(0) - pos_array_[i].first), 2 * (state(1) - pos_array_[i].second), 0;
        }

        linearApproximation.dfdx = C;
        return linearApproximation;
      };

      VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t &state,
                                                                     const PreComputation &preComp) const override
      {
        VectorFunctionQuadraticApproximation quadraticApproximation;

        quadraticApproximation.f = getValue(time, state, preComp);

        quadraticApproximation.dfdx = getLinearApproximation(time, state, preComp).dfdx;

        matrix_t dC = matrix_t::Zero(state.size(), state.size());
        dC(0, 0) = 2;
        dC(3, 0) = 2;
        dC(1, 1) = 2;
        dC(4, 1) = 2;
        dC(0, 3) = 2;
        dC(1, 4) = 2;

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          quadraticApproximation.dfdxx.emplace_back(dC);
        }

        return quadraticApproximation;
      }

    private:
      ObjectCBFConstraint(const ObjectCBFConstraint &other) = default;
      const std::vector<std::pair<scalar_t, scalar_t>> pos_array_;
      const scalar_array_t radius_array_;
      scalar_t alpha_;
    };

  } // namespace object_manipulation
} // namespace ocs2
