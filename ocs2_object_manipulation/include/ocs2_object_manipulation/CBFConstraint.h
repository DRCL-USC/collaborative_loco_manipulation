
#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2
{
  namespace object_manipulation
  {

    class CBF_Constraint final : public StateConstraint
    {
    public:
      CBF_Constraint(vector_t pos)
          : StateConstraint(ConstraintOrder::Quadratic), pos_(pos.head(2)), r_(pos(2)){};

      ~CBF_Constraint() override = default;
      CBF_Constraint *clone() const override { return new CBF_Constraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return 1; };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        vector_t constraint(1);
        constraint << -(r_ + 0.75) * (r_ + 0.75) + (state(0) - pos_(0)) * (state(0) - pos_(0)) + (state(1) - pos_(1)) * (state(1) - pos_(1));
        return constraint;
      };
      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;

        linearApproximation.f = getValue(time, state, preComp);

        matrix_t C = (matrix_t(1, state.size()) << 2 * (state(0) - pos_(0)), 2 * (state(1) - pos_(1)), 0, 0, 0, 0).finished();
        linearApproximation.dfdx = C;
        return linearApproximation;
      };

      VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t &state,
                                                                             const PreComputation &preComp) const override
      {
        VectorFunctionQuadraticApproximation quadraticApproximation;

        quadraticApproximation.f = getValue(time, state, preComp);

        matrix_t C = (matrix_t(1, state.size()) << 2 * (state(0) - pos_(0)), 2 * (state(1) - pos_(1)), 0, 0, 0, 0).finished();
        quadraticApproximation.dfdx = C;

        matrix_t dC = matrix_t::Zero(state.size(), state.size());
        dC(0, 0) = 2;
        dC(1, 1) = 2;
        quadraticApproximation.dfdxx.emplace_back(dC);

        return quadraticApproximation;
      }

    private:
      CBF_Constraint(const CBF_Constraint &other) = default;
      vector_t pos_;
      scalar_t r_;
    };

  } // namespace object_manipulation
} // namespace ocs2
