
#pragma once

#include <ocs2_core/constraint/StateConstraint.h>

namespace ocs2
{
  namespace object_manipulation
  {

    class CBF_Constraint final : public StateConstraint
    {
    public:
      CBF_Constraint(vector_t pos, scalar_t alpha)
          : StateConstraint(ConstraintOrder::Quadratic), pos_(pos.head(2)), r_(pos(2)), alpha_(alpha){};

      ~CBF_Constraint() override = default;
      CBF_Constraint *clone() const override { return new CBF_Constraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return 1; };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        scalar_t B = -(r_ + 0.75) * (r_ + 0.75) + (state(0) - pos_(0)) * (state(0) - pos_(0)) + (state(1) - pos_(1)) * (state(1) - pos_(1)); 
        scalar_t H = alpha_ * B + 2 * (state(0) - pos_(0)) * state(3) + 2 * (state(1) - pos_(1)) * state(4);
        return (vector_t(1) << H).finished();
      };
      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;

        linearApproximation.f = getValue(time, state, preComp);

        matrix_t C = (matrix_t(1, state.size()) << 2 * (state(0) - pos_(0)) + 2 * state(3), 2 * (state(1) - pos_(1)) + 2 * state(4),
                      0, 2 * (state(0) - pos_(0)), 2 * (state(1) - pos_(1)), 0)
                         .finished();
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
        quadraticApproximation.dfdxx.emplace_back(dC);

        return quadraticApproximation;
      }

    private:
      CBF_Constraint(const CBF_Constraint &other) = default;
      vector_t pos_;
      scalar_t r_;
      scalar_t alpha_;
    };

  } // namespace object_manipulation
} // namespace ocs2
