
#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_object_manipulation/LowPassFilter.h>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2
{
    namespace object_manipulation
    {

        class AdaptiveControl : public SolverSynchronizedModule
        {
        public:
            AdaptiveControl(scalar_t mpcDesiredFrequency, const std::string &taskFile) : dt(1 / mpcDesiredFrequency)
            {
                loadData::loadEigenMatrix(taskFile, prefix + "Gamma", Gamma);
                loadData::loadEigenMatrix(taskFile, prefix + "bounds", bounds);
                loadData::loadCppDataType(taskFile, prefix + "Kd", Kd);
                loadData::loadCppDataType(taskFile, prefix + "alpha", alpha);
                loadData::loadCppDataType(taskFile, prefix + "lambda", lambda);

                a_hat.setZero();
                a_hat_dot.setZero();

                Y.resize(3, 4);
                Y.setZero();

                adaptive_law.setZero();
                lpf.reset(new LowPassFilter<vector_t>(alpha));
            };

            vector_t update(vector_t q, vector_t dq, vector_t q_d, vector_t dq_d, vector_t ddq_d)
            {

                vector_t s = (dq - dq_d) + lambda * (q - q_d);
                vector_t dq_r = dq - s;
                vector_t ddq_r = ddq_d - lambda * (dq - dq_d);

                a_hat_dot = Gamma.asDiagonal() * Y.transpose() * s;
                a_hat -= dt * a_hat_dot;

                matrix_t Y_H = (matrix_t(3,4) << ddq_r(0), 0, -sin(q(2)) * ddq_r(2), cos(q(2)) * ddq_r(2),
                    ddq_r(1), 0, -cos(q(2)) * ddq_r(2), -sin(q(2)) * ddq_r(2),
                    0, ddq_r(2), -sin(q(2)) * ddq_r(0) - cos(q(2)) * ddq_r(1), cos(q(2)) * ddq_r(0) - sin(q(2)) * ddq_r(1)).finished();

                matrix_t Y_C = (matrix_t(3,4) << 0, 0, dq(2) * dq_r(2), 0,
                    0, 0, 0, dq(2) * dq_r(2),
                    0, 0, 0, 0).finished();

                Y = Y_H + Y_C;

                return -(Kd * s + Y * a_hat)*0;
            };

            void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                              const ReferenceManagerInterface &referenceManager) override
            {
                if (primalSolution_.stateTrajectory_.size() > 1)
                {

                    vector_t q_d = primalSolution_.stateTrajectory_.back();

                    adaptive_law = update(currentState.head(3), currentState.segment<3>(3), q_d.head(3), q_d.segment<3>(3), vector_t::Zero(3));
                    adaptive_law = lpf->process(adaptive_law);
                    saturate(adaptive_law, -bounds, bounds);
                    // std::cout << "Adaptive control law: " << adaptive_law.transpose() << "\n";
                }
            }

            void postSolverRun(const PrimalSolution &primalSolution) override
            {
                primalSolution_ = primalSolution;
            };

            vector_t getAdaptiveLaw() const
            {
                return adaptive_law;
            };

        private:
            const std::string prefix = "adaptive_control.";
            vector_t Gamma{4}, bounds{3}, adaptive_law{3}, a_hat{4}, a_hat_dot{4};
            scalar_t Kd = 0.0, alpha = 0.1, lambda = 1.0;
            matrix_t Y;
            PrimalSolution primalSolution_;
            scalar_t dt;
            std::unique_ptr<LowPassFilter<vector_t>> lpf;
            
        };

    } // namespace object_manipulation
} // namespace ocs2
