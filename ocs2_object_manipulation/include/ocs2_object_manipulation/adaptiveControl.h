
#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2
{
    namespace object_manipulation
    {

        class AdaptiveControl : public SolverSynchronizedModule
        {
        public:
            AdaptiveControl(scalar_t mpcDesiredFrequency) : dt(1 / mpcDesiredFrequency)
            {
                Gamma.resize(4);
                Gamma << 1, 1, 1, 1;
                Gamma = Gamma ;
                a_hat.resize(4);
                a_hat_dot.resize(4);
                a_hat.setZero();
                a_hat_dot.setZero();

                Y.resize(3, 4);
                Y_H.resize(3, 4);
                Y_C.resize(3, 4);
                Y.setZero();
                Y_H.setZero();
                Y_C.setZero();
                primalSolution_.stateTrajectory_.push_back(vector_t::Zero(6));
                primalSolution_.stateTrajectory_.push_back(vector_t::Zero(6));

                adaptive_law.setZero();
            };

            vector_t update(vector_t currentObservation, vector_t nextObservation, vector_t ddq_d)
            {
                vector_t q = currentObservation.head(3);
                vector_t dq = currentObservation.tail(3);
                vector_t q_d = nextObservation.head(3);
                vector_t dq_d = nextObservation.tail(3);

                vector_t q_e = q - q_d;
                vector_t dq_e = dq - dq_d;

                scalar_t lambda = 1.0;

                vector_t s = dq_e + lambda * q_e;
                vector_t dq_r = dq - s;
                vector_t ddq_r = ddq_d - lambda * (dq - dq_d);

                a_hat_dot = Gamma.asDiagonal() * Y.transpose() * s;
                a_hat -= dt * a_hat_dot; 

                Y_H << ddq_r(0), 0, -sin(q(2)) * ddq_r(2), cos(q(2)) * ddq_r(2),
                    ddq_r(1), 0, -cos(q(2)) * ddq_r(2), -sin(q(2)) * ddq_r(2),
                    0, ddq_r(2), -sin(q(2)) * ddq_r(0) - cos(q(2)) * ddq_r(1), cos(q(2)) * ddq_r(0) - sin(q(2)) * ddq_r(1);

                Y_C << 0, 0, dq(2) * dq_r(2), 0,
                    0, 0, 0, dq(2) * dq_r(2),
                    0, 0, 0, 0;

                Y = Y_H + Y_C;

                return Y * a_hat;
            };

            void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t &currentState,
                              const ReferenceManagerInterface &referenceManager) override
            {
                adaptive_law = update(currentState.head(6), primalSolution_.stateTrajectory_[1], vector_t::Zero(3));
                saturate(adaptive_law, -20, 20);
                std::cout << "Adaptive control law: " << adaptive_law.transpose() << "\n";
                // std::cout << "Target: " << primalSolution_.stateTrajectory_[1].transpose() << "\n";
                // std::cout << "Current: " << currentState.head(6).transpose() << "\n";
            }

            void postSolverRun(const PrimalSolution &primalSolution) override
            {
                primalSolution_ = primalSolution;
            };

            vector_t getAdaptiveLaw() const
            {
                return adaptive_law;
            };

            
            void saturate(Eigen::Vector3d& vec, double min_val, double max_val)
            {
                for (int i = 0; i < vec.size(); ++i)
                {
                    vec(i) = std::min(std::max(vec(i), min_val), max_val);
                }
            }

        private:
            vector_t Gamma;
            vector_t a_hat, a_hat_dot;
            matrix_t Y, Y_H, Y_C;
            Eigen::Vector3d adaptive_law;
            PrimalSolution primalSolution_;
            scalar_t dt;
        };

    } // namespace object_manipulation
} // namespace ocs2
