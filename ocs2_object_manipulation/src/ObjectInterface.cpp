#include <iostream>
#include <string>

#include "ocs2_object_manipulation/ObjectInterface.h"
#include "ocs2_object_manipulation/dynamics/ObjectSystemDynamics.h"

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_object_manipulation/SimpleObstacle.h>
#include <ocs2_object_manipulation/CBFConstraint.h>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2
{
  namespace object_manipulation
  {

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ObjectInterface::ObjectInterface(const std::string &taskFile, const std::string &libraryFolder, bool verbose)
    {
      // check that task file exists
      boost::filesystem::path taskFilePath(taskFile);
      if (boost::filesystem::exists(taskFilePath))
      {
        std::cerr << "[ObjectInterface] Loading task file: " << taskFilePath << "\n";
      }
      else
      {
        throw std::invalid_argument("[ObjectInterface] Task file not found: " + taskFilePath.string());
      }
      // create library folder if it does not exist
      boost::filesystem::path libraryFolderPath(libraryFolder);
      boost::filesystem::create_directories(libraryFolderPath);
      std::cerr << "[ObjectInterface] Generated library path: " << libraryFolderPath << "\n";

      // Default initial condition
      loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
      loadData::loadEigenMatrix(taskFile, "x_final", xFinal_);
      if (verbose)
      {
        std::cerr << "x_init:   " << initialState_.transpose() << "\n";
        std::cerr << "x_final:  " << xFinal_.transpose() << "\n";
      }

      // DDP-MPC settings
      ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
      mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);

      /*
       * ReferenceManager & SolverSynchronizedModule
       */
      referenceManagerPtr_.reset(new ReferenceManager);

      /*
       * Optimal control problem
       */
      // Cost
      matrix_t Q(STATE_DIM, STATE_DIM);
      matrix_t R(INPUT_DIM, INPUT_DIM);
      matrix_t Qf(STATE_DIM, STATE_DIM);
      loadData::loadEigenMatrix(taskFile, "Q", Q);
      loadData::loadEigenMatrix(taskFile, "R", R);
      loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
      if (verbose)
      {
        std::cerr << "Q:  \n"
                  << Q << "\n";
        std::cerr << "R:  \n"
                  << R << "\n";
        std::cerr << "Q_final:\n"
                  << Qf << "\n";
      }

      problem_.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
      problem_.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Qf));

      // Dynamics
      ObjectParameters objectParameters;
      objectParameters.loadSettings(taskFile, "object_parameters", verbose);
      problem_.dynamicsPtr.reset(new ObjectSytemDynamics(objectParameters, libraryFolder, verbose));

      // Rollout
      auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
      rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

      // Constraints

      // Velocity constraint
      const vector_t e = (vector_t(4) << 1, 1, 1, 1).finished();
      const matrix_t C = (matrix_t(4, STATE_DIM) << 0, 0, 0, 1, 0, 0,
                          0, 0, 0, -1, 0, 0,
                          0, 0, 0, 0, 1, 0,
                          0, 0, 0, 0, -1, 0)
                             .finished();

      std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));
      // problem_.stateSoftConstraintPtr->add("VelocityConstraint", std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<LinearStateConstraint>(e, C), std::move(penalty))));

      // CBFs
      vector_t obstaclePos(3);
      obstaclePos << -3.5, -1.5, 0.75;
      obstacles_.push_back(obstaclePos);

      for (int i = 0; i < obstacles_.size(); ++i)
      {
        problem_.stateSoftConstraintPtr->add("Obstacle" + std::to_string(i),
                                             std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<CBF_Constraint>(obstacles_[i]),
                                                                                                std::move(penalty))));
      }

      // Initialization
      objectInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
    }

  } // namespace object_manipulation
} // namespace ocs2
