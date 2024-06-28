#include <iostream>
#include <string>

#include "ocs2_object_manipulation/ObjectInterface.h"

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateInputSoftBoxConstraint.h>

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

      // adaptive control
      adaptiveControlPtr_.reset(new AdaptiveControl(mpcSettings_.mpcDesiredFrequency_, taskFile));

      // Problem settings
      problem_settings_.loadSettings(taskFile, "object_parameters", verbose);
      loadData::loadStdVector(taskFile, "yaw_init", problem_settings_.agents_init_yaw_, verbose);

      // Dynamics
      problem_.dynamicsPtr.reset(new ObjectSytemDynamics(problem_settings_, adaptiveControlPtr_, libraryFolder, verbose));

      // Rollout
      auto rolloutSettings = rollout::loadSettings(taskFile, "rollout", verbose);
      rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

      // Constraints

      // CBFs
      RelaxedBarrierPenalty::Config boundsConfig;
      loadData::loadCppDataType(taskFile, "cbf_penalty_config.mu", boundsConfig.mu);
      loadData::loadCppDataType(taskFile, "cbf_penalty_config.delta", boundsConfig.delta);

      scalar_t alpha;
      loadData::loadCppDataType(taskFile, "cbf_penalty_config.alpha", alpha);

      std::vector<std::pair<scalar_t, scalar_t>> obstacles_pose;
      scalar_array_t obstacles_radius;
      loadData::loadStdVectorOfPair(taskFile, "obstacles.pose", obstacles_pose, verbose);
      loadData::loadStdVector(taskFile, "obstacles.radius", obstacles_radius, verbose);

      // Obstacles
      obstaclesPtr_.reset(new Obstacles(obstacles_pose));

      problem_.stateSoftConstraintPtr->add("Obstacle_object_cbf",
                                           std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectCBFConstraint>(obstaclesPtr_, obstacles_radius, alpha),
                                                                                              std::make_unique<RelaxedBarrierPenalty>(boundsConfig))));

      // Box constraints
      StateInputSoftBoxConstraint::BoxConstraint boxConstraint;

      std::vector<StateInputSoftBoxConstraint::BoxConstraint> stateLimits;
      stateLimits.reserve(STATE_DIM);

      std::vector<StateInputSoftBoxConstraint::BoxConstraint> inputLimits;
      inputLimits.reserve(INPUT_DIM);
      for (int i = 0; i < 2; ++i)
      {
        boxConstraint.index = i;
        boxConstraint.lowerBound = 0;
        boxConstraint.upperBound = 70;
        boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 0.01)));
        inputLimits.push_back(boxConstraint);

        boxConstraint.index = i + 2;
        boxConstraint.lowerBound = -0.25;
        boxConstraint.upperBound = 0.25;
        boxConstraint.penaltyPtr.reset(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(0.1, 1e-3)));
        inputLimits.push_back(boxConstraint);
      }

      auto boxConstraints = std::make_unique<StateInputSoftBoxConstraint>(stateLimits, inputLimits);
      boxConstraints->initializeOffset(0.0, vector_t::Zero(STATE_DIM), vector_t::Zero(INPUT_DIM));

      problem_.softConstraintPtr->add("BoxConstraints", std::move(boxConstraints));

      // Initialization
      objectInitializerPtr_.reset(new DefaultInitializer(INPUT_DIM));
    }

  } // namespace object_manipulation
} // namespace ocs2
