#include "ba_view.hpp"
#include "snavely_reprojection_error.hpp"
#include <ceres/ceres.h>
#include <iostream>

using namespace std;

bool SolveBA(BALProblem &bal_problem, BAHistory &history);

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "usage: bundle_adjustment_ceres bal_data.txt" << endl;
    return 1;
  }

  BALProblem bal_problem(argv[1]);
  if (bal_problem.num_cameras() <= 0 || bal_problem.num_points() <= 0 ||
      bal_problem.num_observations() <= 0) {
    std::cerr << "Expected a BAL dataset with cameras, landmarks and observations.\n";
    return 1;
  }
  bal_problem.Normalize();
  bal_problem.Perturb(0.1, 0.5, 0.5);
  bal_problem.WriteToPLYFile("initial.ply");
  BAHistory history(bal_problem);
  if (!SolveBA(bal_problem, history)) return 1;
  bal_problem.WriteToPLYFile("final.ply");

  std::cout << "Recorded " << history.frames.size() << " states. Reprojection RMS: "
            << history.frames.front().rms << " -> " << history.frames.back().rms << " px\n";
  BAView view(history, bal_problem);
  return view.show();
}

bool SolveBA(BALProblem &bal_problem, BAHistory &history) {
  const int point_block_size = bal_problem.point_block_size();
  const int camera_block_size = bal_problem.camera_block_size();
  double *points = bal_problem.mutable_points();
  double *cameras = bal_problem.mutable_cameras();

  // Observations is 2 * num_observations long array observations
  // [u_1, u_2, ... u_n], where each u_i is two dimensional, the x
  // and y position of the observation.
  const double *observations = bal_problem.observations();
  ceres::Problem problem;

  for (int i = 0; i < bal_problem.num_observations(); ++i) {
    ceres::CostFunction *cost_function;

    // Each Residual block takes a point and a camera as input
    // and outputs a 2 dimensional Residual
    cost_function = SnavelyReprojectionError::Create(observations[2 * i + 0],
                                                     observations[2 * i + 1]);

    // If enabled use Huber's loss function.
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    // Each observation corresponds to a pair of a camera and a point
    // which are identified by camera_index()[i] and point_index()[i]
    // respectively.
    double *camera =
        cameras + camera_block_size * bal_problem.camera_index()[i];
    double *point = points + point_block_size * bal_problem.point_index()[i];

    problem.AddResidualBlock(cost_function, loss_function, camera, point);
  }

  // show some information here ...
  std::cout << "bal problem file loaded..." << std::endl;
  std::cout << "bal problem have " << bal_problem.num_cameras()
            << " cameras and " << bal_problem.num_points() << " points. "
            << std::endl;
  std::cout << "Forming " << bal_problem.num_observations() << " observations. "
            << std::endl;

  std::cout << "Solving ceres BA ... " << endl;
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;
  options.callbacks.push_back(&history);
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
  if (!summary.IsSolutionUsable()) return false;
  history.finish(summary);
  return true;
}
