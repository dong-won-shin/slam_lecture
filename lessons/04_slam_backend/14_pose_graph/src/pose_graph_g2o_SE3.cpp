#include <fstream>
#include <iostream>
#include <unordered_map>

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include "pose_graph_view.hpp"

G2O_USE_TYPE_GROUP(slam3d);

using namespace std;

// Capture the estimates after each real LM iteration without restarting the solver.
class PoseGraphHistory : public g2o::HyperGraphAction {
 public:
  PoseGraphHistory(g2o::SparseOptimizer &optimizer, const vector<g2o::VertexSE3 *> &poses)
      : optimizer_(optimizer), poses_(poses) {}

  void capture() {
    optimizer_.computeActiveErrors();
    PoseGraphState state;
    state.chi2 = optimizer_.activeChi2();
    state.positions.reserve(poses_.size());
    for (const auto *pose : poses_) {
      const auto position = pose->estimate().translation();
      state.positions.emplace_back(position.x(), position.y(), position.z());
    }
    states.push_back(std::move(state));
  }

  HyperGraphAction *operator()(const g2o::HyperGraph *, Parameters *) override {
    capture();
    return this;
  }

  vector<PoseGraphState> states;

 private:
  g2o::SparseOptimizer &optimizer_;
  const vector<g2o::VertexSE3 *> &poses_;
};

int main(int argc, char **argv) {
  if (argc != 2) {
    cout << "Usage: pose_graph_g2o_SE3 sphere.g2o" << endl;
    return 1;
  }
  ifstream fin(argv[1]);
  if (!fin) {
    cout << "file " << argv[1] << " does not exist." << endl;
    return 1;
  }

  typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
  typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>
      LinearSolverType;
  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(true);

  if (!optimizer.load(fin, false) || optimizer.vertices().empty() || optimizer.edges().empty()) {
    cerr << "Expected a graph with SE3 poses and constraints.\n";
    return 1;
  }
  vector<g2o::VertexSE3 *> poses;
  unordered_map<int, size_t> index;
  vector<size_t> anchors;
  for (const auto &entry : optimizer.vertices()) {
    auto *pose = dynamic_cast<g2o::VertexSE3 *>(entry.second);
    if (!pose) {
      cerr << "Expected VERTEX_SE3:QUAT vertices.\n";
      return 1;
    }
    index.emplace(pose->id(), poses.size());
    poses.push_back(pose);
    if (pose->fixed()) anchors.push_back(poses.size() - 1);
  }
  if (anchors.empty()) {
    auto first = min_element(poses.begin(), poses.end(), [](const g2o::VertexSE3 *a, const g2o::VertexSE3 *b) {
      return a->id() < b->id();
    });
    (*first)->setFixed(true);
    anchors.push_back(index.at((*first)->id()));
  }
  vector<PoseConstraint> edges;
  for (const auto *edge : optimizer.edges()) {
    if (!dynamic_cast<const g2o::EdgeSE3 *>(edge)) {
      cerr << "Expected EDGE_SE3:QUAT constraints.\n";
      return 1;
    }
    const int from = edge->vertex(0)->id(), to = edge->vertex(1)->id();
    edges.push_back({index.at(from), index.at(to), abs(static_cast<long long>(from) - to) == 1});
  }
  cout << "read total " << poses.size() << " vertices, " << edges.size() << " edges." << endl;
  if (!optimizer.initializeOptimization()) return 1;
  PoseGraphHistory history(optimizer, poses);
  history.capture();
  optimizer.addPostIterationAction(&history);
  cout << "optimizing ..." << endl;
  const int iterations = optimizer.optimize(30);
  optimizer.removePostIterationAction(&history);
  if (iterations <= 0) return 1;

  cout << "saving optimization results ..." << endl;
  if (!optimizer.save("result.g2o")) return 1;
  PoseGraphView view(history.states, edges, anchors);
  return view.show();
}
