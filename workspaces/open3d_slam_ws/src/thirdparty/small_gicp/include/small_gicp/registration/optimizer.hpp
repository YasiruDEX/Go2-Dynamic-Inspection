// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
#pragma once

#include <iostream>
#include <deque>
#include <small_gicp/util/lie.hpp>
#include <small_gicp/registration/registration_result.hpp>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
namespace small_gicp {

  struct SolutionRemappingHandler
  {
      // Configuration & status
      bool   enabled        = false;           ///< true ➜ `apply()` will project
      double threshold      = 300.0;           ///< σ < threshold = degeneracy
      bool   has_degenerate = false;           ///< at least one axis is bad
  
      // Outputs
      Eigen::Matrix<double, 6, 6> projection =
          Eigen::Matrix<double, 6, 6>::Identity();         ///< P = [ I  0; 0  P₃ ]
      std::vector<int> degenerate_indices;                 ///< bad DOFs (3..5)
  
      void compute(const Eigen::Matrix<double, 6, 6>& H)
      {
          enabled          = false;
          has_degenerate   = false;
          degenerate_indices.clear();
          projection.setIdentity();                    // reset each call
  
          // ---- extract the 3×3 translational block -------------------------
          const Eigen::Matrix<double, 3, 3> H3 = H.bottomRightCorner(3,3);
  
          // ---- eigendecomposition of the 3×3 block -------------------------
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> solver(H3);
          if (solver.info() != Eigen::Success) {
              std::cerr << "SolutionRemappingHandler: eigensolver failed (3×3 block)\n";
              return;
          }
          const auto& eigvals3 = solver.eigenvalues();    // ascending order
          const auto& eigvecs3 = solver.eigenvectors();   // 3×3, columns = e-vectors
  
          // ---- identify *all* degenerate translation DOFs ------------------
          Eigen::Matrix<double, 3, 3> eigvecs3_copy = eigvecs3;
          for (int i = 0; i < 3; ++i) {
              if (eigvals3(i) < threshold) {
                  eigvecs3_copy.col(i).setZero();
                  degenerate_indices.push_back(i + 3);    // map to 6-DOF index 3-5
                  has_degenerate = true;
              }
          }
  
          // ---- build 3×3 projection & embed it in 6×6 ----------------------
          const Eigen::Matrix<double, 3, 3> P3 = eigvecs3 * eigvecs3_copy.transpose();
          projection.bottomRightCorner(3,3) = P3;
          enabled  = has_degenerate;
  
          // ---- diagnostics -------------------------------------------------
          if (enabled) {
              std::ostringstream oss;
              oss << "DEGENERATE translational eigen-directions: ";
              for (std::size_t k = 0; k < degenerate_indices.size(); ++k) {
                  oss << degenerate_indices[k]
                      << (k + 1 < degenerate_indices.size() ? ", " : "");
              }
  
              std::cerr << "\033[1;35m"
                           "█████████████████████████████████████████████████████████████████████████████████████\n"
                        << "██  WARNING: " << oss.str()
                        << "  ██\n"
                           "█████████████████████████████████████████████████████████████████████████████████████\n"
                           "\033[0m";
          }
      }
  
      void apply(Eigen::Matrix<double, 6, 1>& delta) const
      {
          if (enabled) {
              delta = projection * delta;           // modifies only indices 3-5
          }
      }
  };

/// @brief GaussNewton optimizer
struct GaussNewtonOptimizer {
  GaussNewtonOptimizer() : verbose(false), max_iterations(20), lambda(1e-6) {}

  template <
    typename TargetPointCloud,
    typename SourcePointCloud,
    typename TargetTree,
    typename CorrespondenceRejector,
    typename TerminationCriteria,
    typename Reduction,
    typename Factor,
    typename GeneralFactor>
  RegistrationResult optimize(
    const TargetPointCloud& target,
    const SourcePointCloud& source,
    const TargetTree& target_tree,
    const CorrespondenceRejector& rejector,
    const TerminationCriteria& criteria,
    Reduction& reduction,
    const Eigen::Isometry3d& init_T,
    std::vector<Factor>& factors,
    GeneralFactor& general_factor) const {
    //
    if (verbose) {
      std::cout << "--- GN optimization ---" << std::endl;
    }

    RegistrationResult result(init_T);
    for (int i = 0; i < max_iterations && !result.converged; i++) {
      // Linearize
      auto [H, b, e] = reduction.linearize(target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(target, source, target_tree, result.T_target_source, &H, &b, &e);

      // Solve linear system
      const Eigen::Matrix<double, 6, 1> delta = (H + lambda * Eigen ::Matrix<double, 6, 6>::Identity()).ldlt().solve(-b);

      if (verbose) {
        std::cout << "iter=" << i << " e=" << e << " lambda=" << lambda << " dt=" << delta.tail<3>().norm() << " dr=" << delta.head<3>().norm() << std::endl;
      }

      result.converged = criteria.converged(delta);
      result.T_target_source = result.T_target_source * se3_exp(delta);
      result.iterations = i;
      result.H = H;
      result.b = b;
      result.error = e;
    }

    result.num_inliers = std::count_if(factors.begin(), factors.end(), [](const auto& factor) { return factor.inlier(); });

    return result;
  }

  bool verbose;        ///< If true, print debug messages
  int max_iterations;  ///< Max number of optimization iterations
  double lambda;       ///< Damping factor (Increasing this makes optimization slow but stable)
};

/// @brief LevenbergMarquardt optimizer
struct LevenbergMarquardtOptimizer {
  LevenbergMarquardtOptimizer() : verbose(false), max_iterations(20), max_inner_iterations(10), init_lambda(1e-3), lambda_factor(10.0) {}

  template <
    typename TargetPointCloud,
    typename SourcePointCloud,
    typename TargetTree,
    typename CorrespondenceRejector,
    typename TerminationCriteria,
    typename Reduction,
    typename Factor,
    typename GeneralFactor>
  RegistrationResult optimize(
    const TargetPointCloud& target,
    const SourcePointCloud& source,
    const TargetTree& target_tree,
    const CorrespondenceRejector& rejector,
    const TerminationCriteria& criteria,
    Reduction& reduction,
    const Eigen::Isometry3d& init_T,
    std::vector<Factor>& factors,
    GeneralFactor& general_factor) const {
    //
    if (verbose) {
      std::cout << "--- LM optimization ---" << std::endl;
    }

    double lambda = init_lambda;
    RegistrationResult result(init_T);
    for (int i = 0; i < max_iterations && !result.converged; i++) {
      // Linearize
      auto [H, b, e] = reduction.linearize(target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(target, source, target_tree, result.T_target_source, &H, &b, &e);

      // Lambda iteration
      bool success = false;
      for (int j = 0; j < max_inner_iterations; j++) {
        // Solve with damping
        const Eigen::Matrix<double, 6, 1> delta = (H + lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(-b);

        // Validate new solution
        const Eigen::Isometry3d new_T = result.T_target_source * se3_exp(delta);
        double new_e = reduction.error(target, source, new_T, factors);
        general_factor.update_error(target, source, new_T, &e);

        if (verbose) {
          std::cout << "iter=" << i << " inner=" << j << " e=" << e << " new_e=" << new_e << " lambda=" << lambda << " dt=" << delta.tail<3>().norm()
                    << " dr=" << delta.head<3>().norm() << std::endl;
        }

        if (new_e <= e) {
          // Error decreased, decrease lambda
          result.converged = criteria.converged(delta);
          result.T_target_source = new_T;
          lambda /= lambda_factor;
          success = true;
          e = new_e;

          break;
        } else {
          // Failed to decrease error, increase lambda
          lambda *= lambda_factor;
        }
      }

      result.iterations = i;
      result.H = H;
      result.b = b;
      result.error = e;

      if (!success) {
        break;
      }
    }

    result.num_inliers = std::count_if(factors.begin(), factors.end(), [](const auto& factor) { return factor.inlier(); });

    return result;
  }

  bool verbose;              ///< If true, print debug messages
  int max_iterations;        ///< Max number of optimization iterations
  int max_inner_iterations;  ///< Max number of inner iterations (lambda-trial)
  double init_lambda;        ///< Initial lambda (damping factor)
  double lambda_factor;      ///< Lambda increase factor
};

struct RobustLevenbergMarquardtOptimizer {
  RobustLevenbergMarquardtOptimizer()
      : verbose(false),
        max_iterations(20),
        max_inner_iterations(20),
        tau(1e-2) {}

  template <
      typename TargetPointCloud,
      typename SourcePointCloud,
      typename TargetTree,
      typename CorrespondenceRejector,
      typename TerminationCriteria,
      typename Reduction,
      typename Factor,
      typename GeneralFactor>
  RegistrationResult optimize(const TargetPointCloud& target,
                              const SourcePointCloud& source,
                              const TargetTree& target_tree,
                              const CorrespondenceRejector& rejector,
                              const TerminationCriteria& criteria,
                              Reduction& reduction,
                              const Eigen::Isometry3d& init_T,
                              std::vector<Factor>& factors,
                              GeneralFactor& general_factor) const {
    if (verbose) {
      std::cout << "--- LM optimization ---" << std::endl;
    }

    double v = 2.0;
    double lambda = 0.0;

    RegistrationResult result(init_T);
    // SolutionRemappingHandler remap_handler;
    double e = 0.0;

    for (int i = 0; i < max_iterations && !result.converged; ++i) {
      // Linearize system
      auto [H, b, e_init] = reduction.linearize(target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(target, source, target_tree, result.T_target_source, &H, &b, &e_init);
      e = e_init;
      lambda = tau * H.diagonal().maxCoeff();

      // remap_handler.compute(H);

      bool step_accepted = false;

      for (int j = 0; j < max_inner_iterations; ++j) {
        // Apply damping
        Eigen::Matrix<double, 6, 6> H_damped = H + lambda * Eigen::Matrix<double, 6, 6>::Identity();

        // Adaptive diagonal (Jacobi) preconditioner
        Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
        for (int k = 0; k < 6; ++k) {
          double d = std::max(H_damped(k, k), 1e-12);
          P(k, k) = 1.0 / std::sqrt(d);
        }

        // Preconditioned system
        Eigen::Matrix<double, 6, 6> A = P * H_damped * P;
        Eigen::Matrix<double, 6, 1> rhs = -P * b;

        // Solve A * delta_hat = rhs
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<double, 6, 1> delta_hat;
        {
          const double tol = 1e-12;
          Eigen::Matrix<double, 6, 6> S_inv = Eigen::Matrix<double, 6, 6>::Zero();
          const auto& sing = svd.singularValues();
          for (int k = 0; k < 6; ++k)
            if (sing[k] > tol) S_inv(k, k) = 1.0 / sing[k];

          delta_hat = svd.matrixV() * S_inv * svd.matrixU().transpose() * rhs;
        }

        Eigen::Matrix<double, 6, 1> delta = P * delta_hat;
        if (!delta.allFinite()) {
          if (verbose) std::cerr << "[LM] Invalid delta (NaNs)\n";
          break;
        }
        // remap_handler.apply(delta);

        const Eigen::Isometry3d new_T = result.T_target_source * se3_exp(delta);
        double new_e = reduction.error(target, source, new_T, factors);
        general_factor.update_error(target, source, new_T, &new_e);

        // Gain ratio
        double expected = -delta.dot(b) - 0.5 * delta.dot(H * delta);
        double rho = (e - new_e) / std::max(expected, 1e-15);

        if (verbose) {
          double cond_H = svd.singularValues()(0) / std::max(svd.singularValues()(5), 1e-12);
          std::cout << "iter=" << i
                    << " inner=" << j
                    << " e=" << e
                    << " new_e=" << new_e
                    << " ρ=" << rho
                    << " λ=" << lambda
                    << " cond=" << cond_H
                    << " dt=" << delta.tail<3>().norm()
                    << " dr=" << delta.head<3>().norm()
                    << std::endl;
        }

        if (rho > 0 && std::isfinite(new_e) && std::isfinite(delta.norm())) {
          result.T_target_source = new_T;
          e = new_e;
          lambda = lambda * std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3.0));
          lambda = std::max(lambda, 1e-9); // Avoid too small lambda
          step_accepted = true;
          result.converged = criteria.converged(delta) || std::abs(expected) < 1e-12;
          // Reset v for future additive updates
          v = 2.0;
          break;
        } else {
          lambda = lambda + v;    // Additive increase (Nielsen’s method)
          v = v * 2.0;            // Exponential growth to escape poor regions
        }

      }

      result.iterations = i + step_accepted;
      result.H = H;
      result.b = b;
      result.error = e;

      if (!step_accepted) {
        if (verbose) std::cerr << "[LM] Step rejected, terminating\n";
        break;
      }
    }

    result.num_inliers = static_cast<int>(
        std::count_if(factors.begin(), factors.end(),
                      [](const auto& f) { return f.inlier(); }));

    return result;
  }

  bool verbose;
  int max_iterations;
  int max_inner_iterations;
  double  tau;
};

struct NonstandardLevenbergMarquardtOptimizer {
  NonstandardLevenbergMarquardtOptimizer()
      : verbose(false),
        max_iterations(20),
        max_inner_iterations(20),
        tau(1e-2),
        history_size(5) {}

  template <
      typename TargetPointCloud,
      typename SourcePointCloud,
      typename TargetTree,
      typename CorrespondenceRejector,
      typename TerminationCriteria,
      typename Reduction,
      typename Factor,
      typename GeneralFactor>
  RegistrationResult optimize(const TargetPointCloud& target,
                              const SourcePointCloud& source,
                              const TargetTree& target_tree,
                              const CorrespondenceRejector& rejector,
                              const TerminationCriteria& criteria,
                              Reduction& reduction,
                              const Eigen::Isometry3d& init_T,
                              std::vector<Factor>& factors,
                              GeneralFactor& general_factor) const {
    if (verbose) {
      std::cout << "--- nonstandard LM optimization ---" << std::endl;
    }

    double v = 2.0;
    double lambda = 0.0;

    RegistrationResult result(init_T);
    double e = 0.0;

    // Non-monotonic acceptance: keep residual history
    std::deque<double> residual_history(history_size, std::numeric_limits<double>::max());

    for (int i = 0; i < max_iterations && !result.converged; ++i) {
      auto [H, b, e_init] = reduction.linearize(target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(target, source, target_tree, result.T_target_source, &H, &b, &e_init);
      e = e_init;
      lambda = tau * H.diagonal().maxCoeff();

      bool step_accepted = false;

      for (int j = 0; j < max_inner_iterations; ++j) {
        Eigen::Matrix<double, 6, 6> H_damped = H + lambda * Eigen::Matrix<double, 6, 6>::Identity();

        // Jacobi preconditioner
        Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
        for (int k = 0; k < 6; ++k) {
          double d = std::max(H_damped(k, k), 1e-12);
          P(k, k) = 1.0 / std::sqrt(d);
        }

        Eigen::Matrix<double, 6, 6> A = P * H_damped * P;
        Eigen::Matrix<double, 6, 1> rhs = -P * b;

        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix<double, 6, 1> delta_hat;
        {
          const double tol = 1e-12;
          Eigen::Matrix<double, 6, 6> S_inv = Eigen::Matrix<double, 6, 6>::Zero();
          const auto& sing = svd.singularValues();
          for (int k = 0; k < 6; ++k)
            if (sing[k] > tol) S_inv(k, k) = 1.0 / sing[k];
          delta_hat = svd.matrixV() * S_inv * svd.matrixU().transpose() * rhs;
        }

        Eigen::Matrix<double, 6, 1> delta = P * delta_hat;
        if (!delta.allFinite()) {
          if (verbose) std::cerr << "[nonstandardLM] Invalid delta (NaNs)\n";
          break;
        }

        const Eigen::Isometry3d new_T = result.T_target_source * se3_exp(delta);
        double new_e = reduction.error(target, source, new_T, factors);
        general_factor.update_error(target, source, new_T, &new_e);

        // Gain ratio
        double expected = -delta.dot(b) - 0.5 * delta.dot(H * delta);
        double rho = (e - new_e) / std::max(expected, 1e-15);

        if (verbose) {
          double cond_H = svd.singularValues()(0) / std::max(svd.singularValues()(5), 1e-12);
          std::cout << "iter=" << i
                    << " inner=" << j
                    << " e=" << e
                    << " new_e=" << new_e
                    << " ρ=" << rho
                    << " λ=" << lambda
                    << " cond=" << cond_H
                    << " dt=" << delta.tail<3>().norm()
                    << " dr=" << delta.head<3>().norm()
                    << std::endl;
        }

        // Non-monotonic step acceptance:
        bool accept_nonmonotonic = (new_e < *std::max_element(residual_history.begin(), residual_history.end()))
                                   && std::isfinite(new_e) && std::isfinite(delta.norm());

        if ((rho > 0 && std::isfinite(new_e) && std::isfinite(delta.norm())) || accept_nonmonotonic) {
          result.T_target_source = new_T;
          e = new_e;
          lambda = lambda * std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * rho - 1.0, 3.0));
          lambda = std::max(lambda, 1e-9);
          step_accepted = true;
          result.converged = criteria.converged(delta) || std::abs(expected) < 1e-12;
          v = 2.0;
          break;
        } else {
          lambda = lambda + v;
          v = v * 2.0;
        }
      }

      // Update residual history (always add last error)
      if (residual_history.size() >= history_size) residual_history.pop_front();
      residual_history.push_back(e);

      result.iterations = i + step_accepted;
      result.H = H;
      result.b = b;
      result.error = e;

      if (!step_accepted) {
        if (verbose) std::cerr << "[nonstandardLM] Step rejected, terminating\n";
        break;
      }
    }

    result.num_inliers = static_cast<int>(
        std::count_if(factors.begin(), factors.end(),
                      [](const auto& f) { return f.inlier(); }));

    return result;
  }

  bool verbose;
  int max_iterations;
  int max_inner_iterations;
  double tau;
  int history_size;
};

struct HouseholderSolver {
  HouseholderSolver()
      : verbose(false),
        max_iterations(30) {}

  template <
      typename TargetPointCloud,
      typename SourcePointCloud,
      typename TargetTree,
      typename CorrespondenceRejector,
      typename TerminationCriteria,
      typename Reduction,
      typename Factor,
      typename GeneralFactor>
  RegistrationResult optimize(const TargetPointCloud& target,
                              const SourcePointCloud& source,
                              const TargetTree& target_tree,
                              const CorrespondenceRejector& rejector,
                              const TerminationCriteria& criteria,
                              Reduction& reduction,
                              const Eigen::Isometry3d& init_T,
                              std::vector<Factor>& factors,
                              GeneralFactor& general_factor) const {
    if (verbose) std::cout << "--- Householder Full Newton optimisation ---\n";

    RegistrationResult result(init_T);

    for (int i = 0; i < max_iterations && !result.converged; ++i) {
      // Build full Newton equation
      auto [H, b, e_init] = reduction.linearize(
          target, source, target_tree, rejector, result.T_target_source, factors);

      // Solve H δ = –b
      Eigen::Matrix<double, 6, 1> delta = Eigen::LDLT<Eigen::Matrix<double, 6, 6>>(H).solve(-b);
      if (!delta.allFinite()) throw std::runtime_error("[HouseholderFullNewtonSolver] NaNs in δ");

      // Commit step
      result.T_target_source = result.T_target_source * se3_exp(delta);
      double new_e = reduction.error(target, source, result.T_target_source, factors);

      // Bookkeeping
      result.error      = new_e;
      result.H          = H;
      result.b          = b;
      result.iterations = i + 1;

      if (verbose) {
        std::cout << "iter=" << i
                  << "  e=" << e_init
                  << "  new_e=" << new_e
                  << "  |δr|=" << delta.head<3>().norm()
                  << "  |δt|=" << delta.tail<3>().norm()
                  << '\n';
      }

      result.converged = criteria.converged(delta);
    }

    result.num_inliers = static_cast<int>(std::count_if(
        factors.begin(), factors.end(), [](const auto& f) { return f.inlier(); }));

    return result;
  }

  bool verbose;
  int  max_iterations;
};

// struct HouseholderSolver {
//   HouseholderSolver()
//       : verbose(false),
//         max_iterations(30),
//         lambda_init(1e-3),
//         lambda_min(1e-10),
//         lambda_max(1e6),
//         lambda_inc(2.0),       // multiply λ when step is rejected
//         lambda_dec(0.5)        // multiply λ when step is very good
//   {}

//   // ---------------------------------------------------------------------------
//   template <
//       typename TargetPointCloud,
//       typename SourcePointCloud,
//       typename TargetTree,
//       typename CorrespondenceRejector,
//       typename TerminationCriteria,
//       typename Reduction,
//       typename Factor,
//       typename GeneralFactor>
//   RegistrationResult optimize(const TargetPointCloud&  target,
//                               const SourcePointCloud&  source,
//                               const TargetTree&        target_tree,
//                               const CorrespondenceRejector& rejector,
//                               const TerminationCriteria&    criteria,
//                               Reduction&                    reduction,
//                               const Eigen::Isometry3d&      init_T,
//                               std::vector<Factor>&          factors,
//                               GeneralFactor&                /*general_factor*/) const
//   {
//     if (verbose)
//       std::cout << "--- Householder Trust-Region (LM) optimisation ---\n";

//     RegistrationResult result(init_T);
//     double lambda = lambda_init;

//     for (int i = 0; i < max_iterations && !result.converged; ++i) {

//       // 1. linearise around current pose
//       auto [H, b, e_init] = reduction.linearize(
//           target, source, target_tree, rejector, result.T_target_source, factors);

//       // 2. damped system (Levenberg–Marquardt)
//       Eigen::Matrix<double, 6, 6> H_lm = H + lambda * Eigen::Matrix<double, 6, 6>::Identity();
//       Eigen::Matrix<double, 6, 1> delta =
//           Eigen::LDLT<Eigen::Matrix<double, 6, 6>>(H_lm).solve(-b);

//       if (!delta.allFinite())
//         throw std::runtime_error("[HouseholderSolver] NaNs in δ");

//       // 3. evaluate new energy
//       const Eigen::Isometry3d T_new = result.T_target_source * se3_exp(delta);
//       const double            new_e = reduction.error(target, source, T_new, factors);

//       // 4. predicted vs. actual reduction
//       const double pred_red = 0.5 * delta.dot(lambda * delta - b);        // *** FIX #1 ***
//       const double act_red  = e_init - new_e;                             // *** FIX #2 ***

//       const double rho = (pred_red > 0.0) ? (act_red / pred_red) : 0.0;   // *** FIX #3 ***

//       bool step_accepted;
//       if (rho > 0.75 && act_red > 0.0) {               // very good
//         lambda        = std::max(lambda * lambda_dec, lambda_min);
//         step_accepted = true;
//       } else if (rho < 0.25 || act_red < 0.0) {        // poor / uphill
//         lambda        = std::min(lambda * lambda_inc, lambda_max);
//         step_accepted = false;
//       } else {                                         // acceptable
//         step_accepted = true;
//       }

//       if (step_accepted) {
//         result.T_target_source = T_new;
//         result.error           = new_e;
//       }

//       // bookkeeping
//       result.H          = H;
//       result.b          = b;
//       result.iterations = i + 1;

//       if (verbose) {
//         std::cout << "iter=" << i
//                   << "  e="      << e_init
//                   << "  new_e="  << new_e
//                   << "  |δr|="   << delta.head<3>().norm()
//                   << "  |δt|="   << delta.tail<3>().norm()
//                   << "  λ="      << lambda
//                   << "  ρ="      << rho
//                   << (step_accepted ? "  [accepted]" : "  [rejected]")
//                   << '\n';
//       }

//       result.converged = criteria.converged(delta);
//     }

//     result.num_inliers = static_cast<int>(std::count_if(
//         factors.begin(), factors.end(), [](const auto& f) { return f.inlier(); }));

//     return result;
//   }

//   // ---------------------------------------------------------------------------
//   bool   verbose;
//   int    max_iterations;
//   double lambda_init, lambda_min, lambda_max;
//   double lambda_inc,  lambda_dec;
// };

}  // namespace small_gicp
