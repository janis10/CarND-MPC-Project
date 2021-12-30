#ifndef MPC_H
#define MPC_H

#include "Eigen/Core"
#include <vector>

class MPC {
  public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    std::vector<double> Solve(const Eigen::VectorXd &state,
                              const Eigen::VectorXd &coeffs);
};

#endif // MPC_H
