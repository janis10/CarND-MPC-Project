#include "MPC.h"
#include "Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>

using CppAD::AD;
using Eigen::VectorXd;
using std::vector;

// Set global variables for MPC:
// Prediction horizon
int N = 10;
// Time-steps interval
double dt = 0.1;

const double Lf = 2.67;

// Number of states in model
const int numStates = 6;
// Number of inputs in model
const int numInputs = 2;

// Notice: variable 'vars' contains the variables for all N steps of the MPC and
// are arranged as follows:
// [x0 .. xN y0 .. yN psi0 .. psiN v0 .. vN cte0 .. cteN epsi0 .. epsiN
// delta0 .. deltaN a0 .. aN]
//
// Indices of x0, y0, psi0, v0, cte0, epsi0, delta0, a0 in vars for easy access:
vector<int> varsIdcs = {0, N, 2 * N, 3 * N, 4 * N, 5 * N, 6 * N, 7 * N - 1};
// vector<int> varsIdcs(6, 0);
// for (int i = 0; i < numStates; i++) {
//     varsIdcs[i] = i * N;
// }
// for (int i = 0; i < numInputs; i++) {
//     varsIdcs[i] = numStates * N + i * (N - 1);
// }

class FG_eval {
  public:
    // Fitted polynomial coefficients
    VectorXd coeffs;
    FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    void operator()(ADvector &fg, const ADvector &vars) {
        // State: [x y psi v cte epsi], Input: [delta a]
        // `fg` is a vector of the cost constraints, `vars` is a vector of
        // variable values (state & actuators)

        // Start defining the cost function:
        // J = J1 + J2 + J3
        // The first element of fg, i.e., fg[0], contains the cost
        // accumulated over time.
        fg[0] = 0;
        const double c1 = 4000.0, c2 = 600.0, c3 = 1.0;
        const double v_des = 100.0;
        // J1 = sum_{k=0}^{N}(c1*(cte[k])2 + c2*(epsi[k])2 + c3*(v[k]-v_des)2)
        for (int k = 0; k < N; k++) {
            fg[0] += c1 * CppAD::pow(vars[varsIdcs[4] + k], 2);
            fg[0] += c2 * CppAD::pow(vars[varsIdcs[5] + k], 2);
            fg[0] += c3 * CppAD::pow(vars[varsIdcs[3] + k] - v_des, 2);
        }
        // J2 = sum_{k=0}^{N-1}((delta[k])2 + (a[k])2)
        for (int k = 0; k < N - 1; k++) {
            fg[0] += CppAD::pow(vars[varsIdcs[6] + k], 2);
            fg[0] += CppAD::pow(vars[varsIdcs[7] + k], 2);
        }
        // J2 = sum_{k=1}^{N-1}((delta[k]-delta[k-1])2 + (a[k]-a[k-1])2)
        const double c4 = 500.0, c5 = 1.0;
        for (int k = 0; k < N - 2; k++) {
            fg[0] +=
                c4 * CppAD::pow(
                         vars[varsIdcs[6] + k + 1] - vars[varsIdcs[6] + k], 2);
            fg[0] +=
                c5 * CppAD::pow(
                         vars[varsIdcs[7] + k + 1] - vars[varsIdcs[7] + k], 2);
        }

        // State propagation constraints:
        // state[k] = f(state[k-1], input[k-1])
        // We define as follows:
        //
        // fg[1+varsIdcs[0]] = x0
        // fg[1+varsIdcs[0]+1] = x1 - f(x0,input[0])
        // ...
        // fg[1+varsIdcs[0]+N-1] = xΝ-1 - f(xN-2,input[N-2])
        // fg[1+varsIdcs[1]] = y0
        // ...
        // and so on and so forth.
        // Then in the next function we set that these constraints should be
        // zero.

        // Initial state:
        for (int i = 0; i < numStates; i++) {
            fg[1 + varsIdcs[i]] = vars[varsIdcs[i]];
        }
        // Here we propagate the state:
        // nextState = f(currState, currInput)
        for (int k = 1; k < N; k++) {
            vector<AD<double>> nextState(numStates);
            for (int i = 0; i < numStates; i++) {
                nextState[i] = vars[varsIdcs[i] + k];
            }
            vector<AD<double>> currState(numStates);
            for (int i = 0; i < numStates; i++) {
                currState[i] = vars[varsIdcs[i] + k - 1];
            }
            vector<AD<double>> currInput(numStates);
            for (int i = 0; i < numInputs; i++) {
                currInput[i] = vars[varsIdcs[numStates + i] + k - 1];
            }

            // First some values we need:
            AD<double> f0 = coeffs[3] * CppAD::pow(currState[0], 3) +
                            coeffs[2] * CppAD::pow(currState[0], 2) +
                            coeffs[1] * currState[0] + coeffs[0];
            AD<double> psi_des =
                CppAD::atan(coeffs[1] + (2 * coeffs[2] * currState[0]) +
                            (3 * coeffs[3] * CppAD::pow(currState[0], 2)));

            // Now construct the constraints for each state based on the control
            // system.
            // x[k+1] - (x[k] + v[k] * cos(psi[k]) * dt)
            fg[1 + varsIdcs[0] + k] =
                nextState[0] -
                (currState[0] + currState[3] * CppAD::cos(currState[2]) * dt);
            // y[k+1] - (y[k] + v[k] * sin(psi[k]) * dt)
            fg[1 + varsIdcs[1] + k] =
                nextState[1] -
                (currState[1] + currState[3] * CppAD::sin(currState[2]) * dt);
            // psi[k+1] - (psi[k] - v[k] * delta[k] / Lf * dt);
            fg[1 + varsIdcs[2] + k] =
                nextState[2] -
                (currState[2] - currState[3] * currInput[0] / Lf * dt);
            // v[k+1] - (v[k] + a[k] * dt);
            fg[1 + varsIdcs[3] + k] =
                nextState[3] - (currState[3] + currInput[1] * dt);
            // cte[k+1] - ((f0 - y[k]) + (v[k] * sin(epsi[k]) * dt));
            fg[1 + varsIdcs[4] + k] =
                nextState[4] - ((f0 - currState[1]) +
                                (currState[3] * CppAD::sin(currState[5]) * dt));
            // epsi[k+1] - ((psi[k] - psi_des[k]) - v[k] * delta[k] / Lf * dt);
            fg[1 + varsIdcs[5] + k] =
                nextState[5] - ((currState[2] - psi_des) -
                                currState[3] * currInput[0] / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    /* Set up variables */
    // Number of total variables = horizon * states + (horizon-1) * inputs
    size_t n_vars = N * numStates + (N - 1) * numInputs;
    Dvector vars(n_vars);
    // It is initialized to 0, except for the initial state of MPC ...
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }
    // ... which is:
    for (int i = 0; i < numStates; i++) {
        vars[varsIdcs[i]] = state[i];
    }
    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // states are basically unbounded:
    for (int i = 0; i < varsIdcs[6]; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }
    // delta \in [-25 deg, + 25 deg] (but here is in rad)
    for (int i = varsIdcs[6]; i < varsIdcs[7]; i++) {
        vars_lowerbound[i] = -0.436332 * Lf;
        vars_upperbound[i] = 0.436332 * Lf;
    }
    // acceleration \in [-1.0, 1.0]
    for (int i = varsIdcs[7]; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    /* Set up constraints */
    // Number of constraints
    size_t n_constraints = N * numStates;

    // Lower and upper limits for the state transition constraints
    // Constraints are state[k+1] - f(state[k], input[k]) = 0.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    // Now for the initial state: state[0] = state0
    for (int i = 0; i < numStates; i++) {
        constraints_lowerbound[varsIdcs[i]] = state[i];
        constraints_upperbound[varsIdcs[i]] = state[i];
    }

    /**********************************************************************/
    /** The code below is standard from Udacity's original repo **/

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    // NOTE: You don't have to worry about these options
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    //   of sparse routines, this makes the computation MUCH FASTER. If you can
    //   uncomment 1 of these and see if it makes a difference or not but if you
    //   uncomment both the computation time should go up in orders of
    //   magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    /** Until here **/
    /**********************************************************************/

    // Print cost
    auto cost = solution.obj_value;
    std::cout << "Cost: " << cost << std::endl;

    // Return next input
    vector<double> mpcOut(2 + 2 * (N - 1), 0);
    mpcOut[0] = solution.x[varsIdcs[6]];
    mpcOut[1] = solution.x[varsIdcs[7]];
    // Return predicted trajectory (position)
    int k = 0;
    for (int i = 0; i < N - 1; i++) {
        mpcOut[2 + k] = solution.x[varsIdcs[0] + i + 1];
        mpcOut[2 + k + 1] = solution.x[varsIdcs[1] + i + 1];
        k += 2;
    }

    return mpcOut;
}