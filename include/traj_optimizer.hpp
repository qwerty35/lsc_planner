#ifndef LSC_PLANNER_TRAJ_OPTIMIZER_HPP
#define LSC_PLANNER_TRAJ_OPTIMIZER_HPP

#include <sp_const.hpp>
#include <polynomial.hpp>
#include <param.hpp>
#include <mission.hpp>
#include <collision_constraints.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// CPLEX
#include <ilcplex/ilocplex.h>

namespace DynamicPlanning {
    class TrajOptimizer {
    public:
        TrajOptimizer(const Param& param, const Mission& mission, const Eigen::MatrixXd& B);

        void solve(const Agent& agent, const CollisionConstraints& constraints);

        void updateParam(const Param& param);

        traj_t getTrajectory();

        double getQPcost() const;

    private:
        Param param;
        Mission mission;
        Eigen::MatrixXd Q_base, Aeq_base, deq, B;

        // trajectory optimization results
        traj_t trajectory;
        double current_qp_cost;

        // Frequently used constants
        int M, n, phi, dim;

        void buildConstraintMatrices(const Agent& agent);

        // Cost matrix Q
        void buildQBase();

        // Constraint matrix A_eq x > d_eq
        void buildAeqBase();

        void buildDeq(const Agent& agent);

        void populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                           const Agent& agent, const CollisionConstraints& constraints);

        int getTerminalSegments(const Agent& agent) const;
    };
}


#endif //LSC_PLANNER_TRAJ_OPTIMIZER_HPP
