#include "traj_optimizer.hpp"

namespace DynamicPlanning {
    TrajOptimizer::TrajOptimizer(const Param& _param, const Mission& _mission, const Eigen::MatrixXd& _B)
    : param(_param), mission(_mission), B(_B)
    {
        // Initialize trajectory param, offsets
        dim = param.world_dimension;
        M = param.M;
        n = param.n;
        phi = param.phi;
        if (param.N_constraint_segments < 0) {
            param.N_constraint_segments = M;
        }

        // Build constraint matrices
        buildQBase();
        buildAeqBase();
    }

    void TrajOptimizer::buildConstraintMatrices(const Agent& agent) {
        buildDeq(agent);
    }

    traj_t TrajOptimizer::solve(const Agent& agent, const CollisionConstraints& constraints, double& qp_cost){
        // Initialize trajectory
        traj_t trajectory;
        trajectory.resize(M);
        for(int m = 0; m < M; m++){
            trajectory[m].resize(n + 1);
        }
        qp_cost = SP_INFINITY;

        // Construct constraint matrix
        buildConstraintMatrices(agent);

        IloEnv env;
        IloCplex cplex(env);
        IloModel model(env);
        IloNumVarArray var(env);
        IloRangeArray con(env);

        // Set CPLEX parameters
        if(param.multisim_experiment){
            cplex.setParam(IloCplex::Param::Threads, 6); // Due to cmd_publisher_node
        }
        else{
            cplex.setParam(IloCplex::Param::Threads, 10);
        }

        // Set CPLEX algorithm
//        cplex.setParam(IloCplex::Param::TimeLimit, 0.02);
//        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::AutoAlg); //0.015211
//        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal); //0.0111339
        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Dual); //0.0101956
//        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Barrier); //0.0135995
//        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Network); //0.0121052

        // Initialize QP model
        populatebyrow(model, var, con, agent, constraints);
        cplex.extract(model);

        std::string QPmodel_path = param.package_path + "/log/QPmodel.lp";
        std::string conflict_path = param.package_path + "/log/conflict.lp";
        if (param.log) {
            cplex.exportModel(QPmodel_path.c_str());
        } else {
            cplex.setOut(env.getNullStream());
            cplex.setWarning(env.getNullStream());
        }

        // Solve QP
        int offset_dim, offset_seg;
        offset_dim = M * (n + 1);
        offset_seg = n + 1;
        try {
            IloBool success = cplex.solve();

            // Save the result at trajectory
            IloNumArray vals(env);
            cplex.getValues(vals, var);
            for (int m = 0; m < M; m++) {
                for (int i = 0; i < n + 1; i++) {
                    if (dim == 3) {
                        trajectory[m][i] = point_t(vals[0 * offset_dim + m * offset_seg + i],
                                                   vals[1 * offset_dim + m * offset_seg + i],
                                                   vals[2 * offset_dim + m * offset_seg + i]);
                    } else {
                        trajectory[m][i] = point_t(vals[0 * offset_dim + m * offset_seg + i],
                                                   vals[1 * offset_dim + m * offset_seg + i],
                                                   param.world_z_2d);
                    }
                }
            }
            qp_cost = cplex.getObjValue();
            env.end();
        }
        catch (IloException &e) {
            if(not param.log){
                cplex.exportModel(QPmodel_path.c_str());
            }

            if ( ( cplex.getStatus() == IloAlgorithm::Infeasible ) ||
                 ( cplex.getStatus() == IloAlgorithm::InfeasibleOrUnbounded ) ) {
                ROS_ERROR_STREAM("[TrajOptimizer] CPLEX No solution at mav "<< agent.id << ", starting Conflict refinement");
                IloConstraintArray infeas(env);
                IloNumArray preferences(env);

                infeas.add(con);
                for (IloInt i = 0; i < var.getSize(); i++) {
                    if ( var[i].getType() != IloNumVar::Bool ) {
                        infeas.add(IloBound(var[i], IloBound::Lower));
                        infeas.add(IloBound(var[i], IloBound::Upper));
                    }
                }

                for (IloInt i = 0; i<infeas.getSize(); i++) {
                    preferences.add(1.0);  // user may wish to assign unique preferences
                }

                if ( cplex.refineConflict(infeas, preferences) ) {
                    IloCplex::ConflictStatusArray conflict = cplex.getConflict(infeas);
                    env.getImpl()->useDetailedDisplay(IloTrue);
                    std::cout << "Conflict :" << std::endl;
                    for (IloInt i = 0; i < infeas.getSize(); i++) {
                        if ( conflict[i] == IloCplex::ConflictMember)
                            std::cout << "Proved  : c" << i << infeas[i] << std::endl;
                        if ( conflict[i] == IloCplex::ConflictPossibleMember)
                            std::cout << "Possible: c" << i << infeas[i] << std::endl;
                    }
                    cplex.writeConflict(conflict_path.c_str());
                }
                else{
                    ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Conflict could not be refined");
                }
            }
            else{
                ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Concert exception caught: " << e);
            }

            //TODO: find better exception
            throw PlanningReport::QPFAILED;
        }
        catch (...) {
            ROS_ERROR_STREAM("[TrajOptimizer] CPLEX Unknown exception caught at iteration ");
            if(not param.log){
                cplex.exportModel(QPmodel_path.c_str());
            }

            //TODO: find better exception
            throw PlanningReport::QPFAILED;
        }

        return trajectory;
    }

    void TrajOptimizer::updateParam(const Param& _param){
        param = _param;
    }

    // Cost matrix Q
    void TrajOptimizer::buildQBase() {
        Q_base = Eigen::MatrixXd::Zero(n + 1, n + 1);
        for(int k = phi; k > phi - param.phi_n; k--) {
            Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(n + 1, n + 1);
            for (int i = 0; i < n + 1; i++) {
                for (int j = 0; j < n + 1; j++) {
                    if (i + j - 2 * k + 1 > 0)
                        Z(i, j) =
                                (double) coef_derivative(i, k) * coef_derivative(j, k) / (i + j - 2 * k + 1);
                }
            }
            Z = B * Z * B.transpose();
            Z = Z * pow(param.dt, -2 * k + 1);
            Q_base += Z;
        }
    }

    void TrajOptimizer::buildAeqBase() {
        // Build A_0, A_T
        Eigen::MatrixXd A_0 = Eigen::MatrixXd::Zero(n + 1, n + 1);
        Eigen::MatrixXd A_T = Eigen::MatrixXd::Zero(n + 1, n + 1);
        if (phi == 3 && n == 5) {
            A_0 <<  1,   0,   0,   0,   0,   0,
                   -1,   1,   0,   0,   0,   0,
                    1,  -2,   1,   0,   0,   0,
                   -1,   3,  -3,   1,   0,   0,
                    1,  -4,   6,  -4,   1,   0,
                   -1,   5, -10,  10,  -5,   1;

            A_T << 0,   0,   0,   0,   0,   1,
                   0,   0,   0,   0,  -1,   1,
                   0,   0,   0,   1,  -2,   1,
                   0,   0,  -1,   3,  -3,   1,
                   0,   1,  -4,   6,  -4,   1,
                  -1,   5, -10,  10,  -5,   1;
        } else {
            //TODO: Compute A_0, A_T when n is not 5
            throw std::invalid_argument("[TrajOptimizer] Currently, only n=5 is available");
        }

        Eigen::MatrixXd A_waypoints, A_cont;
        Aeq_base = Eigen::MatrixXd::Zero(phi + (M - 1) * phi, M * (n + 1));
        A_waypoints = Eigen::MatrixXd::Zero(phi, M * (n + 1));

        // Build A_waypoints
        int nn = 1;
        for (int i = 0; i < phi; i++) {
            A_waypoints.block(i, 0, 1, n + 1) = pow(param.dt, -i) * nn * A_0.row(i);
            nn = nn * (n - i);
        }

        // Build A_cont
        A_cont = Eigen::MatrixXd::Zero((M - 1) * phi, M * (n + 1));
        for (int m = 1; m < M; m++) {
            nn = 1;
            for (int j = 0; j < phi; j++) {
                A_cont.block(phi * (m - 1) + j, (n + 1) * (m - 1), 1, n + 1) =
                        pow(param.dt, -j) * nn * A_T.row(j);
                A_cont.block(phi * (m - 1) + j, (n + 1) * m, 1, n + 1) =
                        -pow(param.dt, -j) * nn * A_0.row(j);
                nn = nn * (n - j);
            }
        }

        // Build Aeq_base
        Aeq_base << A_waypoints,
                    A_cont;
    }

    // Equality constraints condition vector deq
    void TrajOptimizer::buildDeq(const Agent& agent) {
        deq = Eigen::MatrixXd::Zero(phi + (M - 1) * phi, dim);
        Eigen::MatrixXd d_waypoints, d_cont;
        d_waypoints = Eigen::MatrixXd::Zero(phi, dim);
        d_waypoints(0, 0) = agent.current_state.position.x();
        d_waypoints(1, 0) = agent.current_state.velocity.x();
        d_waypoints(2, 0) = agent.current_state.acceleration.x();
        d_waypoints(0, 1) = agent.current_state.position.y();
        d_waypoints(1, 1) = agent.current_state.velocity.y();
        d_waypoints(2, 1) = agent.current_state.acceleration.y();
        if(dim == 3) {
            d_waypoints(0, 2) = agent.current_state.position.z();
            d_waypoints(1, 2) = agent.current_state.velocity.z();
            d_waypoints(2, 2) = agent.current_state.acceleration.z();
        }
        d_cont = Eigen::MatrixXd::Zero((M - 1) * phi, dim);

        // Build deq
        deq << d_waypoints,
               d_cont;
    }

    void TrajOptimizer::populatebyrow(IloModel model, IloNumVarArray x, IloRangeArray c,
                                      const Agent& agent, const CollisionConstraints& constraints) {
        int offset_slack, offset_dim, offset_seg;
        size_t N_obs;
        offset_slack = dim * M * (n + 1);
        offset_dim = M * (n + 1);
        offset_seg = n + 1;
        N_obs = constraints.getObsSize();
        std::set<int> obs_slack_indices = constraints.getSlackIndices();

        IloEnv env = model.getEnv();
        // Initialize control points variables
        std::string name;
        double lower_bound, upper_bound;
        for (int k = 0; k < dim; k++) {
            for (int m = 0; m < M; m++) {
                for (int i = 0; i < n + 1; i++) {
                    int row = k * offset_dim + m * offset_seg + i;
                    if (k == 0) {
                        name = "x_" + std::to_string(m) + "_" + std::to_string(i);
                        lower_bound = mission.world_min.x();
                        upper_bound = mission.world_max.x();
                    } else if (k == 1) {
                        name = "y_" + std::to_string(m) + "_" + std::to_string(i);
                        lower_bound = mission.world_min.y();
                        upper_bound = mission.world_max.y();
                    } else if (k == 2) {
                        name = "z_" + std::to_string(m) + "_" + std::to_string(i);
                        lower_bound = mission.world_min.z();
                        upper_bound = mission.world_max.z();
                    } else {
                        throw std::invalid_argument("[TrajOptimizer] Invalid output dimension, output_dim > 3");
                    }

                    if(m == 0 and i < 3){
                        x.add(IloNumVar(env, -IloInfinity, IloInfinity)); //Do not adjust constraint at the initial state
                    }
                    else{
                        x.add(IloNumVar(env, lower_bound, upper_bound));
                    }
                    x[row].setName(name.c_str());
                }
            }
        }

        // Initialize slack variables
        if(param.slack_mode == SlackMode::DYNAMICALLIMIT){
            //for compact LSC, use slack variables to relax dynamic feasibility condition
            for(int i = 0; i < 2; i++) {
                for (int m = 0; m < M; m++) {
                    x.add(IloNumVar(env, -IloInfinity, 0));
                    int row = offset_slack + M * i + m;
                    name = "epsilon_slack_" + std::to_string(i) + "_" + std::to_string(m);
                    x[row].setName(name.c_str());
                }
            }
        }
        else if(param.slack_mode == SlackMode::COLLISIONCONSTRAINT or !obs_slack_indices.empty()){
            for(int oi = 0; oi < N_obs; oi++) {
                for (int m = 0; m < M; m++) {
                    x.add(IloNumVar(env, -IloInfinity, 0));
                    int row = offset_slack + M * oi + m;
                    name = "epsilon_slack_" + std::to_string(oi) + "_" + std::to_string(m);
                    x[row].setName(name.c_str());
                }
            }
        }

        // Cost function - 1. jerk
        IloNumExpr cost(env);
        for (int k = 0; k < dim; k++) {
            for (int m = 0; m < M; m++) {
                for (int i = 0; i < n + 1; i++) {
                    int row = k * offset_dim + m * offset_seg + i;
                    for (int j = 0; j < n + 1; j++) {
                        int col = k * offset_dim + m * offset_seg + j;
                        if (Q_base(i, j) != 0 and param.control_input_weight != 0) {
                            cost += param.control_input_weight * Q_base(i, j) * x[row] * x[col];
                        }
                    }
                }
            }
        }
        // Cost function - 2. error to goal
        // 1. isNearGoal
//            if(!isNearGoal) {
//                terminal_weight =
//                        param.w / std::max((agent.desired_goal_position - current_position).norm(), 0.001);
//            }else {
//                terminal_weight = param.w;
//            }
        // 2. distance cost with clamping
//            terminal_weight = std::min(param.w / (agent.desired_goal_position - current_position).norm(), 10.0);
        // 3. simple cost
        int terminal_segments = getTerminalSegments(agent);
        double terminal_weight = param.terminal_weight;
        if (terminal_segments > M) {
            throw std::invalid_argument("[TrajOptimizer] terminal segments is larger than total segments");
        }

        for (int m = M - terminal_segments; m < M; m++) {
            cost += terminal_weight *
                    (x[0 * offset_dim + m * offset_seg + n] - agent.current_goal_position.x()) *
                    (x[0 * offset_dim + m * offset_seg + n] - agent.current_goal_position.x());
            cost += terminal_weight *
                    (x[1 * offset_dim + m * offset_seg + n] - agent.current_goal_position.y()) *
                    (x[1 * offset_dim + m * offset_seg + n] - agent.current_goal_position.y());
            if (dim == 3) {
                cost += terminal_weight *
                        (x[2 * offset_dim + m * offset_seg + n] - agent.current_goal_position.z()) *
                        (x[2 * offset_dim + m * offset_seg + n] - agent.current_goal_position.z());
            }
        }

        // Cost function - 3. slack variables
        if(param.slack_mode == SlackMode::DYNAMICALLIMIT) {
            for(int i = 0; i < 2; i++) {
                for (int m = 0; m < M; m++) {
                    cost += param.slack_collision_weight * ((double) (M - m) / M)
                            * x[offset_slack + M * i + m] * x[offset_slack + M * i + m];
                }
            }
        }
        else if(param.slack_mode == SlackMode::COLLISIONCONSTRAINT or !obs_slack_indices.empty()) {
            for(int oi = 0; oi < N_obs; oi++) {
                for (int m = 0; m < M; m++) {
                    cost += param.slack_collision_weight * ((double) (M - m) / M)
                            * x[offset_slack + M * oi + m] * x[offset_slack + M * oi + m];
                }
            }
        }
        model.add(IloMinimize(env, cost));

        // Equality Constraints
        for (int k = 0; k < dim; k++) {
            for (int i = 0; i < phi + (M - 1) * phi; i++) {
                IloNumExpr expr(env);
                for (int j = 0; j < offset_dim; j++) {
                    if (Aeq_base(i, j) != 0) {
                        expr += Aeq_base(i, j) * x[k * offset_dim + j];
                    }
                }
                c.add(expr == deq(i, k));
                expr.end();
            }
        }

        // Inequality Constraints
        // SFC
        if(param.world_use_octomap) {
            for (int m = 0; m < param.N_constraint_segments; m++) {
                std::vector<LSC> lscs = constraints.getSFC(m).convertToLSCs(param.world_dimension);
                for (const auto &lsc : lscs) {
                    for (int j = 0; j < n + 1; j++) {
                        if (m == 0 and j < phi) {
                            continue; // Do not adjust constraint at initial state
                        }

                        IloNumExpr expr(env);
                        expr += lsc.normal_vector.x() *
                                (x[0 * offset_dim + m * offset_seg + j] - lsc.obs_control_point.x());
                        expr += lsc.normal_vector.y() *
                                (x[1 * offset_dim + m * offset_seg + j] - lsc.obs_control_point.y());
                        if (dim == 3) {
                            expr += lsc.normal_vector.z() *
                                    (x[2 * offset_dim + m * offset_seg + j] - lsc.obs_control_point.z());
                        }
                        expr += -lsc.d;

                        c.add(expr >= 0);
                        expr.end();
                    }
                }
            }
        }

        // LSC or BVC
        for (int oi = 0; oi < N_obs; oi++) {
            for (int m = 0; m < param.N_constraint_segments; m++) {
                for (int i = 0; i < n + 1; i++) {
                    if (m == 0 and i < phi) {
                        continue; // Do not adjust constraint at initial state
                    }

                    LSC rsfc = constraints.getLSC(oi, m, i);
                    IloNumExpr expr(env);
                    expr += rsfc.normal_vector.x() *
                            (x[0 * offset_dim + m * offset_seg + i] - rsfc.obs_control_point.x());
                    expr += rsfc.normal_vector.y() *
                            (x[1 * offset_dim + m * offset_seg + i] - rsfc.obs_control_point.y());
                    if (dim == 3) {
                        expr += rsfc.normal_vector.z() *
                                (x[2 * offset_dim + m * offset_seg + i] - rsfc.obs_control_point.z());
                    }

                    if (param.slack_mode == SlackMode::COLLISIONCONSTRAINT or
                        obs_slack_indices.find(oi) != obs_slack_indices.end()) {
                        expr += -(rsfc.d + x[offset_slack + M * oi + m]);
                    } else {
                        expr += -rsfc.d;
                    }

                    c.add(expr >= 0);
                    expr.end();
                }
            }
        }

        // Dynamic feasibility
        for(int k = 0; k < dim; k++) {
            for (int m = 0; m < M; m++) {
                // Maximum velocity
                for (int i = 0; i < n; i++) {
                    if (m == 0 and (i == 0 or i == 1)) {
                        continue; //Do not adjust constraint at the initial state
                    }
                    else if(param.slack_mode == SlackMode::DYNAMICALLIMIT) {
                        c.add(pow(param.dt, -1) * n *
                              (x[k * offset_dim + m * offset_seg + i + 1] - x[k * offset_dim + m * offset_seg + i]) +
                              x[offset_slack + m] <= agent.max_vel[k]);
                        c.add(-pow(param.dt, -1) * n *
                              (x[k * offset_dim + m * offset_seg + i + 1] - x[k * offset_dim + m * offset_seg + i]) +
                              x[offset_slack + m] <= agent.max_vel[k]);
                    }
                    else {
                        c.add(pow(param.dt, -1) * n *
                              (x[k * offset_dim + m * offset_seg + i + 1] - x[k * offset_dim + m * offset_seg + i]) <=
                              agent.max_vel[k]);
                        c.add(-pow(param.dt, -1) * n *
                              (x[k * offset_dim + m * offset_seg + i + 1] - x[k * offset_dim + m * offset_seg + i]) <=
                              agent.max_vel[k]);
                    }
                }
                // Maximum acceleration
                for (int i = 0; i < n - 1; i++) {
                    if(m == 0 and i == 0){
                        continue; //Do not adjust constraint at initial state
                    }
                    else if(param.slack_mode == SlackMode::DYNAMICALLIMIT) {
                        c.add(pow(param.dt, -2) * n * (n - 1) *
                              (x[k * offset_dim + m * offset_seg + i + 2] -
                               2 * x[k * offset_dim + m * offset_seg + i + 1] +
                               x[k * offset_dim + m * offset_seg + i]) +
                               x[offset_slack + M + m] <=
                              agent.max_acc[k]);
                        c.add(-pow(param.dt, -2) * n * (n - 1) *
                              (x[k * offset_dim + m * offset_seg + i + 2] -
                               2 * x[k * offset_dim + m * offset_seg + i + 1] +
                               x[k * offset_dim + m * offset_seg + i]) +
                               x[offset_slack + M + m] <=
                              agent.max_acc[k]);
                    } else {
                        c.add(pow(param.dt, -2) * n * (n - 1) *
                              (x[k * offset_dim + m * offset_seg + i + 2] -
                               2 * x[k * offset_dim + m * offset_seg + i + 1] +
                               x[k * offset_dim + m * offset_seg + i]) <=
                              agent.max_acc[k]);
                        c.add(-pow(param.dt, -2) * n * (n - 1) *
                              (x[k * offset_dim + m * offset_seg + i + 2] -
                               2 * x[k * offset_dim + m * offset_seg + i + 1] +
                               x[k * offset_dim + m * offset_seg + i]) <=
                               agent.max_acc[k]);
                    }
                }
            }
        }

        // Additional constraints for feasible LSC
        // stop at time horizon
        if(param.planner_mode == PlannerMode::LSC){
            for (int k = 0; k < dim; k++) {
                int m = M - 1;
                for(int i = 1; i < phi; i++){
                    c.add(x[k * offset_dim + m * offset_seg + n] - x[k * offset_dim + m * offset_seg + n - i] == 0);
                }
            }
        }

        model.add(c);
    }

    int TrajOptimizer::getTerminalSegments(const Agent& agent) const{
        int terminal_segments;
        double ideal_flight_time =
                (agent.current_goal_position - agent.current_state.position).norm() / agent.nominal_velocity;
        terminal_segments = std::max(static_cast<int>((M * param.dt - ideal_flight_time + SP_EPSILON) / param.dt),
                                     1); //increase terminal segments near goal point
        return terminal_segments;
    }

}