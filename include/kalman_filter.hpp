#pragma once
#include <dynamic_msgs/State.h>
#include <Eigen/Dense>
#include <sp_const.hpp>

namespace DynamicPlanning {
    class LinearKalmanFilter{
    public:
        void initialize(double _sigma_y_sq, double _sigma_v_sq, double _sigma_a_sq){
            n = 6; // the number of states
            m = 3; // the number of observed states
            phi = 2; // position, velocity
            sigma_y_sq = _sigma_y_sq;
            sigma_v_sq = _sigma_v_sq;
            sigma_a_sq = _sigma_a_sq;

            F = Eigen::MatrixXd::Identity(n, n);
            B = Eigen::MatrixXd::Zero(n, n/phi);

            Q = Eigen::MatrixXd::Zero(n, n);
//            Q.block(0, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * sigma_y_sq;
//            Q.block(n/phi, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * sigma_v_sq;

            H = Eigen::MatrixXd::Zero(m, n);
            H.block(0, 0, m, m) = Eigen::MatrixXd::Identity(m, m);
            R = Eigen::MatrixXd::Identity(m, m) * sigma_y_sq;

            P = Eigen::MatrixXd::Zero(n, n);
            P.block(0, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * 10;
            P.block(n/phi, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * 10;

            Y = Eigen::VectorXd(m);
            X_hat = Eigen::VectorXd(n);

            isFirstInput = true;
        }

        Obstacle filter(const Obstacle& obstacle){
            // Observe
            Y << obstacle.position.x(), obstacle.position.y(), obstacle.position.z();
            ros::Time current_update_time = obstacle.update_time;

            if(isFirstInput){
                X_hat << Y, 0, 0, 0;
                isFirstInput = false;
            }
            else{
                // Update F
                dt = (current_update_time - prev_update_time).toSec();
                F.block(0, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * dt;
                B.block(n/phi, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * dt;
                Q = sigma_a_sq * B * B.transpose();
//                Q.block(0, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * sigma_y_sq;

                // Predict
                X_hat_update = F * X_hat;
                Eigen::MatrixXd P_update = F * P * F.transpose() + Q;

                // Kalman Gain
                Eigen::MatrixXd S = H * P_update * H.transpose() + R;
                Eigen::MatrixXd K = P_update * H.transpose() * S.inverse();

                // Update
                X_hat = X_hat_update + K * (Y - H * X_hat_update);
                P = P_update - K * H * P_update;
            }

            Obstacle result = obstacle;
            result.position = point3d(X_hat(0, 0), X_hat(1, 0), X_hat(2, 0));
            result.velocity = point3d(X_hat(3, 0), X_hat(4, 0), X_hat(5, 0));

//            double v = sqrt(pow(result.velocity.linear.x, 2) + pow(result.velocity.linear.y, 2) + pow(result.velocity.linear.z, 2));
//            std::cout << "dt: " << dt << ", v: " << v << std::endl;

            prev_update_time = current_update_time;
            return result;
        }

        Eigen::MatrixXd getPositionCovariance(){
            return P.block(0, 0, n/phi, n/phi);
        }

        double getUncertaintyRadius(double t_delta){
            F.block(0, n/phi, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * t_delta;
            B.block(n/phi, 0, n/phi, n/phi) = Eigen::MatrixXd::Identity(n/phi, n/phi) * t_delta;
            Q = sigma_a_sq * B * B.transpose();
            Eigen::MatrixXd P_update = F * P * F.transpose() + Q;

            Eigen::MatrixXd SIGMA = P_update.block(0, 0, n/phi, n/phi);
            double uncertainty_radius = 1.33 * sqrt(2.0 * (double)SIGMA.trace()); //TODO: consider vector direction
            return uncertainty_radius;
        }

    private:
        int n, m, phi;
        double sigma_y_sq, sigma_v_sq, sigma_a_sq, dt;
        Eigen::MatrixXd F, B, Q, H, R, P;
        Eigen::VectorXd Y, X_hat, X_hat_update;

        bool isFirstInput;
        ros::Time prev_update_time;
    };
}