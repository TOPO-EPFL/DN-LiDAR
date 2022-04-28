#include <iostream>

#include <cmath>

#include "initial_guess_generator.hpp"

#include "stamped_csv.hpp"

InitialGuessGenerator::InitialGuessGenerator(BodyFrameOrientation bfo, const std::string & gnss_file) : bfo(bfo) {
    StampedCsv gnss_data(gnss_file, 6);

    // read the whole set of positions
    while(gnss_data.next()) {
        timestamps.push_back(gnss_data.getTimestamp());        
        positions.push_back(gnss_data.getCurrent().head(3));
    };
}

Eigen::Vector7d InitialGuessGenerator::initialGuessAt(double query_t, const Eigen::VectorXd & no_swap_q_wrt_to) {
    Eigen::Vector7d ig;

    // find the elements that have to be interpolated

    int up = 0;
    for (int k = 0; k < timestamps.size()-1; k++) { // find the element which comes surely after
        if (timestamps[up] - 1e-6 < query_t ) {
            ++up; 
        }
    };    

    int low = up; 

    bool last_up = true;
    while( up-low+1 < N) {
        if (last_up) {
            if (low != 0) {
                --low;
            } else {
                ++up;
            }

        } else {
            if (up < timestamps.size()-1) {
                ++up;
            } else {
                --low;
            }
        }

        last_up = ! last_up;
    }

    // perform 2nd order polynomial interpolation of the selected N points

    Eigen::MatrixXd J(N, 3);
    Eigen::MatrixXd XYZ(N, 3);

    for (int k = low; k <= up; ++k) {
        J.row(k-low) << 1.0, (timestamps[k] - query_t), (timestamps[k] - query_t)*(timestamps[k] - query_t);

        XYZ.row(k-low) = positions[k];
    }

    // params will be a 3x3 matrix such that each column contain the parameters of the approximating parabola
    // x(t-query_t) = params[0,0] + params[1,0] * (t-query_t) + params[2,0] * (t-query_t)^2
    //
    // so the first row is the interpolated position at query_t and the second is the tangent at query_t

    Eigen::MatrixXd params = (J.transpose() * J).colPivHouseholderQr().solve(J.transpose() * XYZ);

    // get the orientation so that x is directed along the tangent 

    Eigen::Vector3d tangent = params.row(1)/params.row(1).norm();

    double yaw = atan2(tangent(1), tangent(0));
    double pitch = atan2( tangent(2), sqrt( pow(tangent(0),2) + pow(tangent(1),2) ) ); 

    Eigen::Quaterniond q1(cos(yaw/2), 0.0, 0.0, sin(yaw/2));
    Eigen::Quaterniond q2(cos(-pitch/2), 0.0, sin(-pitch/2), 0 );
    Eigen::Quaterniond q = q1*q2;

    if (bfo == FRONTRIGHTDOWN) {
        Eigen::Quaterniond q3(0.0, 1.0, 0.0, 0.0);
        q = q * q3;
    }

    Eigen::Quaterniond qold( no_swap_q_wrt_to(3), no_swap_q_wrt_to(4), no_swap_q_wrt_to(5), no_swap_q_wrt_to(6) );

    if ((q.coeffs() - qold.coeffs()).norm() > (q.coeffs() + qold.coeffs()).norm() ) {
        q.coeffs() = - q.coeffs();
        // std::cerr << "swapping or: " << (no_swap_q_wrt_to.tail(4) - q.coeffs()).norm() << " swp: " << (no_swap_q_wrt_to.tail(4) + q.coeffs()).norm() << std::endl;
        // std::cerr << q.coeffs().transpose() << "  ---  " << no_swap_q_wrt_to.tail(4).transpose() << std::endl; 
    }
    
    ig << params(0, 0), params(0, 1), params(0, 2), q.coeffs()(3), q.coeffs()(0), q.coeffs()(1), q.coeffs()(2);

    return ig;
}
