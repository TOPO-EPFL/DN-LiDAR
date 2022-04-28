#ifndef INITIAL_GUESS_GENERATOR_H_
#define INITIAL_GUESS_GENERATOR_H_

#include <string>
#include <vector>
#include <set>

#include <Eigen/Dense>

namespace Eigen {
    typedef Matrix<double, 1, 7> Vector7d;
}

enum BodyFrameOrientation { FRONTLEFTUP, FRONTRIGHTDOWN };

class InitialGuessGenerator {
    public:

        InitialGuessGenerator(BodyFrameOrientation bfo, const std::string & gnss_file);

        Eigen::Vector7d initialGuessAt(double timestamp, const Eigen::VectorXd & no_swap_q_wrt_to);

    protected:
        BodyFrameOrientation bfo;

        std::vector<double> timestamps;
        std::vector<Eigen::Vector3d> positions;

        int N = 6;
};

#endif