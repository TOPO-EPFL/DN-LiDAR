#ifndef STAMPED_CSV_H_
#define STAMPED_CSV_H_

#include <string>
#include <fstream>

#include <Eigen/Dense>

class StampedCsv {
    public:

        StampedCsv(const std::string &file, int n_cols);

        bool next();
        bool isValid();

        const Eigen::VectorXd & getCurrent();
        double getTimestamp();

    protected:

        int n_cols;

        std::fstream f;

        Eigen::VectorXd current;
        double timestamp;
        bool valid;
};

#endif