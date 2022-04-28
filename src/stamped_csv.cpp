#include <exception>

#include "stamped_csv.hpp"

StampedCsv::StampedCsv(const std::string & file, int n_cols) : current(n_cols), valid(false), timestamp(0) {
    f.open(file);

    if (!f.is_open() || f.eof()) {
        throw std::invalid_argument(file + " not found");
    }
}

bool StampedCsv::next() {
  if (!f.is_open() || f.eof()) {
        valid = false;
        return false;
  }

  char delim;

  f >> timestamp;

  for (int k = 0; k < current.size(); k++) {
    f >> delim;
    f >> current(k);
  }

  valid = !f.eof();
  return valid;
}

const Eigen::VectorXd & StampedCsv::getCurrent() {
    return current;
}

double StampedCsv::getTimestamp() {
    return timestamp;
}

bool StampedCsv::isValid() {
    return valid;
}