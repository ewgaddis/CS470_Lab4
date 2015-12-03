#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter
{
private:
	Eigen::VectorXd mean;
	Eigen::MatrixXd sigma;

	Eigen::MatrixXd sigmaX;
	Eigen::Matrix2d sigmaZ;

	Eigen::MatrixXd F;

	Eigen::MatrixXd H;
	Eigen::MatrixXd HT;

public:
	KalmanFilter(const Eigen::VectorXd &m,
				 const Eigen::MatrixXd &s,
				 const Eigen::MatrixXd &sx,
				 const Eigen::Matrix2d &sz,
				 const Eigen::MatrixXd &_H);

	void update(const Eigen::VectorXd z,
				double elapsedTime,
				double c);

	const Eigen::VectorXd & getMean()  const { return mean;  }
	const Eigen::MatrixXd & getSigma() const { return sigma; }

	const Eigen::MatrixXd & getF() const { return F; }
};

#endif