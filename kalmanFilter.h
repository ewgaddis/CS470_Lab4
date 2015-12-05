#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include "geometry.h"

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
	KalmanFilter(double initialSigmaPos,
				 double initialSigmaVel,
				 double initialSigmaAccel,
				 double sigmaPos,
				 double sigmaVel,
				 double sigmaAccel,
				 double sigmaObs);

	void update(const Eigen::VectorXd &z,
				double elapsedTime,
				double c,
				bool ignoreVel = false,
				bool ignoreAccel = false);

	void predict(double time, double c, Eigen::VectorXd *m,
				 bool ignoreVel = false, bool ignoreAccel = false);

	const Eigen::VectorXd & getMean()  const { return mean;  }
	const Eigen::MatrixXd & getSigma() const { return sigma; }

	const Eigen::MatrixXd & getF() const { return F; }
	const Eigen::MatrixXd & getH() const { return H; }
};

void plotKalmanFilter(KalmanFilter &filter, double predictTime, double predictC,
					  const Vector *actualPos, const Vector *actualVel);

#endif