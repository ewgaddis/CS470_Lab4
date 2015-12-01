#include "kalmanFilter.h"

using namespace Eigen;

KalmanFilter::KalmanFilter(const Eigen::VectorXd &m,
						   const Eigen::MatrixXd &s,
						   const Eigen::MatrixXd &sx,
						   const Eigen::Matrix2d &sz,
						   const Eigen::MatrixXd &_F,
						   const Eigen::MatrixXd &_H) : mean(m),
														sigma(s),
														sigmaX(sx),
														sigmaZ(sz),
														F(_F),
														H(_H),
														FT(_F.transpose()),
														HT(_H.transpose())
{
}

void KalmanFilter::update(const Eigen::VectorXd z)
{
	MatrixXd A(F * sigma * FT);
	MatrixXd B((A + sigmaX) * HT);

	VectorXd u(F * mean);

	MatrixXd K(B * (H * B + sigmaZ).inverse());

	mean  = u + K * (z - H * u);
	sigma = (MatrixXd::Identity(6, 6) - K * H) * (A + sigmaX);
}