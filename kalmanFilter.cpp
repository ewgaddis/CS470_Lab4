#include "kalmanFilter.h"

using namespace Eigen;

KalmanFilter::KalmanFilter(const Eigen::VectorXd &m,
						   const Eigen::MatrixXd &s,
						   const Eigen::MatrixXd &sx,
						   const Eigen::Matrix2d &sz,
						   const Eigen::MatrixXd &_H) : mean(m),
														sigma(s),
														sigmaX(sx),
														sigmaZ(sz),
														F(6, 6),
														H(_H),
														HT(_H.transpose())
{
	F.fill(0.0);
	F(0, 0) = F(3, 3) = 1.0;
	F(1, 1) = F(4, 4) = 1.0;
	F(2, 2) = F(5, 5) = 1.0;
}

void KalmanFilter::update(const VectorXd z,
						  double elapsedTime,
						  double c)
{
	F(0, 1) = F(3, 4) = elapsedTime;
	F(0, 2) = F(3, 5) = elapsedTime * elapsedTime / 2.0;
	F(1, 2) = F(4, 5) = elapsedTime;
	F(2, 1) = F(5, 4) = -c;

	MatrixXd FT(F.transpose());

	MatrixXd A(F * sigma * FT);
	MatrixXd B((A + sigmaX) * HT);

	VectorXd u(F * mean);

	MatrixXd K(B * (H * B + sigmaZ).inverse());

	mean  = u + K * (z - H * u);
	sigma = (MatrixXd::Identity(6, 6) - K * H) * (A + sigmaX);
}