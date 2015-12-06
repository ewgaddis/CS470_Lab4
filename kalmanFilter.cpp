#include "kalmanFilter.h"

#include "gnuplotter.h"

using namespace Eigen;

KalmanFilter::KalmanFilter(double initialSigmaPos,
						   double initialSigmaVel,
						   double initialSigmaAccel,
						   double sigmaPos,
						   double sigmaVel,
						   double sigmaAccel,
						   double sigmaObs) : mean(6),
											  sigma(6, 6),
											  sigmaX(6, 6),
											  sigmaZ(2, 2),
											  F(6, 6),
											  H(2, 6)
{
	mean.fill(0.0);

	sigma.fill(0.0);
	sigma(0, 0) = sigma(3, 3) = initialSigmaPos;
	sigma(1, 1) = sigma(4, 4) = initialSigmaVel;
	sigma(2, 2) = sigma(5, 5) = initialSigmaAccel;

	sigmaX.fill(0.0);
	sigmaX(0, 0) = sigmaX(3, 3) = sigmaPos;
	sigmaX(1, 1) = sigmaX(4, 4) = sigmaVel;
	sigmaX(2, 2) = sigmaX(5, 5) = sigmaAccel;

	sigmaZ.fill(0);
	sigmaZ(0, 0) = sigmaZ(1, 1) = sigmaObs;

	F.fill(0.0);
	F(0, 0) = F(3, 3) = 1.0;
	F(1, 1) = F(4, 4) = 1.0;
	F(2, 2) = F(5, 5) = 1.0;

	H.fill(0.0);
	H(0, 0) = H(1, 3) = 1.0;

	HT = H.transpose();
}

void KalmanFilter::update(const VectorXd &z,
						  double elapsedTime,
						  double c,
						  bool ignoreVel,
						  bool ignoreAccel)
{
	F(0, 1) = F(3, 4) = (ignoreVel ? 0.0 : elapsedTime);
	F(0, 2) = F(3, 5) = (ignoreAccel ? 0.0 : elapsedTime * elapsedTime / 2.0);
	F(1, 2) = F(4, 5) = (ignoreAccel ? 0.0 : elapsedTime);
	F(2, 1) = F(5, 4) = -c;

	MatrixXd FT(F.transpose());

	MatrixXd A(F * sigma * FT);
	MatrixXd B((A + sigmaX) * HT);

	VectorXd u(F * mean);

	MatrixXd K(B * (H * B + sigmaZ).inverse());

	mean  = u + K * (z - H * u);
	sigma = (MatrixXd::Identity(6, 6) - K * H) * (A + sigmaX);
}

void KalmanFilter::predict(double time, double c, Eigen::VectorXd *m,
						   bool ignoreVel, bool ignoreAccel)
{
	F(0, 1) = F(3, 4) = (ignoreVel ? 0.0 : time);
	F(0, 2) = F(3, 5) = (ignoreAccel ? 0.0 : time * time / 2.0);
	F(1, 2) = F(4, 5) = (ignoreAccel ? 0.0 : time);
	F(2, 1) = F(5, 4) = -c;

	(*m) = F * mean;
}

void plotKalmanFilter(KalmanFilter &filter, double predictTime, double predictC,
					  const Vector *actualPos, const Vector *actualVel)
{
	static int number = 0;

	char fileName[256];
	sprintf(fileName, "./Data/kalman%i.gpi", number);

	char title[256];
	sprintf(title, "Kalman Filter #%i", number);

	++number;

	GNUPlotter plotter;

	plotter.createFile(fileName, title);

	Vector meanPos(filter.getMean()(0), filter.getMean()(3));
	Vector meanVel(filter.getMean()(1), filter.getMean()(4));

	plotter.drawCircle(meanPos, 5, 0, 0, 255);
	plotter.drawArrow(meanPos, meanPos + meanVel, 0, 0, 255);

	if(actualPos)
	{
		plotter.drawCircle(*actualPos, 5, 255, 0, 0);

		if(actualVel)
		{
			plotter.drawArrow(*actualPos, *actualPos + *actualVel, 255, 0, 0);
		}
	}

	if(predictTime > 0.0)
	{
		VectorXd p;
		filter.predict(predictTime, predictC, &p);

		plotter.drawCircle(Vector(p(0), p(3)), 5, 0, 255, 0);
		plotter.drawArrow(Vector(p(0), p(3)), Vector(p(0) + p(1), p(3) + p(4)), 0, 255, 0);
	}

	plotter.drawGaussian(filter.getMean(), filter.getSigma());

	plotter.finishFile();
}