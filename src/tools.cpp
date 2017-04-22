#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	/*
	* Calculate the RMSE.
	*/

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() == 0 || (estimations.size() != ground_truth.size())) {
		std::cout << "Error in estimations or ground_truth vector sizes";
		return rmse;
	}


	for (unsigned int i = 0; i < estimations.size(); i++)
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		//std::cout << i << "  " << residual << "\n";
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the sqrt
	rmse = rmse.array().sqrt();

	return rmse;
}
