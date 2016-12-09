// This file is part of Tasks.
//
// Tasks is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Tasks is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Tasks.  If not, see <http://www.gnu.org/licenses/>.


#pragma once

// std
#include <vector>
#include <memory>

// Eigen
#include <Eigen/Core>

// RBDyn
#include <RBDyn/FD.h>
#include <RBDyn/Jacobian.h>

//Forward declare RBDyn
namespace rbd
{
class MultiBody;
class MultiBodyConfig;
}

namespace tasks
{
//Forward declare Contact and SolverData
namespace qp
{
class SolverData;
class FrictionCone;
}


namespace qpgains
{

struct ConstrData
{
	ConstrData(const rbd::MultiBody& mb);
	~ConstrData() {}

	void setGainsList(const rbd::MultiBody& mb, const std::vector<int> & jointsList);
	void updateNrVars(const rbd::MultiBody& mb, const tasks::qp::SolverData& data);

	void insertSpecificLines(std::vector<int>& lineList,
		const Eigen::MatrixXd& matExtract, Eigen::Ref<Eigen::MatrixXd> matOut);

	// Vector of joint index for which gains will be changed
	std::vector<int> gainsJointsList;

	// Vector of qp lines corresponding to joint index for which gains will be changed
	std::vector<int> gainsLinesList;

	// Vector of qp lines corresponding to other joint index
	std::vector<int> noGainsLinesList;

	// Stock mass matrix relative to the rindex robot
	Eigen::MatrixXd H;

	// Stock the Coriolis vector
	Eigen::VectorXd C;

	// Stock the contact matrix
	Eigen::MatrixXd minusJtG;

	// Stock the error vector
	Eigen::VectorXd error;

	// Stock the speed error vector
	Eigen::VectorXd derror;
};


class ConstrDataComputation
{

public:
	ConstrDataComputation(const std::vector<rbd::MultiBody>& mbs, int robotIndex);
	~ConstrDataComputation() {}

	void updateNrVars(const rbd::MultiBody &mb, const tasks::qp::SolverData &data);
	void q0(const std::vector<std::vector<double> >& q0);
	void alpha0(const std::vector<std::vector<double> >& alpha0);

	void computeGainsConstrMatrices(const std::vector<rbd::MultiBody>& mbs,
		const std::vector<rbd::MultiBodyConfig>& mbcs,
		const std::shared_ptr<ConstrData>& stockData);

private:
	void computeFDMatrices(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc,
		Eigen::MatrixXd& H, Eigen::VectorXd& C);
	void computePDError(const rbd::MultiBodyConfig& mbc, const std::vector<int>& gainsJointsList,
		Eigen::VectorXd& e, Eigen::VectorXd& de) const;
	void computeContactMatrix(const rbd::MultiBody& mb, const rbd::MultiBodyConfig& mbc,
		Eigen::MatrixXd& minusJtG);

private:
	struct ContactData
	{
		ContactData() {}
		ContactData(const rbd::MultiBody& mb,
			const std::string& bodyName, int lambdaBegin,
			std::vector<Eigen::Vector3d> points,
			const std::vector<tasks::qp::FrictionCone>& cones);
		~ContactData() {}


		int bodyIndex, lambdaBegin;
		rbd::Jacobian jac;
		std::vector<Eigen::Vector3d> points;
		// BEWARE generator are minus to avoid one multiplication by -1 in the
		// update method
		std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic> > minusGenerators;
	};

private:
	int robotIndex_, lambdaBegin_, nrParam_, nrDof_;
	rbd::ForwardDynamics fd_;
	std::vector<std::vector<double> > q0_, alpha0_;
	Eigen::MatrixXd fullJacLambda_, jacTrans_, jacLambda_;
	std::vector<ContactData> cont_;
};


}

}
