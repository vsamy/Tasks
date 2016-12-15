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

/** This structure keeps all the computation datas.
 * It holds information of desired adaptive joints,
 * equation of motion matrices and vectors,
 * contact matrices,
 * and position and velocity errors
 */
struct ConstrData
{
	/** Constructor
		* @param mb The multi body of the robot
		*/
	ConstrData(const rbd::MultiBody& mb);
	/** Destructor */
	~ConstrData() = default;

	/** Set the desired compliant joints
		* @param mb The multi body of the robot
		* @param jointsList The list of the joints to be compliant
		*/
	void setGainsList(const rbd::MultiBody& mb, const std::vector<int>& jointsList);
	/** Update the system size
		* @param mb Multi bodies of the robot
		* @param data The datas of the solver
		*/	
	void updateNrVars(const rbd::MultiBody& mb, const tasks::qp::SolverData& data);

	/** Insert the specific rows of a matrix into another matrix in a list order.
		* @param lineList The list of rows to copy
		* @param matExtract The matrix from which the row is copied
		* @param matOut The maxtrix to which the row is copied
		*/
	void insertSpecificLines(std::vector<int>& lineList,
		const Eigen::MatrixXd& matExtract, Eigen::Ref<Eigen::MatrixXd> matOut);

	std::vector<int> gainsJointsList; /**< Vector of joint index for which gains will be changed */
	std::vector<int> gainsLinesList; /**< Vector of qp lines corresponding to joint index for which gains will be changed */
	std::vector<int> noGainsLinesList; /**< Vector of qp lines corresponding to other joint index */
	Eigen::MatrixXd H; /**< Stock mass matrix relative to the rindex robot */
	Eigen::VectorXd C; /**< Stock the Coriolis and gravity vector */
	Eigen::MatrixXd minusJtG; /**< Stock the contact matrix */
	Eigen::VectorXd error; /**< Stock the position error vector */
	Eigen::VectorXd derror; /**< Stock the velocity error vector */
};


/** This class makes all the computations required for updating the constraints
	* It stocks every results into a @see ConstrData structure.
	*/
class ConstrDataComputation
{

public:
	/** Constructor
	 	* @param mbs Multi bodies of all robots
		* @param robotIndex The index of the robot
		*/
	ConstrDataComputation(const std::vector<rbd::MultiBody>& mbs, int robotIndex);
	/** Destructor */
	~ConstrDataComputation() = default;

	/** Update the system size
		* @param mb Multi bodies of all the robots
		* @param data The datas of the solver
		*/	
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData &data);
	/** Set the position reference vector
		* @param q0 The vector of vector of the position reference vector
		*/
	void q0(const std::vector<std::vector<double>>& q0);
	/** Set the velocity reference vector
		* @param alpha0 The vector of vector of the velocity reference vector
		*/
	void alpha0(const std::vector<std::vector<double>>& alpha0);

	/** Compute all the necessary matrices and vectors of the equation of motion and PD control.
		* @param mbs Multi bodies of all the robots
		* @param mbcs Multi bodies configs of all the robots
		* @param stockData The structure in which the results are copied
		*/
	void computeGainsConstrMatrices(const std::vector<rbd::MultiBody>& mbs,	const std::vector<rbd::MultiBodyConfig>& mbcs,
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
		std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> minusGenerators;
	};

private:
	int robotIndex_, lambdaBegin_, nrParam_, nrDof_;
	rbd::ForwardDynamics fd_;
	std::vector<std::vector<double>> q0_, alpha0_;
	Eigen::MatrixXd fullJacLambda_, jacTrans_, jacLambda_;
	std::vector<ContactData> cont_;
};


}

}
