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
#include <memory>
#include <vector>

// Inherited header
#include "QPSolver.h"

// Task
#include "QPGainsConstrData.h"

// Forward declare RBDyn
namespace rbd
{
class MultiBody;
class MultiBodyConfig;
}

namespace tasks
{

namespace qpgains
{

/**
	* The adaptive qp class.
	* This class handles all the constraints and tasks inside @see tasks::qpgains plus @see tasks::qp.
	*
	* @warning Some constraints become incompatible with the adaptive qp if @see QPGainsSolver::setGainsList is used.
	* Those constraints are: 
	*   - @see tasks::qp::JointLimitsConstr (replaced by @see tasks::qpgains::JointLimitsNoGainsConstr)
	*   - @see tasks::qp::DamperJointLimitsConstr (replaced by @see tasks::qpgains::DamperJointLimitsNoGainsConstr)
	*   - @see tasks::qp::MotionConstr (replaced by @see tasks::qpgains::MotionGainsConstr)
	*   - @see tasks::qp::MotionSpringConstr (has no replacement yet)
	*   - @see tasks::qp::MotionPolyConstr (has no replacement yet)
	*/
class TASKS_DLLAPI QPGainsSolver : public tasks::qp::QPSolver
{
public:
	/** Constructor */
	QPGainsSolver();
	/** Destructor */
	~QPGainsSolver() = default;

	/** Make the qp know that it may uses its adaptive form
		* @param mbs The vector of all robot
		* @param robotIndex The index of the robot
		*/
	void addRobotToAdaptiveQP(const std::vector<rbd::MultiBody> & mbs, int robotIndex);
	/** Set the position reference vector for a specific robot.
		* The robot should have been added before calling this function @see addRobotToAdaptiveQP
		* @param robotIndex The index of the robot
		* @param q0 The vector of vector of the position reference vector
		* @throw std::runtime_error if the robot has not been added yet.
		*/
	void setRobotq0(int robotIndex, const std::vector<std::vector<double> > &q0);
	/** Set the velocity reference vector for a specific robot.
		* The robot should have been added before calling this function @see addRobotToAdaptiveQP
		* @param robotIndex The index of the robot
		* @param alpha0 The vector of vector of the velocity reference vector
		* @throw std::runtime_error if the robot has not been added yet.
		*/
	void setRobotAlpha0(int robotIndex, const std::vector<std::vector<double> > &alpha0);
	/** Set the velocity reference vector for a specific robot.
		* The robot should have been added before calling this function @see addRobotToAdaptiveQP
		* @param robotIndex The index of the robot
		* @param q0 The vector of vector of the velocity reference vector
		* @throw std::runtime_error if the robot has not been added yet.
		* @warning You need to call @see nrVars after this function and beforedoing any @see solve
		*/
	void setGainsList(const std::vector<rbd::MultiBody> &mbs, int robotIndex, const std::vector<int> &gainsList);
	/** Update @see rbd::MultiBodyConfig with solver's last results
		* @param mbc The MultiBodyConfig of the robot
		* @param robotIndex The index of the robot
		*/
	void updateMbc(rbd::MultiBodyConfig& mbc, int robotIndex) const override;

	/** Update the tasks and constraints.
		* @param mbs All the robot multi bodies.
		* @param uni The unilateral constacts
		* @param bi The bilateral contacts
		* @warning This needs to be called at least one timee before running the qp, and when all the desired constraints has been added to the qp.
		*/		
	void nrVars(const std::vector<rbd::MultiBody>& mbs, std::vector<tasks::qp::UnilateralContact> uni,
		std::vector<tasks::qp::BilateralContact> bi) override;

	/** Get a const-copy of the shared pointer of @see ConstrData.
		* This function is intended for the use of tasks and constraints
		* @param robotIndex The index of the robot
		* @return A const-copy of the ConstrData of the desired robot
		*/
	const std::shared_ptr<ConstrData> getConstrData(int robotIndex) const;

	/** Return a vector of gains of all robots 
		* @return The vector
		*/
	Eigen::VectorXd gainsVec() const;
	/** Return a vector of gains of a specific robot
		* @param robotIndex The index of the robot
		* @return The vector
		*/
	Eigen::VectorXd gainsVec(int robotIndex) const;

protected:
	void preUpdate(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs) override;
	std::size_t elemPosByRobotIndex(int robotIndex, const std::string& funName) const;

protected:
	std::vector<int> robotIndex_;
	std::vector<std::unique_ptr<ConstrDataComputation>> constrDataCompute_;
	std::vector<std::shared_ptr<ConstrData>> constrDataStock_;
};


} // namespace qpgains

} // namespace tasks