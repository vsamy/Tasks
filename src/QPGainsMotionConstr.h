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

// Eigen
#include <Eigen/Core>

// Solver
#include "QPGainsSolver.h"

// Forward declare RBDyn
namespace rbd
{
class MultiBody;
class MultiBodyConfig;
}

namespace tasks
{
// Forward declare TorqueBound 
class TorqueBound;

// Forward ConstrData
namespace qpgains
{
class ConstrData;

/**
	* Bound constraint for \f$K\f$ and \f$B\f$
	*/
class TASKS_DLLAPI BoundGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Bound>
{
public:
	/** Constructor
		* @param mbs Multi bodies of all robots
		* @param robotIndex Constrained robot Index in mbs
		*/
	BoundGainsConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex);

	/** Assign the pointer to the problem datas.
		@param sol The solver the constraint belongs to
		@warning This must be called before any update.
		*/
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	/** Update the system size
	 * @param mbs Multi bodies of all robots
	 * @param data The datas of the solver
	 */
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	/** Update the constraint
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 * @param data The datas of the solver
	 */
	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	/** Return the name of the constraint
	 * @return The name of the constraint
	 */
	std::string nameBound() const override;
	/** Return the name of the joint that fails when the qp fails.
	 * @param mb Multi bodies of all robots
	 * @param line The line for which the qp failed
	 * @return The name of the joint
	 */
	std::string descBound(const std::vector<rbd::MultiBody>& mbs, int line) override;

	/** Gives the position of bound constraint inthe qp's optimization vector
		* @return The position
		*/
	int beginVar() const override;

	/** Get the computed lower limit
		* @return The lower limite
		*/
	const Eigen::VectorXd& Lower() const override;
	/** Get the computed upper limit
		* @return The upper limite
		*/
	const Eigen::VectorXd& Upper() const override;

private:
	int robotIndex_, gainsBegin_;
	std::shared_ptr<ConstrData> constrData_;
	Eigen::VectorXd lower_, upper_;
};


/** The equation of motion as a constraint for non adaptive joints.
	* It replaces the @tasks::qp::MotionConstr for the adaptive qp.
	* The constraint is \f$\underline{\tau} \leq H\ddot{q} + c -J^{T}G\lambda \leq \overline{\tau}\f$.
	*/
class TASKS_DLLAPI MotionGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::GenInequality>
{
public:
	/** Constructor
		* @param mbs Multi bodies of all robots
		* @param robotIndex Constrained robot Index in mbs
		* @param tb The torque limits
		*/
	MotionGainsConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex, const TorqueBound& tb);

	/** Compute the torques of the @see ConstrData::noGainsLinesList
		@param alphaD The generalized acceleration vector
		@param lambda The contacts
		*/
	void computeTorque(const Eigen::VectorXd& alphaD, const Eigen::VectorXd& lambda);
	/** Return the computed torques of the @see ConstrData::noGainsLinesList
		@return The torque vector
		*/
	const Eigen::VectorXd& torque() const;
	/** Save the computed torque into the multi body config of the robot.
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 */
	void torque(const std::vector<rbd::MultiBody>& mbs,	std::vector<rbd::MultiBodyConfig>& mbcs) const;

	/** Assign the pointer to the problem datas.
		@param sol The solver the constraint belongs to
		@warning This must be called before any update.
		*/
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	/** Update the system size
	 * @param mbs Multi bodies of all robots
	 * @param data The datas of the solver
	 */
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	/** Update the constraint
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 * @param data The datas of the solver
	 */
	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	/** Return the name of the constraint
	 * @return The name of the constraint
	 */
	std::string nameGenInEq() const override;
	/** Return the name of the joint that fails when the qp fails.
	 * @param mb Multi bodies of all robots
	 * @param line The line for which the qp failed
	 * @return The name of the joint
	 */
	std::string descGenInEq(const std::vector<rbd::MultiBody>& mbs, int line) override;

	/** Return the number of constraints
	 * @return The number of constraints
	 */
	int maxGenInEq() const override;

	/** Get the computed constraint matrix
		* @return The matrix
		*/
	const Eigen::MatrixXd& AGenInEq() const override;
	/** Get the computed lower limit
		* @return The lower limite
		*/
	const Eigen::VectorXd& LowerGenInEq() const override;
	/** Get the computed upper limit
		* @return The upper limite
		*/
	const Eigen::VectorXd& UpperGenInEq() const override;

protected:
	int robotIndex_, alphaDBegin_, nrDof_, lambdaBegin_, nrLambda_;
	std::shared_ptr<ConstrData> constrData_;
	std::size_t nrLines_;

	Eigen::VectorXd curTorque_;

	Eigen::MatrixXd A_;
	Eigen::VectorXd AL_, AU_;
	Eigen::VectorXd torqueL_, torqueU_;
};


/** The equation of motion as a constraint for adaptive joints.
	* The constraint is \f$H\ddot{q} + c -J^{T}G\lambda = Ke + B\dot{e}\f$.
	*/
class TASKS_DLLAPI MotionGainsEqualConstr : public tasks::qp::ConstraintFunction<tasks::qp::Equality>
{
public:
	/** Constructor
		* @param mbs Multi bodies of all robots
		* @param robotIndex Constrained robot Index in mbs
		*/
	MotionGainsEqualConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex);

	/** Assign the pointer to the problem datas.
		@param sol The solver the constraint belongs to
		@warning This must be called before any update.
		*/
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	/** Update the system size
	 * @param mbs Multi bodies of all robots
	 * @param data The datas of the solver
	 */
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	/** Update the constraint
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 * @param data The datas of the solver
	 */
	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	/** Return the name of the constraint
	 * @return The name of the constraint
	 */
	std::string nameEq() const override;
	/** Return the name of the joint that fails when the qp fails.
	 * @param mb Multi bodies of all robots
	 * @param line The line for which the qp failed
	 * @return The name of the joint
	 */
	std::string descEq(const std::vector<rbd::MultiBody>& mbs, int line) override;

	/** Return the number of constraints
	 * @return The number of constraints
	 */
	int maxEq() const override;

	/** Get the computed constraint matrix
		* @return The matrix
		*/
	const Eigen::MatrixXd& AEq() const override;
	/** Get the computed constraint vector
		* @return The vector
		*/
	const Eigen::VectorXd& bEq() const override;

protected:
	int robotIndex_, alphaDBegin_, nrDof_, nrLambda_, lambdaBegin_;
	int gainsBegin_;
	std::shared_ptr<ConstrData> constrData_;
	std::size_t nrLines_;

	Eigen::VectorXd curTorque_;

	Eigen::MatrixXd A_;
	Eigen::VectorXd b_;
};


/** This constraint ensures that the torques are under their respective limits
	* The constraint is \f$\underline{\tau} \leq Ke + B\dot{e} \leq \overline{\tau}\f$.
	*/
class TASKS_DLLAPI TorquePDConstr : public tasks::qp::ConstraintFunction<tasks::qp::GenInequality>
{
public:
	/** Constructor
		* @param mbs Multi bodies of all robots
		* @param robotIndex Constrained robot Index in mbs
		* @param tb The torque limits
		*/
	TorquePDConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex, const TorqueBound& tb);

	/** Compute the torques of the @see ConstrData::gainsLinesList
		@param gains The gains \f$K\f$ and \f$B\f$
		*/
	void computeMotorTorque(const Eigen::VectorXd& gains);
	/** Return the computed torques of the @see ConstrData::noGainsLinesList
		@return The torque vector
		*/
	const Eigen::VectorXd& motorTorque() const;
	/** Save the computed torque into the multi body config of the robot.
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 */
	void motorTorque(const std::vector<rbd::MultiBody>& mbs, std::vector<rbd::MultiBodyConfig>& mbcs) const;

	/** Assign the pointer to the problem datas.
		@param sol The solver the constraint belongs to
		@warning This must be called before any update.
		*/
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	/** Update the system size
	 * @param mbs Multi bodies of all robots
	 * @param data The datas of the solver
	 */
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	/** Update the constraint
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 * @param data The datas of the solver
	 */
	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	/** Return the name of the constraint
	 * @return The name of the constraint
	 */
	std::string nameGenInEq() const override;
	/** Return the name of the joint that fails when the qp fails.
	 * @param mb Multi bodies of all robots
	 * @param line The line for which the qp failed
	 * @return The name of the joint
	 */
	std::string descGenInEq(const std::vector<rbd::MultiBody>& mbs, int line) override;

	/** Return the number of constraints
	 * @return The number of constraints
	 */
	int maxGenInEq() const override;

	/** Get the computed constraint matrix
		* @return The matrix
		*/
	const Eigen::MatrixXd& AGenInEq() const override;
	/** Get the computed lower limit
		* @return The lower limite
		*/
	const Eigen::VectorXd& LowerGenInEq() const override;
	/** Get the computed upper limit
		* @return The upper limite
		*/
	const Eigen::VectorXd& UpperGenInEq() const override;

protected:
	int robotIndex_, nrDof_, gainsBegin_;
	std::shared_ptr<ConstrData> constrData_;
	std::size_t nrLines_;

	Eigen::VectorXd curTorque_, torqueL_, torqueU_;

	Eigen::MatrixXd A_;
	Eigen::VectorXd AL_, AU_;
};

} // namespace qpgains

} // namespace tasks
