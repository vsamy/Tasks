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


class TASKS_DLLAPI BoundGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Bound>
{
public:
	/**
		* @param mbs Multi-robot system.
		* @param robotIndex Constrained robot Index in mbs.
		*/
	BoundGainsConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex);

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	// Constraint
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;

	void update(const std::vector<rbd::MultiBody>& /* mbs */, const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
		const tasks::qp::SolverData& /* data */) override;

	std::string nameBound() const override;
	std::string descBound(const std::vector<rbd::MultiBody>& mbs, int line) override;

	// Bound Constraint
	int beginVar() const override;

	const Eigen::VectorXd& Lower() const override;
	const Eigen::VectorXd& Upper() const override;

private:
	int robotIndex_, gainsBegin_;
	std::shared_ptr<ConstrData> constrData_;
	Eigen::VectorXd lower_, upper_;
};



class TASKS_DLLAPI MotionGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::GenInequality>
{
public:
	MotionGainsConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex, const TorqueBound& tb);

	void computeTorque(const Eigen::VectorXd& alphaD, const Eigen::VectorXd& lambda);
	const Eigen::VectorXd& torque() const;
	void torque(const std::vector<rbd::MultiBody>& mbs,	std::vector<rbd::MultiBodyConfig>& mbcs) const;

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	// Constraint
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;

	void update(const std::vector<rbd::MultiBody>& mbs,	const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	// Description
	std::string nameGenInEq() const override;
	std::string descGenInEq(const std::vector<rbd::MultiBody>& mbs, int line) override;

	// Inequality Constraint
	int maxGenInEq() const override;

	const Eigen::MatrixXd& AGenInEq() const override;
	const Eigen::VectorXd& LowerGenInEq() const override;
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



class TASKS_DLLAPI MotionGainsEqualConstr : public tasks::qp::ConstraintFunction<tasks::qp::Equality>
{
public:
	MotionGainsEqualConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex);

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	// Constraint
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;

	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	// Description
	std::string nameEq() const override;
	std::string descEq(const std::vector<rbd::MultiBody>& mbs, int line) override;

	// Inequality Constraint
	int maxEq() const override;

	const Eigen::MatrixXd& AEq() const override;
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


class TASKS_DLLAPI TorquePDConstr : public tasks::qp::ConstraintFunction<tasks::qp::GenInequality>
{
public:
	TorquePDConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex, const TorqueBound& tb);

	void computeMotorTorque(const Eigen::VectorXd& gains);
	const Eigen::VectorXd& motorTorque() const;
	void motorTorque(const std::vector<rbd::MultiBody>& mbs, std::vector<rbd::MultiBodyConfig>& mbcs) const;

	// Add the pointer to the problem datas
    // Need to be called right after addToSolver
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	// Constraint
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;

	void update(const std::vector<rbd::MultiBody>& mb, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	// Description
	std::string nameGenInEq() const override;
	std::string descGenInEq(const std::vector<rbd::MultiBody>& mbs, int line) override;

	// Inequality Constraint
	int maxGenInEq() const override;

	const Eigen::MatrixXd& AGenInEq() const override;
	const Eigen::VectorXd& LowerGenInEq() const override;
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
