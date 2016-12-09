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


class BoundGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Bound>
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
		constrData_ = sol.getConstrData();
	}

	// Constraint
	virtual void updateNrVars(const std::vector<rbd::MultiBody>& mbs,
		const tasks::qp::SolverData& data);

	virtual void update(const std::vector<rbd::MultiBody>& /* mbs */,
		const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
		const tasks::qp::SolverData& /* data */);

	virtual std::string nameBound() const;
	virtual std::string descBound(const std::vector<rbd::MultiBody>& mbs, int line);

	// Bound Constraint
	virtual int beginVar() const;

	virtual const Eigen::VectorXd& Lower() const;
	virtual const Eigen::VectorXd& Upper() const;

private:
	int robotIndex_, gainsBegin_;
	std::shared_ptr<ConstrData> constrData_;
	Eigen::VectorXd lower_, upper_;
};



class MotionConstr : public tasks::qp::ConstraintFunction<tasks::qp::GenInequality>
{
public:
	MotionConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
		const TorqueBound& tb);

	void computeTorque(const Eigen::VectorXd& alphaD,
		const Eigen::VectorXd& lambda);
	const Eigen::VectorXd& torque() const;
	void torque(const std::vector<rbd::MultiBody>& mbs,
		std::vector<rbd::MultiBodyConfig>& mbcs) const;

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData();
	}

	// Constraint
	virtual void updateNrVars(const std::vector<rbd::MultiBody>& mbs,
		const tasks::qp::SolverData& data);

	virtual void update(const std::vector<rbd::MultiBody>& mbs,
		const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data);

	// Description
	virtual std::string nameGenInEq() const;
	virtual std::string descGenInEq(const std::vector<rbd::MultiBody>& mbs, int line);

	// Inequality Constraint
	virtual int maxGenInEq() const;

	virtual const Eigen::MatrixXd& AGenInEq() const;
	virtual const Eigen::VectorXd& LowerGenInEq() const;
	virtual const Eigen::VectorXd& UpperGenInEq() const;

protected:
	int robotIndex_, alphaDBegin_, nrDof_, lambdaBegin_, nrLambda_;
	std::shared_ptr<ConstrData> constrData_;
	std::size_t nrLines_;

	Eigen::VectorXd curTorque_;

	Eigen::MatrixXd A_;
	Eigen::VectorXd AL_, AU_;
	Eigen::VectorXd torqueL_, torqueU_;
};



class MotionGainsEqualConstr : public tasks::qp::ConstraintFunction<tasks::qp::Equality>
{
public:
	MotionGainsEqualConstr(const std::vector<rbd::MultiBody>& mbs,
		int robotIndex);

	Eigen::MatrixXd getA() const
	{
		return A_;
	}

	Eigen::VectorXd getb() const
	{
		return b_;
	}

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData();
	}

	// Constraint
	virtual void updateNrVars(const std::vector<rbd::MultiBody>& mbs,
		const tasks::qp::SolverData& data);

	void update(const std::vector<rbd::MultiBody>& mbs,
		const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data);

	// Description
	virtual std::string nameEq() const;
	virtual std::string descEq(const std::vector<rbd::MultiBody>& mbs, int line);

	// Inequality Constraint
	virtual int maxEq() const;

	virtual const Eigen::MatrixXd& AEq() const;
	virtual const Eigen::VectorXd& bEq() const;

protected:
	int robotIndex_, alphaDBegin_, nrDof_, nrLambda_, lambdaBegin_;
	int gainsBegin_;
	std::shared_ptr<ConstrData> constrData_;
	std::size_t nrLines_;

	Eigen::VectorXd curTorque_;

	Eigen::MatrixXd A_;
	Eigen::VectorXd b_;
};


class TorquePDConstr : public tasks::qp::ConstraintFunction<tasks::qp::GenInequality>
{
public:
	TorquePDConstr(const std::vector<rbd::MultiBody>& mbs,
		int robotIndex, const TorqueBound& tb);

	void computeMotorTorque(const Eigen::VectorXd& gains);
	const Eigen::VectorXd& motorTorque() const;
	void motorTorque(const std::vector<rbd::MultiBody>& mbs,
		std::vector<rbd::MultiBodyConfig>& mbcs) const;

	Eigen::MatrixXd getA() const
	{
		return A_;
	}

	Eigen::VectorXd getAL() const
	{
		return AL_;
	}

	Eigen::VectorXd getAU() const
	{
		return AU_;
	}

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData();
	}

	// Constraint
	virtual void updateNrVars(const std::vector<rbd::MultiBody>& mbs,
		const tasks::qp::SolverData& data);

	void update(const std::vector<rbd::MultiBody>& mb,
		const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data);

	// Description
	virtual std::string nameGenInEq() const;
	virtual std::string descGenInEq(const std::vector<rbd::MultiBody>& mbs, int line);

	// Inequality Constraint
	virtual int maxGenInEq() const;

	virtual const Eigen::MatrixXd& AGenInEq() const;
	virtual const Eigen::VectorXd& LowerGenInEq() const;
	virtual const Eigen::VectorXd& UpperGenInEq() const;

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
