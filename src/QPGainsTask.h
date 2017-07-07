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

// Solver
#include "QPGainsSolver.h"

// Eigen
#include <Eigen/Core>

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
// Forward declare ConstrData
class ConstrData;


class TASKS_DLLAPI GainsTask : public tasks::qp::Task
{
public:
	GainsTask(int robotIndex, double weight=1);

	std::pair<int, int> begin() const override
	{
		return std::make_pair(gainsBegin_, gainsBegin_);
	}

	// Add the pointer to the problem datas
	void configureTask(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	void update(const std::vector<rbd::MultiBody>& mbs,	const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	const Eigen::MatrixXd& Q() const override;
	const Eigen::VectorXd& C() const override;

private:
	int robotIndex_, gainsBegin_;
	std::shared_ptr<ConstrData> constrData_;

	Eigen::MatrixXd Q_;
	Eigen::VectorXd C_;
};


class TASKS_DLLAPI TorqueTask : public tasks::qp::Task
{
public:
	TorqueTask(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
				double weight);

	TorqueTask(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
				const Eigen::VectorXd& jointSelect,
				double weight);

	TorqueTask(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
				const std::string& efName,
				double weight);


	// Add the pointer to the problem datas
	void configureTask(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	virtual void updateNrVars(const std::vector<rbd::MultiBody>& mbs,
		const tasks::qp::SolverData& data);
	virtual void update(const std::vector<rbd::MultiBody>& mbs,
		const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data);

	virtual std::pair<int, int> begin() const
	{
		return std::make_pair(0, 0);
	}

	virtual const Eigen::MatrixXd& Q() const
	{
		return Q_;
	}

	virtual const Eigen::VectorXd& C() const
	{
		return C_;
	}

	virtual const Eigen::VectorXd& jointSelect() const
	{
		return jointSelector_;
	}

private:
	int robotIndex_;
	int alphaDBegin_, lambdaBegin_;
	std::shared_ptr<ConstrData> constrData_;
	Eigen::VectorXd jointSelector_;
	Eigen::MatrixXd Q_;
	Eigen::VectorXd C_;
};

} // namespace qpgains

} // namespace tasks
