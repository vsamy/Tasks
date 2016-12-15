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


} // namespace qpgains

} // namespace tasks
