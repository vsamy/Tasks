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

// associated header
#include "QPGainsTask.h"

// RBDyn
#include "RBDyn/MultiBodyConfig.h"

// ConstrData
#include "QPGainsConstrData.h"

namespace tasks
{

namespace qpgains
{

/**
	*												GainsTask
	*/


GainsTask::GainsTask(int robotIndex, double weight):
	tasks::qp::Task(weight),
	robotIndex_(robotIndex),
	gainsBegin_(-1),
	constrData_(nullptr),
	Q_(),
	C_()
{}

void GainsTask::updateNrVars(const std::vector<rbd::MultiBody>& /* mbs */,
	const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	gainsBegin_ = data.gainsBegin(robotIndex_);
	std::size_t gainsLength = data.gains(robotIndex_); // = length of K+B

	Q_.setIdentity(gainsLength, gainsLength);
	C_.setZero(gainsLength);
}

void GainsTask::update(const std::vector<rbd::MultiBody>& /* mbs */,
	const std::vector<rbd::MultiBodyConfig>& /* mbcs */, const tasks::qp::SolverData& /* data */)
{
	/*
	const rbd::MultiBodyConfig& mbc = mbcs[robotIndex_];
	const std::vector<int>& list = constrData_->gainsJointsList;
	std::size_t gainsLength = list.size();

	std::size_t line = 0;
	for(int jInd: list)
	{
		C_[line] = -2*mbc.jointGainsK[jInd][0];
		C_[line + gainsLength] = -2*mbc.jointGainsB[jInd][0];
	}
	*/
}

const Eigen::MatrixXd& GainsTask::Q() const
{
	return Q_;
}

const Eigen::VectorXd& GainsTask::C() const
{
	return C_;
}

} // namespace qpgains

} // namespace tasks
