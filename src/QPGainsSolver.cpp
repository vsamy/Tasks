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
#include "QPGainsSolver.h"

// includes
// std
#include <limits>
#include <utility>
#include <algorithm>
#include <stdexcept>

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

// Tasks
#include "GenQPSolver.h"

namespace tasks
{

namespace qpgains
{


QPGainsSolver::QPGainsSolver() :
	tasks::qp::QPSolver(),
	constrDataCompute_(),
	constrDataStock_()
{
}


void QPGainsSolver::addRobotToAdaptiveQP(const std::vector<rbd::MultiBody>& mbs, int robotIndex)
{
	if(std::find(robotIndex_.cbegin(), robotIndex_.cend(), robotIndex) == robotIndex_.cend())
	{
		robotIndex_.push_back(robotIndex);
		constrDataCompute_.emplace_back(std::make_unique<ConstrDataComputation>(mbs, robotIndex));
		constrDataStock_.emplace_back(std::make_shared<ConstrData>(mbs[robotIndex]));
	}
}


void QPGainsSolver::setRobotq0(int robotIndex, const std::vector<std::vector<double> > &q0)
{
	auto index = elemPosByRobotIndex(robotIndex, "setRobotq0");
	constrDataCompute_[index]->q0(q0);
}


void QPGainsSolver::setRobotAlpha0(int robotIndex, const std::vector<std::vector<double> > &alpha0)
{
	auto index = elemPosByRobotIndex(robotIndex, "setRobotAlpha0");
	constrDataCompute_[index]->alpha0(alpha0);
}


void QPGainsSolver::setGainsList(const std::vector<rbd::MultiBody> &mbs, int robotIndex, const std::vector<int> &gainsList)
{
	auto index = elemPosByRobotIndex(robotIndex, "setGainsList");
	constrDataStock_[index]->setGainsList(mbs[robotIndex], gainsList);
}


void QPGainsSolver::updateMbc(rbd::MultiBodyConfig& mbc, int robotIndex) const
{
	rbd::vectorToParam(solver_->result().segment(data_.alphaDBegin_[robotIndex], 
												 data_.alphaD_[robotIndex]),
					   mbc.alphaD);

	auto it = std::find(robotIndex_.cbegin(), robotIndex_.cend(), robotIndex);
	if(it != robotIndex_.cend())
	{
		auto index = std::distance(robotIndex_.cbegin(), it);
		const std::vector<int>& list = constrDataStock_[index]->gainsJointsList;
		int line = data_.alphaDBegin_[robotIndex];
		for (auto jIndex: list)
		{
			mbc.q[jIndex][0] = solver_->result()[line];
			++line;
		}

		line = data_.gainsBegin_[robotIndex];

		for (auto jIndex: list)
		{
			mbc.jointGainsK[jIndex][0] = solver_->result()[line];
			++line;
		}

		for (auto jIndex: list)
		{
			mbc.jointGainsB[jIndex][0] = solver_->result()[line];
			++line;
		}

		assert(line - data_.gainsBegin_[robotIndex] == data_.gains(robotIndex));
	}
}


void QPGainsSolver::nrVars(const std::vector<rbd::MultiBody>& mbs, std::vector<tasks::qp::UnilateralContact> uni,
	std::vector<tasks::qp::BilateralContact> bi)
{
	data_.alphaD_.resize(mbs.size());
	data_.alphaDBegin_.resize(mbs.size());

	data_.uniCont_ = std::move(uni);
	data_.biCont_ = std::move(bi);

	int nrContacts = data_.nrContacts();

	data_.lambda_.resize(nrContacts);
	data_.lambdaBegin_.resize(nrContacts);

	data_.gains_.resize(mbs.size());
	data_.gainsBegin_.resize(mbs.size());

	data_.mobileRobotIndex_.clear();
	data_.normalAccB_.resize(mbs.size());

	int cumAlphaD = 0;
	for(std::size_t r = 0; r < mbs.size(); ++r)
	{
		const rbd::MultiBody& mb = mbs[r];
		data_.alphaD_[r] = mb.nrDof();
		data_.alphaDBegin_[r] = cumAlphaD;
		data_.normalAccB_[r].resize(mb.nrBodies(),
			sva::MotionVecd(Eigen::Vector6d::Zero()));
		cumAlphaD += mb.nrDof();
		if(mb.nrDof() > 0)
		{
			data_.mobileRobotIndex_.push_back(int(r));
		}
	}
	data_.totalAlphaD_ = cumAlphaD;

	int cumLambda = cumAlphaD;
	int cIndex = 0;
	data_.allCont_.clear();
	// counting unilateral contact
	for(const tasks::qp::UnilateralContact& c: data_.uniCont_)
	{
		data_.lambdaBegin_[cIndex] = cumLambda;
		int lambda = 0;
		for(std::size_t p = 0; p < c.r1Points.size(); ++p)
		{
			lambda += c.nrLambda(int(p));
		}
		data_.lambda_[cIndex] = lambda;
		cumLambda += lambda;
		++cIndex;

		data_.allCont_.emplace_back(c);
	}
	data_.nrUniLambda_ = cumLambda - cumAlphaD;

	// counting bilateral contact
	for(const tasks::qp::BilateralContact& c: data_.biCont_)
	{
		data_.lambdaBegin_[cIndex] = cumLambda;
		int lambda = 0;
		for(std::size_t p = 0; p < c.r1Points.size(); ++p)
		{
			lambda += c.nrLambda(int(p));
		}
		data_.lambda_[cIndex] = lambda;
		cumLambda += lambda;
		++cIndex;

		data_.allCont_.emplace_back(c);
	}
	data_.nrBiLambda_ = cumLambda - data_.nrUniLambda_ - cumAlphaD;
	data_.totalLambda_ = data_.nrUniLambda_ + data_.nrBiLambda_;

	for(int rI: robotIndex_)
	{
		constrDataStock_[rI]->updateNrVars(mbs[rI], data_);
		constrDataCompute_[rI]->updateNrVars(mbs, data_);
	}
	int cumGains = cumLambda;
	for(std::size_t r = 0; r < mbs.size(); ++r)
		data_.gains_[r] = 0;

	for(int rI: robotIndex_)
	{
		data_.gainsBegin_[rI] = cumGains;
		data_.gains_[rI] = 2*static_cast<int>(constrDataStock_[rI]->gainsLinesList.size());
		cumGains += 2*static_cast<int>(constrDataStock_[rI]->gainsLinesList.size());
	}
	data_.totalGains_ = cumGains - cumLambda;

	data_.nrVars_ = data_.totalAlphaD_ + data_.totalLambda_ + data_.totalGains_;

	for(tasks::qp::Task* t: tasks_)
		t->updateNrVars(mbs, data_);

	for(tasks::qp::Constraint* c: constr_)
		c->updateNrVars(mbs, data_);

	solver_->updateSize(data_.nrVars_, maxEqLines_, maxInEqLines_, maxGenInEqLines_);
}


const std::shared_ptr<ConstrData> QPGainsSolver::getConstrData(int robotIndex) const
{
	auto index = elemPosByRobotIndex(robotIndex, "getConstrData");
	return constrDataStock_[index];
}


Eigen::VectorXd QPGainsSolver::gainsVec() const
{
	return solver_->result().segment(data_.gainsBegin(), data_.totalGains_);
}


Eigen::VectorXd QPGainsSolver::gainsVec(int robotIndex) const
{
	return solver_->result().segment(data_.gainsBegin_[robotIndex],	data_.gains_[robotIndex]);
}


void QPGainsSolver::preUpdate(const std::vector<rbd::MultiBody> &mbs, const std::vector<rbd::MultiBodyConfig> &mbcs)
{
	data_.computeNormalAccB(mbs, mbcs);
	for(int rI: robotIndex_)
		constrDataCompute_[rI]->computeGainsConstrMatrices(mbs, mbcs, constrDataStock_[rI]);

	for(std::size_t i = 0; i < constr_.size(); ++i)
		constr_[i]->update(mbs, mbcs, data_);

	for(std::size_t i = 0; i < tasks_.size(); ++i)
		tasks_[i]->update(mbs, mbcs, data_);

	solver_->updateMatrix(tasks_, eqConstr_, inEqConstr_, genInEqConstr_, boundConstr_);
}


std::size_t QPGainsSolver::elemPosByRobotIndex(int robotIndex, const std::string& funName) const
{
	auto it = std::find(robotIndex_.cbegin(), robotIndex_.cend(), robotIndex);
	if(it != robotIndex_.cend())
		return std::distance(robotIndex_.cbegin(), it);
	else
		throw(std::runtime_error("Bad robot index in " + funName));
}


} // namespace qpgains

} // namespace tasks
