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
#include "QPGainsConstr.h"

// Datas
#include "QPGainsConstrData.h"

// Eigen
#include <unsupported/Eigen/Polynomials>

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

// Tasks
#include "Bounds.h"
#include "utils.h"


namespace tasks
{

namespace qpgains
{


/**
	*												OverDampedGainsConstr
	*/


OverDampedGainsConstr::OverDampedGainsConstr(const std::vector<rbd::MultiBody>& /* mbs */,
	int robotIndex) :
	robotIndex_(robotIndex),
	nrLines_(0),
	constrData_(nullptr),
	Aineq_(),
	bineq_()
{
}


void OverDampedGainsConstr::updateNrVars(const std::vector<rbd::MultiBody>& /* mbs */, const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	int gainsBegin = data.gainsBegin(robotIndex_);
	nrLines_ = data.gains(robotIndex_) / 2;

	// Aineq = [0 0 0.025 -1]
	Aineq_.resize(nrLines_, data.nrVars());
	Aineq_.leftCols(gainsBegin).setZero();
	Aineq_.block(0, gainsBegin, nrLines_, gainsBegin + nrLines_).noalias() = Eigen::MatrixXd::Identity(nrLines_, nrLines_) * 0.025;
	Aineq_.rightCols(nrLines_) = -Eigen::MatrixXd::Identity(nrLines_, nrLines_);

	// bineq = [-5]
	bineq_.setConstant(nrLines_, -5);
}

void OverDampedGainsConstr::update(const std::vector<rbd::MultiBody>& /* mbs */,
	const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
	const tasks::qp::SolverData& /* data */)
{
	// Linearization around old K?
}


std::string OverDampedGainsConstr::nameInEq() const
{
	return "OverDampedGainsConstr";
}


std::string OverDampedGainsConstr::descInEq(const std::vector<rbd::MultiBody>& mbs,
	int line)
{
	return std::string("Joint: ") + mbs[robotIndex_].joint(constrData_->gainsJointsList[line]).name();
}


int OverDampedGainsConstr::maxInEq() const
{
	return static_cast<int>(nrLines_);
}


const Eigen::MatrixXd& OverDampedGainsConstr::AInEq() const
{
	return Aineq_;
}


const Eigen::VectorXd& OverDampedGainsConstr::bInEq() const
{
	return bineq_;
}



/**
	*												JointLimitsNoGainsConstr
	*/


JointLimitsNoGainsConstr::JointLimitsNoGainsConstr(const std::vector<rbd::MultiBody>& mbs,
	int robotIndex, QBound bound, double step):
	robotIndex_(robotIndex),
	alphaDBegin_(-1),
	step_(step),
	constrData_(nullptr),
	qMin_(),
	qMax_(),
	qVec_(),
	alphaVec_(),
	lower_(),
	upper_()
{
	assert(std::size_t(robotIndex_) < mbs.size() && robotIndex_ >= 0);

	const rbd::MultiBody& mb = mbs[robotIndex_];

	// Remove alphaDOffset hack from original version (tasks::qp::JointLimitsConstr)
	qMin_.resize(mb.nrParams());
	qMax_.resize(mb.nrParams());
	qVec_.resize(mb.nrParams());
	alphaVec_.resize(mb.nrDof());

	rbd::paramToVector(bound.lQBound, qMin_);
	rbd::paramToVector(bound.uQBound, qMax_);
}


void JointLimitsNoGainsConstr::updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	const rbd::MultiBody& mb = mbs[robotIndex_];
	alphaDBegin_ = data.alphaDBegin(robotIndex_);

	// reset lower and upper in case of gainsList changes.
	lower_.setConstant(mb.nrDof(), -std::numeric_limits<double>::infinity());
	upper_.setConstant(mb.nrDof(), std::numeric_limits<double>::infinity());
}


void JointLimitsNoGainsConstr::update(const std::vector<rbd::MultiBody>& /* mbs */, const std::vector<rbd::MultiBodyConfig>& mbcs,
	const tasks::qp::SolverData& /* data */)
{
	const rbd::MultiBodyConfig& mbc = mbcs[robotIndex_];

	double dts = step_*step_*0.5;

	rbd::paramToVector(mbc.q, qVec_);
	rbd::paramToVector(mbc.alpha, alphaVec_);

	Eigen::VectorXd derivativePart = qVec_ + alphaVec_*step_;
	for(auto it = constrData_->noGainsLinesList.cbegin(); it != constrData_->noGainsLinesList.cend(); ++it)
	{
		lower_(*it) = qMin_(*it) - derivativePart(*it);
		upper_(*it) = qMax_(*it) - derivativePart(*it);
	}

	lower_ /= dts;
	upper_ /= dts;
}


std::string JointLimitsNoGainsConstr::nameBound() const
{
	return "JointLimitsNoGainsConstr";
}


std::string JointLimitsNoGainsConstr::descBound(const std::vector<rbd::MultiBody>& mbs, int line)
{
	int jIndex = tasks::qp::findJointFromVector(mbs[robotIndex_], line, true);
	return std::string("Joint: ") + mbs[robotIndex_].joint(jIndex).name();
}


int JointLimitsNoGainsConstr::beginVar() const
{
	return alphaDBegin_;
}


const Eigen::VectorXd& JointLimitsNoGainsConstr::Lower() const
{
	return lower_;
}


const Eigen::VectorXd& JointLimitsNoGainsConstr::Upper() const
{
	return upper_;
}


/**
	*												DamperJointLimitsNoGainsConstr
	*/


DamperJointLimitsNoGainsConstr::DamperJointLimitsNoGainsConstr(
	const std::vector<rbd::MultiBody>& mbs, int robotIndex,
	const QBound& qBound, const AlphaBound& aBound,
	double interPercent, double securityPercent,
	double damperOffset, double step):
	robotIndex_(robotIndex),
	alphaDBegin_(-1),
	data_(),
	constrData_(nullptr),
	lower_(mbs[robotIndex].nrDof()),
	upper_(mbs[robotIndex].nrDof()),
	step_(step),
	damperOff_(damperOffset)
{
	assert(std::size_t(robotIndex_) < mbs.size() && robotIndex_ >= 0);

	const rbd::MultiBody& mb = mbs[robotIndex_];

	for(int i = 0; i < mb.nrJoints(); ++i)
	{
		if(mb.joint(i).dof() == 1)
		{
			double dist = (qBound.uQBound[i][0] - qBound.lQBound[i][0]);
			data_.emplace_back(qBound.lQBound[i][0], qBound.uQBound[i][0],
				aBound.lAlphaBound[i][0], aBound.uAlphaBound[i][0],
				dist*interPercent, dist*securityPercent,
				mb.jointPosInDof(i), i);
		}
	}
}


void DamperJointLimitsNoGainsConstr::updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	const rbd::MultiBody& mb = mbs[robotIndex_];
	alphaDBegin_ = data.alphaDBegin(robotIndex_);

	// reset lower and upper in case of gainsList changes.
	lower_.setConstant(mb.nrDof(), -std::numeric_limits<double>::infinity());
	upper_.setConstant(mb.nrDof(), std::numeric_limits<double>::infinity());
}


void DamperJointLimitsNoGainsConstr::update(const std::vector<rbd::MultiBody>& /* mbs */, const std::vector<rbd::MultiBodyConfig>& mbcs, 
	const tasks::qp::SolverData& /* data */)
{
	const rbd::MultiBodyConfig& mbc = mbcs[robotIndex_];

	for(DampData& d: data_)
	{
		// Do not perform a damping limit for joints that are using the adaptive qp.
		if(std::find(constrData_->noGainsLinesList.cbegin(), constrData_->noGainsLinesList.cend(), d.alphaDBegin) == constrData_->noGainsLinesList.cend())
			continue;

		double ld = mbc.q[d.jointIndex][0] - d.min;
		double ud = d.max - mbc.q[d.jointIndex][0];
		double alpha = mbc.alpha[d.jointIndex][0];

		lower_[d.alphaDBegin] = (d.minVel - alpha)/step_;
		upper_[d.alphaDBegin] = (d.maxVel - alpha)/step_;

		if(ld < d.iDist)
		{
			// damper(dist) < alpha
			// dist > 0 -> negative < alpha -> joint angle can decrease
			// dist < 0 -> positive < alpha -> joint angle must increase
			if(d.state != DampData::Low)
			{
				d.damping =
					std::abs(computeDamping(alpha, ld, d.iDist, d.sDist)) + damperOff_;
				d.state = DampData::Low;
			}

			double damper = -computeDamper(ld, d.iDist, d.sDist, d.damping);
			lower_[d.alphaDBegin] = std::max((damper - alpha)/step_,
				lower_[d.alphaDBegin]);
		}
		else if(ud < d.iDist)
		{
			// alpha < damper(dist)
			// dist > 0 -> alpha < positive -> joint angle can increase
			// dist < 0 -> alpha < negative -> joint angle must decrease
			if(d.state != DampData::Upp)
			{
				d.damping =
					std::abs(computeDamping(alpha, ud, d.iDist, d.sDist)) + damperOff_;
				d.state = DampData::Upp;
			}

			double damper = computeDamper(ud, d.iDist, d.sDist, d.damping);
			upper_[d.alphaDBegin] = std::min((damper - alpha)/step_,
				upper_[d.alphaDBegin]);
		}
		else
		{
			d.state = DampData::Free;
		}
	}
}


std::string DamperJointLimitsNoGainsConstr::nameBound() const
{
	return "DamperJointLimitsNoGainsConstr";
}


std::string DamperJointLimitsNoGainsConstr::descBound(
	const std::vector<rbd::MultiBody>& mbs, int line)
{
	int jIndex = tasks::qp::findJointFromVector(mbs[robotIndex_], line, true);
	return std::string("Joint: ") + mbs[robotIndex_].joint(jIndex).name();
}


int DamperJointLimitsNoGainsConstr::beginVar() const
{
	return alphaDBegin_;
}


const Eigen::VectorXd& DamperJointLimitsNoGainsConstr::Lower() const
{
	return lower_;
}


const Eigen::VectorXd& DamperJointLimitsNoGainsConstr::Upper() const
{
	return upper_;
}


double DamperJointLimitsNoGainsConstr::computeDamping(double alpha, double dist,
	double iDist, double sDist)
{
	return ((iDist - sDist)/(dist - sDist))*alpha;
}


double DamperJointLimitsNoGainsConstr::computeDamper(double dist,
	double iDist, double sDist ,double damping)
{
	return damping*((dist - sDist)/(iDist - sDist));
}



} // namespace qpgains

} // namespace tasks