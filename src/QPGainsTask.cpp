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

void GainsTask::update(const std::vector<rbd::MultiBody>& /* mbs */, const std::vector<rbd::MultiBodyConfig>& /* mbcs */, 
	const tasks::qp::SolverData& /* data */)
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


/**
	*												TorqueTask
	*/

TorqueTask::TorqueTask(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
                       double weight):
  Task(weight),
  robotIndex_(robotIndex),
  alphaDBegin_(-1),
  lambdaBegin_(-1),
  jointSelector_(mbs[robotIndex].nrDof()),
  Q_(mbs[robotIndex].nrDof(), mbs[robotIndex].nrDof()),
  C_(mbs[robotIndex].nrDof())
{
  jointSelector_.setOnes();
}

TorqueTask::TorqueTask(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
                       const Eigen::VectorXd& jointSelect,
                       double weight):
  Task(weight),
  robotIndex_(robotIndex),
  alphaDBegin_(-1),
  lambdaBegin_(-1),
  jointSelector_(jointSelect),
  Q_(mbs[robotIndex].nrDof(), mbs[robotIndex].nrDof()),
  C_(mbs[robotIndex].nrDof())
{
}

TorqueTask::TorqueTask(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
                       const std::string& efName,
                       double weight):
  Task(weight),
  robotIndex_(robotIndex),
  alphaDBegin_(-1),
  lambdaBegin_(-1),
  jointSelector_(mbs[robotIndex].nrDof()),
  Q_(mbs[robotIndex].nrDof(), mbs[robotIndex].nrDof()),
  C_(mbs[robotIndex].nrDof())
{
  rbd::Jacobian jac(mbs[robotIndex], efName);
  jointSelector_.setZero();
  for(auto i : jac.jointsPath())
  {
    //Do not add root joint !
    if(i != 0)
    {
    jointSelector_.segment(mbs[robotIndex].jointPosInDof(i),
                           mbs[robotIndex].joint(i).dof()).setOnes();
    }
  }
}

void TorqueTask::updateNrVars(const std::vector<rbd::MultiBody>& /* mbs */,
                              const tasks::qp::SolverData& data)
{
  assert(constrData_ != nullptr);
  if (constrData_->gainsJointsList.size() > 0)
  	throw std::runtime_error("Torque Task does not yet work with adaptive QP.");
  alphaDBegin_ = data.alphaDBegin(robotIndex_);
  lambdaBegin_ = data.lambdaBegin();
  Q_.resize(data.nrVars(), data.nrVars());
  C_.resize(data.nrVars());
}

void TorqueTask::update(const std::vector<rbd::MultiBody>& /* mbs */,
                        const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
                        const tasks::qp::SolverData& data)
{
  Eigen::MatrixXd motionMat(data.totalAlphaD(), data.nrVars());
  motionMat << constrData_->H, constrData_->minusJtG;
  Q_.noalias() = motionMat.transpose()*jointSelector_.asDiagonal()*motionMat;
  C_.noalias() = constrData_->C.transpose()*jointSelector_.asDiagonal()*motionMat;
  //C_.setZero();
}

} // namespace qpgains

} // namespace tasks
