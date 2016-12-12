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
// includes
// Eigen
#include <unsupported/Eigen/Polynomials>

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

// Tasks
#include "Bounds.h"
#include "utils.h"
#include "QPGainsConstrData.h"


namespace tasks
{

namespace qpgains
{


/**
	*												BoundGainsConstr
	*/


BoundGainsConstr::BoundGainsConstr(const std::vector<rbd::MultiBody>& /* mbs */,
	int robotIndex) :
	robotIndex_(robotIndex),
	gainsBegin_(-1),
	constrData_(nullptr),
	lower_(),
	upper_()
{
}


void BoundGainsConstr::updateNrVars(const std::vector<rbd::MultiBody>& /* mbs */,
	const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	gainsBegin_ = data.gainsBegin(robotIndex_);
	int nrVars = data.gains(robotIndex_);
	lower_.setConstant(nrVars, 0);
	upper_.setConstant(nrVars, std::numeric_limits<double>::infinity());
}

void BoundGainsConstr::update(const std::vector<rbd::MultiBody>& /* mbs */,
	const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
	const tasks::qp::SolverData& /* data */)
{
}


std::string BoundGainsConstr::nameBound() const
{
	return "BoundGainsConstr";
}


std::string BoundGainsConstr::descBound(const std::vector<rbd::MultiBody>& mbs,
	int line)
{
	std::size_t goodLine = line % constrData_->gainsLinesList.size();
	int jIndex = tasks::qp::findJointFromVector(mbs[robotIndex_], constrData_->gainsLinesList[goodLine], false);
	return std::string("Joint: ") + mbs[robotIndex_].joint(jIndex).name();
}


int BoundGainsConstr::beginVar() const
{
	return gainsBegin_;
}


const Eigen::VectorXd& BoundGainsConstr::Lower() const
{
	return lower_;
}


const Eigen::VectorXd& BoundGainsConstr::Upper() const
{
	return upper_;
}




/**
	*															MotionGainsConstr
	*/


MotionGainsConstr::MotionGainsConstr(const std::vector<rbd::MultiBody>& mbs,
	int robotIndex, const TorqueBound& tb):
	robotIndex_(robotIndex),
	alphaDBegin_(-1),
	nrDof_(mbs[robotIndex_].nrDof()),
	lambdaBegin_(-1),
	nrLambda_(0),
	constrData_(nullptr),
	nrLines_(0),
	curTorque_(nrDof_),
	A_(),
	AL_(),
	AU_(),
	torqueL_(mbs[robotIndex].nrDof()),
	torqueU_(mbs[robotIndex].nrDof())
{
	assert(std::size_t(robotIndex_) < mbs.size() && robotIndex_ >= 0);
	rbd::paramToVector(tb.lTorqueBound, torqueL_);
	rbd::paramToVector(tb.uTorqueBound, torqueU_);
}


void MotionGainsConstr::computeTorque(const Eigen::VectorXd& alphaD, const Eigen::VectorXd& lambda)
{
	Eigen::VectorXd torqueSelect(constrData_->noGainsLinesList.size());
	Eigen::VectorXd CSelect(constrData_->noGainsLinesList.size());
	constrData_->insertSpecificLines(constrData_->noGainsLinesList,
		constrData_->C, CSelect);

	torqueSelect = A_.block(0, alphaDBegin_, nrLines_, nrDof_)*alphaD.segment(alphaDBegin_, nrDof_) +\
		CSelect + A_.block(0, lambdaBegin_, nrLines_, nrLambda_)*lambda;

	curTorque_.setZero();
	int pos = 0;
	for(int line: constrData_->noGainsLinesList)
	{
		curTorque_[line] = torqueSelect[pos];
		++pos;
	}
}


const Eigen::VectorXd& MotionGainsConstr::torque() const
{
	return curTorque_;
}


void MotionGainsConstr::torque(const std::vector<rbd::MultiBody>& /* mbs */,
	std::vector<rbd::MultiBodyConfig>& mbcs) const
{
	rbd::vectorToParam(curTorque_, mbcs[robotIndex_].jointTorque);
}


void MotionGainsConstr::updateNrVars(const std::vector<rbd::MultiBody>& /* mbs */,
	const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	alphaDBegin_ = data.alphaDBegin(robotIndex_);
	lambdaBegin_ = data.lambdaBegin();
	nrLambda_ = data.totalLambda();

	nrLines_ = constrData_->noGainsLinesList.size();

	AL_.setZero(nrLines_);
	AU_.setZero(nrLines_);
	A_.setZero(nrLines_, data.nrVars());
}


void MotionGainsConstr::update(const std::vector<rbd::MultiBody>& /* mbs */,
	const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
	const tasks::qp::SolverData& /* data */)
{
	using namespace Eigen;

	if(constrData_->noGainsLinesList.size() == 0)
		return; // If all motors of all joints are used do nothing. This never happend when there is a free-flyer

	// tauMin -C <= H*alphaD - J^t G lambda <= tauMax - C

	// A = [H JtG 0 0]
	constrData_->insertSpecificLines(constrData_->noGainsLinesList,
		constrData_->H, A_.block(0, alphaDBegin_, nrLines_, nrDof_));

	constrData_->insertSpecificLines(constrData_->noGainsLinesList,
		constrData_->minusJtG, A_.block(0, lambdaBegin_, nrLines_, nrLambda_));


	// b = tau - C
	constrData_->insertSpecificLines(constrData_->noGainsLinesList,
		torqueL_ - constrData_->C, AL_);
	constrData_->insertSpecificLines(constrData_->noGainsLinesList,
		torqueU_ - constrData_->C, AU_);
}


int MotionGainsConstr::maxGenInEq() const
{
	return static_cast<int>(nrLines_);
}


const Eigen::MatrixXd& MotionGainsConstr::AGenInEq() const
{
	return A_;
}


const Eigen::VectorXd& MotionGainsConstr::LowerGenInEq() const
{
	return AL_;
}


const Eigen::VectorXd& MotionGainsConstr::UpperGenInEq() const
{
	return AU_;
}


std::string MotionGainsConstr::nameGenInEq() const
{
	return "MotionGainsConstr";
}


std::string MotionGainsConstr::descGenInEq(const std::vector<rbd::MultiBody>& mbs,
	int line)
{
	int jIndex = tasks::qp::findJointFromVector(mbs[robotIndex_], constrData_->noGainsLinesList[line], true);
	return std::string("Joint: ") + mbs[robotIndex_].joint(jIndex).name();
}


/**
	*															MotionGainsEqualConstr
	*/


MotionGainsEqualConstr::MotionGainsEqualConstr(const std::vector<rbd::MultiBody>& mbs,
	int robotIndex):
	robotIndex_(robotIndex),
	alphaDBegin_(-1),
	nrDof_(mbs[robotIndex_].nrDof()),
	nrLambda_(-1),
	lambdaBegin_(-1),
	gainsBegin_(-1),
	constrData_(nullptr),
	nrLines_(0),
	curTorque_(nrDof_),
	A_(),
	b_()
{
	assert(std::size_t(robotIndex_) < mbs.size() && robotIndex_ >= 0);
}


void MotionGainsEqualConstr::updateNrVars(const std::vector<rbd::MultiBody>& /* mbs */,
	const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	alphaDBegin_ = data.alphaDBegin(robotIndex_);
	gainsBegin_ = data.gainsBegin(robotIndex_);
	lambdaBegin_ = data.lambdaBegin();
	nrLambda_ = data.totalLambda();

	nrLines_ = constrData_->gainsLinesList.size(); // = length of gains

	A_.setZero(nrLines_, data.nrVars());
	b_.setZero(nrLines_);
}


void MotionGainsEqualConstr::update(const std::vector<rbd::MultiBody>& /* mbs */,
	const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
	const tasks::qp::SolverData& /* data */)
{
	if(constrData_->gainsLinesList.size() == 0)
		return; // If no motor is used do nothing

	// H*alphaD - J^t G lambda + K*(q - q0) + B*alpha = -C

	// A = [H -JtG -e -de]
	constrData_->insertSpecificLines(constrData_->gainsLinesList,
		constrData_->H, A_.block(0, alphaDBegin_, nrLines_, nrDof_));

	constrData_->insertSpecificLines(constrData_->gainsLinesList,
		constrData_->minusJtG, A_.block(0, lambdaBegin_, nrLines_, nrLambda_));

	// We can't use insertSpecificLines for error and derror
	// Because we want to insert a vecteur in matrix
	for(std::size_t i = 0; i < nrLines_; ++i)
		A_(i, gainsBegin_+i) = -constrData_->error[i]; // <=> A_.block(0, gainsBegin_, nrLines_, nrLines_) = -constrData_->error.asDiagonal()

	for(std::size_t i = 0; i < nrLines_; ++i)
		A_(i, gainsBegin_+nrLines_+i) = -constrData_->derror[i]; // <=> A_.block(0, gainsBegin_ + nrLines_, nrLines_, nrLines_) = -constrData_->derror.asDiagonal()

	// BEq = -C
	constrData_->insertSpecificLines(constrData_->gainsLinesList,
		-constrData_->C, b_);
}

int MotionGainsEqualConstr::maxEq() const
{
	return static_cast<int>(nrLines_);
}


const Eigen::MatrixXd& MotionGainsEqualConstr::AEq() const
{
	return A_;
}


const Eigen::VectorXd& MotionGainsEqualConstr::bEq() const
{
	return b_;
}


std::string MotionGainsEqualConstr::nameEq() const
{
	return "MotionGainsEqualConstr";
}


std::string MotionGainsEqualConstr::descEq(const std::vector<rbd::MultiBody>& mbs,
	int line)
{
	int jIndex = tasks::qp::findJointFromVector(mbs[robotIndex_], constrData_->gainsLinesList[line], false);
	return std::string("Joint: ") + mbs[robotIndex_].joint(jIndex).name();
}


/**
	*															TorquePDConstr
	*/


TorquePDConstr::TorquePDConstr(const std::vector<rbd::MultiBody>& mbs,
	int robotIndex, const TorqueBound& tb):
	robotIndex_(robotIndex),
	nrDof_(mbs[robotIndex_].nrDof()),
	gainsBegin_(-1),
	constrData_(nullptr),
	nrLines_(0),
	curTorque_(nrDof_),
	torqueL_(mbs[robotIndex].nrDof()),
	torqueU_(mbs[robotIndex].nrDof()),
	A_(),
	AL_(),
	AU_()
{
	assert(std::size_t(robotIndex_) < mbs.size() && robotIndex_ >= 0);
	rbd::paramToVector(tb.lTorqueBound, torqueL_);
	rbd::paramToVector(tb.uTorqueBound, torqueU_);
}


void TorquePDConstr::computeMotorTorque(const Eigen::VectorXd& gains)
{
	const Eigen::VectorXd& e = constrData_->error;
	const Eigen::VectorXd& de = constrData_->derror;
	const Eigen::VectorXd& K = gains.segment(0, nrLines_);
	const Eigen::VectorXd& B = gains.segment(nrLines_, nrLines_);

	curTorque_.setZero();
	int pos = 0;
	for(auto line: constrData_->gainsLinesList) // gainsLinesList is sorted by joint Index
	{
		curTorque_[line] = K[pos]*e[pos] + B[pos]*de[pos];
		++pos;
	}
}


const Eigen::VectorXd& TorquePDConstr::motorTorque() const
{
	return curTorque_;
}


void TorquePDConstr::motorTorque(const std::vector<rbd::MultiBody>& /* mbs */,
	std::vector<rbd::MultiBodyConfig>& mbcs) const
{
	rbd::vectorToParam(curTorque_, mbcs[robotIndex_].jointMotorTorque);
}


void TorquePDConstr::updateNrVars(const std::vector<rbd::MultiBody>& /* mbs */,
	const tasks::qp::SolverData& data)
{
	assert(constrData_ != nullptr);
	gainsBegin_ = data.gainsBegin(robotIndex_);
	nrLines_ = constrData_->gainsLinesList.size(); // = gains' length

	/// @todo don't use nrDof and totalLamdba but max dof of a jacobian
	/// and max lambda of a contact.
	A_.setZero(nrLines_, data.nrVars());
	AL_.setZero(nrLines_);
	AU_.setZero(nrLines_);
}


void TorquePDConstr::update(const std::vector<rbd::MultiBody>& /* mbs */,
	const std::vector<rbd::MultiBodyConfig>& /* mbcs */,
	const tasks::qp::SolverData& /* data */)
{
	if(constrData_->gainsLinesList.empty())
		return; // If no motor is used do nothing

	// tauMin <= K*(q0 - q) - B*alpha <= tauMax

	// A = [0 0 e de]

	// We can't use insertSpecificLines for error and derror
	// Because we want to insert a vecteur in matrix
	for(std::size_t i = 0; i < nrLines_; ++i)
		A_(i, gainsBegin_+i) = constrData_->error[i]; // <=> A_.block(0, gainsBegin_, nrLines_, nrLines_) = constrData_->error.asDiagonal()

	for(std::size_t i = 0; i < nrLines_; ++i)
		A_(i, gainsBegin_+nrLines_+i) = constrData_->derror[i]; // <=> A_.block(0, gainsBegin_ + nrLines_, nrLines_, nrLines_) = constrData_->derror.asDiagonal()

	// b = tau_bound
	constrData_->insertSpecificLines(constrData_->gainsLinesList,
		torqueL_, AL_);
	constrData_->insertSpecificLines(constrData_->gainsLinesList,
		torqueU_, AU_);
}

int TorquePDConstr::maxGenInEq() const
{
	return static_cast<int>(nrLines_);
}


const Eigen::MatrixXd& TorquePDConstr::AGenInEq() const
{
	return A_;
}


const Eigen::VectorXd& TorquePDConstr::LowerGenInEq() const
{
	return AL_;
}


const Eigen::VectorXd& TorquePDConstr::UpperGenInEq() const
{
	return AU_;
}


std::string TorquePDConstr::nameGenInEq() const
{
	return "TorquePDConstr";
}


std::string TorquePDConstr::descGenInEq(const std::vector<rbd::MultiBody>& mbs,
	int line)
{
	int jIndex = tasks::qp::findJointFromVector(mbs[robotIndex_], constrData_->gainsLinesList[line], false);
	return std::string("Joint: ") + mbs[robotIndex_].joint(jIndex).name();
}


} // namespace qpgains

} // namespace tasks
