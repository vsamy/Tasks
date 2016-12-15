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


#include "QPGainsConstrData.h"

// SolverData and FictionCone
#include "QPSolverData.h"

// RBDyn
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>

// std
#include <algorithm>


namespace tasks
{

namespace qpgains
{


/*
 *							ConstrData
 */


ConstrData::ConstrData(const rbd::MultiBody& mb) :
	gainsJointsList(),
	gainsLinesList(),
	noGainsLinesList(mb.nrDof()),
	H(),
	C(),
	minusJtG(),
	error(),
	derror()
{
	std::iota(noGainsLinesList.begin(), noGainsLinesList.end(), 0);
}

void ConstrData::setGainsList(const rbd::MultiBody& mb,
	const std::vector<int>& jointsList)
{
	gainsJointsList.resize(jointsList.size());
	std::copy(jointsList.begin(), jointsList.end(), gainsJointsList.begin());
	std::sort(gainsJointsList.begin(), gainsJointsList.end());

	// Check that index in [0,nrJoints)
	assert(*std::min_element(gainsJointsList.begin(), gainsJointsList.end()) >= 0);
	assert(*std::max_element(gainsJointsList.begin(), gainsJointsList.end()) <= mb.nrJoints() -1);
	// Check that the dof of the joints are 1 (Two motors can't act on one joint)
	assert(std::all_of(gainsJointsList.begin(), gainsJointsList.end(), [&mb](int i)
	{
		return mb.joint(i).dof() == 1;
	}));

	std::size_t pos = 0;
	int line = 0;
	gainsLinesList.clear();
	noGainsLinesList.clear();

	auto addLines = [mb, &line](int i, std::vector<int>& vec)
	{
		for(int dof = 0; dof < mb.joint(i).dof(); ++dof)
		{
			vec.push_back(line);
			line += 1;
		}
	};

	for(int ind = 0; ind < mb.nrJoints(); ++ind)
	{
		if(ind == gainsJointsList[pos])
		{
			addLines(ind, gainsLinesList);
			pos += 1;
		}
		else
		{
			addLines(ind, noGainsLinesList);
		}
	}
}

void ConstrData::updateNrVars(const rbd::MultiBody& mb,	const tasks::qp::SolverData& data)
{
	H.setZero(mb.nrDof(), mb.nrDof());
	minusJtG.setZero(mb.nrDof(), data.totalLambda());
	C.setZero(mb.nrDof());
	error.resize(gainsLinesList.size());
	derror.resize(gainsLinesList.size());
}


void ConstrData::insertSpecificLines(std::vector<int>& lineList,
	const Eigen::MatrixXd &matExtract, Eigen::Ref<Eigen::MatrixXd> matOut)
{
	// Check MatOut is consistent with the linelist
	assert(lineList.size() == static_cast<std::size_t>(matOut.rows()));
	// Check matExtract is consistent with the linelist
	assert(lineList.size() <= static_cast<std::size_t>(matExtract.rows()));
	assert(*std::max_element(lineList.begin(), lineList.end()) <= matExtract.rows() - 1);
	// Check that matOut and matExtract have the same numbers of columns
	assert(matOut.cols() == matExtract.cols());

	for(std::size_t ind = 0; ind < lineList.size(); ++ind)
		matOut.row(ind) = matExtract.row(lineList[ind]);
}


/*
 *							ConstrDataComputation
 */


ConstrDataComputation::ContactData::ContactData(const rbd::MultiBody& mb,
	const std::string& bodyName, int lB,
	std::vector<Eigen::Vector3d> pts,
	const std::vector<tasks::qp::FrictionCone>& cones):
	bodyIndex(),
	lambdaBegin(lB),
	jac(mb, bodyName),
	points(std::move(pts)),
	minusGenerators(cones.size())
{
	bodyIndex = jac.jointsPath().back();
	for(std::size_t i = 0; i < cones.size(); ++i)
	{
		minusGenerators[i].resize(3, cones[i].generators.size());
		for(std::size_t j = 0; j < cones[i].generators.size(); ++j)
		{
			minusGenerators[i].col(j) = -cones[i].generators[j];
		}
	}
}


ConstrDataComputation::ConstrDataComputation(const std::vector<rbd::MultiBody>& mbs,
	int robotIndex) :
	robotIndex_(robotIndex),
	lambdaBegin_(-1),
	nrParam_(mbs[robotIndex_].nrParams()),
	nrDof_(mbs[robotIndex_].nrDof()),
	fd_(mbs[robotIndex_]),
	q0_(mbs[robotIndex_].nrJoints()),
	alpha0_(mbs[robotIndex_].nrJoints()),
	fullJacLambda_(),
	jacTrans_(6, mbs[robotIndex_].nrDof()),
	jacLambda_(),
	cont_()
{
	for(int ind = 0; ind < mbs[robotIndex_].nrJoints(); ++ind)
	{
		q0_[ind].resize(mbs[robotIndex_].joint(ind).params());
		alpha0_[ind].resize(mbs[robotIndex_].joint(ind).dof());
	}
}


void ConstrDataComputation::updateNrVars(const std::vector<rbd::MultiBody> &mbs, const tasks::qp::SolverData& data)
{
	const auto& mb = mbs[robotIndex_];
	lambdaBegin_ = data.lambdaBegin();

	cont_.clear();
	const auto& cCont = data.allContacts();
	for(std::size_t i = 0; i < cCont.size(); ++i)
	{
		const tasks::qp::BilateralContact& c = cCont[i];
		if(robotIndex_ == c.contactId.r1Index)
		{
			cont_.emplace_back(mb, c.contactId.r1BodyName, data.lambdaBegin(static_cast<int>(i)),
				c.r1Points, c.r1Cones);
		}
		// we don't use else to manage self contact on the robot
		if(robotIndex_ == c.contactId.r2Index)
		{
			cont_.emplace_back(mb, c.contactId.r2BodyName, data.lambdaBegin(static_cast<int>(i)),
				c.r2Points, c.r2Cones);
		}
	}

	/// @todo don't use nrDof and totalLamdba but max dof of a jacobian
	/// and max lambda of a contact.
	jacLambda_.resize(data.totalLambda(), nrDof_);
	fullJacLambda_.resize(data.totalLambda(), nrDof_);
}


void ConstrDataComputation::computeGainsConstrMatrices(const std::vector<rbd::MultiBody> &mbs,
	const std::vector<rbd::MultiBodyConfig> &mbcs, const std::shared_ptr<ConstrData> &stockData)
{
	computeFDMatrices(mbs[robotIndex_], mbcs[robotIndex_], stockData->H, stockData->C);
	computePDError(mbcs[robotIndex_], stockData->gainsJointsList, stockData->error, stockData->derror);
	computeContactMatrix(mbs[robotIndex_], mbcs[robotIndex_], stockData->minusJtG);
}


void ConstrDataComputation::q0(const std::vector<std::vector<double> > &q0)
{
	q0_ = q0;
}


void ConstrDataComputation::alpha0(const std::vector<std::vector<double> > &alpha0)
{
	alpha0_ = alpha0;
}


void ConstrDataComputation::computeFDMatrices(const rbd::MultiBody& mb,
	const rbd::MultiBodyConfig& mbc, Eigen::MatrixXd& H, Eigen::VectorXd& C)
{
	fd_.computeH(mb, mbc);
	fd_.computeC(mb, mbc);
	H = fd_.H();
	C = fd_.C();
}


void ConstrDataComputation::computePDError(const rbd::MultiBodyConfig &mbc,
	const std::vector<int>& gainsJointsList, Eigen::VectorXd &e, Eigen::VectorXd &de) const
{
	for(std::size_t jInd = 0; jInd < gainsJointsList.size(); ++jInd)
	{
		e[jInd] = q0_[gainsJointsList[jInd]][0] - mbc.q[gainsJointsList[jInd]][0];
		de[jInd] = alpha0_[gainsJointsList[jInd]][0] - mbc.alpha[gainsJointsList[jInd]][0];
	}
}


void ConstrDataComputation::computeContactMatrix(const rbd::MultiBody &mb,
	const rbd::MultiBodyConfig &mbc, Eigen::MatrixXd& minusJtG)
{
	for(std::size_t i = 0; i < cont_.size(); ++i)
	{
		const Eigen::MatrixXd& jac = cont_[i].jac.bodyJacobian(mb, mbc);

		ContactData& cd = cont_[i];
		int lambdaOffset = 0;
		for(std::size_t j = 0; j < cd.points.size(); ++j)
		{
			int nrLambda = static_cast<int>(cd.minusGenerators[j].cols());
			// we translate the jacobian to the contact point
			// then we compute the jacobian against lambda J_l = J^T C
			// to apply fullJacobian on it we must have robot dof on the column so
			// J_l^T = (J^T C)^T = C^T J
			cd.jac.translateBodyJacobian(jac, mbc, cd.points[j], jacTrans_);
			jacLambda_.block(0, 0, nrLambda, cd.jac.dof()).noalias() =
				(cd.minusGenerators[j].transpose()*jacTrans_.block(3, 0, 3, cd.jac.dof()));

			cd.jac.fullJacobian(mb,
				jacLambda_.block(0, 0, nrLambda, cd.jac.dof()),
				fullJacLambda_);

			minusJtG.block(0, cd.lambdaBegin - lambdaBegin_ + lambdaOffset,
				nrDof_, nrLambda).noalias() =
					fullJacLambda_.block(0, 0, nrLambda, nrDof_).transpose();
			lambdaOffset += nrLambda;
		}
	}
}

}

}
