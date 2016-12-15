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
#include <vector>

// Inherited header
#include "QPSolver.h"

// Task
#include "QPGainsConstrData.h"

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

class QPGainsSolver : public tasks::qp::QPSolver
{
public:
	QPGainsSolver();
	~QPGainsSolver() {}

    void addRobotToAdaptiveQP(const std::vector<rbd::MultiBody> & mbs, int robotIndex);
	void setRobotq0(int robotIndex, const std::vector<std::vector<double> > &q0);
	void setRobotAlpha0(int robotIndex, const std::vector<std::vector<double> > &alpha0);
	void setGainsList(const std::vector<rbd::MultiBody> &mbs, int robotIndex, std::vector<int> &gainsList);
	void updateMbc(rbd::MultiBodyConfig& mbc, int robotIndex) const override;

	void nrVars(const std::vector<rbd::MultiBody>& mbs, std::vector<tasks::qp::UnilateralContact> uni,
		std::vector<tasks::qp::BilateralContact> bi) override;

	const std::shared_ptr<ConstrData> getConstrData(int robotIndex) const;

	const Eigen::VectorXd gainsVec() const;
	const Eigen::VectorXd gainsVec(int rIndex) const;

protected:
	void preUpdate(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs) override;
    std::size_t elemPosByRobotIndex(int robotIndex, const std::string& funName) const;

protected:
    std::vector<int> robotIndex_;
	std::vector<std::unique_ptr<ConstrDataComputation>> constrDataCompute_;
	std::vector<std::shared_ptr<ConstrData>> constrDataStock_;
};


}

}
