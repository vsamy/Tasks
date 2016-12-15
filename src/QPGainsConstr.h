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
// Forward declare QBound 
class QBound;
class AlphaBound;

// Forward ConstrData
namespace qpgains
{
class ConstrData;

/**
	* Avoid to reach articular position limits based on direct integration.
	* This is similar to @see tasks::qp::JointLimitsConstr.
	* This constraint will perform for each Non-Selected (NS) joint
	* when using the adaptive qp.
	* This constraint can be impossible to fulfill when articulation velocity
	* is really high. Always prefer to use DamperJointLimitsConstr.
	*/
class TASKS_DLLAPI JointLimitsNoGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Bound>
{
public:
	/**
		* @param mbs Multi-robot system.
		* @param robotIndex Constrained robot Index in mbs.
		* @param bound Articular position bounds.
		* @param step Time step in second.
		*/
	JointLimitsNoGainsConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
		QBound bound, double step);

	// Constraint
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	void update(const std::vector<rbd::MultiBody>& mbs,	const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	std::string nameBound() const override;
	std::string descBound(const std::vector<rbd::MultiBody>& mbs, int line) override;

	// Bound Constraint
	int beginVar() const override;

	const Eigen::VectorXd& Lower() const override;
	const Eigen::VectorXd& Upper() const override;

private:
	int robotIndex_, alphaDBegin_, alphaDOffset_;
	double step_;
	std::shared_ptr<ConstrData> constrData_;
	Eigen::VectorXd qMin_, qMax_;
	Eigen::VectorXd qVec_, alphaVec_;
	Eigen::VectorXd lower_, upper_;
};



/**
	* Avoid to reach articular position and velocity limits based on
	* a velocity damper.
	* This is similar to @see tasks::qp::DamperJointLimitsConstr.
	* This constraint will perform for each Non-Selected (NS) joint
	* when using the adaptive qp.
	*/
class TASKS_DLLAPI DamperJointLimitsNoGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Bound>
{
public:
	/**
		* @param mbs Multi-robot system.
		* @param robotIndex Constrained robot Index in mbs.
		* @param qBound Articular position bounds.
		* @param aBound Articular velocity bounds.
		* @param interPercent \f$ interPercent (\overline{q} - \underline{q}) \f$
		* @param securityPercent \f$ securityPercent (\overline{q} - \underline{q}) \f$
		* @param damperOffset \f$ \xi_{\text{off}} \f$
		* @param step Time step in second.
		*/
	DamperJointLimitsNoGainsConstr(const std::vector<rbd::MultiBody>& mbs,
		int robotIndex, const QBound& qBound, const AlphaBound& aBound,
		double interPercent, double securityPercent, double damperOffset, double step);

	// Constraint
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	void update(const std::vector<rbd::MultiBody>& mbs,	const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	// Add the pointer to the problem datas
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	std::string nameBound() const override;
	std::string descBound(const std::vector<rbd::MultiBody>& mbs, int line) override;

	// Bound Constraint
	int beginVar() const override;

	const Eigen::VectorXd& Lower() const override;
	const Eigen::VectorXd& Upper() const override;

	/// compute damping that avoid speed jump
	double computeDamping(double alpha, double dist, double iDist, double sDist);
	double computeDamper(double dist, double iDist, double sDist, double damping);

private:
	struct DampData
	{
		enum State {Low, Upp, Free};

		DampData(double mi, double ma, double miV, double maV,
				 double idi, double sdi, int aDB, int i):
			min(mi), max(ma), minVel(miV), maxVel(maV), iDist(idi), sDist(sdi),
			jointIndex(i), alphaDBegin(aDB), damping(0.), state(Free)
		{}

		double min, max;
		double minVel, maxVel;
		double iDist, sDist;
		int jointIndex;
		int alphaDBegin;
		double damping;
		State state;
	};

private:
	int robotIndex_, alphaDBegin_;
	std::vector<DampData> data_;
	std::shared_ptr<ConstrData> constrData_;

	Eigen::VectorXd lower_, upper_;
	double step_;
	double damperOff_;
};

} // namespace qpgains

} // namespace tasks