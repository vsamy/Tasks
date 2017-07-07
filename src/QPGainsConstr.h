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
	* Over damped system constraint for \f$K\f$ and \f$B\f$. 
	* Linearization of the 2 times square root of K at K=400 => B >= 0.05K + 20
	*/
class TASKS_DLLAPI OverDampedGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Inequality>
{
public:
	/** Constructor
		* @param mbs Multi bodies of all robots
		* @param robotIndex Constrained robot Index in mbs
		*/
	OverDampedGainsConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex);

	/** Assign the pointer to the problem datas.
		@param sol The solver the constraint belongs to
		@warning This must be called before any update.
		*/
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	/** Update the system size
	 * @param mbs Multi bodies of all robots
	 * @param data The datas of the solver
	 */
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	/** Update the constraint
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 * @param data The datas of the solver
	 */
	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	/** Return the name of the constraint
	 * @return The name of the constraint
	 */
	std::string nameInEq() const override;
	/** Return the name of the joint that fails when the qp fails.
	 * @param mb Multi bodies of all robots
	 * @param line The line for which the qp failed
	 * @return The name of the joint
	 */
	std::string descInEq(const std::vector<rbd::MultiBody>& mbs, int line) override;

	/** Return the number of constraints
	 * @return The number of constraints
	 */
	int maxInEq() const override;

	/** Get the computed matrix part
		* @return The matrix
		*/
	const Eigen::MatrixXd& AInEq() const override;
	/** Get the computed vector part
		* @return The vector
		*/
	const Eigen::VectorXd& bInEq() const override;

private:
	int robotIndex_, nrLines_;
	std::shared_ptr<ConstrData> constrData_;
	Eigen::MatrixXd Aineq_;
	Eigen::VectorXd bineq_;
};


/** Avoid to reach articular position limits based on direct integration.
	* This is similar to @see tasks::qp::JointLimitsConstr.
	* This constraint will perform for each Non-Selected (NS) joint
	* when using the adaptive qp.
	* This constraint can be impossible to fulfill when articulation velocity
	* is really high. Always prefer to use DamperJointLimitsConstr.
	*/
class TASKS_DLLAPI JointLimitsNoGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Bound>
{
public:
	/** Constructor
		* @param mbs Multi-robot system.
		* @param robotIndex Constrained robot Index in mbs.
		* @param bound Articular position bounds.
		* @param step Time step in second.
		*/
	JointLimitsNoGainsConstr(const std::vector<rbd::MultiBody>& mbs, int robotIndex,
		QBound bound, double step);

	/** Assign the pointer to the problem datas.
		@param sol The solver the constraint belongs to
		@warning This must be called before any update.
		*/
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	/** Update the system size
	 * @param mbs Multi bodies of all robots
	 * @param data The datas of the solver
	 */
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	/** Update the constraint
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 * @param data The datas of the solver
	 */
	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	/** Return the name of the constraint
	 * @return The name of the constraint
	 */
	std::string nameBound() const override;
	/** Return the name of the joint that fails when the qp fails.
	 * @param mb Multi bodies of all robots
	 * @param line The line for which the qp failed
	 * @return The name of the joint
	 */
	std::string descBound(const std::vector<rbd::MultiBody>& mbs, int line) override;

	/** Gives the position of bound constraint inthe qp's optimization vector
		* @return The position
		*/
	int beginVar() const override;

	/** Get the computed lower limit
		* @return The lower limite
		*/
	const Eigen::VectorXd& Lower() const override;
	/** Get the computed upper limit
		* @return The upper limite
		*/
	const Eigen::VectorXd& Upper() const override;

private:
	int robotIndex_, alphaDBegin_, alphaDOffset_;
	double step_;
	std::shared_ptr<ConstrData> constrData_;
	Eigen::VectorXd qMin_, qMax_;
	Eigen::VectorXd qVec_, alphaVec_;
	Eigen::VectorXd lower_, upper_;
};



/** Avoid to reach articular position and velocity limits based on a velocity damper.
	* This is similar to @see tasks::qp::DamperJointLimitsConstr.
	* This constraint will perform for each Non-Selected (NS) joint
	* when using the adaptive qp.
	*/
class TASKS_DLLAPI DamperJointLimitsNoGainsConstr : public tasks::qp::ConstraintFunction<tasks::qp::Bound>
{
public:
	/** Constructor
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

	/** Assign the pointer to the problem datas.
		@param sol The solver the constraint belongs to
		@warning This must be called before any update.
		*/
	void configureConstraint(const QPGainsSolver& sol)
	{
		constrData_ = sol.getConstrData(robotIndex_);
	}

	/** Update the system size
	 * @param mbs Multi bodies of all robots
	 * @param data The datas of the solver
	 */
	void updateNrVars(const std::vector<rbd::MultiBody>& mbs, const tasks::qp::SolverData& data) override;
	/** Update the constraint
	 * @param mb Multi bodies of all robots
	 * @param mbcs Multi bodies configs of all robots
	 * @param data The datas of the solver
	 */
	void update(const std::vector<rbd::MultiBody>& mbs, const std::vector<rbd::MultiBodyConfig>& mbcs,
		const tasks::qp::SolverData& data) override;

	/** Return the name of the constraint
	 * @return The name of the constraint
	 */
	std::string nameBound() const override;
	/** Return the name of the joint that fails when the qp fails.
	 * @param mb Multi bodies of all robots
	 * @param line The line for which the qp failed
	 * @return The name of the joint
	 */
	std::string descBound(const std::vector<rbd::MultiBody>& mbs, int line) override;

	/** Gives the position of bound constraint inthe qp's optimization vector
		* @return The position
		*/
	int beginVar() const override;

	/** Get the computed lower limit
		* @return The lower limite
		*/
	const Eigen::VectorXd& Lower() const override;
	/** Get the computed upper limit
		* @return The upper limite
		*/
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