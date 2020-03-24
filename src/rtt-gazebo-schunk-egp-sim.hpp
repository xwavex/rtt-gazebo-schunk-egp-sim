/* ============================================================
 *
 * This file is a part of CoSiMA (CogIMon) project
 *
 * Copyright (C) 2020 by Dennis Leroy Wigand <dwigand@techfak.uni-bielefeld.de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the ``LGPL''),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CoR-Lab, Research Institute for Cognition and Robotics
 *     Bielefeld University
 *
 * ============================================================ */

#pragma once

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <Eigen/Dense>
#include <Eigen/LU>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>
#include <string>
#include <fstream>
#include <streambuf>
#include <sstream>

#include <boost/shared_ptr.hpp>

#include <kdl/velocityprofile_trap.hpp>

#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/robot/JointState.hpp>

namespace cosima
{

class SchunkEgpSim : public RTT::TaskContext
{
public:
	SchunkEgpSim(std::string const &name);
	bool configureHook();
	void WorldUpdateBegin();
	void WorldUpdateEnd();
	// virtual ~SchunkEgpSim(){}

protected:
	bool getModel(const std::string &model_name);
	bool getModelWithPrefix(const std::string &model_name, const std::string &urdf_prefix);
	void gazeboUpdateHook(gazebo::physics::ModelPtr model);
	bool gazeboConfigureHook(gazebo::physics::ModelPtr model);
	void setGains(double p, double i, double d);
	void setTrapGeneratorLimits(const double maxvel, const double maxacc);

	void moveTo(double percent_amount);
	void open();
	void close();


	double getOrocosTime();

	// gazebo-related connections
	gazebo::event::ConnectionPtr world_begin;
	gazebo::event::ConnectionPtr world_end;

	// physical entities
	gazebo::physics::ModelPtr model;
	gazebo::physics::LinkPtr link_finger_1;
	gazebo::physics::LinkPtr link_finger_2;
	gazebo::physics::JointPtr joint_finger_1;
	gazebo::physics::JointPtr joint_finger_2;
	std::string name_joint_finger_1;
	std::string name_joint_finger_2;

	// controller
	gazebo::physics::JointControllerPtr _gazebo_position_joint_controller;
	gazebo::common::PID pid_gains;

	// trapez profile
	KDL::VelocityProfile_Trap trap_generator;
	double maxvel;
	double maxacc;

	// ports
	RTT::OutputPort<rstrt::robot::JointState> out_JointFeedback_port;
	rstrt::robot::JointState out_JointFeedback;

	RTT::InputPort<rstrt::kinematics::JointAngles> in_JointPositionCtrl_port;
	RTT::FlowStatus in_JointPositionCtrl_flow;
	rstrt::kinematics::JointAngles in_JointPositionCtrl;

	RTT::OutputPort<bool> out_converged_port;
	bool out_converged;

	void readSim();
	void writeSim();

	void writeFeedbackToOrocos();
	void readCommandsFromOrocos();

	void move(float amount);

private:
	bool is_configured;
	std::string urdf_prefix;

	double upperPos, lowerPos;
	double starttrajectory_time;

	double move_speed;

	double convergence_pos_threshold, convergence_vel_threshold;
};

} // namespace cosima