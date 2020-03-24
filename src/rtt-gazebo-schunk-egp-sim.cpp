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

#include "rtt-gazebo-schunk-egp-sim.hpp"
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>
#include <sstream>

using namespace cosima;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

#define PRELOG(X) (RTT::log(RTT::X) << "[" << this->getName() << "] ")

SchunkEgpSim::SchunkEgpSim(const std::string &name) : TaskContext(name),
														is_configured(false), 
														out_converged(true),
														maxvel(3.0),
														maxacc(1),
														upperPos(0.01) /* values from SDF */,
														lowerPos(0) /* values from SDF */,
														starttrajectory_time(0),
														move_speed(3.0),
														convergence_pos_threshold(0.01),
														convergence_vel_threshold(0.01)
{
	this->provides("gazebo")->addOperation("WorldUpdateBegin",
										   &SchunkEgpSim::WorldUpdateBegin, this, RTT::ClientThread);
	this->provides("gazebo")->addOperation("WorldUpdateEnd",
										   &SchunkEgpSim::WorldUpdateEnd, this, RTT::ClientThread);

	this->addOperation("getModel", &SchunkEgpSim::getModel, this, ClientThread);
	this->addOperation("getModelWithPrefix", &SchunkEgpSim::getModelWithPrefix, this, ClientThread);

	this->addOperation("setGains", &SchunkEgpSim::setGains, this, ClientThread);
	this->addOperation("setTrapGeneratorLimits", &SchunkEgpSim::setTrapGeneratorLimits, this, ClientThread);
	
	this->addOperation("moveTo", &SchunkEgpSim::moveTo, this, ClientThread);
	this->addOperation("open", &SchunkEgpSim::open, this, ClientThread);
	this->addOperation("close", &SchunkEgpSim::close, this, ClientThread);

	this->addProperty("out_converged", out_converged);
	this->addProperty("move_speed", move_speed);
	this->addProperty("convergence_pos_threshold", convergence_pos_threshold);
	this->addProperty("convergence_vel_threshold", convergence_vel_threshold);
	this->addProperty("upperPos", upperPos);

	world_begin = gazebo::event::Events::ConnectWorldUpdateBegin(
		boost::bind(&SchunkEgpSim::WorldUpdateBegin, this));
	world_end = gazebo::event::Events::ConnectWorldUpdateEnd(
		boost::bind(&SchunkEgpSim::WorldUpdateEnd, this));

	// Defaults
	// pid_gains.SetPGain(1);
	// pid_gains.SetIGain(0.1);
	// pid_gains.SetDGain(0.01);

	pid_gains.SetPGain(100);
	pid_gains.SetIGain(1);
	pid_gains.SetDGain(30);

	trap_generator = KDL::VelocityProfile_Trap(maxvel, maxacc);
}

bool SchunkEgpSim::getModel(const std::string &model_name)
{
	getModelWithPrefix(model_name, "");
}

bool SchunkEgpSim::getModelWithPrefix(const std::string &model_name, const std::string &urdf_prefix)
{
	if (model)
	{
		PRELOG(Warning) << "Model [" << model_name << "] already loaded!"
					 << endlog();
		return true;
	}
	gazebo::printVersion();
	if (!gazebo::physics::get_world())
	{
		PRELOG(Error) << "getWorldPtr does not seem to exists" << endlog();
		return false;
	}
	model = gazebo::physics::get_world()->GetModel(model_name);
	this->urdf_prefix = urdf_prefix;
	if (model)
	{
		if (model.get() == NULL)
		{
			PRELOG(Error) << "Model could not be retrieved from the model pointer!" << endlog();
			return false;
		}
		else
		{
			PRELOG(Info) << "Model [" << model_name << "] successfully loaded!" << endlog();
			return true;
		}
	}
	else
	{
		PRELOG(Error) << "Model [" << model_name << "] could NOT be loaded!"
				   << endlog();
		return false;
	}
	return bool(model);
}

void SchunkEgpSim::setTrapGeneratorLimits(const double maxvel, const double maxacc)
{
	this->maxvel = maxvel;
	this->maxacc = maxacc;

	trap_generator.SetMax(this->maxvel, this->maxacc);
}

void SchunkEgpSim::setGains(double p, double i, double d)
{
	pid_gains.SetPGain(p);
	pid_gains.SetIGain(i);
	pid_gains.SetDGain(d);
}

bool SchunkEgpSim::configureHook()
{
	if (!model)
	{
		PRELOG(Warning) << "Model is NOT loaded!" << endlog();
		return false;
	}
	else if (model.get() == NULL)
	{
		PRELOG(Error) << "Model could not be retrieved from the model pointer!" << endlog();
		return false;
	}

	// ########## GET LINKS AND JOINTS ##########
	link_finger_1 = model->GetLink(urdf_prefix + "SchunkEGP40_Finger1_link");
	if (!link_finger_1)
	{
		PRELOG(Error) << "Link: " << urdf_prefix << "SchunkEGP40_Finger1_link" << " not found!" << endlog();
		return false;
	}
	
	link_finger_2 = model->GetLink(urdf_prefix + "SchunkEGP40_Finger2_link");
	if (!link_finger_2)
	{
		PRELOG(Error) << "Link: " << urdf_prefix << "SchunkEGP40_Finger2_link" << " not found!" << endlog();
		return false;
	}
	// 	finger_1_prox_link->SetGravityMode(false);

	joint_finger_1 = model->GetJoint(urdf_prefix + "SchunkEGP40_Finger1_joint");
	if (!joint_finger_1)
	{
		PRELOG(Error) << "Joint [" << (urdf_prefix + "SchunkEGP40_Finger1_joint") << "] not found!" << endlog();
		return false;
	}
	else if (joint_finger_1.get() == NULL)
	{
		PRELOG(Error) << "Joint [" << (urdf_prefix + "SchunkEGP40_Finger1_joint") << "] not found!" << endlog();
		return false;
	}
	else
	{
		name_joint_finger_1 = joint_finger_1->GetName();
		PRELOG(Info) << "Joint [" << (urdf_prefix + "SchunkEGP40_Finger1_joint") << "] found as " << name_joint_finger_1 << endlog();
	}
	
	joint_finger_2 = model->GetJoint(urdf_prefix + "SchunkEGP40_Finger2_joint");
	if (!joint_finger_2)
	{
		PRELOG(Error) << "Joint [" << (urdf_prefix + "SchunkEGP40_Finger2_joint") << "] not found!" << endlog();
		return false;
	}
	else if (joint_finger_2.get() == NULL)
	{
		PRELOG(Error) << "Joint [" << (urdf_prefix + "SchunkEGP40_Finger2_joint") << "] not found!" << endlog();
		return false;
	}
	else
	{
		name_joint_finger_2 = joint_finger_2->GetName();
		PRELOG(Info) << "Joint [" << (urdf_prefix + "SchunkEGP40_Finger2_joint") << "] found as " << name_joint_finger_2 << endlog();
	}

	// ########## INIT CONTROLLER ##########
	_gazebo_position_joint_controller.reset(new gazebo::physics::JointController(model));

	
	// _gazebo_position_joint_controller->AddJoint(model->GetJoint(name_joint_finger_1));
	// _gazebo_position_joint_controller->AddJoint(model->GetJoint(name_joint_finger_2));
	_gazebo_position_joint_controller->AddJoint(joint_finger_1);
	_gazebo_position_joint_controller->AddJoint(joint_finger_2);

	_gazebo_position_joint_controller->SetPositionPID(joint_finger_1->GetScopedName(), pid_gains);
	_gazebo_position_joint_controller->SetPositionPID(joint_finger_2->GetScopedName(), pid_gains);

	_gazebo_position_joint_controller->SetPositionTarget(joint_finger_1->GetScopedName(), upperPos);
	_gazebo_position_joint_controller->SetPositionTarget(joint_finger_2->GetScopedName(), upperPos);

	// PRELOG(Info) << "Set Gains to P = " << pid_gains.GetPGain() << ", I = " << pid_gains.GetIGain() << ", D = " << pid_gains.GetDGain() << endlog();

	// ########## INIT TRAPEZ ##########
	trap_generator = KDL::VelocityProfile_Trap(maxvel, maxacc);

	// ########## SETUP PORTS ##########
	out_JointFeedback = rstrt::robot::JointState(1);
	this->addPort("out_JointFeedback_port", out_JointFeedback_port).doc("Output port for the joint feedback.");
	out_JointFeedback_port.setDataSample(out_JointFeedback);

	this->addPort("in_JointPositionCtrl_port", in_JointPositionCtrl_port).doc("Input port for commands based on position control mode");
	in_JointPositionCtrl_flow = RTT::NoData;
	in_JointPositionCtrl = rstrt::kinematics::JointAngles(1);

	out_converged = true;
	this->addPort("out_converged", out_converged_port).doc("Output port for the convergency criterion.");
	out_converged_port.setDataSample(out_converged);

	this->is_configured = true;
	return true;
}

double SchunkEgpSim::getOrocosTime()
{
	return 1E-9 * RTT::os::TimeService::ticks2nsecs(
					  RTT::os::TimeService::Instance()->getTicks());
}

void SchunkEgpSim::WorldUpdateBegin()
{
	if (!is_configured) // || !isRunning())
		return;
	readCommandsFromOrocos();
	writeSim();
}

void SchunkEgpSim::readCommandsFromOrocos()
{
	in_JointPositionCtrl_flow = in_JointPositionCtrl_port.readNewest(in_JointPositionCtrl);
	if (in_JointPositionCtrl_flow == RTT::NewData)
	{
		trap_generator.SetProfile(joint_finger_1->GetAngle(0).Radian(), in_JointPositionCtrl.angles(0) * upperPos);
		out_converged = false;
		this->out_converged_port.write(out_converged);
		starttrajectory_time = getOrocosTime();
	}
}

void SchunkEgpSim::writeSim()
{
	double t = getOrocosTime() - starttrajectory_time;
	if (!out_converged)
	{
		// double step1 = joint_finger_1->GetAngle(0).Radian() + t * move_speed;
		// double step2 = joint_finger_2->GetAngle(0).Radian() + t * move_speed;
		double step1 = t * move_speed;
		double step2 = t * move_speed;
		if (step1 > (in_JointPositionCtrl.angles(0) * upperPos)) {
			step1 = in_JointPositionCtrl.angles(0) * upperPos;
		}
		if (step2 > (in_JointPositionCtrl.angles(0) * upperPos)) {
			step2 = in_JointPositionCtrl.angles(0) * upperPos;
		}
		// if (t <= trap_generator.Duration())
		// {
			// _gazebo_position_joint_controller->SetPositionTarget(joint_finger_1->GetScopedName(), trap_generator.Pos(t));
			// _gazebo_position_joint_controller->SetPositionTarget(joint_finger_2->GetScopedName(), trap_generator.Pos(t));
			_gazebo_position_joint_controller->SetPositionTarget(joint_finger_1->GetScopedName(), step1);
			_gazebo_position_joint_controller->SetPositionTarget(joint_finger_2->GetScopedName(), step2);
		// }
		// else
		// {
		// 	_gazebo_position_joint_controller->SetPositionTarget(joint_finger_1->GetScopedName(), in_JointPositionCtrl.angles(0) * upperPos);
		// 	_gazebo_position_joint_controller->SetPositionTarget(joint_finger_2->GetScopedName(), in_JointPositionCtrl.angles(0) * upperPos);
		// }
		
	}
	else
	{
		// Is this necessary?
		_gazebo_position_joint_controller->SetPositionTarget(joint_finger_1->GetScopedName(), in_JointPositionCtrl.angles(0) * upperPos);
		_gazebo_position_joint_controller->SetPositionTarget(joint_finger_2->GetScopedName(), in_JointPositionCtrl.angles(0) * upperPos);
	}
	
	_gazebo_position_joint_controller->Update();
}

void SchunkEgpSim::WorldUpdateEnd()
{
	if (!is_configured) // || !isRunning())
		return;
	readSim();
	writeFeedbackToOrocos();
}

void SchunkEgpSim::readSim()
{
	out_JointFeedback.angles[0] = joint_finger_1->GetAngle(0).Radian() / upperPos;
	out_JointFeedback.velocities[0] = joint_finger_1->GetVelocity(0);
	out_JointFeedback.torques[0] = joint_finger_1->GetForce(0u);

	PRELOG(Info) << "torques[0] = " << joint_finger_1->GetForce(0u) << ", torques[1] = " << joint_finger_1->GetForce(0u) << endlog();
	PRELOG(Info) << "[0]body1Force = X: " << joint_finger_1->GetForceTorque(0u).body1Force.x << ", Y: " << joint_finger_1->GetForceTorque(0u).body1Force.y << ", Z: " << joint_finger_1->GetForceTorque(0u).body1Force.z << endlog();
	PRELOG(Info) << "[0]body2Force = X: " << joint_finger_1->GetForceTorque(0u).body2Force.x << ", Y: " << joint_finger_1->GetForceTorque(0u).body2Force.y << ", Z: " << joint_finger_1->GetForceTorque(0u).body2Force.z << endlog();
	PRELOG(Info) << "[1]body1Force = X: " << joint_finger_2->GetForceTorque(0u).body1Force.x << ", Y: " << joint_finger_2->GetForceTorque(0u).body1Force.y << ", Z: " << joint_finger_2->GetForceTorque(0u).body1Force.z << endlog();
	PRELOG(Info) << "[1]body2Force = X: " << joint_finger_2->GetForceTorque(0u).body2Force.x << ", Y: " << joint_finger_2->GetForceTorque(0u).body2Force.y << ", Z: " << joint_finger_2->GetForceTorque(0u).body2Force.z << endlog();

	// Check for convergence
	if (!out_converged)
	{
		// Check for force-dependent convergence
		// if (link_finger_1->GetRelativeForce())
		// {

		// }
		if (fabs(out_JointFeedback.angles[0] - in_JointPositionCtrl.angles(0)) < convergence_pos_threshold)
		{
			// Goal position considered to be reached
			if (fabs(out_JointFeedback.velocities[0]) < convergence_vel_threshold)
			{
				// Almost no velocity
				out_converged = true;
				this->out_converged_port.write(out_converged);
			}
		}
	}
}

void SchunkEgpSim::writeFeedbackToOrocos()
{
	this->out_JointFeedback_port.write(out_JointFeedback);
}

void SchunkEgpSim::moveTo(double percent_amount)
{
	in_JointPositionCtrl.angles(0) = percent_amount;
	trap_generator.SetProfileDuration(joint_finger_1->GetAngle(0).Radian(), in_JointPositionCtrl.angles(0) * upperPos, move_speed);
	out_converged = false;
	this->out_converged_port.write(out_converged);
	starttrajectory_time = getOrocosTime();
}

void SchunkEgpSim::open()
{
	moveTo(0.0);
}

void SchunkEgpSim::close()
{
	moveTo(1.0);
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(cosima::SchunkEgpSim)