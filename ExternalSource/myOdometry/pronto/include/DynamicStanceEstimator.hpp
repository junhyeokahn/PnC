/* Copyright (c) 2015-2021
 * Istituto Italiano di Tecnologia (IIT), University of Oxford
 * All rights reserved.
 *
 * Author: Marco Camurri (mcamurri@robots.ox.ac.uk)
 *
 * This file is part of pronto_quadruped,
 * a library for leg odometry on quadruped robots.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include "ExternalSource/myOdometry/pronto/include/StanceEstimator.hpp"

#include <ExternalSource/myOdometry/pronto/include/feet_contact_forces.h>
#include <ExternalSource/myOdometry/pronto/include/feet_jacobians.h>
#include <ExternalSource/myOdometry/pronto/include/jsim.h>
#include <ExternalSource/myOdometry/pronto/include/inverse_dynamics.h>
#include <ExternalSource/myOdometry/pronto/include/joint_id_tricks.h>
#include <ExternalSource/myOdometry/pronto/include/forward_kinematics.h>

namespace pronto {
namespace quadruped {

using InverseDynamics = InverseDynamicsBase;
using JSIM = JSIMBase ;
using LegScalarMap = LegDataMap<double>;
using Wrench = iit::rbd::ForceVector;

/**
 * @brief The DynamicStanceEstimator class computes the stance status from
 * the discrepancy between the floating base and joint space portions of the
 * Newtown-Euler equations.
 */
class DynamicStanceEstimator : public StanceEstimator {
public:
    DynamicStanceEstimator(InverseDynamics& inverse_dynamics,
                           JSIM& jsim,
                           FeetContactForces& feet_contact_forces,
                           ForwardKinematics& forward_kinematics);

    bool getStance(LegBoolMap& stance,
                   LegScalarMap& stance_probability) override;

    /**
     * @brief getDynamicsViolation computes the difference in force as per the
     * base and the joints (the two lines of the Newton-Euler equations.
     * @return a data structure containing for each foot the difference in
     * Ground Reaction Force
     */
    std::vector<Wrench> getDynamicsViolation();
    std::vector<Wrench> getGRFnormLegs();
    Wrench getGRFnormBase();

protected:
    ForwardKinematics& forward_kinematics_;

    std::vector<Wrench> dynamics_violation_;
    std::vector<Wrench> wrench_legs;
    InverseDynamics& inverse_dynamics_;
    JSIM& jsim_;

    double violation_threshold_;
    Wrench wrench_base;

protected:
    Wrench getBaseWrench(const JointState& q,
                         const JointState& qd,
                         const Quaterniond &orient,
                         const JointState& qdd = JointState::Zero(),
                         const Vector3d& xd = Vector3d::Zero(),
                         const Vector3d& xdd = Vector3d::Zero(),
                         const Vector3d& omega = Vector3d::Zero(),
                         const Vector3d& omegad = Vector3d::Zero());
};
}
} // namespace pronto
