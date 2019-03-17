#pragma once

#include "RigidBody.h"
#include <array>

using Eigen::Vector2d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

class HingeJoint {
public:

	HingeJoint(int rb0Idx, int rb1Idx, Vector2d local0, Vector2d local1)
		: rbIdx({rb0Idx, rb1Idx}), local({local0, local1}) {
	}

	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb0, const RigidBody &rb1) const {
		Vector2d p0 = rb0.pWorld(x, local[0]);
		Vector2d p1 = rb1.pWorld(x, local[1]);
        return (p1-p0);
	}

	MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb0, const RigidBody &rb1) const {
		MatrixXd dCdx(2, 6);

        ////////////////////////
        // Task 1.2
        ////////////////////////

		return dCdx;
	}

public:
	std::array<int, 2> rbIdx;
	std::array<Vector2d, 2> local;
};

class FixedJoint
{
public:
	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb) const {
        return (pos - rb.pWorld(x, localPos));
	}

	MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb) const {
        MatrixXd dCdx(2, 3);

        ////////////////////////
        // Task 1.2
        ////////////////////////

		return dCdx;
	}

public:
    Vector2d pos;       // world coordinates
    int rbIdx;          // index of rigid body
    Vector2d localPos;  // position in rigid body coordinates
};

class FixedAngleJoint
{
public:
	VectorXd computeConstraints(const VectorXd &x, const RigidBody &rb) const {
		VectorXd c(1);
		c << (rb.theta(x) - angle);
		return c;
	}

	MatrixXd computeJacobian(const VectorXd x, const RigidBody &rb) const {
		MatrixXd dCdx(1, 3);

        ////////////////////////
        // Task 1.2
        ////////////////////////


		return dCdx;
	}

public:
	int rbIdx;
	mutable double angle;
};
