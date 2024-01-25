// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.controller;

import edu.wpi.first.wpilibj.GenericHID;

public class ControllerIOJoystick extends ControllerIO {
    GenericHID controller = new GenericHID(0);

    @Override
    public double xTranslation() {
        return -controller.getRawAxis(1);
    }

    @Override
    public double yTranslation() {
        return -controller.getRawAxis(0);
    }

    @Override
    public double angularVelocity() {
        return -controller.getRawAxis(2);
    }

    @Override
    public int anglePosition() {
        return controller.getPOV();
    }

    @Override
    public boolean cross() {
        return controller.getRawButton(6);
    }
}
