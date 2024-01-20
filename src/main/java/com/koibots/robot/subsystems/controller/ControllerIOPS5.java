// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.controller;

import edu.wpi.first.wpilibj.PS5Controller;

public class ControllerIOPS5 extends ControllerIO {

    PS5Controller controller;

    public ControllerIOPS5() {
        controller = new PS5Controller(0);
    }

    @Override
    public double xTranslation() {
        return -controller.getLeftY();
    }

    @Override
    public double yTranslation() {
        return -controller.getLeftX();
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
        return controller.getCrossButton();
    }
}
