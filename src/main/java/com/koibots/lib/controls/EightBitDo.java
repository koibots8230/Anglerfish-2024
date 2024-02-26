// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.lib.controls;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class EightBitDo {
    private GenericHID controller;

    public EightBitDo(int port) {
        controller = new GenericHID(port);
    }

    public boolean getA() {
        return controller.getRawButton(2);
    }

    public boolean getB() {
        return controller.getRawButton(1);
    }

    public boolean getX() {
        return controller.getRawButton(4);
    }

    public boolean getY() {
        return controller.getRawButton(3);
    }

    public boolean getLeftBumper() {
        return controller.getRawButton(5);
    }

    public boolean getRightBumper() {
        return controller.getRawButton(6);
    }

    public boolean getMinus() {
        return controller.getRawButton(7);
    }

    public boolean getPlus() {
        return controller.getRawButton(8);
    }

    public boolean getLeftStick() {
        return controller.getRawButton(9);
    }

    public boolean getRightStick() {
        return controller.getRawButton(10);
    }

    public int getPOV() {
        return controller.getPOV();
    }

    public double getLeftY() {
        return controller.getRawAxis(0);
    }

    public double getLeftX() {
        return controller.getRawAxis(1);
    }

    public double getRightY() {
        return controller.getRawAxis(4);
    }

    public double getRightX() {
        return controller.getRawAxis(5);
    }

    public double getLeftTrigger() {
        return controller.getRawAxis(2);
    }

    public double getRightTrigger() {
        return controller.getRawAxis(3);
    }

    public void setRumble(RumbleType type, double strength) {
        controller.setRumble(type, strength);
    }
}
