// Copyright 2024 (c) FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems.controller;

public abstract class ControllerIO {
    public abstract double xTranslation();

    public abstract double yTranslation();

    public abstract double angularVelocity();

    public abstract int anglePosition();

    public abstract boolean cross();
}
