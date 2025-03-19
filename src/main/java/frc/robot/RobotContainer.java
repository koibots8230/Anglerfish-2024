// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  private XboxController controller;
  private final Swerve swerve;

  public RobotContainer() {
    controller = new XboxController(0);
    swerve =  new Swerve(true);
    configureBindings();
  }

  private void configureBindings() {
    swerve.fieldOrientedWhilePointingCommand(() -> controller.getLeftX(), () -> controller.getLeftY());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
