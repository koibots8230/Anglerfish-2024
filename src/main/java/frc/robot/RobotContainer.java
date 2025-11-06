// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

  private XboxController controller;

  private final Swerve swerve;
  private final Indexer indexer;
  private final Shooter shooter;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    swerve = new Swerve(isReal);
    indexer = new Indexer();
    shooter = new Shooter();

    Monologue.setupMonologue(this, "Robot", false, false);

    configureBindings();
    subsystemDefualtCommands();
  }

  private void configureBindings() {
    Trigger shoot = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    shoot.onTrue(
      Commands.sequence(
        shooter.setSpeedCommand(0.55, 0.55),
        Commands.waitSeconds(0.2),
        indexer.setSpeedCommand(0.5)
      )
    );
    shoot.onFalse(
      Commands.sequence(
        indexer.setSpeedCommand(0),
        shooter.setSpeedCommand(0, 0)
      )
    );

    Trigger intake = new Trigger(() -> controller.getRightTriggerAxis() > 0.15);
    intake.onTrue(
      Commands.sequence(
        indexer.setSpeedCommand(-0.2),
        shooter.setSpeedCommand(-0.2, -0.2),
        Commands.waitUntil(indexer::sensorTriggered),
        Commands.waitUntil(() -> !indexer.sensorTriggered()),
        shooter.setSpeedCommand(0, 0),
        indexer.setSpeedCommand(0.1),
        Commands.waitUntil(() -> indexer.sensorTriggered()),
        indexer.setSpeedCommand(0)
      )
    );
    intake.onFalse(
      Commands.sequence(
        indexer.setSpeedCommand(0),
        shooter.setSpeedCommand(0, 0)
      )
    );
  }

  private void subsystemDefualtCommands() {
    swerve.setDefaultCommand(
        swerve.fieldOrientedCommand(
            (() -> -1 * controller.getLeftY()),
            (() -> -1 * controller.getLeftX()),
            (() -> -1 * controller.getRightX())));
  }

  public Command getAutonomousCommand() {
    return new WaitCommand(15);
  }
}
