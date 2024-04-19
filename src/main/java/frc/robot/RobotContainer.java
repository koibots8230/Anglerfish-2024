// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.controller;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Trigger intakeTrigger;
  private final Trigger speakerShooterTrigger;
  private final Trigger ampShooterTrigger;
  private final Trigger sendToShooterTrigger;
  // The robot's subsystems and commands are defined here...
  private final Shooter shooterSubsystem = new Shooter();
  private final Intake intakeSubsystem = new Intake();
  private final Indexer indexerSubsystem = new Indexer();

  private final IntakeCommand intakeCommand =  new IntakeCommand(indexerSubsystem, intakeSubsystem);

  public RobotContainer() {
    intakeTrigger = new Trigger(() -> controller.CONTROLLER.getRightTriggerAxis() > 0.15);
    speakerShooterTrigger = new Trigger(() -> controller.OPERATOR_CONTROLLER.getRawButtonPressed(6));
    ampShooterTrigger = new Trigger(() -> controller.OPERATOR_CONTROLLER.getRawButtonPressed(5));
    sendToShooterTrigger = new Trigger(() -> controller.OPERATOR_CONTROLLER.getRawButtonPressed(7) && shooterSubsystem.shooterAtSpeedAmp() || shooterSubsystem.shooterAtSpeedSpeaker());

    

    configureBindings();
  }

  
  private void configureBindings() {

    intakeTrigger.onTrue(intakeCommand);
    intakeTrigger.onFalse(new ParallelCommandGroup(
      new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(0.0)),
      new InstantCommand(() -> intakeSubsystem.setIntakeVelocity(0.0))
    ));

    Trigger reverse = new Trigger(controller.CONTROLLER::getRightBumper);
    reverse.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> intakeSubsystem.setIntakeVelocity (-(PIDConstants.INTAKE_SETPOINT))), 
      new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(-(PIDConstants.INDEXER_SETPOINT)))
    ));
    reverse.onFalse(new ParallelCommandGroup( 
    new InstantCommand(() -> intakeSubsystem.setIntakeVelocity(0.0)),
    new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(0.0))
    ));

    speakerShooterTrigger.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> shooterSubsystem.setTopShooterVelocity(PIDConstants.TOP_SHOOTER_SETPOINT)),
      new InstantCommand(() -> shooterSubsystem.setBottomShooterVelocity(PIDConstants.BOTTOM_SHOOTER_SETPOINT))
    ));
    speakerShooterTrigger.onFalse(new ParallelCommandGroup(
      new InstantCommand(() -> shooterSubsystem.setTopShooterVelocity(0.0)),
      new InstantCommand(() -> shooterSubsystem.setBottomShooterVelocity(0.0))
    ));

    ampShooterTrigger.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> shooterSubsystem.setTopShooterVelocity(PIDConstants.TOP_SHOOTER_SETPOINT)),
      new InstantCommand(() -> shooterSubsystem.setBottomShooterVelocity(PIDConstants.BOTTOM_SHOOTER_SETPOINT))
    ));
    ampShooterTrigger.onFalse(new ParallelCommandGroup(
      new InstantCommand(() -> shooterSubsystem.setTopShooterVelocity(0.0)),
      new InstantCommand(() -> shooterSubsystem.setBottomShooterVelocity(0.0))
    ));

    sendToShooterTrigger.onTrue(new ParallelCommandGroup(
      new InstantCommand(() -> indexerSubsystem.setIndexerVelocity(PIDConstants.SEND_TO_SHOOTER_SETPOINT))
    ));


  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
