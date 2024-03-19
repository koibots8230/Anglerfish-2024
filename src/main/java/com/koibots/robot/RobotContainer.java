// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;
import com.koibots.lib.controls.EightBitDo;
import com.koibots.lib.sysid.SysIDMechanism;
import com.koibots.robot.Constants.*;
import com.koibots.robot.autos.SysID;
import com.koibots.robot.commands.Intake.IntakeCommand;
import com.koibots.robot.commands.Intake.IntakeShooter;
import com.koibots.robot.commands.Scoring.Shoot;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import com.koibots.robot.commands.Swerve.TestDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class RobotContainer {
    static EightBitDo driveController = new EightBitDo(0);
    static GenericHID operatorPad = new GenericHID(1);
    List<Command> autos = new ArrayList<>();

    List<SendableChooser<Boolean>> modulesEnabled = new ArrayList<>();

    SendableChooser<Translation2d> startingPosition = new SendableChooser<>();
    List<SendableChooser<Command>> actions = new ArrayList<>();

    public RobotContainer() {
        for (int a = 0; a < 4; a++) {
            SendableChooser<Boolean> module = new SendableChooser<>();

            module.setDefaultOption("Enabled", true);
            module.addOption("Disabled", false);

            SmartDashboard.putData("Module " + a, module);

            modulesEnabled.add(a, module);
        }

        startingPosition.setDefaultOption("PLEASE INPUT!!", new Translation2d());
        Enumeration<String> startingPosNames = AutoConstants.STARTING_POSITIONS.keys();
        while (startingPosNames.hasMoreElements()) {
            String key = startingPosNames.nextElement();
            startingPosition.addOption(key, AutoConstants.STARTING_POSITIONS.get(key));
        }

        for (int a = 0; a < 5; a++) {
            SendableChooser<Command> action = new SendableChooser<>();

            action.setDefaultOption("Nothing", new InstantCommand());

            Enumeration<String> actionNames = AutoConstants.AUTO_ACTIONS.keys();
            while (actionNames.hasMoreElements()) {
                String key = actionNames.nextElement();
                action.addOption(key, AutoConstants.AUTO_ACTIONS.get(key));
            }

            SmartDashboard.putData("Action " + a, action);

            actions.add(a, action);
        }
    }

    public void registerAutos() {
        
        // autos.add(CalibX.command.get());
        // autos.add(CalibY.command.get());
        // autos.add(CalibTheta.command.get());
        autos.add(new SysID(SysIDMechanism.Drive, () -> driveController.getY()));
        autos.add(new SysID(SysIDMechanism.Turn, () -> driveController.getY()));

        SmartDashboard.putString("Auto Routine", "0");
    }

    public void configureButtonBindings() {
        Swerve.get()
                .setDefaultCommand(
                        new FieldOrientedDrive(
                                () -> -driveController.getLeftY(),
                                () -> -driveController.getLeftX(),
                                () -> -driveController.getRightX(),
                                () -> driveController.getPOV(),
                                () -> driveController.getB()));

        Trigger intake = new Trigger(() -> driveController.getRightTrigger() > 0.15);
        intake.onTrue(new IntakeCommand(false));
        intake.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Intake.get().setVelocity(RPM.of(0)), Intake.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));

        Trigger shoot = new Trigger(() -> driveController.getLeftTrigger() > 0.3);
        shoot.onTrue(
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                        SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                        false));
        shoot.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get())));

        Trigger shootAmp = new Trigger(() -> driveController.getLeftBumper());
        shootAmp.onTrue(
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.get(1).get(0),
                        SetpointConstants.SHOOTER_SPEEDS.get(1).get(1),
                        false));
        shootAmp.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get())));

        Trigger zero = new Trigger(() -> driveController.getA());
        zero.onTrue(new InstantCommand(() -> Swerve.get().zeroGyro()));

        Trigger intakeShooter = new Trigger(() -> driveController.getRightBumper());
        intakeShooter.onTrue(
                new ParallelRaceGroup(
                        new StartEndCommand(
                                () -> Shooter.get().setVelocity(RPM.of(-800), RPM.of(-602)),
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get()),
                        new IntakeShooter()));
        intakeShooter.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));
    }

    public void configureTestBinds() {
        Swerve.get()
                .setDefaultCommand(
                        new TestDrive(
                                () -> -driveController.getRightX(),
                                () -> -driveController.getRightY(),
                                () -> -driveController.getLeftX(),
                                () -> driveController.getPOV(),
                                () -> driveController.getB(),
                                modulesEnabled));
    }

    public Command getAutonomousRoutine() {
        //Swerve.get().resetOdometry(new Pose2d(startingPosition.getSelected(), Swerve.get().getGyroAngle()));
        return autos.get(1);
    }

    public static void rumbleController(double strength) {
        driveController.setRumble(RumbleType.kBothRumble, strength);
    }
}
