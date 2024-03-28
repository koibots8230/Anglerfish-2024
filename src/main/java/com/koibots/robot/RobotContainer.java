// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.controls.EightBitDo;
import com.koibots.robot.Constants.*;
import com.koibots.robot.autos.JankAutos;
import com.koibots.robot.commands.Intake.IntakeCommand;
import com.koibots.robot.commands.Intake.IntakeShooter;
import com.koibots.robot.commands.Scoring.FeedNote;
import com.koibots.robot.commands.Shooter.SpinUpShooter;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import com.koibots.robot.commands.Swerve.TestDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class RobotContainer {
    static EightBitDo driveController = new EightBitDo(0);
    static GenericHID operatorPad = new GenericHID(1);

    List<SendableChooser<Boolean>> modulesEnabled = new ArrayList<>();

    SendableChooser<Pose2d> startingPosition = new SendableChooser<>();
    SendableChooser<Command> autos = new SendableChooser<>();

    public RobotContainer() {

        if (AutoConstants.IS_RED) {
            AutoConstants.STARTING_POSITIONS.replace(
                    "Subwoofer - Left", new Pose2d(0.668, 4.39, new Rotation2d(Math.PI / 3)));
            AutoConstants.STARTING_POSITIONS.replace(
                    "Subwoofer - Right", new Pose2d(0.668, 6.72, new Rotation2d(-Math.PI / 3)));

            Pose2d temp = AutoConstants.SCORING_POSITIONS[0];
            AutoConstants.SCORING_POSITIONS[0] = AutoConstants.SCORING_POSITIONS[2];
            AutoConstants.SCORING_POSITIONS[2] = temp;

            Translation2d tempNote = AutoConstants.NOTE_POSITIONS[0];
            AutoConstants.NOTE_POSITIONS[0] = AutoConstants.NOTE_POSITIONS[2];
            AutoConstants.NOTE_POSITIONS[2] = tempNote;
        }

        JankAutos jankAutos = new JankAutos();

        for (int a = 0; a < 4; a++) {
            SendableChooser<Boolean> module = new SendableChooser<>();

            module.setDefaultOption("Enabled", true);
            module.addOption("Disabled", false);

            SmartDashboard.putData("Module " + a, module);

            modulesEnabled.add(a, module);
        }
        startingPosition.setDefaultOption("PLEASE INPUT!!", new Pose2d());
        Enumeration<String> startingPosNames = AutoConstants.STARTING_POSITIONS.keys();
        while (startingPosNames.hasMoreElements()) {
            String key = startingPosNames.nextElement();
            startingPosition.addOption(key, AutoConstants.STARTING_POSITIONS.get(key));
        }

        SmartDashboard.putData("Starting Pos", startingPosition);

        Method[] autoOptions = jankAutos.getClass().getDeclaredMethods();
        autos.setDefaultOption("Nothing", new InstantCommand());
        for (Method autoOption : autoOptions) {
            try {
                autos.addOption(autoOption.getName(), (Command) autoOption.invoke(jankAutos));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } catch (IllegalArgumentException e) {
                System.out.println(" >> " + autoOption.getName());
            } catch (InvocationTargetException e) {
                e.printStackTrace();
            }
        }
        SmartDashboard.putData("Autos", autos);
    }

    public void registerAutos() {

        // autos.add(CalibX.command.get());
        // autos.add(CalibY.command.get());
        // autos.add(CalibTheta.command.get());

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

        Trigger zero = new Trigger(() -> driveController.getA());
        zero.onTrue(new InstantCommand(() -> Swerve.get().zeroGyro()));

        Trigger intake = new Trigger(() -> driveController.getRightTrigger() > 0.15);
        intake.onTrue(new IntakeCommand(false));
        intake.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Intake.get().setVelocity(RPM.of(0)), Intake.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
                        new InstantCommand(() -> RobotContainer.rumbleController(0))));

        Trigger spinUpSpeaker = new Trigger(() -> operatorPad.getRawButton(6));
        spinUpSpeaker.onTrue(
                new SpinUpShooter(
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.bottomSpeed));
        spinUpSpeaker.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get())));

        Trigger spinUpAmp = new Trigger(() -> operatorPad.getRawButton(5));
        spinUpAmp.onTrue(
                new SpinUpShooter(
                        SetpointConstants.SHOOTER_SPEEDS.AMP.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.AMP.bottomSpeed));
        spinUpAmp.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get())));

        Trigger feedNote = new Trigger(() -> operatorPad.getRawButton(7));
        feedNote.onTrue(new FeedNote());
        feedNote.onFalse(
                new InstantCommand(() -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()));

        Trigger intakeShooter = new Trigger(() -> driveController.getRightBumper());
        intakeShooter.onTrue(
                new ParallelRaceGroup(
                        new StartEndCommand(
                                () ->
                                        Shooter.get()
                                                .setVelocity(
                                                        SetpointConstants.SHOOTER_SPEEDS
                                                                .INTAKE
                                                                .topSpeed,
                                                        SetpointConstants.SHOOTER_SPEEDS
                                                                .INTAKE
                                                                .bottomSpeed),
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get()),
                        new IntakeShooter()));
        intakeShooter.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
                        new InstantCommand(() -> RobotContainer.rumbleController(0))));

        Trigger reverse = new Trigger(() -> operatorPad.getRawButton(3));
        reverse.onTrue(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () ->
                                        Intake.get()
                                                .setVelocity(
                                                        SetpointConstants.INTAKE_REVERSE_SPEED),
                                Intake.get()),
                        new InstantCommand(
                                () ->
                                        Indexer.get()
                                                .setVelocity(
                                                        SetpointConstants.INTAKE_INDEXER_SPEED
                                                                .times(-1)),
                                Indexer.get()),
                        new InstantCommand(
                                () ->
                                        Shooter.get()
                                                .setVelocity(
                                                        SetpointConstants.SHOOTER_SPEEDS
                                                                .REVERSE
                                                                .topSpeed,
                                                        SetpointConstants.SHOOTER_SPEEDS
                                                                .REVERSE
                                                                .topSpeed),
                                Shooter.get())));
        reverse.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Intake.get().setVelocity(RPM.of(0)), Intake.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get())));
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
        Swerve.get().resetOdometry(startingPosition.getSelected());
        return autos.getSelected();
    }

    public static void rumbleController(double strength) {
        driveController.setRumble(RumbleType.kBothRumble, strength);
    }
}
