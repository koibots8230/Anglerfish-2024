// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.autos.AutoCommands.*;
import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;
import static java.lang.StrictMath.PI;

import com.koibots.lib.controls.EightBitDo;
import com.koibots.lib.sysid.SysIDMechanism;
import com.koibots.robot.Constants.*;
import com.koibots.robot.autos.DriveDistance;
import com.koibots.robot.autos.SysID;
import com.koibots.robot.commands.Intake.IntakeCommand;
import com.koibots.robot.commands.Intake.IntakeShooter;
import com.koibots.robot.commands.Scoring.Shoot;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import com.koibots.robot.commands.Swerve.TestDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
    static EightBitDo driveController = new EightBitDo(0);
    static GenericHID operatorPad = new GenericHID(1);
    List<Command> autos = new ArrayList<>();

    List<SendableChooser<Boolean>> modulesEnabled = new ArrayList<>();

    public RobotContainer() {
        for (int a = 0; a < 4; a++) {
            SendableChooser<Boolean> module = new SendableChooser<>();

            module.setDefaultOption("Enabled", true);
            module.addOption("Disabled", false);

            SmartDashboard.putData("Module " + a, module);

            modulesEnabled.add(a, module);
        }
    }

    public void registerAutos() {
        autos.add(new InstantCommand());
        autos.add(
                new SequentialCommandGroup(
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false),
                        new DriveDistance(Inches.of(50), new Rotation2d(0))));
        autos.add(
                new SequentialCommandGroup(
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false),
                        new ParallelCommandGroup(
                                new DriveDistance(Inches.of(60), new Rotation2d(0)),
                                new IntakeCommand(false)),
                        new DriveDistance(Inches.of(60), new Rotation2d(-PI)),
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false)));
        autos.add(
                new SequentialCommandGroup(
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false),
                        new ParallelCommandGroup(
                                new DriveDistance(Inches.of(60), new Rotation2d(0)),
                                new IntakeCommand(false)),
                        new DriveDistance(Inches.of(60), new Rotation2d(-PI)),
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false),
                        new ParallelCommandGroup(
                                new DriveDistance(
                                        Inches.of(Math.sqrt(5002.515625)),
                                        new Rotation2d(Math.atan(57 / 41.875))),
                                new IntakeCommand(false)),
                        new DriveDistance(
                                Inches.of(Math.sqrt(5002.515625)),
                                new Rotation2d(Math.atan(57 / 41.875) - PI)),
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false)));
        autos.add(
                new SequentialCommandGroup(
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false),
                        new ParallelCommandGroup(
                                new DriveDistance(Inches.of(60), new Rotation2d(0)),
                                new IntakeCommand(false)),
                        new DriveDistance(Inches.of(60), new Rotation2d(-PI)),
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false),
                        new ParallelCommandGroup(
                                new DriveDistance(
                                        Inches.of(Math.sqrt(5002.515625)),
                                        new Rotation2d(Math.atan(57.0 / 41.875))),
                                new IntakeCommand(false)),
                        new DriveDistance(
                                Inches.of(Math.sqrt(5002.515625)),
                                new Rotation2d(Math.atan(57.0 / 41.875) - PI)),
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false),
                        new ParallelCommandGroup(
                                new DriveDistance(
                                        Inches.of(Math.sqrt(5002.515625)),
                                        new Rotation2d(Math.atan(-57.0 / 41.875))),
                                new IntakeCommand(false)),
                        new DriveDistance(
                                Inches.of(Math.sqrt(5002.515625)),
                                new Rotation2d(Math.atan(-57.0 / 41.875) - PI)),
                        new Shoot(
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
                                SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
                                false)));
        autos.add(CalibX.command.get());
        autos.add(CalibY.command.get());
        autos.add(CalibTheta.command.get());
        autos.add(new SysID(SysIDMechanism.Swerve, () -> driveController.getB()));

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
                new InstantCommand(() -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)), Shooter.get()),
                new InstantCommand(() -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())
            )
        );
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
        return autos.get(1);
    }

    public static void rumbleController(double strength) {
        driveController.setRumble(RumbleType.kBothRumble, strength);
    }
}
