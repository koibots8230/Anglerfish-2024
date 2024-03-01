// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.autos.AutoCommands.*;
import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.controls.EightBitDo;
import com.koibots.lib.sysid.SysIDMechanism;
import com.koibots.robot.Constants.*;
import com.koibots.robot.autos.SysID;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
    static EightBitDo driveController = new EightBitDo(0);
    static GenericHID operatorPad = new GenericHID(1);
    List<Command> autos = new ArrayList<>();

    public void registerAutos() {
        autos.add(new InstantCommand());
        autos.add(new SysID(SysIDMechanism.Elevator, () -> driveController.getB()));
        autos.add(new SysID(SysIDMechanism.Swerve, () -> driveController.getB()));
        autos.add(
                new SequentialCommandGroup(
                        B1_A_N1_S1.command.get(),
                        S1_N4_S2.command.get(),
                        S2_N5_S2.command.get(),
                        S2_N6_S2.command.get()));

        SmartDashboard.putString("Auto Routine", "0");
    }

    public void configureButtonBindings() {
        Swerve.get()
                .setDefaultCommand(
                        new FieldOrientedDrive(
                                () -> -driveController.getRightX(),
                                () -> -driveController.getRightY(),
                                () -> -driveController.getLeftX(),
                                () -> driveController.getPOV(),
                                () -> driveController.getB()));

        Trigger intake = new Trigger(() -> driveController.getRightTrigger() > 0.15);
        intake.onTrue(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () ->
                                        Intake.get()
                                                .setVelocity(
                                                        SetpointConstants.INTAKE_TARGET_VELOCITY),
                                Intake.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(1000)), Indexer.get())));
        intake.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Intake.get().setVelocity(RPM.of(0)), Intake.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));

        Trigger shoot = new Trigger(() -> driveController.getLeftTrigger() > 0.15);
        shoot.onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(5000).times(2048)), Shooter.get()),
                        new WaitCommand(1),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(1000)), Indexer.get())));
        shoot.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0)), Shooter.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())));
    }

    public Command getAutonomousRoutine() {
        return autos.get(Integer.parseInt(SmartDashboard.getString("Auto Routine", "0")));
    }

    public static void rumbleController(double strength) {
        driveController.setRumble(RumbleType.kBothRumble, strength);
    }
}
