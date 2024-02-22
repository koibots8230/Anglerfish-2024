// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.autos.AutoCommands.*;
import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.sysid.SysIDMechanism;
import com.koibots.robot.Constants.*;
import com.koibots.robot.autos.SysID;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
    static GenericHID controller = new GenericHID(0);
    List<Command> autos = new ArrayList<>();

    public void registerAutos() {
        autos.add(new InstantCommand());
        autos.add(new SysID(SysIDMechanism.Elevator, () -> controller.getRawButton(1)));
        autos.add(new SysID(SysIDMechanism.Swerve, () -> controller.getRawButton(1)));
        autos.add(
                new SequentialCommandGroup(
                        B1_A_N1_S1.command, S1_N4_S2.command, S2_N5_S2.command, S2_N6_S2.command));

        SmartDashboard.putString("Auto Routine", "0");
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    public void configureButtonBindings() {
        Swerve.get()
                .setDefaultCommand(
                        new FieldOrientedDrive(
                                () -> -controller.getRawAxis(1),
                                () -> -controller.getRawAxis(0),
                                () -> -controller.getRawAxis(4),
                                () -> controller.getPOV(),
                                () -> controller.getRawButton(1)));

        Trigger intake = new Trigger(() -> controller.getRawButton(5));
        intake.onTrue(
                new InstantCommand(
                        () -> Intake.get().setVelocity(SetpointConstants.INTAKE_TARGET_VELOCITY),
                        Intake.get()));
        // intake.onTrue(new IntakeCommand());
        intake.onFalse(new InstantCommand(() -> Intake.get().setVelocity(RPM.of(0)), Intake.get()));

        Trigger indexer = new Trigger(() -> controller.getRawButton(6));
        indexer.onTrue(
                new InstantCommand(() -> Indexer.get().setVelocity(RPM.of(1000)), Indexer.get()));
        indexer.onFalse(
                new InstantCommand(() -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()));

        // Trigger elevator = new Trigger(() -> controller.getRawButton(2));
        // elevator.onTrue(
        //         new InstantCommand(
        //                 () -> Elevator.get().setPostion(ElevatorConstants.AMP_POSITION),
        //                 Elevator.get()));
        // // elevator.onTrue(new Climb(false));

        // Elevator.get()
        //         .setDefaultCommand(
        //                 new InstantCommand(
        //                         () -> Elevator.get().setPostion(ElevatorConstants.LOAD_POSITION),
        //                         Elevator.get()));

        // Trigger pivot = new Trigger(() -> controller.getRawAxis(2) > 0.15);
        // pivot.onTrue(
        //         new InstantCommand(
        //                 () -> PlopperPivot.get().setPosition(PlopperPivotConstants.AMP_POSITION),
        //                 PlopperPivot.get()));
        // // pivot.onTrue(new ScoreAmp(false));
        // PlopperPivot.get()
        //         .setDefaultCommand(
        //                 new InstantCommand(
        //                         () ->
        //                                 PlopperPivot.get()
        //
        // .setPosition(PlopperPivotConstants.LOAD_POSITION),
        //                         PlopperPivot.get()));

        // Trigger loadPlopper = new Trigger(() -> controller.getRawButton(3));
        // loadPlopper.onTrue(new RunPlopper(true));

        // Trigger unloadPlopper = new Trigger(() -> controller.getRawButton(4));
        // unloadPlopper.onTrue(new RunPlopper(false));

        // Plopper.get()
        //         .setDefaultCommand(
        //                 new InstantCommand(
        //                         () -> Plopper.get().setVelocity(RPM.of(0)), Plopper.get()));

        Trigger shoot = new Trigger(() -> controller.getRawAxis(3) > 0.15);
        shoot.onTrue(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(2000)), Shooter.get()));
        shoot.onFalse(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0)), Shooter.get()));
    }

    public Command getAutonomousRoutine() {
        return autos.get(Integer.parseInt(SmartDashboard.getString("Auto Routine", "0")));
    }

    public static void rumbleController(double strength) {
        controller.setRumble(RumbleType.kBothRumble, strength);
    }
}
