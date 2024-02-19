// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.autos.AutoCommands.*;
import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.sysid.SysIDMechanism;
import com.koibots.robot.Constants.*;
import com.koibots.robot.autos.SysID;
import com.koibots.robot.commands.Intake.IntakeCommand;
import com.koibots.robot.commands.Intake.RunIndexer;
import com.koibots.robot.commands.Ploppervator.RunPlopper;
import com.koibots.robot.commands.Scoring.Climb;
import com.koibots.robot.commands.Scoring.ScoreAmp;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

        Intake.get()
                .setDefaultCommand(
                        new ConditionalCommand(
                                new InstantCommand(
                                        () ->
                                                Intake.get()
                                                        .setVelocity(
                                                                IntakeConstants.TARGET_VELOCITY),
                                        Intake.get()),
                                new InstantCommand(
                                        () -> Intake.get().setVelocity(RPM.of(0)), Intake.get()),
                                () -> controller.getRawButton(5)));

        // Intake.get()
        //         .setDefaultCommand(
        //                 new ConditionalCommand(
        //                         new IntakeCommand(false),
        //                         new InstantCommand(
        //                                 () -> Intake.get().setVelocity(RPM.of(0)), Intake.get()),
        //                         () -> controller.getRawButton(4)));

        Indexer.get()
                .setDefaultCommand(
                        new ConditionalCommand(
                                new RunIndexer(),
                                new InstantCommand(
                                        () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
                                () -> controller.getRawButton(6)));

        Elevator.get()
                .setDefaultCommand(
                        new ConditionalCommand(
                                new InstantCommand(
                                        () ->
                                                Elevator.get()
                                                        .setPostion(ElevatorConstants.AMP_POSITION),
                                        Elevator.get()),
                                new InstantCommand(
                                        () ->
                                                Elevator.get()
                                                        .setPostion(
                                                                ElevatorConstants.LOAD_POSITION),
                                        Elevator.get()),
                                () -> controller.getRawButton(2)));
        
        // Elevator.get()
        //         .setDefaultCommand(
        //                 new ConditionalCommand(
        //                         new Climb(false),
        //                         new InstantCommand(
        //                                 () ->
        //                                         Elevator.get()
        //                                                 .setPostion(
        //                                                         ElevatorConstants.LOAD_POSITION),
        //                                 Elevator.get()),
        //                         () -> controller.getRawButton(0)));

        PlopperPivot.get()
                .setDefaultCommand(
                        new ConditionalCommand(
                                new InstantCommand(
                                        () ->
                                                PlopperPivot.get()
                                                        .setPosition(
                                                                PlopperPivotConstants.AMP_POSITION),
                                        PlopperPivot.get()),
                                new InstantCommand(
                                        () ->
                                                PlopperPivot.get()
                                                        .setPosition(
                                                                PlopperPivotConstants
                                                                        .LOAD_POSITION),
                                        PlopperPivot.get()),
                                () -> controller.getRawAxis(2) > 0.15));
        
        // PlopperPivot.get()
        //         .setDefaultCommand(
        //                 new ConditionalCommand(
        //                         new ScoreAmp(false),
        //                         new InstantCommand(
        //                                 () ->
        //                                         PlopperPivot.get()
        //                                                 .setPosition(
        //                                                         PlopperPivotConstants
        //                                                                 .LOAD_POSITION),
        //                                 PlopperPivot.get()),
        //                         () -> controller.getRawAxis(2) > 0.15));

        Plopper.get()
                .setDefaultCommand(
                        new ConditionalCommand(
                                new RunPlopper(true),
                                new ConditionalCommand(
                                        new RunPlopper(false),
                                        new InstantCommand(
                                                () -> Plopper.get().setVelocity(RPM.of(0)),
                                                Plopper.get()),
                                        () -> controller.getRawButton(4)),
                                () -> controller.getRawButton(3)));

        Shooter.get()
                .setDefaultCommand(
                        new ConditionalCommand(
                                new InstantCommand(
                                        () -> Shooter.get().setVelocity(ShooterConstants.SPEED),
                                        Shooter.get()),
                                new InstantCommand(
                                        () -> Shooter.get().setVelocity(RPM.of(0)), Shooter.get()),
                                () -> controller.getRawAxis(3) > 0.15));

        // Shooter.get()
        //         .setDefaultCommand(
        //                 new ConditionalCommand(
        //                         new Shoot(ShooterConstants.SPEED, false),
        //                         new InstantCommand(
        //                                 () -> Shooter.get().setVelocity(RPM.of(0)),
        // Shooter.get()),
        //                         () -> controller.getRawAxis(3) > 0.15));
    }

    public Command getAutonomousRoutine() {
        return autos.get(Integer.parseInt(SmartDashboard.getString("Auto Routine", "0")));
    }

    public static void rumbleController(double strength) {
        controller.setRumble(RumbleType.kBothRumble, strength);
    }
}
