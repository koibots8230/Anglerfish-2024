// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.*;

import java.util.function.BooleanSupplier;
import com.koibots.robot.commands.ElevatorControl;
import com.koibots.robot.commands.FieldOrientedDrive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        GenericHID controller;
        GenericHID oppController;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                controller = new GenericHID(0);
                oppController = new GenericHID(1);
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
         * subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
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

                Elevator.get().setDefaultCommand(new ElevatorControl(() -> controller.getRawAxis(3)));

                Intake.get()
                                .setDefaultCommand(
                                                new ConditionalCommand(
                                                                new InstantCommand(
                                                                                () -> Intake.get()
                                                                                                .setIntakeVoltsWithTargetRPM(
                                                                                                                3000),
                                                                                Intake.get()),
                                                                new InstantCommand(
                                                                                () -> Intake.get()
                                                                                                .setIntakeVoltsWithTargetRPM(
                                                                                                                0),
                                                                                Intake.get()),
                                                                () -> controller.getRawButton(5)));

                LEDs.get()
                                .setDefaultCommand(
                                                new ConditionalCommand(
                                                                new InstantCommand(
                                                                                () -> LEDs.get().writeSPI(
                                                                                                new byte[] { 0x00 }), LEDs.get()),
                                                                new ConditionalCommand(
                                                                                new InstantCommand(
                                                                                                () -> LEDs.get().writeSPI(
                                                                                                                new byte[] { 0x01 }), LEDs.get()),
                                                                                new ConditionalCommand(
                                                                                                new InstantCommand(
                                                                                                                () -> LEDs.get().writeSPI(
                                                                                                                                new byte[] { 0x10 }), LEDs.get()),
                                                                                                                                
                                                                                                new ConditionalCommand(
                                                                                                                new InstantCommand(
                                                                                                                                () -> LEDs.get().writeSPI(
                                                                                                                                                new byte[] { 0x11 }), LEDs.get()),
                                                                                                                new InstantCommand(),
                                                                                                                () -> oppController.getRawButton(
                                                                                                                                3)),
                                                                                                () -> oppController.getRawButton(
                                                                                                                2)),
                                                                                () -> oppController.getRawButton(1)),
                                                                () -> oppController.getRawButton(0)));

        }
}