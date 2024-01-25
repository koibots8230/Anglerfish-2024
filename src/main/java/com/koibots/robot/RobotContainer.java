// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.robot.commands.FieldOrientedDrive;
import com.koibots.robot.subsystems.controller.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.Function;
import java.util.function.Supplier;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    SendableChooser<Supplier<ControllerIO>> controllerChooser;

    // Graph of algorithms here: https://www.desmos.com/calculator/w738aldioj
    enum ScalingAlgorithm {
        Linear((x) -> x),
        Squared((x) -> Math.signum(x) * x * x),
        Cubed((x) -> x * x * x),
        Cosine((x) -> (-Math.signum(x) * Math.cos(Math.PI * 0.5 * x)) + (1 * Math.signum(x))),
        CubedSquareRoot((x) -> Math.signum(x) * Math.sqrt(Math.abs(x * x * x)));

        public final Function<Double, Double> algorithm;

        ScalingAlgorithm(Function<Double, Double> algorithm) {
            this.algorithm = algorithm;
        }
    }

    SendableChooser<ScalingAlgorithm> scalingChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        controllerChooser = new SendableChooser<>();

        controllerChooser.setDefaultOption("PS5 Controller", ControllerIOPS5::new);
        controllerChooser.addOption("Xbox Controller", ControllerIOXbox::new);
        // TODO: Implement
        // each of these controllers
        // controllerChooser.addOption("Drone Controller", ControllerIODrone::new);
        // controllerChooser.addOption("Flight Controller", ControllerIOJoystick::new);

        scalingChooser.setDefaultOption("Linear", ScalingAlgorithm.Linear);
        scalingChooser.addOption("Squared", ScalingAlgorithm.Squared);
        scalingChooser.addOption("Cubed", ScalingAlgorithm.Cubed);
        scalingChooser.addOption("Cosine", ScalingAlgorithm.Cosine);
        scalingChooser.addOption("Fancy", ScalingAlgorithm.CubedSquareRoot);

        SmartDashboard.putData("Scaling Algorithm", scalingChooser);
        SmartDashboard.putData("Controller Chooser", controllerChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    public void configureButtonBindings() {
        ControllerIO controller = controllerChooser.getSelected().get();

        Swerve.get()
                .setDefaultCommand(
                        new FieldOrientedDrive(
                                controller::xTranslation,
                                controller::yTranslation,
                                controller::angularVelocity,
                                controller::anglePosition,
                                controller::cross,
                                scalingChooser.getSelected().algorithm));
    }
}
