package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.robot.commands.teleop.SwerveCommand;
import com.koibots.robot.subsystems.ShooterPivot.ShooterPivot;
import com.koibots.robot.subsystems.controller.ControllerIO;
import com.koibots.robot.subsystems.controller.ControllerIOPS5;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final CommandPS4Controller m_operatorController = new CommandPS4Controller(1);
    LoggedDashboardChooser<Supplier<ControllerIO>> controllerChooser;

    // Graph of algorithms here: https://www.desmos.com/calculator/w738aldioj
    enum ScalingAlgorithm {
        Linear((x) -> x),
        Squared((x) -> Math.signum(x) * x * x),
        Cubed((x) -> x * x * x),
        Cosine((x) -> (-Math.signum(x) * Math.cos(Math.PI * 0.5 * x)) + (1 * Math.signum(x))),
        CubedSquareRoot((x) -> Math.signum(x) * Math.sqrt(Math.abs(x * x * x)));

        public final Function<Double, Double> algorithm;

        private ScalingAlgorithm(Function<Double, Double> algorithm) {
            this.algorithm = algorithm;
        }
    }

    LoggedDashboardChooser<ScalingAlgorithm> scalingChooser = new LoggedDashboardChooser<>("Scaling Algorithm");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(Robot.Mode mode) {
        switch (mode) {
            case REPLAY:
                DriverStation.reportError("Replay not supported", false);
                throw new RuntimeException("Replay not supported");
            case REAL:
            case SIM:
                controllerChooser = new LoggedDashboardChooser<>("Controller Chooser");

                controllerChooser.addDefaultOption("PS5 Controller", ControllerIOPS5::new);

                scalingChooser.addDefaultOption("Linear", ScalingAlgorithm.Linear);
                scalingChooser.addOption("Squared", ScalingAlgorithm.Squared);
                scalingChooser.addOption("Cubed", ScalingAlgorithm.Cubed);
                scalingChooser.addOption("Cosine", ScalingAlgorithm.Cosine);
                scalingChooser.addOption("Fancy", ScalingAlgorithm.CubedSquareRoot);

                break;
        }
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
        ControllerIO controller = controllerChooser.get().get();
        ShooterPivot shooterPivot = new ShooterPivot();

        Swerve.get().setDefaultCommand(new SwerveCommand(
                controller::xTranslation,
                controller::yTranslation,
                controller::angularVelocity,
                controller::anglePosition,
                controller::cross,
                scalingChooser.get().algorithm));
        
        Trigger shooterAmp = m_operatorController.L2();
        Trigger shooterSpeaker = m_operatorController.R2();
        Trigger shooterLoad = m_operatorController.cross();

        shooterAmp.whileTrue(new InstantCommand(() -> shooterPivot.setShooterPosition(Constants.AMP_SHOOTER_RADIANS)));
        shooterSpeaker.whileTrue(new InstantCommand(() -> shooterPivot.setShooterPosition(Constants.SPEAKER_SHOOTER_RADIANS)));
        shooterLoad.whileTrue(new InstantCommand(() -> shooterPivot.setShooterPosition(Constants.LOAD_SHOOTER_RADIANS)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}