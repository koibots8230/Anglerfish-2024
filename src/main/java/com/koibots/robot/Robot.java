// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.lib.auto.AutonomousRegister;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.recordMetadata("RobotName", "Swerve Chassis");
        Logger.addDataReceiver(new WPILOGWriter());

        if (!DriverStation.isFMSAttached()) {
            Logger.addDataReceiver(new NT4Publisher());
            SmartDashboard.putData(CommandScheduler.getInstance());
        }

        if (isReal()) {
            LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev);
        }

        Logger.start();

        // Instantiate our RobotContainer.
        // This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode-specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.
        // This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods.
        // This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        AutonomousRegister.startSelectedAuto();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.configureButtonBindings();
    }

    @Override
    public void disabledInit() {
        Swerve.get().stop();
    }

    @Override
    public void disabledPeriodic() {}
}
