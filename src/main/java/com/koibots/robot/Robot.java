// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.lib.dashboard.Alert;
import com.koibots.lib.sysid.SysIdTest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
    private enum AutoMode {
        TextGenerated,
        Chooser,
        SysId,
    }

    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private final SendableChooser<AutoMode> autoModeChooser = new SendableChooser<>();
    private final SendableChooser<SysIdTest> sysidRoutineChooser = new SendableChooser<>();
    private final SendableChooser<SysIdRoutine.Mechanism> sysidMechanismChooser =
            new SendableChooser<>();
    private final Field2d field = new Field2d();

    private final Alert sysIdMechanismAlert =
            new Alert("No SysId Mechanism Selected", Alert.AlertType.WARNING);
    private final Alert sysIdRoutineAlert =
            new Alert("No SysId Routine Selected", Alert.AlertType.WARNING);

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
            // Vision.get();
        }

        Logger.start();

        // Instantiate our RobotContainer.
        // This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        autoModeChooser.addOption("Text Input", AutoMode.TextGenerated);
        autoModeChooser.addOption("Legacy Selector", AutoMode.TextGenerated);
        autoModeChooser.setDefaultOption("SysId", AutoMode.TextGenerated);

        sysidMechanismChooser.addOption(
                "Swerve Drive",
                new SysIdRoutine.Mechanism(
                        (voltage) -> Swerve.get().setDriveVoltages(voltage),
                        null,
                        Swerve.get(),
                        "Swerve Drive"));

        sysidRoutineChooser.addOption("Forward Dynamic", SysIdTest.ForwardDynamic);
        sysidRoutineChooser.addOption("Reverse Dynamic", SysIdTest.ReverseDynamic);
        sysidRoutineChooser.addOption("Forward Quasistatic", SysIdTest.ForwardQuasistatic);
        sysidRoutineChooser.addOption("Reverse Quasistatic", SysIdTest.ReverseQuasistatic);

        SmartDashboard.putString("Test", "text");
        SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);
        SmartDashboard.putData("SysId Mechanism Chooser", sysidMechanismChooser);
        SmartDashboard.putData("SysId Routine Chooser", sysidRoutineChooser);
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
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        } else {
            DriverStation.reportWarning("Autonomous command is null", false);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        // Swerve.get().setBrake(true);

        robotContainer.configureButtonBindings();
    }

    @Override
    public void disabledInit() {
        Swerve.get().stop();
    }

    @Override
    public void disabledPeriodic() {
    }
}
