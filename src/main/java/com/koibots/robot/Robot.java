// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.Swerve;

import com.koibots.lib.sysid.SysIdTest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
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
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private final SendableChooser<SysIdTest> sysidRoutineChooser = new SendableChooser<>();
    private final SendableChooser<SysIdRoutine.Mechanism> sysidMechanismChooser =
            new SendableChooser<>();

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
        }

        if (isReal()) {
            LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev);
        }

        Logger.start();

        // Instantiate our RobotContainer.
        // This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        SmartDashboard.putData("SysId Mechanism Chooser", sysidMechanismChooser);
        SmartDashboard.putData("SysId Routine Chooser", sysidRoutineChooser);
        SmartDashboard.putBoolean("Enable SysId", false);
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
        autonomousCommand = robotContainer.getAutonomousCommand();
        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
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

        robotContainer.configureButtonBindings();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        Logger.end();

        DataLogManager.start();
        URCL.start();

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
    }

    @Override
    public void testPeriodic() {
        if (SmartDashboard.getBoolean("Enable SysId", false)) {
            final var mechanism = sysidMechanismChooser.getSelected();
            if (mechanism == null) {
                DriverStation.reportError("No SysId Mechanism Selected", false);
                return;
            }

            final var routine = sysidRoutineChooser.getSelected();
            if (routine == null) {
                DriverStation.reportError("No SysId Routine Selected", false);
                return;
            }

            Command sysidCommand =
                    switch (routine) {
                        case ForwardDynamic -> new SysIdRoutine(
                                        new SysIdRoutine.Config(), mechanism)
                                .dynamic(SysIdRoutine.Direction.kForward);
                        case ReverseDynamic -> new SysIdRoutine(
                                        new SysIdRoutine.Config(), mechanism)
                                .dynamic(SysIdRoutine.Direction.kReverse);
                        case ForwardQuasistatic -> new SysIdRoutine(
                                        new SysIdRoutine.Config(), mechanism)
                                .quasistatic(SysIdRoutine.Direction.kForward);
                        case ReverseQuasistatic -> new SysIdRoutine(
                                        new SysIdRoutine.Config(), mechanism)
                                .quasistatic(SysIdRoutine.Direction.kReverse);
                    };

            if (mechanism.m_name.equals("Swerve")) {
                sysidCommand.beforeStarting(
                        () ->
                                Swerve.get()
                                        .setModuleStates(
                                                new SwerveModuleState[] {
                                                    new SwerveModuleState(0, new Rotation2d()),
                                                    new SwerveModuleState(0, new Rotation2d()),
                                                    new SwerveModuleState(0, new Rotation2d()),
                                                    new SwerveModuleState(0, new Rotation2d())
                                                }));
            }
        }
    }

    @Override
    public void testExit() {
        Logger.start();
    }

    @Override
    public void disabledInit() {
        Swerve.get().stop();
    }

    @Override
    public void simulationInit() {
        super.simulationInit();
    }

    @Override
    public void simulationPeriodic() {
        Constants.FIELD.setRobotPose(Swerve.get().getEstimatedPose());
    }
}
