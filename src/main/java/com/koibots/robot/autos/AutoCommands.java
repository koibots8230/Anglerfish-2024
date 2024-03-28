// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.autos;

// import static com.koibots.robot.subsystems.Subsystems.Swerve;

// import com.choreo.lib.Choreo;
// import com.koibots.robot.Constants.SetpointConstants;
// import com.koibots.robot.commands.Intake.IntakeCommand;
// import com.koibots.robot.commands.Scoring.Shoot;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import java.util.function.Supplier;

// public enum AutoCommands {
//     B1_A_N1_S1(
//             followChoreoTrajectory("B1_A_N1_S1.1"),
//             // new ScoreAmp(),
//             new ParallelCommandGroup(followChoreoTrajectory("B1_A_N1_S1.2"), new
// IntakeCommand()),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S1_N4_S2(
//             followChoreoTrajectory("S1_N4_S2.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S1_N4_S2.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S2_N5_S2(
//             followChoreoTrajectory("S2_N5_S2.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S2_N5_S2.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S2_N6_S2(
//             followChoreoTrajectory("S2_N6_S2.1"),
//             new IntakeCommand(),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false),
//             followChoreoTrajectory("S2_N6_S2.2")),
//     B2_S2(
//             followChoreoTrajectory("B2_S2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S2_N1_S2(
//             followChoreoTrajectory("S2_N1_S2.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S2_N1_S2.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S2_N2_S3(
//             followChoreoTrajectory("S2_N2_S3.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S2_N2_S3.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S2_N1_S3(
//             followChoreoTrajectory("S2_N1_S3.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S2_N1_S3.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S2_N4_S2(
//             followChoreoTrajectory("S2_N4_S2.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S2_N4_S2.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     B3_S4(
//             followChoreoTrajectory("B3_S4"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S4_N3_S4(
//             followChoreoTrajectory("S4_N3_S4.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S4_N3_S4.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S4_N7_S4(
//             followChoreoTrajectory("S4_N7_S4.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S4_N7_S4.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     S4_N6_S4(
//             followChoreoTrajectory("S4_N6_S4.1"),
//             new IntakeCommand(),
//             followChoreoTrajectory("S4_N6_S4.2"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     B1_A_N4_S1(
//             followChoreoTrajectory("B1_A_N4_S1.1"),
//             // new ScoreAmp(),
//             followChoreoTrajectory("B1_A_N4_S1.2"),
//             new IntakeCommand(),
//             followChoreoTrajectory("B1_A_N4_S1.3"),
//             new Shoot(
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(0),
//                     SetpointConstants.SHOOTER_SPEEDS.get(0).get(1),
//                     false)),
//     CalibX(followChoreoTrajectory("x_calibration")),
//     CalibY(followChoreoTrajectory("y_calibration")),
//     CalibTheta(followChoreoTrajectory("theta_calibration"));

//     public final Supplier<Command> command;

//     AutoCommands(Command... commands) {
//         this.command = () -> new SequentialCommandGroup(commands);
//     }

//     private static Command followChoreoTrajectory(String trajectory) {
//         return Choreo.choreoSwerveCommand(
//                 Choreo.getTrajectory(trajectory),
//                 Swerve.get()::getEstimatedPose,
//                 Swerve.get().xController,
//                 Swerve.get().yController,
//                 Swerve.get().thetaController,
//                 Swerve.get()::driveRobotRelative,
//                 () -> {
//                     var alliance = DriverStation.getAlliance();
//                     return alliance.filter(value -> value == DriverStation.Alliance.Red)
//                             .isPresent();
//                 },
//                 Swerve.get());
//     }
// }
