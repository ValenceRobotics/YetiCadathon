// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Joystick m_joystick1 = new Joystick(Constants.OI.kJoystick1Port);
  private final Joystick m_joystick2 = new Joystick(Constants.OI.kJoystick2Port);
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final Button m_buttonIntake = new JoystickButton(m_joystick1, Constants.OI.kIntakeButton);
  private final Button m_buttonOuttake = new JoystickButton(m_joystick1, Constants.OI.kOutTakeButton);

  private final Command m_tankDriveCommand = new RunCommand(
      () -> m_drivetrainSubsystem.tankDrive(m_joystick1.getY(), m_joystick2.getY()), m_drivetrainSubsystem);
  private final Command m_intakeCommand = new RunCommand(
    () -> m_intakeSubsystem.Intake(Constants.kIntakeMotorSpeed), m_intakeSubsystem);
  private final Command m_outtakeCommand = new RunCommand(
      () -> m_intakeSubsystem.Outtake(Constants.kIntakeMotorSpeed), m_intakeSubsystem);
  private final Command m_defaultIntakeCommand = new RunCommand(
    () -> m_intakeSubsystem.Reset(),  
  m_intakeSubsystem);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    m_drivetrainSubsystem.setDefaultCommand(m_tankDriveCommand);
    m_intakeSubsystem.setDefaultCommand(m_defaultIntakeCommand);
  }

  /**
   * Button configuration
   */
  private void configureButtonBindings() {
    m_buttonIntake.whileHeld(m_intakeCommand);
    m_buttonOuttake.whileHeld(m_outtakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltsSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      10);
    
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond, 
        Constants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.kDriveKinematics)
        .addConstraint(voltageConstraint);
      
      // TODO: Actually make this
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
        ), 
        new Pose2d(3, 0, new Rotation2d(0)), 
        trajectoryConfig);
      // = TrajectoryGenerator.generateTrajectory(waypoints, config)

      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        m_drivetrainSubsystem::getOdometry, 
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltsSecondsSquaredPerMeter),
        Constants.kDriveKinematics, 
        m_drivetrainSubsystem::getWheelSpeeds, 
        new PIDController(Constants.kPDriveVel, 0.0 , 0.0), 
        new PIDController(Constants.kPDriveVel, 0.0 , 0.0), 
        m_drivetrainSubsystem::tankDriveVolts, 
        m_drivetrainSubsystem);

      m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose());

      return ramseteCommand
        .andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0))
        .andThen(() -> m_intakeSubsystem.Outtake(Constants.kIntakeMotorSpeed))
        .andThen(new WaitCommand(Constants.kAutoOuttakeTime))
        .andThen(() -> m_intakeSubsystem.Reset());
  }
}
