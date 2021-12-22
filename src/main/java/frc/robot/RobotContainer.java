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
import frc.robot.subsystems.HingeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  // Subsystems
  private final IntakeSubsystem m_intakeSubsystemFront = new IntakeSubsystem(Constants.kIntakeMotorPortFront);
  private final IntakeSubsystem m_intakeSubsystemBack = new IntakeSubsystem(Constants.kIntakeMotorPortBack);

  private final HingeSubsystem m_hingeSubsystemFront = new HingeSubsystem(Constants.kHingeMotorPortFront);
  private final HingeSubsystem m_hingeSubsystemBack = new HingeSubsystem(Constants.kHingeMotorPortBack);

  // Buttons
  private final Button m_buttonIntakeFront = new JoystickButton(m_joystick1, Constants.OI.kIntakeButtonFront);
  private final Button m_buttonOuttakeFront = new JoystickButton(m_joystick1, Constants.OI.kOutTakeButtonFront);

  private final Button m_buttonHingeDownFront = new JoystickButton(m_joystick1, Constants.OI.kHingeDownButtonFront);
  private final Button m_buttonHingeUpFront = new JoystickButton(m_joystick1, Constants.OI.kHingeUpButtonFront);

  private final Button m_buttonIntakeBack = new JoystickButton(m_joystick2, Constants.OI.kIntakeButtonBack);
  private final Button m_buttonOuttakeBack = new JoystickButton(m_joystick2, Constants.OI.kOutTakeButtonBack);

  private final Button m_buttonHingeDownBack = new JoystickButton(m_joystick1, Constants.OI.kHingeDownButtonBack);
  private final Button m_buttonHingeUpBack = new JoystickButton(m_joystick1, Constants.OI.kHingeUpButtonBack);

  // TODO: Move these into a commands file
  private final Command m_tankDriveCommand = new RunCommand(
      () -> m_drivetrainSubsystem.tankDrive(m_joystick1.getY(), m_joystick2.getY()), m_drivetrainSubsystem);

  private final Command m_intakeCommandFront = new RunCommand(
    () -> m_intakeSubsystemFront.Intake(Constants.kIntakeMotorSpeed), m_intakeSubsystemFront);
  private final Command m_outtakeCommandFront = new RunCommand(
      () -> m_intakeSubsystemFront.Outtake(Constants.kIntakeMotorSpeed), m_intakeSubsystemFront);
  private final Command m_defaultIntakeCommandFront = new RunCommand(
    () -> m_intakeSubsystemFront.Reset(),  
    m_intakeSubsystemFront);

  private final Command m_intakeCommandBack = new RunCommand(
    () -> m_intakeSubsystemBack.Intake(Constants.kIntakeMotorSpeed), m_intakeSubsystemBack);
  private final Command m_outtakeCommandBack = new RunCommand(
      () -> m_intakeSubsystemBack.Outtake(Constants.kIntakeMotorSpeed), m_intakeSubsystemBack);
  private final Command m_defaultIntakeCommandBack = new RunCommand(
    () -> m_intakeSubsystemBack.Reset(),  
    m_intakeSubsystemBack);

    private final Command m_hingeDownCommandFront = new RunCommand(
      () -> m_hingeSubsystemFront.Down(Constants.kHingeMotorSpeed), m_hingeSubsystemFront);
    private final Command m_hingeUpCommandFront = new RunCommand(
        () -> m_hingeSubsystemFront.Up(Constants.kHingeMotorSpeed), m_hingeSubsystemFront);
    private final Command m_defaultHingeCommandFront = new RunCommand(
      () -> m_hingeSubsystemFront.Reset(),  
      m_hingeSubsystemFront);

    private final Command m_hingeDownCommandBack = new RunCommand(
      () -> m_hingeSubsystemBack.Down(Constants.kHingeMotorSpeed), m_hingeSubsystemBack);
    private final Command m_hingeUpCommandBack = new RunCommand(
        () -> m_hingeSubsystemBack.Up(Constants.kHingeMotorSpeed), m_hingeSubsystemBack);
    private final Command m_defaultHingeCommandBack = new RunCommand(
      () -> m_hingeSubsystemBack.Reset(),  
      m_hingeSubsystemBack);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();

    m_drivetrainSubsystem.setDefaultCommand(m_tankDriveCommand);

    m_intakeSubsystemFront.setDefaultCommand(m_defaultIntakeCommandFront);
    m_intakeSubsystemBack.setDefaultCommand(m_defaultIntakeCommandBack);

    m_hingeSubsystemFront.setDefaultCommand(m_defaultHingeCommandFront);
    m_hingeSubsystemBack.setDefaultCommand(m_defaultHingeCommandBack);
  }

  /**
   * Button configuration
   */
  private void configureButtonBindings() {
    m_buttonIntakeFront.whileHeld(m_intakeCommandFront);
    m_buttonOuttakeFront.whileHeld(m_outtakeCommandFront);

    m_buttonIntakeBack.whileHeld(m_intakeCommandBack);
    m_buttonOuttakeBack.whileHeld(m_outtakeCommandBack);

    m_buttonHingeDownFront.whileHeld(m_hingeDownCommandFront);
    m_buttonHingeUpFront.whileHeld(m_hingeUpCommandFront);

    m_buttonHingeDownBack.whileHeld(m_hingeDownCommandBack);
    m_buttonHingeUpBack.whileHeld(m_hingeUpCommandBack);
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
      
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
          new Translation2d(10, -5),
          new Translation2d(17, -9)
        ), 
        new Pose2d(-19, -9, new Rotation2d(0)), 
        trajectoryConfig);

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

      return new 
        // Intake front ball
        InstantCommand(() -> m_hingeSubsystemFront.Down(Constants.kHingeMotorSpeed))
        .andThen(new WaitCommand(Constants.kAutoHingeTime))
        .andThen(() -> m_hingeSubsystemFront.Reset())
        .andThen(() -> m_intakeSubsystemFront.Intake(Constants.kIntakeMotorSpeed))
        .andThen(new WaitCommand(Constants.kAutoIntakeTime))
        .andThen(() -> m_intakeSubsystemFront.Reset())
        // Intake back ball
        .andThen(() -> m_hingeSubsystemBack.Down(Constants.kHingeMotorSpeed))
        .andThen(new WaitCommand(Constants.kAutoHingeTime))
        .andThen(() -> m_hingeSubsystemBack.Reset())
        .andThen(() -> m_intakeSubsystemBack.Intake(Constants.kIntakeMotorSpeed))
        .andThen(new WaitCommand(Constants.kAutoIntakeTime))
        .andThen(() -> m_intakeSubsystemBack.Reset())
        // Follow trajectory
        .andThen(ramseteCommand)
        .andThen(() -> m_drivetrainSubsystem.tankDriveVolts(0, 0))
        // OutTake all balls into low goal
        .andThen(() -> m_hingeSubsystemFront.Down(Constants.kAutoHingeTime))
        .andThen(new WaitCommand(Constants.kAutoHingeTime))
        .andThen(() -> m_hingeSubsystemFront.Reset())
        .andThen(() -> m_intakeSubsystemFront.Outtake(Constants.kIntakeMotorSpeed))
        .andThen(() -> m_intakeSubsystemBack.Intake(Constants.kIntakeMotorSpeed))
        .andThen(new WaitCommand(Constants.kAutoIntakeTime * 2))
        .andThen(() -> m_intakeSubsystemFront.Reset())
        .andThen(() -> m_intakeSubsystemBack.Reset());
  }
}
