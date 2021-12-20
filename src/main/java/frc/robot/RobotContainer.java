// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    // TODO: Make an autonomous command
    return null;
  }
}
