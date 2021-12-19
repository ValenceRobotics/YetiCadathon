package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_leftFront = new WPI_TalonFX(Constants.kMotor1Port);
    private final WPI_TalonFX m_rightFront = new WPI_TalonFX(Constants.kMotor2Port);
    private final WPI_TalonFX m_leftFollow = new WPI_TalonFX(Constants.kMotor3Port);
    private final WPI_TalonFX m_rightFollow = new WPI_TalonFX(Constants.kMotor4Port);

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftFront, m_rightFront);

    public DrivetrainSubsystem() {
        m_leftFront.configFactoryDefault();
        m_rightFront.configFactoryDefault();
        m_leftFollow.configFactoryDefault();
        m_rightFollow.configFactoryDefault();

        m_leftFollow.follow(m_leftFront);
        m_rightFollow.follow(m_rightFront);

        // Might need to be changed
        m_rightFront.setInverted(TalonFXInvertType.Clockwise);
        m_leftFront.setInverted(TalonFXInvertType.CounterClockwise);
        m_rightFollow.setInverted(TalonFXInvertType.FollowMaster);
        m_leftFollow.setInverted(TalonFXInvertType.FollowMaster);

        m_rightFront.setSensorPhase(true);
        m_leftFront.setSensorPhase(true);

        m_differentialDrive.setRightSideInverted(true);
    }

    public void tankDrive(double left, double right) {
        m_differentialDrive.tankDrive(left, right);
        m_differentialDrive.feed();
    }

    // TODO: Look into sensor value stuff
}
