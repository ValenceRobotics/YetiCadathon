package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_leftFront = new WPI_TalonFX(Constants.kMotor1Port);
    private final WPI_TalonFX m_rightFront = new WPI_TalonFX(Constants.kMotor2Port);
    private final WPI_TalonFX m_leftFollow = new WPI_TalonFX(Constants.kMotor3Port);
    private final WPI_TalonFX m_rightFollow = new WPI_TalonFX(Constants.kMotor4Port);

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftFront, m_rightFront);

    AnalogGyro m_gyro = new AnalogGyro(Constants.kGyroPort);

    Field2d m_field = new Field2d();
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

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

        SmartDashboard.putData("Field", m_field);

        // Some values of this might need to be changed too
        m_rightFront.setSensorPhase(true);
        m_leftFront.setSensorPhase(true);

        m_differentialDrive.setRightSideInverted(true);
    }

    public void tankDrive(double left, double right) {
        m_differentialDrive.tankDrive(left, right);
        m_differentialDrive.feed();
    }

    @Override
    public void periodic() {
        m_odometry.update(m_gyro.getRotation2d(),
         motorUnitsToMeters(m_leftFront.getSelectedSensorPosition()),
         motorUnitsToMeters(m_rightFront.getSelectedSensorPosition()));

         m_field.setRobotPose(m_odometry.getPoseMeters());
    }

    private double motorUnitsToMeters(double sensorValue) {
        return (((double) sensorValue / Constants.kEncoderTicksPerRev) / Constants.kSensorGearRatio) * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadius));
    }
}
