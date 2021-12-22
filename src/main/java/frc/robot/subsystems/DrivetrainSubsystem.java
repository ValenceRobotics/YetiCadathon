package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_leftFront = new WPI_TalonFX(Constants.Drivetrain.kMotor1Port);
    private final WPI_TalonFX m_rightFront = new WPI_TalonFX(Constants.Drivetrain.kMotor2Port);
    private final WPI_TalonFX m_leftFollow = new WPI_TalonFX(Constants.Drivetrain.kMotor3Port);
    private final WPI_TalonFX m_rightFollow = new WPI_TalonFX(Constants.Drivetrain.kMotor4Port);

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftFront, m_rightFront);

    AnalogGyro m_gyro = new AnalogGyro(Constants.Drivetrain.kGyroPort);

    Field2d m_field = new Field2d();
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    public DrivetrainSubsystem() {
        m_leftFront.configFactoryDefault();
        m_rightFront.configFactoryDefault();
        m_leftFollow.configFactoryDefault();
        m_rightFollow.configFactoryDefault();

        m_leftFront.setNeutralMode(NeutralMode.Brake);
        m_rightFront.setNeutralMode(NeutralMode.Brake);
        m_leftFollow.setNeutralMode(NeutralMode.Brake);
        m_rightFollow.setNeutralMode(NeutralMode.Brake);

        m_leftFollow.follow(m_leftFront);
        m_rightFollow.follow(m_rightFront);

        m_rightFront.setInverted(TalonFXInvertType.CounterClockwise);
        m_leftFront.setInverted(TalonFXInvertType.Clockwise);

        m_rightFollow.setInverted(TalonFXInvertType.FollowMaster);
        m_leftFollow.setInverted(TalonFXInvertType.FollowMaster);

        m_differentialDrive.setRightSideInverted(false);


        SmartDashboard.putData("Field", m_field);
    }

    public void tankDrive(double left, double right) {
        m_differentialDrive.tankDrive(left, right);
        m_differentialDrive.feed();
    }

    public void tankDriveVolts(double left, double right) {
        m_leftFront.setVoltage(left);
        m_rightFront.setVoltage(-right);
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
        return (((double) sensorValue / Constants.Drivetrain.kEncoderTicksPerRev) / Constants.Drivetrain.kSensorGearRatio) * (2 * Math.PI * Units.inchesToMeters(Constants.Drivetrain.kWheelRadius));
    }

    public Pose2d getOdometry() {
        return m_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(motorUnitsToMeters(m_leftFront.getSelectedSensorPosition()), motorUnitsToMeters(m_rightFront.getSelectedSensorPosition()));
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }
}
