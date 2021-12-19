package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
    // Commented out for simulations sake currently
    // private final WPI_TalonFX m_leftFront = new WPI_TalonFX(Constants.kMotor1Port);
    // private final WPI_TalonFX m_rightFront = new WPI_TalonFX(Constants.kMotor2Port);
    // private final WPI_TalonFX m_leftFollow = new WPI_TalonFX(Constants.kMotor3Port);
    // private final WPI_TalonFX m_rightFollow = new WPI_TalonFX(Constants.kMotor4Port);

    private final WPI_TalonSRX m_leftFront = new WPI_TalonSRX(Constants.kMotor1Port);
    private final WPI_TalonSRX m_rightFront = new WPI_TalonSRX(Constants.kMotor2Port);
    private final WPI_TalonSRX m_leftFollow = new WPI_TalonSRX(Constants.kMotor3Port);
    private final WPI_TalonSRX m_rightFollow = new WPI_TalonSRX(Constants.kMotor4Port);

    private final DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftFront, m_rightFront);

    AnalogGyro m_gyro = new AnalogGyro(Constants.kGyroPort);

    // Sim setup
    // TalonSRXSimCollection m_leftSim = new TalonSRXSimCollection(m_leftFront);
    // TalonSRXSimCollection m_rightSim = new TalonSRXSimCollection(m_rightFront);
    TalonSRXSimCollection m_leftSim = m_leftFront.getSimCollection();
    TalonSRXSimCollection m_rightSim = m_rightFront.getSimCollection();
    AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

    DifferentialDrivetrainSim m_differentialDrivetrainSim = new DifferentialDrivetrainSim(
        DCMotor.getCIM(Constants.kMotorsPerSide),
        Constants.kGearRatio,
        Constants.kRobotMOI,
        Constants.kRobotMass,
        Units.inchesToMeters(Constants.kWheelRadius),
        Constants.kDistanceBetweenWheels,

        // TODO: Look into simulated noise
        null
    );
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
        // Commented out for simulations sake currently
        // m_rightFront.setInverted(TalonFXInvertType.Clockwise);
        // m_leftFront.setInverted(TalonFXInvertType.CounterClockwise);
        // m_rightFollow.setInverted(TalonFXInvertType.FollowMaster);
        // m_leftFollow.setInverted(TalonFXInvertType.FollowMaster);

        SmartDashboard.putData("Field", m_field);

        // Some sim values might need to be changed too
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

    @Override
    public void simulationPeriodic() {
        m_differentialDrivetrainSim.setInputs(m_leftFront.getMotorOutputVoltage(), m_rightFront.getMotorOutputVoltage());
        m_differentialDrivetrainSim.update(Constants.kSimDelta);

        m_leftSim.setQuadratureRawPosition(distanceToSensorUnits(m_differentialDrivetrainSim.getLeftPositionMeters()));
        m_leftSim.setQuadratureVelocity(velocityToSensorUnits(m_differentialDrivetrainSim.getLeftVelocityMetersPerSecond()));
        m_rightSim.setQuadratureRawPosition(distanceToSensorUnits(m_differentialDrivetrainSim.getRightPositionMeters()));
        m_rightSim.setQuadratureVelocity(velocityToSensorUnits(m_differentialDrivetrainSim.getRightVelocityMetersPerSecond()));

        m_gyroSim.setAngle(-m_differentialDrivetrainSim.getHeading().getDegrees());

        m_leftSim.setBusVoltage(RobotController.getBatteryVoltage());
        m_rightSim.setBusVoltage(RobotController.getBatteryVoltage());
    }

    private double motorUnitsToMeters(double sensorValue) {
        return (((double) sensorValue / Constants.kEncoderTicksPerRev) / Constants.kSensorGearRatio) * (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadius));
    }

    private int velocityToSensorUnits(double velocity) {
        double rotationsPer100ms = velocity / (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadius)) * Constants.kSensorGearRatio / Constants.k100msPerSecond;
        return (int)(rotationsPer100ms * Constants.kEncoderTicksPerRev);
    }

    private int distanceToSensorUnits(double position) {
        double rotations = position / (2 * Math.PI * Units.inchesToMeters(Constants.kWheelRadius)) * Constants.kSensorGearRatio;
        return (int)(rotations * Constants.kEncoderTicksPerRev);
    }
}
