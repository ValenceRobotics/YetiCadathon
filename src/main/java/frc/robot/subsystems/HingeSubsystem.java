package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HingeSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_hingeMotor;
    private final DutyCycleEncoder m_encoder;

    public HingeSubsystem(int motorPort, int encoderPort) {
        m_hingeMotor = new WPI_TalonFX(motorPort);
        m_hingeMotor.configFactoryDefault();
        m_hingeMotor.setNeutralMode(NeutralMode.Brake);

        m_encoder = new DutyCycleEncoder(encoderPort);
        m_encoder.reset();
        m_encoder.setDistancePerRotation(360);
    }

    public void Down(double speed) {
        m_hingeMotor.set(speed);
    }

    public void Up(double speed) {
        m_hingeMotor.set(-speed);
    }

    public void Reset() {
        m_hingeMotor.set(0);
    }

    public double GetEncoderDistance() {
        return m_encoder.getDistance();
    }
}
