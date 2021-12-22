package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HingeSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_hingeMotor;

    public HingeSubsystem(int motorPort) {
        m_hingeMotor = new WPI_TalonFX(motorPort);
        m_hingeMotor.configFactoryDefault();
        m_hingeMotor.setNeutralMode(NeutralMode.Brake);
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
}
