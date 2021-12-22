package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_intakeMotor;

    public IntakeSubsystem(int motorPort) {
        m_intakeMotor = new WPI_TalonFX(motorPort);
        m_intakeMotor.configFactoryDefault();
        m_intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void Intake(double speed) {
        m_intakeMotor.set(speed);
    }

    public void Outtake(double speed) {
        m_intakeMotor.set(-speed);
    }

    public void Reset() {
        m_intakeMotor.set(0);
    }
}
