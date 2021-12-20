package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final WPI_TalonFX m_intakeMotor = new WPI_TalonFX(Constants.kIntakeMotorPort);

    public IntakeSubsystem() {
        m_intakeMotor.configFactoryDefault();
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
