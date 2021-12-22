package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.HingeSubsystem;

public class HingeCommand extends PIDCommand {
    public HingeCommand(double targetDegrees, HingeSubsystem hingeSubsystem) {
        super(
            new PIDController(Constants.Hinge.kHingeSetP, Constants.Hinge.kHingeSetI, Constants.Hinge.kHingeSetD),
            hingeSubsystem::GetEncoderDistance,
            targetDegrees,
            output -> hingeSubsystem.Down(output),
            hingeSubsystem
        );
    
        getController().enableContinuousInput(0, 360);

        getController().setTolerance(Constants.Hinge.kHingeSetTolerance);
    }
    
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }
}
