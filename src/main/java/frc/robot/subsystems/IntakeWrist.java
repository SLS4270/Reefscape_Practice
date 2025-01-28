package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWrist extends SubsystemBase{
    
    TalonFX wristIntake1;
    TalonFX wristIntake2;

    public IntakeWrist() {
        wristIntake1 = new TalonFX(Constants.intakeWristID1);
        wristIntake2 = new TalonFX(Constants.intakeWristID2);
    }

    public void runWristToPos(double pos) {

    }

    @Override
    public void periodic() {
        
    }
}
