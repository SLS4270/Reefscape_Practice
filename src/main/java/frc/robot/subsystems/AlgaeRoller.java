package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeRoller extends SubsystemBase {
    TalonFX rollerMotor;
    
    public AlgaeRoller() {
        rollerMotor = new TalonFX(Constants.ballIntakeID);
    }
}
