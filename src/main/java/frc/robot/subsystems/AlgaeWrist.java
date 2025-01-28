package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeWrist extends SubsystemBase {
    TalonFX algaeWrist;

    public AlgaeWrist() {
        algaeWrist = new TalonFX(Constants.ballWristID);
    }

}
