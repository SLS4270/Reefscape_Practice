package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    TalonFX lElevator;
    TalonFX rElevator;

    public Elevator() {
        lElevator = new TalonFX(Constants.elevatorID1);
        rElevator = new TalonFX(Constants.elevatorID2);
    }
    
    public void runElevatorToPos(double pos) {

    }

    @Override
    public void periodic() {
        
    }
}
