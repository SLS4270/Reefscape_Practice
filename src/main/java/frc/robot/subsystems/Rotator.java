package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Rotator extends SubsystemBase {

    TalonFX rotatorMotor;
    DutyCycleEncoder rotationSensor;

    public Rotator() {
        rotatorMotor = new TalonFX(Constants.rotatorID);
        rotationSensor = new DutyCycleEncoder(0);
    }

    public void runRotatorToPos(double pos) {
    }

    @Override
    public void periodic() {
        
    }
    
}
