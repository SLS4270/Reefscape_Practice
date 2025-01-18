package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase{

    SparkMax RollerSparkMax;
    
    public Roller() {
        RollerSparkMax = new SparkMax(5, MotorType.kBrushless);
    }

    public void SetRollerPower(double RollerPower) {
        RollerSparkMax.set(RollerPower);
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}
