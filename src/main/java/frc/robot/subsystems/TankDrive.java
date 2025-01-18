package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase{

    SparkMax lSparkMax;
    SparkMax rSparkMax;
    
    public TankDrive() {
        lSparkMax = new SparkMax(1, MotorType.kBrushless);
        rSparkMax = new SparkMax(3, MotorType.kBrushless);
    }

    public void setDrivePower(double leftSide, double rightSide) {
        lSparkMax.set(leftSide);
        rSparkMax.set(rightSide);
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}
