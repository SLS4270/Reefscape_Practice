package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase{

    SparkMax lSparkMax;
    SparkMax rSparkMax;
    
    public TankDrive() {

    }

    public void setDrivePower(double leftSide, double rightSide) {

    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}
