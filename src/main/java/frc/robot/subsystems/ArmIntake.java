package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmIntake extends SubsystemBase{
    SparkMax armIntake;
    CANrange rangeFinder;

    public ArmIntake() {
        armIntake = new SparkMax(0, MotorType.kBrushless);
        rangeFinder = new CANrange(40);
    }

    public void spinArmIntake(double power) {

    }

    @Override
    public void periodic() {
        
    }
}
