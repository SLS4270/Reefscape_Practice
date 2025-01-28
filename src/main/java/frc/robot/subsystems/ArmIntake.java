package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmIntake extends SubsystemBase{
    SparkMax armIntake;
    CANrange rangeFinder;

    public ArmIntake() {
        armIntake = new SparkMax(Constants.armIntakeID, MotorType.kBrushless);
        rangeFinder = new CANrange(40);
    }

    public void spinArmIntake(double power) {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance", rangeFinder.getDistance().getValueAsDouble());
    }
}
