package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmIntake extends SubsystemBase{
    TalonFX armIntake;
    CANrange rangeFinder;
    public static boolean objectInClaw;

    public ArmIntake() {
        armIntake = new TalonFX(Constants.armIntakeID);
        armIntake.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withSupplyCurrentLimitEnable(true));
        rangeFinder = new CANrange(40);
        rangeFinder.getConfigurator().apply(new ProximityParamsConfigs().withProximityThreshold(0.037));
        objectInClaw = false;
    }

    public void spinArmIntake(double power) {
        armIntake.set(power);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance", rangeFinder.getDistance().getValueAsDouble());
        objectInClaw = rangeFinder.getIsDetected().getValue();
    }
}
