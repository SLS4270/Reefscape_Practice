package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    TalonFX intake;

    public Intake() {
        intake = new TalonFX(Constants.intakeID);
        intake.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true));
        intake.setNeutralMode(NeutralModeValue.Brake);
    }

    public void spinIntake(double power) {
        intake.set(power);
    }
}