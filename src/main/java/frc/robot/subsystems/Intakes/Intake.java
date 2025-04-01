package frc.robot.subsystems.Intakes;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Intake extends SubsystemBase {
    TalonFX intake;

    public Intake() {
        intake = new TalonFX(Constants.intakeID);
        intake.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withSupplyCurrentLimitEnable(true));
        intake.setNeutralMode(NeutralModeValue.Brake);
        intake.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    }

    public void spinIntake(double power) {
        intake.set(power);
    }
}