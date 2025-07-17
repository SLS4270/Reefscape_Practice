package frc.robot.subsystems.Intakes;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class L1Intake extends SubsystemBase {
    TalonFX intake;

    public L1Intake() {
        intake = new TalonFX(Constants.l1IntakeID);
        intake.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withSupplyCurrentLimitEnable(true));
        intake.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        intake.setNeutralMode(NeutralModeValue.Brake);
    }

    public void spinIntake(double power) {
        intake.setControl(new DutyCycleOut(power).withEnableFOC(true));
    }
}