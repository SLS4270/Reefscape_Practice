package frc.robot.subsystems.Intakes;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ClimbIntake extends SubsystemBase {
    TalonFX cageIntake;

    public ClimbIntake() {
        cageIntake = new TalonFX(Constants.climbIntakeID);
        cageIntake.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withSupplyCurrentLimitEnable(true));
        cageIntake.setNeutralMode(NeutralModeValue.Brake);
    }

    public void spinClimbIntake(double power) {
        cageIntake.set(power);
    }
}