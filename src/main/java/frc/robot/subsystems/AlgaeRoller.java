package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeRoller extends SubsystemBase {
    TalonFX rollerMotor;
    
    public AlgaeRoller() {
        rollerMotor = new TalonFX(Constants.ballIntakeID);
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(60));
    }

    public void spinAlgaeRoller(double power) {
        rollerMotor.set(power);
    }
}
