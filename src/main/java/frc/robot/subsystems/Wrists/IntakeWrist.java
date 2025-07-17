package frc.robot.subsystems.Wrists;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeWrist extends SubsystemBase{
    
    TalonFX wristIntake1;

    public IntakeWrist() {
        wristIntake1 = new TalonFX(Constants.intakeWristID1);

        wristIntake1.setNeutralMode(NeutralModeValue.Brake);
        wristIntake1.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(200)
            .withMotionMagicExpo_kV(0.00001)
            .withMotionMagicExpo_kA(0.00001));
        wristIntake1.getConfigurator().apply(new Slot0Configs().withKP(1));
        wristIntake1.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true));
    }

    public void runWristToPos(double lPos) {
        wristIntake1.setControl(new MotionMagicExpoVoltage(lPos).withEnableFOC(true));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("lPositionWrist", wristIntake1.getPosition().getValueAsDouble());
    }

    public double getLPos() {
        return wristIntake1.getPosition().getValueAsDouble();
    }
}
