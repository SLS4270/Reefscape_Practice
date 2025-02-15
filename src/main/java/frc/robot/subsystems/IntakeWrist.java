package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeWrist extends SubsystemBase{
    
    TalonFX wristIntake1;
    TalonFX wristIntake2;

    public IntakeWrist() {
        wristIntake1 = new TalonFX(Constants.intakeWristID1);
        wristIntake2 = new TalonFX(Constants.intakeWristID2);

        wristIntake1.setNeutralMode(NeutralModeValue.Brake);
        wristIntake1.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(200)
            .withMotionMagicExpo_kV(0.00001)
            .withMotionMagicExpo_kA(0.00001));
        wristIntake1.getConfigurator().apply(new Slot0Configs().withKP(1));

        wristIntake2.setNeutralMode(NeutralModeValue.Brake);
        wristIntake2.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
        .withMotionMagicCruiseVelocity(200)
        .withMotionMagicExpo_kV(0.00001)
        .withMotionMagicExpo_kA(0.00001));
        wristIntake2.getConfigurator().apply(new Slot0Configs().withKP(1));
    }

    public void runWristToPos(double lPos, double rPos) {
        wristIntake1.setControl(new MotionMagicExpoVoltage(lPos));
        wristIntake2.setControl(new MotionMagicExpoVoltage(rPos));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("lPositionWrist", wristIntake1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("rPositionWrist", wristIntake2.getPosition().getValueAsDouble());
    }

    //stow: L: -3.3 R: 3.8
    //intake L: -31.8 R: 32.5
}
