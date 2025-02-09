package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
            .withMotionMagicCruiseVelocity(150));
    }

    public void runWristToPos(double pos) {

    }

    @Override
    public void periodic() {
        
    }
}
