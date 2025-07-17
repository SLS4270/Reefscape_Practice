package frc.robot.subsystems.Wrists;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climb extends SubsystemBase {

    TalonFX climb;

    public Climb() {
        climb = new TalonFX(Constants.climbID);

        climb.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(200)
            .withMotionMagicExpo_kV(0.00001)
            .withMotionMagicExpo_kA(0.00001));

        climb.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        climb.getConfigurator().apply(new Slot0Configs().withKP(1));
        climb.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climbPos", climb.getPosition().getValueAsDouble());
    }

    public void setClimbPower(double speed) {
        climb.set(speed);
    }

    public void setClimbToPos(double pos) {
        climb.setControl(new MotionMagicExpoVoltage(pos));
    }
}
