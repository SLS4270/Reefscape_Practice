package frc.robot.subsystems.Wrists;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Rotator extends SubsystemBase {

    TalonFX rotatorMotor;
    DutyCycleEncoder rotationSensor;
    CANdi candi;

    public Rotator() {
        var config = new TalonFXConfiguration();
        rotatorMotor = new TalonFX(Constants.rotatorID);
        rotatorMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationSensor = new DutyCycleEncoder(9);
        rotatorMotor.getConfigurator().apply(config.MotionMagic
            .withMotionMagicCruiseVelocity(200)
            .withMotionMagicExpo_kV(0.00001)
            .withMotionMagicExpo_kA(0.00001));
        rotatorMotor.getConfigurator().apply(new Slot0Configs().withKP(35));
        candi = new CANdi(59);
        rotatorMotor.getConfigurator().apply(new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANdiPWM1).withFeedbackRemoteSensorID(59));
        rotatorMotor.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withSupplyCurrentLimitEnable(true));
    }

    public void runRotatorToPos(double pos, double maxVelo) {
        rotatorMotor.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(maxVelo));
        rotatorMotor.setControl(new MotionMagicExpoVoltage(pos).withEnableFOC(true));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("pivotPos", rotatorMotor.getPosition().getValueAsDouble()); 
        SmartDashboard.putNumber("absEncoder", rotationSensor.get());
        SmartDashboard.putNumber("candiPosition", candi.getPWM1Position().getValueAsDouble());
    }

    
    public double getPos() {
        return rotatorMotor.getPosition().getValueAsDouble();
    }
}
