package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    TalonFX indexer;
    CANrange indexerSensor;

    public Indexer() {
        indexer = new TalonFX(Constants.indexerID);
        indexer.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true));
        indexerSensor = new CANrange(0);
    }

    public void spinIndexer(double speed) {
        indexer.set(speed);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("distance", indexerSensor.getDistance().getValueAsDouble());
    }
    
}
