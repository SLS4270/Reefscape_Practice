package frc.robot.subsystems.Intakes;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Indexer extends SubsystemBase {
    TalonFX indexer;

    public Indexer() {
        indexer = new TalonFX(Constants.indexerID);
        indexer.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true));
    }

    public void spinIndexer(double speed) {
        indexer.setControl(new DutyCycleOut(speed).withEnableFOC(true));
    }
  
}
