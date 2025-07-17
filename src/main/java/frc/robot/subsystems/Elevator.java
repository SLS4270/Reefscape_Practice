package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CoralLevels;

public class Elevator extends SubsystemBase {

    private static CoralLevels currentLevel;
    public static CoralLevels rElevatorLevel;

    TalonFX lElevator;
    TalonFX rElevator;

    public Elevator() {
        lElevator = new TalonFX(Constants.rElevatorID1);
        rElevator = new TalonFX(Constants.rElevatorID2);

        lElevator.setNeutralMode(NeutralModeValue.Brake);
        lElevator.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(300)
            .withMotionMagicExpo_kV(0.0001)
            .withMotionMagicExpo_kA(0.00001));
        lElevator.getConfigurator().apply(new Slot0Configs().withKP(1).withGravityType(GravityTypeValue.Elevator_Static).withKG(0.5));
        lElevator.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true));

        rElevator.setNeutralMode(NeutralModeValue.Brake);
        rElevator.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(300)
            .withMotionMagicExpo_kV(0.0001)
            .withMotionMagicExpo_kA(0.00001));
        rElevator.getConfigurator().apply(new Slot0Configs().withKP(1).withGravityType(GravityTypeValue.Elevator_Static).withKG(0.5));
        rElevator.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(40)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40)
        .withSupplyCurrentLimitEnable(true));
        currentLevel = CoralLevels.L4;
        rElevatorLevel = CoralLevels.L4;

        lElevator.setControl(new Follower(Constants.rElevatorID2, true));
    }
    
    public void runElevatorToPos(double rPos) {
        rElevator.setControl(new MotionMagicExpoVoltage(rPos).withEnableFOC(true));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("lPosition", lElevator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("rPosition", rElevator.getPosition().getValueAsDouble());
        SmartDashboard.putString("Current Level", getCurrentLevel().toString());
        rElevatorLevel = getCurrentLevel();
        SmartDashboard.putString("rEleLevel", rElevatorLevel.toString());
    }

    public static void setCurrentLevel(CoralLevels level) {
        currentLevel = level;
    }

    public static CoralLevels getCurrentLevel() {
        return currentLevel;
    }

    public double getLPos() {
        return lElevator.getPosition().getValueAsDouble();
    }

    public double getRPos() {
        return rElevator.getPosition().getValueAsDouble();
    }
}
