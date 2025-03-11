package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.StateCommands.CoralLevels;

public class Elevator extends SubsystemBase {

    private static CoralLevels currentLevel;
    public static CoralLevels elevatorLevel;

    TalonFX lElevator;
    TalonFX rElevator;

    public Elevator() {
        lElevator = new TalonFX(Constants.elevatorID1);
        rElevator = new TalonFX(Constants.elevatorID2);

        lElevator.setNeutralMode(NeutralModeValue.Brake);
        lElevator.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(150)
            .withMotionMagicExpo_kV(0.01)
            .withMotionMagicExpo_kA(0.001));
        lElevator.getConfigurator().apply(new Slot0Configs().withKP(1).withGravityType(GravityTypeValue.Elevator_Static).withKG(0.1));

        rElevator.setNeutralMode(NeutralModeValue.Brake);
        rElevator.getConfigurator().apply(new TalonFXConfiguration().MotionMagic
            .withMotionMagicCruiseVelocity(150)
            .withMotionMagicExpo_kV(0.01)
            .withMotionMagicExpo_kA(0.001));
        rElevator.getConfigurator().apply(new Slot0Configs().withKP(1).withGravityType(GravityTypeValue.Elevator_Static).withKG(0.1));
        rElevator.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(60)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(60)
        .withSupplyCurrentLimitEnable(true));
        currentLevel = CoralLevels.L4;
        elevatorLevel = CoralLevels.L4;
    }
    
    public void runElevatorToPos(double lPos, double rPos) {
        lElevator.setControl(new MotionMagicExpoVoltage(lPos));
        rElevator.setControl(new MotionMagicExpoVoltage(rPos));
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("lPosition", lElevator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("rPosition", rElevator.getPosition().getValueAsDouble());
        SmartDashboard.putString("Current Level", getCurrentLevel().toString());
        elevatorLevel = getCurrentLevel();
        SmartDashboard.putString("eleLevel", elevatorLevel.toString());
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
