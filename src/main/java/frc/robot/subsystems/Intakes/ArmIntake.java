package frc.robot.subsystems.Intakes;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ArmIntake extends SubsystemBase{
    TalonFX armIntake;
    CANrange rangeFinder;
    public static boolean objectInClaw;
    private static CoralIntakeState intakeState;
    public static CoralIntakeState coralIntakeState;

    private static BallIntakeState ballIntakeState;
    public static BallIntakeState publicBallIntakeState;
    
    public static enum CoralIntakeState {
        CoralIntaking,
        NotCoralIntaking
    }

    public static enum BallIntakeState {
        BallIntaking,
        NotBallIntaking
    }

    public ArmIntake() {
        armIntake = new TalonFX(Constants.armIntakeID);
        armIntake.setNeutralMode(NeutralModeValue.Brake);
        armIntake.getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(50)
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimit(50)
        .withSupplyCurrentLimitEnable(true));
        rangeFinder = new CANrange(47);
        rangeFinder.getConfigurator().apply(new ProximityParamsConfigs()
        .withProximityThreshold(0.08)
        .withMinSignalStrengthForValidMeasurement(0.02)
        .withProximityHysteresis(0.039));
        objectInClaw = false;
        coralIntakeState = CoralIntakeState.NotCoralIntaking;
        
    }

    public void spinArmIntake(double power) {
        armIntake.setControl(new DutyCycleOut(power).withEnableFOC(true));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("distance", rangeFinder.getDistance().getValueAsDouble());
        objectInClaw = rangeFinder.getIsDetected().getValue();

        coralIntakeState = getCoralIntakeState();
        publicBallIntakeState = getBallIntakeState();
        SmartDashboard.putBoolean("ObjectInClaw", objectInClaw);
        SmartDashboard.putBoolean("isCoralIntaking", coralIntakeState == CoralIntakeState.CoralIntaking);
    }

    public static void setCoralIntakeState(CoralIntakeState state) {
        intakeState = state;
    }

    private CoralIntakeState getCoralIntakeState() {
        return intakeState;
    }

    public static void setBallIntakeState(BallIntakeState state) {
        ballIntakeState = state;
    }

    private BallIntakeState getBallIntakeState() {
        return ballIntakeState;
    }
}
