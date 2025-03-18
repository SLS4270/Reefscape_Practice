package frc.robot.subsystems.Wrists;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Climb extends SubsystemBase{
    SparkMax climbMotor;
    SparkClosedLoopController pidController;
    SparkBaseConfig configs;

    public Climb() {
        climbMotor = new SparkMax(Constants.climbID, MotorType.kBrushless);
        configs = new SparkMaxConfig();
        configs.apply(new ClosedLoopConfig().pidf(1, 0, 0, 0)
                .apply(new MAXMotionConfig()
                .allowedClosedLoopError(5)
                .maxVelocity(100)
                .maxAcceleration(15)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal))
                );
        climbMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidController = climbMotor.getClosedLoopController();
        
        climbMotor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climbPos", climbMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("kP", climbMotor.configAccessor.closedLoop.getP());
    }

    public void setClimbPower(double speed) {
        climbMotor.set(speed);
    }

    public void setClimbToPos(double pos) {
        pidController.setReference(pos, ControlType.kPosition);
    }
}
