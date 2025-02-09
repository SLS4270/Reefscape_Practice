package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    TalonFX intake;

    public Intake() {
        intake = new TalonFX(Constants.intakeID);
    }

    public void spinIntake(double power) {
        intake.set(power);
    }
}
