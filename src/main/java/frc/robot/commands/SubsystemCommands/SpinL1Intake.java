package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intakes.L1Intake;

public class SpinL1Intake extends Command {
    L1Intake s_Intake;
    double power;

    public SpinL1Intake(L1Intake subsys, double power) {
        s_Intake = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        s_Intake.spinIntake(power);
    }
    
}
