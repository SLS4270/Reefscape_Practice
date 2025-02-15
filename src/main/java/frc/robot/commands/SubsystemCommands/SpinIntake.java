package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SpinIntake extends Command {
    Intake s_Intake;
    double power;

    public SpinIntake(Intake subsys, double power) {
        s_Intake = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        s_Intake.spinIntake(power);
    }
    
}
