package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intakes.ClimbIntake;

public class SpinClimbIntake extends Command {
    ClimbIntake s_ClimbIntake;
    double power;

    public SpinClimbIntake(ClimbIntake subsys, double power) {
        s_ClimbIntake = subsys;
        this.power = power;

        addRequirements(subsys);
    }
    
    @Override
    public void execute() {
        s_ClimbIntake.spinClimbIntake(power);
    }
}
