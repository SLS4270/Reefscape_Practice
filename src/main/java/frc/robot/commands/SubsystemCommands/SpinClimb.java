package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrists.Climb;

public class SpinClimb extends Command {
    Climb s_Climb;
    double power;

    public SpinClimb(Climb subsys, double power) {
        s_Climb = subsys;
        this.power = power;

        addRequirements(subsys);
    }
    
    @Override
    public void execute() {
        s_Climb.setClimbPower(power);
    }
}
