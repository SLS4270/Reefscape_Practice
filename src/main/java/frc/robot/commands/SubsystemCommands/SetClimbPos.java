package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrists.Climb;

public class SetClimbPos extends Command {
    Climb s_Climb;
    double pos;

    public SetClimbPos(Climb subsys, double pos) {
        s_Climb = subsys;
        this.pos = pos;

        addRequirements(subsys);
    }
    
    @Override
    public void execute() {
        s_Climb.setClimbToPos(pos);
    }
}
