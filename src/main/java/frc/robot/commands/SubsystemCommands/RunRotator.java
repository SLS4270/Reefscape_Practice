package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrists.Rotator;

public class RunRotator extends Command {
    Rotator s_Rotator;
    double pos;
    double maxVelo;

    public RunRotator(Rotator subsys, double pos, double maxVelo) {
        s_Rotator = subsys;
        this.pos = pos;
        this.maxVelo = maxVelo;

        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        s_Rotator.runRotatorToPos(pos, maxVelo);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_Rotator.getPos() - pos) < 0.365;
    }
}
