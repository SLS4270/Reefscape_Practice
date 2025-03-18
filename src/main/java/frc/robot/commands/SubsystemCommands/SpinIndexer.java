package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intakes.Indexer;

public class SpinIndexer extends Command {
    Indexer s_Indexer;
    double power;

    public SpinIndexer(Indexer subsys, double power) {
        s_Indexer = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        s_Indexer.spinIndexer(power);
    }
}
