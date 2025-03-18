package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrists.IntakeWrist;

public class RunIntakeWrist extends Command {
    IntakeWrist s_Wrist;
    double lPos;

    public RunIntakeWrist(IntakeWrist subsys, double lPos) {
        s_Wrist = subsys;
        this.lPos = lPos;

        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        s_Wrist.runWristToPos(lPos);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(s_Wrist.getLPos() - lPos) < 1.5);
    }
}
