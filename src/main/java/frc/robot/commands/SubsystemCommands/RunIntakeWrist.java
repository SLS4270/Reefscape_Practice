package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWrist;

public class RunIntakeWrist extends Command {
    IntakeWrist s_Wrist;
    double lPos;
    double rPos;

    public RunIntakeWrist(IntakeWrist subsys, double lPos, double rPos) {
        s_Wrist = subsys;
        this.lPos = lPos;
        this.rPos = rPos;

        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        s_Wrist.runWristToPos(lPos, rPos);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(s_Wrist.getLPos() - lPos) < 1.5) || (Math.abs(s_Wrist.getRPos() - rPos) < 1.5);
    }
}
