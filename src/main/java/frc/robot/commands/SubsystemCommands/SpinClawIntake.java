package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmIntake;

public class SpinClawIntake extends Command {
    ArmIntake s_ArmIntake;
    double power;
    
    public SpinClawIntake(ArmIntake subsys, double power) {
        s_ArmIntake = subsys;
        this.power = power;

        addRequirements(subsys);
    }

    @Override
    public void execute() {
        s_ArmIntake.spinArmIntake(power);
    }
}
