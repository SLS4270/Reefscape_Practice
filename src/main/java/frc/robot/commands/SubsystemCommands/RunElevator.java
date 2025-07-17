package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class RunElevator extends Command {
    Elevator s_Elevator;
    double rPos;

    public RunElevator(Elevator subsys, double rPos) {
        s_Elevator = subsys;
        this.rPos = rPos;

        addRequirements(subsys);
    }

    @Override
    public void initialize() {
        s_Elevator.runElevatorToPos(rPos);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(s_Elevator.getRPos() - rPos) < 5);
    }
}
