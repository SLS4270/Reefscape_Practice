package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDStates;

public class SetLEDs extends Command {
    LEDs s_LEDs;
    LEDStates state;

    public SetLEDs(LEDs subsys, LEDStates state) {
        s_LEDs = subsys;
        this.state = state;

        addRequirements(subsys);
    }
    
    @Override
    public void initialize() {
        s_LEDs.setLEDS(state);
    }
}
