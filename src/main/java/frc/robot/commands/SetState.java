package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SetState extends SequentialCommandGroup {
    
    public enum States {
        Default,
        Intaking,
        Outtaking,
        PrepScoreL1,
        PrepScoreL2,
        PrepScoreL3,
        PrepScoreL4,
        Scoring,
        Climbing
    }

    public SetState (States state) {
        switch (state) {
            case Default:        
                break;
            case Intaking:
                break;
            case Outtaking:
                break;
            case PrepScoreL1:
                break;
            case PrepScoreL2:
                break;
            case PrepScoreL3:
                break;
            case PrepScoreL4:
                break;
            case Scoring:
                break;
            case Climbing:
                break;
        }
    }
}
