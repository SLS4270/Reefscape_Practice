package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.StateCommands.CoralLevels;

public class SetState extends SequentialCommandGroup {
    
    public enum States {
        Default,
        Indexing,
        Intaking,
        Outtaking,
        PrepScoreL1,
        PrepScoreL2,
        PrepScoreL3,
        PrepScoreL4,
        ScoringL1,
        ScoringL2,
        ScoringL3,
        ScoringL4,
        Climbing,
        Return,
        AlgaeHigh,
        AlgaeLow,
        BargeScore,
        Processor,
        GroundAlgae,
        ScoreGroundAlgae,
        GroundAlgaeIntake,
        BackwardBarge
    }

    public SetState (States state) {
        switch (state) {
            case Default:      
                addCommands(
                    StateCommands.defaultState()
                );  
                break;
            case Intaking:
                addCommands(
                    StateCommands.intakingState()  
                );
                break;
            case Outtaking:
                addCommands(
                    StateCommands.outtakeState()
                );
                break;
            case PrepScoreL1:
                addCommands(
                    StateCommands.prepCoralScore(CoralLevels.L1)
                );
                break;
            case PrepScoreL2:
                addCommands(
                    StateCommands.prepCoralScore(CoralLevels.L2)
                );
                break;
            case PrepScoreL3:
                addCommands(
                    StateCommands.prepCoralScore(CoralLevels.L3)
                );
                break;
            case PrepScoreL4:
                addCommands(
                    StateCommands.prepCoralScore(CoralLevels.L4)
                );
                break;
            case ScoringL4:
                addCommands(
                    StateCommands.scoringState(CoralLevels.L4)
                );
                break;
            case ScoringL3:
                addCommands(
                    StateCommands.scoringState(CoralLevels.L3)
                );
                break;

            case ScoringL2:
                addCommands(
                    StateCommands.scoringState(CoralLevels.L2)
                );
                break;
            case ScoringL1:
                addCommands(
                    StateCommands.scoringState(CoralLevels.L1)
                );
                break;
            case Climbing:
                addCommands(
                    StateCommands.prepClimb()
                );
                break;
            case Indexing:
                addCommands(
                    StateCommands.indexingState()
                );
                break;
            case Return:
                addCommands(
                    StateCommands.returnState()
                );
                break;
            case AlgaeHigh:
                addCommands(
                    StateCommands.algaeIntakingL2()
                );
                break;
            case AlgaeLow:
                addCommands(
                    StateCommands.algaeIntakingL1()
                );
                break;
            case BargeScore:
                addCommands(
                    StateCommands.bargeScore()
                );
                break;
            case Processor:
                addCommands(
                    StateCommands.processorState()
                );
                break;
            case GroundAlgae:
                addCommands(
                    StateCommands.algaeGround()
                );
                break;
            case ScoreGroundAlgae:
                addCommands(
                    StateCommands.releaseAlgaeGround()
                );
                break;
            case GroundAlgaeIntake:
                addCommands(
                    StateCommands.intakeGroundAlgae()
                );
                break;
            case BackwardBarge:
                addCommands(
                    StateCommands.backwardBargeState()
                );
                break;
        }
    }
}
