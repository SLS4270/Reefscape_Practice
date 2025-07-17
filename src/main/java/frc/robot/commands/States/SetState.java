package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.CoralLevels;

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
        PrepScoreL4Auto,
        ScoringL1,
        ScoringL2,
        ScoringL3,
        ScoringL4,
        ScoringL4Auto,
        Climbing,
        Return,
        AlgaeHigh,
        AlgaeLow,
        BargeScore,
        Processor,
        BackwardBarge,
        IntakeL1,
        SmartScoreL1,
        SmartScoreL2,
        SmartScoreL3,
        SmartScoreL4,
        PrepThrow,
        ThrowAlgae,
        IndexingAuto
    }

    public SetState (States state) {
        switch (state) {
            case IntakeL1:
                addCommands(
                    StateCommands.L1intake()
                );
                break;
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
            case PrepScoreL4Auto:
                addCommands(
                    StateCommands.prepCoralScore(CoralLevels.L4Auto)
                );
                break;
            case ScoringL4Auto:
                addCommands(
                    StateCommands.scoringState(CoralLevels.L4Auto)
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
            case BackwardBarge:
                addCommands(
                    StateCommands.backwardBargeState()
                );
                break;
            case SmartScoreL1:
                addCommands(
                    StateCommands.smartScoringState(CoralLevels.L1)
                );
                break;
            case SmartScoreL2:
                addCommands(
                    StateCommands.smartScoringState(CoralLevels.L2)
                );
                break;
            case SmartScoreL3:
                addCommands(
                    StateCommands.smartScoringState(CoralLevels.L3)
                );
                break;
            case SmartScoreL4:
                addCommands(
                    StateCommands.smartScoringState(CoralLevels.L4)
                );
                break;
            case ThrowAlgae:
                addCommands(
                    StateCommands.throwAlgae()
                );
                break;
            case PrepThrow:
                addCommands(
                    StateCommands.prepAlgaeThrow()
                );
                break;
            case IndexingAuto:
                addCommands(
                    StateCommands.indexingStateAuto()
                );
                break;
        }
    }
}
