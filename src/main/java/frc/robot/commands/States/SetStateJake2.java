package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.CoralLevels;

public class SetStateJake2 extends SequentialCommandGroup {
    
    public enum StatesJake2 {
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
        GroundAlgae,
        ScoreGroundAlgae,
        GroundAlgaeIntake,
        BackwardBarge,
        IntakeL1,
        SmartScoreL1,
        SmartScoreL2,
        SmartScoreL3,
        SmartScoreL4,
        BallReturn,
        PrepThrow,
        ThrowAlgae
    }

    public SetStateJake2 (StatesJake2 state) {
        switch (state) {
            case IntakeL1:
                addCommands(
                    StateCommandsJake2.L1intake()
                );
                break;
            case Default:      
                addCommands(
                    StateCommandsJake2.defaultState()
                );  
                break;
            case Intaking:
                addCommands(
                    StateCommandsJake2.intakingState()  
                );
                break;
            case Outtaking:
                addCommands(
                    StateCommandsJake2.outtakeState()
                );
                break;
            case PrepScoreL1:
                addCommands(
                    StateCommandsJake2.prepCoralScore(CoralLevels.L1)
                );
                break;
            case PrepScoreL2:
                addCommands(
                    StateCommandsJake2.prepCoralScore(CoralLevels.L2)
                );
                break;
            case PrepScoreL3:
                addCommands(
                    StateCommandsJake2.prepCoralScore(CoralLevels.L3)
                );
                break;
            case PrepScoreL4:
                addCommands(
                    StateCommandsJake2.prepCoralScore(CoralLevels.L4)
                );
                break;
            case PrepScoreL4Auto:
                addCommands(
                    StateCommandsJake2.prepCoralScore(CoralLevels.L4Auto)
                );
                break;
            case ScoringL4Auto:
                addCommands(
                    StateCommandsJake2.scoringState(CoralLevels.L4Auto)
                );
                break;
            case ScoringL4:
                addCommands(
                    StateCommandsJake2.scoringState(CoralLevels.L4)
                );
                break;
            case ScoringL3:
                addCommands(
                    StateCommandsJake2.scoringState(CoralLevels.L3)
                );
                break;

            case ScoringL2:
                addCommands(
                    StateCommandsJake2.scoringState(CoralLevels.L2)
                );
                break;
            case ScoringL1:
                addCommands(
                    StateCommandsJake2.scoringState(CoralLevels.L1)
                );
                break;
            case Climbing:
                addCommands(
                    StateCommandsJake2.prepClimb()
                );
                break;
            case Indexing:
                addCommands(
                    StateCommandsJake2.indexingState()
                );
                break;
            case Return:
                addCommands(
                    StateCommandsJake2.returnState()
                );
                break;
            case AlgaeHigh:
                addCommands(
                    StateCommandsJake2.algaeIntakingL2()
                );
                break;
            case AlgaeLow:
                addCommands(
                    StateCommandsJake2.algaeIntakingL1()
                );
                break;
            case BargeScore:
                addCommands(
                    StateCommandsJake2.bargeScore()
                );
                break;
            case Processor:
                addCommands(
                    StateCommandsJake2.processorState()
                );
                break;
            case GroundAlgae:
                addCommands(
                    StateCommandsJake2.algaeGround()
                );
                break;
            case ScoreGroundAlgae:
                addCommands(
                    StateCommandsJake2.releaseAlgaeGround()
                );
                break;
            case GroundAlgaeIntake:
                addCommands(
                    StateCommandsJake2.intakeGroundAlgae()
                );
                break;
            case BackwardBarge:
                addCommands(
                    StateCommandsJake2.backwardBargeState()
                );
                break;
            case SmartScoreL1:
                addCommands(
                    StateCommandsJake2.smartScoringState(CoralLevels.L1)
                );
                break;
            case SmartScoreL2:
                addCommands(
                    StateCommandsJake2.smartScoringState(CoralLevels.L2)
                );
                break;
            case SmartScoreL3:
                addCommands(
                    StateCommandsJake2.smartScoringState(CoralLevels.L3)
                );
                break;
            case SmartScoreL4:
                addCommands(
                    StateCommandsJake2.smartScoringState(CoralLevels.L4)
                );
                break;
            case BallReturn:
                addCommands(
                    StateCommandsJake2.ballDefaultState()
                );
                break;
            case ThrowAlgae:
                addCommands(
                    StateCommandsJake2.throwAlgae()
                );
                break;
            case PrepThrow:
                addCommands(
                    StateCommandsJake2.prepAlgaeThrow()
                );
                break;
        }
    }
}
