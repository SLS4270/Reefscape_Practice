package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SubsystemCommands.RunClimb;
import frc.robot.commands.SubsystemCommands.RunElevator;
import frc.robot.commands.SubsystemCommands.RunIntakeWrist;
import frc.robot.commands.SubsystemCommands.RunRotator;
import frc.robot.commands.SubsystemCommands.SpinArmIntake;
import frc.robot.commands.SubsystemCommands.SpinClimbIntake;
import frc.robot.commands.SubsystemCommands.SpinIndexer;
import frc.robot.commands.SubsystemCommands.SpinIntake;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CoralLevels;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intakes.ArmIntake;
import frc.robot.subsystems.Intakes.ArmIntake.BallIntakeState;
import frc.robot.subsystems.Intakes.ArmIntake.CoralIntakeState;

public class StateCommands {
    
    public static Command defaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristUp),
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -0.03),
                new SpinIntake(RobotContainer.s_Intake, 0),
                new RunElevator(RobotContainer.s_Elevator, -2.0, Constants.JakeSetpoints.Elevator.eleDefault),
                new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotDefault,200),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.NotCoralIntaking)),
                new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.NotBallIntaking))
            )
        );
    }

    public static Command groundBallDefaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristUp),
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -0.03),
                new SpinIntake(RobotContainer.s_Intake, 0.1),
                new RunElevator(RobotContainer.s_Elevator, -2.0, Constants.JakeSetpoints.Elevator.eleDefault),
                new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotDefault,200),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.NotCoralIntaking)),
                new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.NotBallIntaking))
            )
        );
    }

    public static Command ballDefaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristUp),
                new SpinIntake(RobotContainer.s_Intake, 0),
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                // new SpinArmIntake(RobotContainer.s_ArmIntake, -1),
                new RunElevator(RobotContainer.s_Elevator, -2.0, Constants.JakeSetpoints.Elevator.eleDefault),
                new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotBallDefault,200),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.NotCoralIntaking)),
                new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.NotBallIntaking))
            )
        );
    }

    public static Command intakingState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristIntake),
            new SpinIntake(RobotContainer.s_Intake, -1),
            new SpinIndexer(RobotContainer.s_Indexer, 0.5),
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotIntake, 200),//0
            new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.eleIntake),
            new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.CoralIntaking))
        );
    }

    public static Command L1intake() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristIntake),
            new SpinIntake(RobotContainer.s_Intake, 1)
        );
    }

    public static Command indexingState() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.8), 
                new SpinIndexer(RobotContainer.s_Indexer, -1),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -1),
                new SpinIntake(RobotContainer.s_Intake, -1),
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristUp),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.CoralIntaking))
            ),
            defaultState()
        );
    }

    public static Command prepCoralScore(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.1),
                        new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.elePrepL2),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new SpinIndexer(RobotContainer.s_Indexer, 0),
                        new SpinIntake(RobotContainer.s_Intake, 0)          
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotPrepL2, 2)//17
                );
            case L3:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.3),
                        new RunElevator(RobotContainer.s_Elevator, -21.3, Constants.JakeSetpoints.Elevator.elePrepL3),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)             
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotPrepL3, 2)//18
                );
            case L4:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -53.6, Constants.JakeSetpoints.Elevator.elePrepL4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)           
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotPrepL4, 2)//20
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -53.6, Constants.JakeSetpoints.Elevator.elePrepL4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotPrepL4, 2)           
                    )
                );            
            default://L1
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -6.8, Constants.JakeSetpoints.Elevator.elePrepL1),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),           
                        new SpinIndexer(RobotContainer.s_Indexer, 0),
                        new SpinIntake(RobotContainer.s_Intake, 0)
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotPrepL1, 2)//10
                );   
        }
    }

    public static Command returnState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristUp),
            new SpinIntake(RobotContainer.s_Intake, 0),
            new SpinIndexer(RobotContainer.s_Indexer, 0),
            new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotReturn, 200),//0
            new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.eleDefault)
        );
    }

    public static Command scoringState(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotReturn, 1)
                    )
                );
            case L3:
                return new SequentialCommandGroup(
                    new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotReturn, 1)
                );
            case L4: 
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotReturn, 1)//0
                    )
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotScoreL4Auto, 20)//0
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.3),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 1)
                    )
                );
            default://L1
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0.25)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotReturn, 200)//0
                    ),
                    new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.elePrepL2)
                );
        }
    }

    public static Command smartScoringState(CoralLevels level) {
        return new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator);
    }

    public static Command algaeIntakingL2() {
        return new ParallelCommandGroup(
            // new RunElevator(RobotContainer.s_Elevator, -35.5, 35.7),//18.8
            // new RunRotator(RobotContainer.s_Rotator, 0.37, 200),//13//0.404
            new RunElevator(RobotContainer.s_Elevator, -35.5, Constants.JakeSetpoints.Elevator.eleAlgaeL2),//18.8
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotAlgaeIntake, 200),//13//0.404
            new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.BallIntaking)),
            new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75)
        );
    }

    public static Command algaeIntakingL1() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, -5.69, Constants.JakeSetpoints.Elevator.eleAlgaeL1),
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotAlgaeIntake, 200),//13
            new InstantCommand(() ->    ArmIntake.setBallIntakeState(BallIntakeState.BallIntaking)),
            new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75)
        );
    }

    public static Command bargeScore() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, -53.8, 54),
            new RunRotator(RobotContainer.s_Rotator, 0.57, 200)//0.24
        );
    }

    public static Command prepClimb() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotPrepClimb, 200),
            new RunClimb(RobotContainer.s_Climb, Constants.JakeSetpoints.Climb.climbOut),
            new SpinClimbIntake(RobotContainer.s_ClimbIntake, -0.25)
        );
    }

    public static Command processorState() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotProcessor, 200),
            new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.elePrepL2)
        );
    }

    public static Command algaeGround() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -22.7),
            new SpinIntake(RobotContainer.s_Intake, -0.15)
        );
    }

    public static Command releaseAlgaeGround() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new WaitCommand(0.3),
                new SpinIntake(RobotContainer.s_Intake, 1)
            ),
            defaultState()
        );
    }

    public static Command intakeGroundAlgae() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -25.84),
            new SpinIntake(RobotContainer.s_Intake, -0.35)
        );
    }

    public static Command outtakeState() {
        return new SequentialCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristOuttake),
            new SpinIntake(RobotContainer.s_Intake, -0.4)
        );
    }

    public static Command backwardBargeState() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, 0.65, 0.5),
            new RunElevator(RobotContainer.s_Elevator, 0, 54)
        );
    }

    public static Command prepAlgaeThrow() {
        return new ParallelCommandGroup(
                new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.eleThrowAlgae),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.7),
                    new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotBargeThrowPrep, 1)
                ),
                new ParallelRaceGroup(
                    new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75),
                    new WaitCommand(0.01)
                )
        );

    }
    public static Command throwAlgae() {
        return new SequentialCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotBargeThrowRelease, 1),
            new SpinArmIntake(RobotContainer.s_ArmIntake, 1)
        );
    }

    public static Command transferBall() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.eleThrowAlgae),
                new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotIntake, 200),
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristTransfer),
                new ParallelRaceGroup(
                    new WaitCommand(1),
                    new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75)
                )
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(1), 
                new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.eleTransferDown),
                new SpinIntake(RobotContainer.s_Intake, -0.25)
            ),
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristIntake),
                new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.eleTransferStow),
                new ParallelRaceGroup(
                    new WaitCommand(0.1),
                    new SpinIntake(RobotContainer.s_Intake, 0)
                )
            ),
            new RunRotator(RobotContainer.s_Rotator, Constants.JakeSetpoints.Rotator.rotDefault, 1),
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.JakeSetpoints.IntakeWrist.wristUp),
            new RunElevator(RobotContainer.s_Elevator, 0, Constants.JakeSetpoints.Elevator.eleDefault)
        );
    }


}