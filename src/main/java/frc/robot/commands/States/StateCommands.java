package frc.robot.commands.States;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.SubsystemCommands.RunElevator;
import frc.robot.commands.SubsystemCommands.RunIntakeWrist;
import frc.robot.commands.SubsystemCommands.RunRotator;
import frc.robot.commands.SubsystemCommands.SpinArmIntake;
import frc.robot.commands.SubsystemCommands.SpinIndexer;
import frc.robot.commands.SubsystemCommands.SpinIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intakes.ArmIntake;
import frc.robot.subsystems.Intakes.ArmIntake.BallIntakeState;
import frc.robot.subsystems.Intakes.ArmIntake.CoralIntakeState;

public class StateCommands {

    public enum CoralLevels {
        L1,
        L2,
        L3,
        L4,
        L4Auto
    }
    
    public static Command defaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, -3.3),
                new SpinIntake(RobotContainer.s_Intake, 0),
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -0.03),
                new RunElevator(RobotContainer.s_Elevator, -2.0, 4.2),
                new RunRotator(RobotContainer.s_Rotator, 0.605,200),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.NotCoralIntaking)),
                new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.NotBallIntaking))
            )
        );
    }

    public static Command ballDefaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, -3.3),
                new SpinIntake(RobotContainer.s_Intake, 0),
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                // new SpinArmIntake(RobotContainer.s_ArmIntake, -1),
                new RunElevator(RobotContainer.s_Elevator, -2.0, 4.2),
                new RunRotator(RobotContainer.s_Rotator, 0.625,0.5),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.NotCoralIntaking)),
                new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.NotBallIntaking))
            )
        );
    }

    public static Command intakingState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -31.95),
            new SpinIntake(RobotContainer.s_Intake, -1),
            new SpinIndexer(RobotContainer.s_Indexer, 0.5),
            new RunRotator(RobotContainer.s_Rotator, 0.115, 200),//0
            new RunElevator(RobotContainer.s_Elevator, 0, 1.3)
        );
    }

    public static Command L1intake() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -31.95),
            new SpinIntake(RobotContainer.s_Intake, -0.25)
        );
    }

    public static Command indexingState() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.8), 
                new SpinIndexer(RobotContainer.s_Indexer, -1),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -1),
                new SpinIntake(RobotContainer.s_Intake, -1),
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, -3.3),
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
                        new RunElevator(RobotContainer.s_Elevator, 0, 0.66),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)            
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.425, 2)//17
                );
            case L3:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.3),
                        new RunElevator(RobotContainer.s_Elevator, -21.3, 18.16),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)             
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.46, 2)//18
                );
            case L4:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -53.6, 52.8),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)           
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.498, 2)//20
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -53.6, 52.8),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new RunRotator(RobotContainer.s_Rotator, 0.498, 2)           
                    )
                );            
            default://L1
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -6.8, 7.1),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)           
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.285, 2)//10
                );   
        }
    }

    public static Command returnState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -3.3),
            new SpinIntake(RobotContainer.s_Intake, 0),
            new SpinIndexer(RobotContainer.s_Indexer, 0),
            new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
            new RunRotator(RobotContainer.s_Rotator, 0.11, 200),//0
            new RunElevator(RobotContainer.s_Elevator, 0, 4.2)
        );
    }

    public static Command scoringState(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        new RunRotator(RobotContainer.s_Rotator, 0.11, 1)
                    )
                );
            case L3:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, 0.31, 200)//12
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.1),
                        new RunElevator(RobotContainer.s_Elevator, 0, 0)
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.57, 200)//24
                );
            case L4: 
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, 0.11, 1)//0
                    )
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, 0.4, 20)//0
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
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0.2)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new RunRotator(RobotContainer.s_Rotator, 0.11, 200)//0
                    ),
                    new RunElevator(RobotContainer.s_Elevator, 0, 0.66)
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
            new RunElevator(RobotContainer.s_Elevator, -35.5, 31.5),//18.8
            new RunRotator(RobotContainer.s_Rotator, 0.375, 200),//13//0.404
            new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.BallIntaking)),
            new SpinArmIntake(RobotContainer.s_ArmIntake, -0.5)
        );
    }

    public static Command algaeIntakingL1() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, -5.69, 5.95),
            new RunRotator(RobotContainer.s_Rotator, 0.377, 200),//13
            new InstantCommand(() -> ArmIntake.setBallIntakeState(BallIntakeState.BallIntaking)),
            new SpinArmIntake(RobotContainer.s_ArmIntake, -0.5)
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
            new RunRotator(RobotContainer.s_Rotator, 0.295, 200)
        );
    }

    public static Command processorState() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, 0.27, 200),
            new RunElevator(RobotContainer.s_Elevator, 0, 0.66)
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
        return new ParallelCommandGroup(
            new SpinIntake(RobotContainer.s_Intake, 0.4),
            new SpinIndexer(RobotContainer.s_Indexer, 0.5),
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -10.0)
        );
    }

    public static Command backwardBargeState() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, 0.65, 0.5),
            new RunElevator(RobotContainer.s_Elevator, 0, 54)
        );
    }

    public static Command throwAlgae() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunElevator(RobotContainer.s_Elevator, 0, 54),
                new RunRotator(RobotContainer.s_Rotator, 0.15, 0.5),
                new ParallelRaceGroup(
                    new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75),
                    new WaitCommand(0.01)
                )
            ),
            new RunRotator(RobotContainer.s_Rotator, 0.65, 0.75),
            new SpinArmIntake(RobotContainer.s_ArmIntake, 1)
        );
    }
}