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
import frc.robot.commands.SubsystemCommands.SetLEDs;
import frc.robot.commands.SubsystemCommands.SpinArmIntake;
import frc.robot.commands.SubsystemCommands.SpinIndexer;
import frc.robot.commands.SubsystemCommands.SpinIntake;
import frc.robot.commands.SubsystemCommands.SpinL1Intake;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CoralLevels;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intakes.ArmIntake;
import frc.robot.subsystems.Intakes.ArmIntake.CoralIntakeState;
import frc.robot.subsystems.LEDs.LEDStates;

public class StateCommands {
    
    public static Command defaultState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.Setpoints.IntakeWrist.wristUp),
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -0.03),
                new SpinIntake(RobotContainer.s_Intake, 0),
                new SpinL1Intake(RobotContainer.s_L1Intake, 0),
                new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleDefault),
                new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotDefault,200),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.NotCoralIntaking)),
                new SetLEDs(RobotContainer.s_LEDs, LEDStates.Default)
            )
        );
    }
    public static Command defaultStateAuto() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -0.03),
                new SpinIntake(RobotContainer.s_Intake, 0),
                new SpinL1Intake(RobotContainer.s_L1Intake, 0),
                new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleDefault),
                new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotDefault,200),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.NotCoralIntaking)),
                new SetLEDs(RobotContainer.s_LEDs, LEDStates.Default)
            )
        );
    }

    public static Command intakingState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.Setpoints.IntakeWrist.wristIntake),
            new SpinIntake(RobotContainer.s_Intake, 1),
            new SpinL1Intake(RobotContainer.s_L1Intake, 1),
            new SpinIndexer(RobotContainer.s_Indexer, 0.5),
            new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotIntake, 200),
            new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleIntake),
            new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.CoralIntaking)),
            new SetLEDs(RobotContainer.s_LEDs, LEDStates.Intaking)
        );
    }

    public static Command L1intake() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.Setpoints.IntakeWrist.wristIntake),
            new SpinL1Intake(RobotContainer.s_L1Intake, 0.7),
            new SpinIntake(RobotContainer.s_Intake, -0.03)
        );
    }

    public static Command indexingState() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpinIndexer(RobotContainer.s_Indexer, -1),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -1),
                new SpinIntake(RobotContainer.s_Intake, 1),
                new SpinL1Intake(RobotContainer.s_L1Intake, 1),
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.Setpoints.IntakeWrist.wristUp),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.CoralIntaking))
            )
        );
    }
    public static Command indexingStateAuto() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new WaitCommand(0.8), 
                new SpinIndexer(RobotContainer.s_Indexer, -1),
                new SpinArmIntake(RobotContainer.s_ArmIntake, -1),
                new SpinIntake(RobotContainer.s_Intake, 1),
                new SpinL1Intake(RobotContainer.s_L1Intake, 1),
                new InstantCommand(() -> ArmIntake.setCoralIntakeState(CoralIntakeState.CoralIntaking))
            ),
            defaultStateAuto()
        );
    }

    public static Command prepCoralScore(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.1),
                        new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL2),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new SpinIndexer(RobotContainer.s_Indexer, 0),
                        new SpinIntake(RobotContainer.s_Intake, 0),
                        new SpinL1Intake(RobotContainer.s_L1Intake, 0)     
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotPrepL2, 2)
                );
            case L3:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.3),
                        new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL3),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)           
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotPrepL3, 2)
                );
            case L4:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)       
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotPrepL4, 2)
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotPrepL4, 2)       
                    )
                );            
            default:
                //L1
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL1),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),           
                        new SpinIndexer(RobotContainer.s_Indexer, 0),
                        new SpinIntake(RobotContainer.s_Intake, 0),
                        new SpinL1Intake(RobotContainer.s_L1Intake, 0)
                    ),
                    new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotPrepL1, 2)
                );   
        }
    }

    public static Command returnState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.Setpoints.IntakeWrist.wristUp),
            new SpinIntake(RobotContainer.s_Intake, 0),
            new SpinIndexer(RobotContainer.s_Indexer, 0),
            new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
            new SpinL1Intake(RobotContainer.s_L1Intake, 0),
            new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotReturn, 200),
            new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleDefault),
            new SetLEDs(RobotContainer.s_LEDs, LEDStates.Default)
        );
    }

    public static Command scoringState(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.25),
                        new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotReturn, 0.9)
                    )
                );
            case L3:
                return new SequentialCommandGroup(
                    new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotReturn, 0.9)
                );
            case L4: 
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotReturn, 0.95)
                    )
                );
            case L4Auto:
                return new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotScoreL4Auto, 20)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.3),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 1)
                    )
                );
            default:
            //L1
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0.25)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotReturn, 200)
                    ),
                    new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL2)
                );
        }
    }

    public static Command smartScoringState(CoralLevels level) {
        return new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator);
    }

    public static Command algaeIntakingL2() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleAlgaeL2),
            new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotAlgaeIntake, 200),
            new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75)
        );
    }

    public static Command algaeIntakingL1() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleAlgaeL1),
            new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotAlgaeIntake, 200),
            new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75)
        );
    }

    public static Command bargeScore() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, 54),
            new RunRotator(RobotContainer.s_Rotator, 0.57, 200)
        );
    }

    public static Command prepClimb() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotPrepClimb, 200),
            new RunClimb(RobotContainer.s_Climb, Constants.Setpoints.Climb.climbOut),
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.Setpoints.IntakeWrist.wristClimbPrep),
            new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL2)
        );
    }

    public static Command processorState() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotProcessor, 1),
            new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rElePrepL2)
        );
    }

    public static Command outtakeState() {
        return new SequentialCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, Constants.Setpoints.IntakeWrist.wristOuttake),
            new ParallelCommandGroup(
                new SpinIntake(RobotContainer.s_Intake, -0.4),
                new SpinIndexer(RobotContainer.s_Indexer, 0.5),
                new SpinL1Intake(RobotContainer.s_L1Intake, -0.3)
            )
        );
    }

    public static Command backwardBargeState() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, 0.65, 0.9),
            new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleThrowAlgae)
        );
    }

    public static Command prepAlgaeThrow() {
        return new ParallelCommandGroup(
                new RunElevator(RobotContainer.s_Elevator, Constants.Setpoints.Elevator.rEleThrowAlgae),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.7),
                    new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotBargeThrowPrep, 1)
                ),
                new ParallelRaceGroup(
                    new SpinArmIntake(RobotContainer.s_ArmIntake, -0.75),
                    new WaitCommand(0.01)
                )
        );

    }
    public static Command throwAlgae() {
        return new SequentialCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, Constants.Setpoints.Rotator.rotBargeThrowRrElease, 0.95),
            new ParallelRaceGroup(
                new SpinArmIntake(RobotContainer.s_ArmIntake, 1),
                new WaitCommand(1)
            )    
        );
    }
}