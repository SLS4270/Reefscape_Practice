package frc.robot.commands;

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
import frc.robot.commands.SubsystemCommands.SetClimbPos;
import frc.robot.commands.SubsystemCommands.SpinArmIntake;
import frc.robot.commands.SubsystemCommands.SpinIndexer;
import frc.robot.commands.SubsystemCommands.SpinIntake;
import frc.robot.subsystems.ArmIntake;
import frc.robot.subsystems.Elevator;

public class StateCommands {

    // private static CoralLevels currentLevel = CoralLevels.L3;

    public enum CoralLevels {
        L1,
        L2,
        L3,
        L4
    }
    
    public static Command defaultState() {
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, -3.3, 3.8),
                new SpinIntake(RobotContainer.s_Intake, 0),
                new SpinIndexer(RobotContainer.s_Indexer, 0),
                new SpinArmIntake(RobotContainer.s_ArmIntake, 0.15),
                new RunElevator(RobotContainer.s_Elevator, -2.0, 2.3),
                new WaitCommand(0.1)
            ),
            new RunRotator(RobotContainer.s_Rotator, 0.855,200)//25
        );
    }

    public static Command intakingState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -31.8, 32.5),
            new SpinIntake(RobotContainer.s_Intake, -1),
            new SpinIndexer(RobotContainer.s_Indexer, 0.5),
            new SpinArmIntake(RobotContainer.s_ArmIntake, 1),
            new RunRotator(RobotContainer.s_Rotator, 0.35, 200),//0
            new RunElevator(RobotContainer.s_Elevator, 0, 0)
        );
    }

    public static Command indexingState() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(1), 
                new SpinIndexer(RobotContainer.s_Indexer, -1),
                new SpinArmIntake(RobotContainer.s_ArmIntake, 1),
                new SpinIntake(RobotContainer.s_Intake, -0.75),
                new RunIntakeWrist(RobotContainer.s_IntakeWrist, -3.3, 3.8)
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
                        new RunElevator(RobotContainer.s_Elevator, 0, 0),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)                 
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.69, 200)//17
                );
            case L3:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.3),
                        new RunElevator(RobotContainer.s_Elevator, -21.3, 20.6),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)                
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.71, 200)//18
                );
            case L4:
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -53.6, 52.8),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)              
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.748, 200)//20
                );
            default://L1
                return new SequentialCommandGroup(
                    new InstantCommand(() -> Elevator.setCurrentLevel(level), RobotContainer.s_Elevator),
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunElevator(RobotContainer.s_Elevator, -6.8, 7.1),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)              
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.545, 200)//10
                );   
        }
    }

    public static Command returnState() {
        return new ParallelCommandGroup(
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -3.3, 3.8),
            new SpinIntake(RobotContainer.s_Intake, 0),
            new SpinIndexer(RobotContainer.s_Indexer, 0),
            new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
            new RunRotator(RobotContainer.s_Rotator, 0.35, 200),//0
            new RunElevator(RobotContainer.s_Elevator, 0, 0)
        );
    }

    public static Command scoringState(CoralLevels level) {
        switch (level) {    
            case L2:
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(1),
                        new RunRotator(RobotContainer.s_Rotator, 0.586, 200),//12
                        new SpinArmIntake(RobotContainer.s_ArmIntake, -0.1)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.15),
                        new RunElevator(RobotContainer.s_Elevator, -10.1, 10.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.2),
                        new RunRotator(RobotContainer.s_Rotator, 0.35, 200)//0
                    ),
                    new RunElevator(RobotContainer.s_Elevator, 0, 0)
                );
            case L3:
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, 0.586, 200)//12
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.1),
                        new RunElevator(RobotContainer.s_Elevator, 0, 0)
                    ),
                    new RunRotator(RobotContainer.s_Rotator, 0.82, 200)//24
                );
            case L4:
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.5),
                        new RunRotator(RobotContainer.s_Rotator, 0.35, 2)//0
                    ),
                    new RunElevator(RobotContainer.s_Elevator, 0, 0)
                );
            default://L1
                return new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, -0.2)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(0.4),
                        new SpinArmIntake(RobotContainer.s_ArmIntake, 0),
                        new RunRotator(RobotContainer.s_Rotator, 0.35, 200)//0
                    ),
                    new RunElevator(RobotContainer.s_Elevator, 0, 0)
                );
        }
    }

    public static Command algaeIntakingL2() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, -35.5, 35.7),
            new RunRotator(RobotContainer.s_Rotator, 0.61, 200)//13
        );
    }

    public static Command algaeIntakingL1() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, -5.69, 5.95),
            new RunRotator(RobotContainer.s_Rotator, 0.617, 200)//13
        );
    }

    public static Command bargeScore() {
        return new ParallelCommandGroup(
            new RunElevator(RobotContainer.s_Elevator, -53.8, 54),
            new RunRotator(RobotContainer.s_Rotator, 0.82, 200)//0.24
        );
    }

    public static Command prepClimb() {
        return new ParallelCommandGroup(
            new RunRotator(RobotContainer.s_Rotator, 0.545, 200),
            new RunIntakeWrist(RobotContainer.s_IntakeWrist, -5, 5.5)
            // new SetClimbPos(RobotContainer.s_Climb, 150)
        );
    }
}
