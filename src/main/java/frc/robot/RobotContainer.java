// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SetState;
import frc.robot.commands.SwerveTurn;
import frc.robot.commands.SwerveTurnRed;
import frc.robot.commands.SetState.States;
import frc.robot.commands.StateCommands.CoralLevels;
import frc.robot.commands.SubsystemCommands.SpinArmIntake;
import frc.robot.commands.SubsystemCommands.SpinClimb;
import frc.robot.commands.SwerveTurn.Side;
import frc.robot.subsystems.ArmIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.Rotator;
import frc.robot.subsystems.ArmIntake.CoralIntakeState;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public static final double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

  public static final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public static final CommandJoystick buttons = new CommandJoystick(1);

  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain
  public static final IntakeWrist s_IntakeWrist = new IntakeWrist();
  public static final Intake s_Intake = new Intake();
  public static final Indexer s_Indexer = new Indexer();
  public static final ArmIntake s_ArmIntake = new ArmIntake();
  public static final Rotator s_Rotator = new Rotator();
  public static final Elevator s_Elevator = new Elevator();
  public static final Climb s_Climb = new Climb();


  private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public static final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private static SendableChooser<Command> m_chooser;

  public RobotContainer() {
    // drivetrain.configNeutralMode(NeutralModeValue.Brake);
    drivetrain.configPathPlanner();
    NamedCommands.registerCommand("PrepScoreL4", new SetState(States.PrepScoreL4Auto));
    NamedCommands.registerCommand("SpinIndexer", new SetState(States.Indexing));
    NamedCommands.registerCommand("Intake", new SetState(States.Intaking));
    NamedCommands.registerCommand("Outtake", new SpinArmIntake(s_ArmIntake, 1));
    NamedCommands.registerCommand("ScoreL4", new SetState(States.ScoringL4Auto));
    NamedCommands.registerCommand("A1 Prep", new SetState(States.AlgaeLow));
    NamedCommands.registerCommand("A2 Prep", new SetState(States.AlgaeHigh));
    NamedCommands.registerCommand("AlgaeIntake", new SpinArmIntake(s_ArmIntake, -0.25));
    NamedCommands.registerCommand("ClawStop", new SpinArmIntake(s_ArmIntake, 0));
    NamedCommands.registerCommand("BargePrep", new SetState(States.BackwardBarge));
    NamedCommands.registerCommand("ArmReturn", new SetState(States.Default));
    NamedCommands.registerCommand("Slam4", new SetState(States.ScoringL4));
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("2.5", new Autos("3-Left"));
    m_chooser.addOption("L4 + 2 Algae", new Autos("L4 + Alagae"));
    m_chooser.addOption("Push", new Autos("PUSH"));
    m_chooser.addOption("4Coral?", new Autos("3-Right"));

    SmartDashboard.putData("Auto", m_chooser);
    drivetrain.configNeutralMode(NeutralModeValue.Brake);
    drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) 
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
    ));
    configureBindings();
  }

  private void configureBindings() {
    Trigger isRedAlliance = new Trigger(() -> Robot.alliance == Alliance.Red);

    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L4).onTrue(new SetState(States.ScoringL4)).onFalse(new SetState(States.Default));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L3).onTrue(new SetState(States.ScoringL3)).onFalse(new SetState(States.Default));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L2).onTrue(new SetState(States.ScoringL2)).onFalse(new SetState(States.Return));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L1).onTrue(new SetState(States.ScoringL1)).onFalse(new SetState(States.Return));
    m_driverController.leftTrigger().onTrue(new SetState(States.Intaking)).onFalse(new SetState(States.Indexing));
    m_driverController.rightBumper().onTrue(new SetState(States.Outtaking)).onFalse(new SetState(States.Default));

    m_driverController.button(3).onTrue(new SpinArmIntake(s_ArmIntake, -0.5));
    m_driverController.button(4).onTrue(new SpinArmIntake(s_ArmIntake, 1)).onFalse(new SpinArmIntake(s_ArmIntake,0));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> drivetrain.tareSwerve(), drivetrain));
    m_driverController.leftBumper().onTrue(new SetState(States.IntakeL1)).onFalse(new SetState(States.Default));

    m_driverController.a().and(() -> drivetrain.angleToTurnTo <= 1.0).and(() -> drivetrain.angleToTurnTo >= -1.0).whileTrue(new SwerveTurn(0, Side.Left));
    m_driverController.a().and(() -> drivetrain.angleToTurnTo <= -119.0).and(() -> drivetrain.angleToTurnTo >= -121.0).whileTrue(new SwerveTurn(-120, Side.Left));
    m_driverController.a().and(() -> drivetrain.angleToTurnTo <= 61.0).and(() -> drivetrain.angleToTurnTo >= 59.0).whileTrue(new SwerveTurn(60, Side.Left));
    m_driverController.a().and(() -> drivetrain.angleToTurnTo <= 121.0).and(() -> drivetrain.angleToTurnTo >= 119.0).whileTrue(new SwerveTurn(120, Side.Left));
    m_driverController.a().and(() -> drivetrain.angleToTurnTo <= -179.0).and(() -> drivetrain.angleToTurnTo >= -181.0).whileTrue(new SwerveTurn(179, Side.Left));
    m_driverController.a().and(() -> drivetrain.angleToTurnTo <= -59.0).and(() -> drivetrain.angleToTurnTo >= -61.0).whileTrue(new SwerveTurn(-60, Side.Left));


    m_driverController.b().and(() -> drivetrain.angleToTurnTo <= 1.0).and(() -> drivetrain.angleToTurnTo >= -1.0).whileTrue(new SwerveTurn(0, Side.Right));
    m_driverController.b().and(() -> drivetrain.angleToTurnTo <= -119.0).and(() -> drivetrain.angleToTurnTo >= -121.0).whileTrue(new SwerveTurn(-120, Side.Right));
    m_driverController.b().and(() -> drivetrain.angleToTurnTo <= 61.0).and(() -> drivetrain.angleToTurnTo >= 59.0).whileTrue(new SwerveTurn(60, Side.Right));
    m_driverController.b().and(() -> drivetrain.angleToTurnTo <= 121.0).and(() -> drivetrain.angleToTurnTo >= 119.0).whileTrue(new SwerveTurn(120, Side.Right));
    m_driverController.b().and(() -> drivetrain.angleToTurnTo <= -179.0).and(() -> drivetrain.angleToTurnTo >= -181.0).whileTrue(new SwerveTurn(179, Side.Right));
    m_driverController.b().and(() -> drivetrain.angleToTurnTo <= -59.0).and(() -> drivetrain.angleToTurnTo >= -61.0).whileTrue(new SwerveTurn(-60, Side.Right));

    //Red
    m_driverController.a().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= 1.0).and(() -> drivetrain.angleToTurnTo >= -1.0).whileTrue(new SwerveTurnRed(179, Side.Left));
    m_driverController.a().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= -119.0).and(() -> drivetrain.angleToTurnTo >= -121.0).whileTrue(new SwerveTurnRed(60, Side.Left));
    m_driverController.a().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= 61.0).and(() -> drivetrain.angleToTurnTo >= 59.0).whileTrue(new SwerveTurnRed(-120, Side.Left));
    m_driverController.a().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= 121.0).and(() -> drivetrain.angleToTurnTo >= 119.0).whileTrue(new SwerveTurnRed(-60, Side.Left));
    m_driverController.a().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= -179.0).and(() -> drivetrain.angleToTurnTo >= -181.0).whileTrue(new SwerveTurnRed(0, Side.Left));
    m_driverController.a().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= -59.0).and(() -> drivetrain.angleToTurnTo >= -61.0).whileTrue(new SwerveTurnRed(120, Side.Left));

    m_driverController.b().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= 1.0).and(() -> drivetrain.angleToTurnTo >= -1.0).whileTrue(new SwerveTurnRed(179, Side.Right));
    m_driverController.b().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= -119.0).and(() -> drivetrain.angleToTurnTo >= -121.0).whileTrue(new SwerveTurnRed(60, Side.Right));
    m_driverController.b().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= 61.0).and(() -> drivetrain.angleToTurnTo >= 59.0).whileTrue(new SwerveTurnRed(-120, Side.Right));
    m_driverController.b().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= 121.0).and(() -> drivetrain.angleToTurnTo >= 119.0).whileTrue(new SwerveTurnRed(-60, Side.Right));
    m_driverController.b().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= -179.0).and(() -> drivetrain.angleToTurnTo >= -181.0).whileTrue(new SwerveTurnRed(0, Side.Right));
    m_driverController.b().and(isRedAlliance).and(() -> drivetrain.angleToTurnTo <= -59.0).and(() -> drivetrain.angleToTurnTo >= -61.0).whileTrue(new SwerveTurnRed(120, Side.Right));

    buttons.button(12).onTrue(new SetState(States.PrepScoreL1));
    // buttons.button(11).onTrue(new SetState(States.PrepScoreL2));
    // buttons.button(2).onTrue(new SetState(States.PrepScoreL3));
    // buttons.button(1).onTrue(new SetState(States.PrepScoreL4));

    // buttons.button(12).onTrue(new SetState(States.SmartScoreL1));
    buttons.button(11).onTrue(new SetState(States.SmartScoreL2));
    buttons.button(2).onTrue(new SetState(States.SmartScoreL3));
    buttons.button(1).onTrue(new SetState(States.SmartScoreL4));

    buttons.button(10).onTrue(new SetState(States.BallReturn));
    buttons.button(9).onTrue(new SetState(States.AlgaeLow));
    buttons.button(8).onTrue(new SetState(States.AlgaeHigh));

    buttons.button(6).onFalse(new SetState(States.Processor));
    buttons.button(7).onTrue(new SetState(States.BackwardBarge));

    buttons.button(3).onTrue(new SetState(States.Climbing));
    buttons.button(5).onTrue(new SpinClimb(s_Climb, 1)).onFalse(new SpinClimb(s_Climb, 0));
    buttons.button(4).onTrue(new SpinClimb(s_Climb, -1)).onFalse(new SpinClimb(s_Climb, 0));
  }

  public void teleopTriggers() {
    Trigger objectInClaw = new Trigger(() -> ArmIntake.objectInClaw);
    Trigger isCoralIntaking = new Trigger(() -> ArmIntake.coralIntakeState == CoralIntakeState.CoralIntaking);
    // Trigger isBallIntaking = new Trigger(() -> ArmIntake.publicBallIntakeState == BallIntakeState.BallIntaking);
    Trigger withinDistance = new Trigger(() -> drivetrain.distanceToTag() <= 1.15);//meters
    objectInClaw.and(isCoralIntaking).onTrue(new SetState(States.Default));
    // objectInClaw.and(isBallIntaking).onTrue(new SetState(States.BallReturn));

    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L4).whileTrue(new SetState(States.PrepScoreL4));
    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L3).whileTrue(new SetState(States.PrepScoreL3));
    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L2).whileTrue(new SetState(States.PrepScoreL2));
    // (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L1).whileTrue(new SetState(States.PrepScoreL1));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
