// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.States.SetState;
import frc.robot.commands.States.SetStateJake2;
import frc.robot.commands.States.SetState.States;
import frc.robot.commands.States.SetStateJake2.StatesJake2;
import frc.robot.commands.SubsystemCommands.RunClimb;
import frc.robot.commands.SubsystemCommands.SetLEDs;
import frc.robot.commands.SubsystemCommands.SpinArmIntake;
import frc.robot.commands.SubsystemCommands.SpinClimb;
import frc.robot.commands.SubsystemCommands.SpinClimbIntake;
import frc.robot.commands.SubsystemCommands.SpinIntake;
import frc.robot.commands.TeleopAlignment.BargeAlign;
import frc.robot.commands.TeleopAlignment.SwerveTurn;
import frc.robot.commands.TeleopAlignment.SwerveTurnRed;
import frc.robot.commands.TeleopAlignment.SwerveTurn.Side;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TunerConstantsJake2;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Intakes.ArmIntake;
import frc.robot.subsystems.Intakes.ClimbIntake;
import frc.robot.subsystems.Intakes.Indexer;
import frc.robot.subsystems.Intakes.Intake;
import frc.robot.subsystems.Intakes.ArmIntake.BallIntakeState;
import frc.robot.subsystems.Intakes.ArmIntake.CoralIntakeState;
import frc.robot.subsystems.LEDs.LEDStates;
import frc.robot.subsystems.Wrists.Climb;
import frc.robot.subsystems.Wrists.IntakeWrist;
import frc.robot.subsystems.Wrists.Rotator;
import frc.robot.constants.Constants.CoralLevels;

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

  public static final CommandSwerveDrivetrain drivetrain = TunerConstantsJake2.createDrivetrain(); // My drivetrain
  public static final IntakeWrist s_IntakeWrist = new IntakeWrist();
  public static final Intake s_Intake = new Intake();
  public static final Indexer s_Indexer = new Indexer();
  public static final ArmIntake s_ArmIntake = new ArmIntake();
  public static final Rotator s_Rotator = new Rotator();
  public static final Elevator s_Elevator = new Elevator();
  public static final Climb s_Climb = new Climb();
  public static final ClimbIntake s_ClimbIntake = new ClimbIntake();
  public static final LEDs s_LEDs = new LEDs();


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
    drivetrain.configPathPlanner();

    configNamedCommandsJake2();
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
    // s_LEDs.setDefaultCommand(new SetLEDs(s_LEDs, LEDStates.Default));
    configureBindingsJake2();
  }

  private void configureBindings() {
    Trigger isRedAlliance = new Trigger(() -> Robot.alliance == Alliance.Red);

    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L4).onTrue(new SetState(States.ScoringL4)).onFalse(new SetState(States.Default));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L3).onTrue(new SetState(States.ScoringL3)).onFalse(new SetState(States.Return));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L2).onTrue(new SetState(States.ScoringL2)).onFalse(new SetState(States.Return));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L1).onTrue(new SetState(States.ScoringL1)).onFalse(new SetState(States.Return));
    m_driverController.leftTrigger().onTrue(new SetState(States.Intaking)).onFalse(new SetState(States.Indexing));
    m_driverController.rightBumper().onTrue(new SetState(States.Outtaking)).onFalse(new SetState(States.Default));

    m_driverController.button(3).onTrue(new SpinArmIntake(s_ArmIntake, -0.75));
    m_driverController.button(4).onTrue(new SpinArmIntake(s_ArmIntake, 1)).onFalse(new SpinArmIntake(s_ArmIntake,0));
    m_driverController.pov(0).onTrue(new InstantCommand(() -> drivetrain.tarePose(), drivetrain));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> drivetrain.tareSwerve(), drivetrain));
    m_driverController.leftBumper().onTrue(new SetState(States.IntakeL1)).onFalse(new SetState(States.BallIdle));

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

    // buttons.button(12).onTrue(new SetState(States.PrepScoreL1));
    // buttons.button(11).onTrue(new SetState(States.PrepScoreL2));
    // buttons.button(2).onTrue(new SetState(States.PrepScoreL3));
    // buttons.button(1).onTrue(new SetState(States.PrepScoreL4));

    buttons.button(12).onTrue(new SetState(States.SmartScoreL1));
    buttons.button(11).onTrue(new SetState(States.SmartScoreL2));
    buttons.button(2).onTrue(new SetState(States.SmartScoreL3));
    buttons.button(1).onTrue(new SetState(States.SmartScoreL4));

    buttons.button(10).onTrue(new SetState(States.BallReturn));
    buttons.button(9).onTrue(new SetState(States.AlgaeLow));
    buttons.button(8).onTrue(new SetState(States.AlgaeHigh));

    buttons.button(6).onTrue(new SetState(States.Processor));
    buttons.button(7).whileTrue(new BargeAlign().alongWith(new SetState(States.PrepThrow))).onFalse(new SetState(States.ThrowAlgae));

    buttons.button(5).onTrue(new SetState(States.Climbing));
    buttons.button(4).onTrue(new RunClimb(s_Climb, Constants.JakeSetpoints.Climb.climbIn).alongWith(new SpinClimbIntake(s_ClimbIntake, 0)));

    buttons.button(3).onTrue(new SetState(States.TransferBall));
  }

  private void configureBindingsJake2() {
    Trigger isRedAlliance = new Trigger(() -> Robot.alliance == Alliance.Red);

    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L4).onTrue(new SetStateJake2(StatesJake2.ScoringL4)).onFalse(new SetStateJake2(StatesJake2.Default));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L3).onTrue(new SetStateJake2(StatesJake2.ScoringL3)).onFalse(new SetStateJake2(StatesJake2.Return));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L2).onTrue(new SetStateJake2(StatesJake2.ScoringL2)).onFalse(new SetStateJake2(StatesJake2.Return));
    m_driverController.rightTrigger().and(() -> Elevator.elevatorLevel == CoralLevels.L1).onTrue(new SetStateJake2(StatesJake2.ScoringL1)).onFalse(new SetStateJake2(StatesJake2.Return));
    m_driverController.leftTrigger().onTrue(new SetStateJake2(StatesJake2.Intaking)).onFalse(new SetStateJake2(StatesJake2.Indexing));
    m_driverController.rightBumper().onTrue(new SetStateJake2(StatesJake2.Outtaking)).onFalse(new SetStateJake2(StatesJake2.Default));

    m_driverController.button(3).onTrue(new SpinArmIntake(s_ArmIntake, -0.75));
    m_driverController.button(4).onTrue(new SpinArmIntake(s_ArmIntake, 1)).onFalse(new SpinArmIntake(s_ArmIntake,0));
    m_driverController.pov(0).onTrue(new InstantCommand(() -> drivetrain.tarePose(), drivetrain));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> drivetrain.tareSwerve(), drivetrain));
    m_driverController.leftBumper().onTrue(new SetStateJake2(StatesJake2.IntakeL1)).onFalse(new SetStateJake2(StatesJake2.Default));

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

    // buttons.button(12).onTrue(new SetStateJake2(StatesJake2.PrepScoreL1));
    // buttons.button(11).onTrue(new SetStateJake2(StatesJake2.PrepScoreL2));
    // buttons.button(2).onTrue(new SetStateJake2(StatesJake2.PrepScoreL3));
    // buttons.button(1).onTrue(new SetStateJake2(StatesJake2.PrepScoreL4));

    buttons.button(12).onTrue(new SetStateJake2(StatesJake2.SmartScoreL1));
    buttons.button(11).onTrue(new SetStateJake2(StatesJake2.SmartScoreL2));
    buttons.button(2).onTrue(new SetStateJake2(StatesJake2.SmartScoreL3));
    buttons.button(1).onTrue(new SetStateJake2(StatesJake2.SmartScoreL4));

    buttons.button(10).onTrue(new SetStateJake2(StatesJake2.BallReturn));
    buttons.button(9).onTrue(new SetStateJake2(StatesJake2.AlgaeLow));
    buttons.button(8).onTrue(new SetStateJake2(StatesJake2.AlgaeHigh));

    buttons.button(6).onTrue(new SetStateJake2(StatesJake2.Processor));
    buttons.button(7).whileTrue(new BargeAlign().alongWith(new SetStateJake2(StatesJake2.PrepThrow))).onFalse(new SetStateJake2(StatesJake2.ThrowAlgae));

    buttons.button(5).onTrue(new SetStateJake2(StatesJake2.Climbing));
    buttons.button(4).onTrue(new RunClimb(s_Climb, Constants.Jake2Setpoints.Climb.climbIn).alongWith(new SpinClimbIntake(s_ClimbIntake, 0)));
  }

  private void configNamedCommands() {
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
  }
  private void configNamedCommandsJake2() {
    NamedCommands.registerCommand("PrepScoreL4", new SetStateJake2(StatesJake2.PrepScoreL4Auto));
    NamedCommands.registerCommand("SpinIndexer", new SetStateJake2(StatesJake2.Indexing));
    NamedCommands.registerCommand("Intake", new SetStateJake2(StatesJake2.Intaking));
    NamedCommands.registerCommand("Outtake", new SpinArmIntake(s_ArmIntake, 1));
    NamedCommands.registerCommand("ScoreL4", new SetStateJake2(StatesJake2.ScoringL4Auto));
    NamedCommands.registerCommand("A1 Prep", new SetStateJake2(StatesJake2.AlgaeLow));
    NamedCommands.registerCommand("A2 Prep", new SetStateJake2(StatesJake2.AlgaeHigh));
    NamedCommands.registerCommand("AlgaeIntake", new SpinArmIntake(s_ArmIntake, -0.25));
    NamedCommands.registerCommand("ClawStop", new SpinArmIntake(s_ArmIntake, 0));
    NamedCommands.registerCommand("BargePrep", new SetStateJake2(StatesJake2.BackwardBarge));
    NamedCommands.registerCommand("ArmReturn", new SetStateJake2(StatesJake2.Default));
    NamedCommands.registerCommand("Slam4", new SetStateJake2(StatesJake2.ScoringL4));
  }



  public void teleopTriggers() {
    Trigger objectInClaw = new Trigger(() -> ArmIntake.objectInClaw);
    Trigger isCoralIntaking = new Trigger(() -> ArmIntake.coralIntakeState == CoralIntakeState.CoralIntaking);
    // Trigger isBallIntaking = new Trigger(() -> ArmIntake.publicBallIntakeState == BallIntakeState.BallIntaking);
    Trigger withinDistance = new Trigger(() -> drivetrain.distanceToTag() <= 1.15);//meters
    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L4).onTrue(new SetState(States.Default));
    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L3).onTrue(new SetState(States.Default));
    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L1).onTrue(new SetState(States.PrepScoreL1));
    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L2).onTrue(new SetState(States.PrepScoreL2));
    // objectInClaw.and(isBallIntaking).onTrue(new SetState(States.BallReturn));

    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L4).whileTrue(new SetState(States.PrepScoreL4));
    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L3).whileTrue(new SetState(States.PrepScoreL3));
    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L2).whileTrue(new SetState(States.PrepScoreL2));
    // (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L1).whileTrue(new SetState(States.PrepScoreL1));
  }
  public void teleopTriggersJake2() {
    Trigger objectInClaw = new Trigger(() -> ArmIntake.objectInClaw);
    Trigger isCoralIntaking = new Trigger(() -> ArmIntake.coralIntakeState == CoralIntakeState.CoralIntaking);
    // Trigger isBallIntaking = new Trigger(() -> ArmIntake.publicBallIntakeState == BallIntakeState.BallIntaking);
    Trigger withinDistance = new Trigger(() -> drivetrain.distanceToTag() <= 1.15);//meters
    Trigger withinScoringDistance = new Trigger(() -> drivetrain.distanceToTag() <= 0.7);//meters

    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L4).onTrue(new SetStateJake2(StatesJake2.Default));
    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L3).onTrue(new SetStateJake2(StatesJake2.Default));
    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L1).onTrue(new SetStateJake2(StatesJake2.PrepScoreL1));
    objectInClaw.and(isCoralIntaking).and(() -> Elevator.elevatorLevel == CoralLevels.L2).onTrue(new SetStateJake2(StatesJake2.PrepScoreL2));

    // objectInClaw.and(isBallIntaking).onTrue(new SetStateJake2(StatesJake2.BallReturn));

    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L4).whileTrue(new SetStateJake2(StatesJake2.PrepScoreL4));
    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L3).whileTrue(new SetStateJake2(StatesJake2.PrepScoreL3));
    (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L2).whileTrue(new SetStateJake2(StatesJake2.PrepScoreL2));

    (m_driverController.a().or(m_driverController.b())).whileTrue(new SetLEDs(s_LEDs, LEDStates.NotInScoringPosition));
    (m_driverController.a().or(m_driverController.b())).and(withinScoringDistance).onTrue(new SetLEDs(s_LEDs, LEDStates.InScoringPosition));
    // (m_driverController.a().or(m_driverController.b())).and(withinDistance).and(() -> Elevator.elevatorLevel == CoralLevels.L1).whileTrue(new SetStateJake2(StatesJake2.PrepScoreL1));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
