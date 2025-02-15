// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.SetState;
import frc.robot.commands.SwerveTurn;
import frc.robot.commands.SetState.States;
import frc.robot.commands.StateCommands.CoralLevels;
import frc.robot.commands.SubsystemCommands.RunIntakeWrist;
import frc.robot.commands.SubsystemCommands.RunRotator;
import frc.robot.commands.SubsystemCommands.SpinArmIntake;
import frc.robot.commands.SubsystemCommands.SpinClimb;
import frc.robot.commands.SubsystemCommands.SetClimbPos;
import frc.robot.commands.SubsystemCommands.SpinIntake;
import frc.robot.subsystems.ArmIntake;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.Rotator;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12VoltsMps desired top speed
  public static final double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
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
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  public static final SwerveRequest.RobotCentric driveRC = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private static SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // drivetrain.configNeutralMode(NeutralModeValue.Brake);
    // drivetrain.configPathPlanner();
    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Test", new Autos("AutoTest"));
    // Configure the trigger bindings
   

    
    SmartDashboard.putData("Auto", m_chooser);
    drivetrain.configNeutralMode(NeutralModeValue.Brake);
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                       // negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Trigger objectInClaw = new Trigger(() -> ArmIntake.objectInClaw);
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // m_driverController.leftBumper().whileTrue(drivetrain.applyRequest(() -> driveRC
    // .withVelocityX(CommandSwerveDrivetrain.limelight_forward(0))
    // .withVelocityY(CommandSwerveDrivetrain.limelight_side_to_side_proportional(-20))
    // .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    // m_driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> driveRC
    // .withVelocityX(CommandSwerveDrivetrain.limelight_forward(0))
    // .withVelocityY(CommandSwerveDrivetrain.limelight_side_to_side_proportional(20))
    // .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    // m_driverController.x().onTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset(), drivetrain));

    m_driverController.leftBumper().and(() -> Elevator.elevatorLevel == CoralLevels.L4).onTrue(new SetState(States.ScoringL4)).onFalse(new SetState(States.Default));
    m_driverController.leftBumper().and(() -> Elevator.elevatorLevel == CoralLevels.L3).onTrue(new SetState(States.ScoringL3)).onFalse(new SetState(States.Default));
    m_driverController.leftBumper().and(() -> Elevator.elevatorLevel == CoralLevels.L2).onTrue(new SetState(States.ScoringL2)).onFalse(new SetState(States.Return));
    m_driverController.leftBumper().and(() -> Elevator.elevatorLevel == CoralLevels.L1).onTrue(new SetState(States.ScoringL1)).onFalse(new SetState(States.Return));
    m_driverController.leftTrigger().onTrue(new SetState(States.Intaking)).onFalse(new SetState(States.Indexing));
    m_driverController.button(3).onTrue(new SpinArmIntake(s_ArmIntake, 0.5)).onFalse(new SpinArmIntake(s_ArmIntake, 0.15));
    m_driverController.button(4).onTrue(new SpinArmIntake(s_ArmIntake, -1)).onFalse(new SpinArmIntake(s_ArmIntake,0));
    m_driverController.pov(180).onTrue(new InstantCommand(() -> drivetrain.tareSwerve(), drivetrain));

    m_driverController.pov(0).and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.pov(0).and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.pov(90).and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    m_driverController.pov(90).and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));

    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> driveRC
    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
    .withVelocityY(CommandSwerveDrivetrain.limelight_side_to_side_proportional(-15))
    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    m_driverController.b().whileTrue(drivetrain.applyRequest(() -> driveRC
    .withVelocityX(-joystick.getLeftY() * MaxSpeed)
    .withVelocityY(CommandSwerveDrivetrain.limelight_side_to_side_proportional(15))
    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    buttons.button(1).onTrue(new SetState(States.PrepScoreL1));
    buttons.button(2).onTrue(new SetState(States.PrepScoreL2));
    buttons.button(3).onTrue(new SetState(States.PrepScoreL3));
    buttons.button(4).onTrue(new SetState(States.PrepScoreL4));
    buttons.button(5).onTrue(new SetState(States.AlgaeLow));
    buttons.button(6).onTrue(new SetState(States.AlgaeHigh));
    buttons.button(7).onTrue(new SpinClimb(s_Climb, 0.75)).onFalse(new SpinClimb(s_Climb, 0));
    buttons.button(8).onTrue(new SetState(States.Default));
    buttons.button(9).onTrue(new SetState(States.BargeScore));
    buttons.button(10).onTrue(new SpinClimb(s_Climb, -0.75)).onFalse(new SpinClimb(s_Climb, 0));
    buttons.button(11).onTrue(new SetState(States.Climbing));
    buttons.button(12).onFalse(new SetClimbPos(s_Climb, 0));

    objectInClaw.onTrue(new SpinArmIntake(s_ArmIntake, 0.15));

    // m_driverController.leftBumper().onTrue(new SpinIntake(s_Intake, -1)).onFalse(new SpinIntake(s_Intake, 0));
    // m_driverController.leftBumper().onTrue(new RunRotator(s_Rotator, 26)).onFalse(new RunRotator(s_Rotator, 0.4));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
