package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    public double angleToTurnTo;

    StructPublisher<Pose2d> publisher;
    StructPublisher<Pose2d> publisher2;

    StructArrayPublisher<SwerveModuleState> swerveStates;
    StructPublisher<ChassisSpeeds> swerveSpeeds;
    StructPublisher<Rotation2d> swerveRotation;
    Pose2d pose;
    Pose2d nearestTag;
    SwerveDrivePoseEstimator poseEstimator;
    public boolean hasTargets;
    public double distance;


    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    public SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();
        publisher2 = NetworkTableInstance.getDefault()
            .getStructTopic("nearestTag", Pose2d.struct).publish();

        swerveStates = NetworkTableInstance.getDefault().getStructArrayTopic("states", SwerveModuleState.struct).publish();
        swerveSpeeds = NetworkTableInstance.getDefault().getStructTopic("speeds", ChassisSpeeds.struct).publish();
        swerveRotation = NetworkTableInstance.getDefault().getStructTopic("rotation", Rotation2d.struct).publish();
        this.seedFieldCentric();
        LimelightHelpers.setCameraPose_RobotSpace("limelight-front", 0.234, 0.02, 0.254, 0, 0, 21.45);
        LimelightHelpers.setCameraPose_RobotSpace("limelight", 0.234, -0.02, 0.254, 0, 0, -21.45);
        poseEstimator = new SwerveDrivePoseEstimator(
            getKinematics(), 
            this.getPigeon2().getRotation2d(),
            this.getState().ModulePositions, 
            new Pose2d());
        angleToTurnTo = 0.0;
        pose = new Pose2d();
        nearestTag = new Pose2d();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front").tagCount > 0 || LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").tagCount > 0)  {
            hasTargets = true;
        } else {
            hasTargets = false;
        }
        angleToTurnTo = findClosestApriltag(Constants.allApriltagPoses).getRotation().getDegrees() - 180;

        SmartDashboard.putNumber("turnToWhatAngle", angleToTurnTo);
        SmartDashboard.putNumber("currentAngle", this.getPigeon2().getRotation2d().getDegrees());
        SmartDashboard.putNumber("LimelightTest", LimelightHelpers.getTX("limelight-front"));
        updateOdometry();
        pose = poseEstimator.getEstimatedPosition();
        nearestTag = findClosestApriltag(Constants.allApriltagPoses);
        distance = poseEstimator.getEstimatedPosition().getTranslation().getDistance(findClosestApriltag(Constants.allApriltagPoses).getTranslation());
        SmartDashboard.putNumber("distanceToTag", distanceToTag());

        publisher.set(pose);
        publisher2.set(findClosestApriltag(Constants.allApriltagPoses));

        swerveStates.set(this.getState().ModuleStates);
        swerveSpeeds.set(this.getState().Speeds);
        swerveRotation.set(this.getState().Pose.getRotation());
    }

    public void updateOdometry() {
        poseEstimator.update(this.getPigeon2().getRotation2d(), this.getState().ModulePositions);
        LimelightHelpers.SetRobotOrientation("limelight-front", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        if (this.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() < 720 && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front").tagCount > 0) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.8, 0.8, 9999999));
            poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front").pose, 
                                          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front").timestampSeconds);
            this.setVisionMeasurementStdDevs(VecBuilder.fill(0.8, 0.8, 9999999));
            this.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front").pose, 
                                          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front").timestampSeconds);
        } 
        if (this.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() < 720 && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").tagCount > 0) {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.8, 0.8, 9999999));
            poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose, 
                                          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").timestampSeconds);
            this.setVisionMeasurementStdDevs(VecBuilder.fill(0.8, 0.8, 9999999));
            this.addVisionMeasurement(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose, 
                                          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").timestampSeconds);
        } 
    }

    public void configPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> poseEstimator.getEstimatedPosition(), 
                this.poseEstimator::resetPose, 
                () -> this.getState().Speeds, 
                (speeds, feedforwards) -> setControl(
                    new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                    ), 
                new PPHolonomicDriveController(
                    new PIDConstants(3.52, 0, 0),
                    new PIDConstants(3.52, 0, 0)
                ), 
                config, 
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
                this);
        } catch (Exception e) {
            e.printStackTrace();
        }

    }
      public static double limelight_side_to_side_proportional(double offset, String limelightName) {    
        if (LimelightHelpers.getFiducialID(limelightName) > 0) {
            double kP = .005;
            double yVelocity = -(LimelightHelpers.getTX(limelightName) + offset) * kP * RobotContainer.MaxSpeed;
            return yVelocity;
        } else {
            return 0.0;
        }
    }

    public static double limelight_forward(double offset) {
        if (LimelightHelpers.getFiducialID("limelight-front") > 0) {
            double kP = .0078;
    
            // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
            // your limelight 3 feed, tx should return roughly 31 degrees.
            double xVelocity = (LimelightHelpers.getTY("limelight-front") + offset) * kP * RobotContainer.MaxSpeed;
    
    
            return xVelocity;
        } else {
            return 0.0;
        }
    }

    public Pose2d findClosestApriltag(Pose2d[] apriltagPoses) {
        return poseEstimator.getEstimatedPosition().nearest(Arrays.asList(apriltagPoses));
    }

    public Command followPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void tareSwerve() {
        this.getPigeon2().setYaw(0);
    }

    public void tarePose() {
        this.resetPose(poseEstimator.getEstimatedPosition());
    }

    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    public double distanceToTag() {
        return distance;
    }

    public void resetOdom(Pose2d pose) {
        this.getPigeon2().setYaw(pose.getRotation().getDegrees() + 90.0);
        poseEstimator.resetPose(pose);
    }
}
