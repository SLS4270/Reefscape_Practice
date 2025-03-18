package frc.robot.commands.TeleopAlignment;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class SwerveTurnRed extends SequentialCommandGroup{

    public static enum Side {
        Left,
        Right
    }
    
    public SwerveTurnRed(double angleToTurnTo, SwerveTurn.Side side) {
        SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(Rotation2d.fromDegrees(angleToTurnTo))
            .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01)
            .withDeadband(RobotContainer.MaxSpeed * 0.008);
        angle.HeadingController = new PhoenixPIDController(4.52, 0, 0);
        angle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        double xKP = 4.25;
        double yKP = 4.25;
        // double driveRadius = 0.343;
        double driveRadius = 0.41;
        if (side == SwerveTurn.Side.Right) {
            addCommands(
                RobotContainer.drivetrain.applyRequest(() -> angle
                    .withVelocityX(
                        (-(RobotContainer.drivetrain.findClosestApriltag(Constants.allApriltagPoses).getX() + 0.18 * Math.sin(Math.toRadians(-angleToTurnTo)))
                         + RobotContainer.drivetrain.getPoseEstimate().getX()
                          - (driveRadius * Math.cos(Math.toRadians(angleToTurnTo))))
                          * xKP)
                    .withVelocityY(
                            (-(RobotContainer.drivetrain.findClosestApriltag(Constants.allApriltagPoses).getY() + 0.18 * Math.cos(Math.toRadians(-angleToTurnTo)))
                                + RobotContainer.drivetrain.getPoseEstimate().getY()
                                 - (driveRadius * Math.sin(Math.toRadians(angleToTurnTo)))) 
                                * yKP)
                )
            );
        } else {
            addCommands(
                RobotContainer.drivetrain.applyRequest(() -> angle
                    .withVelocityX(
                        (-(RobotContainer.drivetrain.findClosestApriltag(Constants.allApriltagPoses).getX() - 0.18 * Math.sin(Math.toRadians(-angleToTurnTo)))
                         + RobotContainer.drivetrain.getPoseEstimate().getX() - (driveRadius * Math.cos(Math.toRadians(angleToTurnTo))))
                          * xKP)
                    .withVelocityY(
                            (-(RobotContainer.drivetrain.findClosestApriltag(Constants.allApriltagPoses).getY() - 0.18 * Math.cos(Math.toRadians(-angleToTurnTo)))
                                + RobotContainer.drivetrain.getPoseEstimate().getY() - (driveRadius * Math.sin(Math.toRadians(angleToTurnTo)))) 
                                * yKP)
                            )
            );
        }

    }
}

