package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveTurn extends SequentialCommandGroup{

    public static enum Side {
        Left,
        Right
    }
    
    public SwerveTurn () {
        // if (side == Side.Right) {
        //     addCommands(
                
        //     );
        // } else {
        // }

        addCommands(
            RobotContainer.drivetrain.teleopPathfollowing(Constants.apriltagPosesXYBlueCoral)
        );

    }
}

// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.RobotContainer;

// public class SwerveTurn extends SequentialCommandGroup{

//     public static enum Side {
//         Left,
//         Right
//     }
    
//     public SwerveTurn (double angleToTurnTo, Side side) {
//         SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
//             .withTargetDirection(Rotation2d.fromDegrees(angleToTurnTo))
//             .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01)
//             .withDeadband(RobotContainer.MaxSpeed * 0.01);
//         angle.HeadingController = new PhoenixPIDController(4.5, 0, 0);
//         // SmartDashboard.putNumber("SwerveTurnAngle", angleToTurnTo);

//         // double xKP = 2.5;
//         // double yKP = 2.5;
//         // double driveRadius = 0.343;
//         if (side == Side.Right) {
//             addCommands(
//                 RobotContainer.drivetrain.applyRequest(() -> angle
//                     // .withVelocityX(
//                     //     DriverStation.getAlliance().get() == Alliance.Red ? 
//                     //     -(RobotContainer.drivetrain.findClosestApriltag(
//                     //         Constants.apriltagPosesXYRedCoral
//                     //     ).getX() - RobotContainer.drivetrain.getPoseEstimate().getX() - driveRadius) * xKP :

//                     //     (RobotContainer.drivetrain.findClosestApriltag(
//                     //         Constants.apriltagPosesXYBlueCoral
//                     //     ).getX() - RobotContainer.drivetrain.getPoseEstimate().getX() - driveRadius) * xKP)
//                     // .withVelocityY(
//                     //     DriverStation.getAlliance().get() == Alliance.Red ? 
//                     //     ((RobotContainer.drivetrain.findClosestApriltag(Constants.apriltagPosesXYRedCoral
//                     //     ).getY() - (0.125 
//                     //     // * Math.cos(angleToTurnTo)
//                     //     ))
//                     //         - RobotContainer.drivetrain.getPoseEstimate().getY()) * yKP : 

//                     //         ((RobotContainer.drivetrain.findClosestApriltag(Constants.apriltagPosesXYBlueCoral
//                     //         ).getY() - (0.23 
//                     //         * Math.cos(angleToTurnTo)
//                     //         ))
//                     //             - RobotContainer.drivetrain.getPoseEstimate().getY()) 
//                     //             * yKP)
//                                 )
//             );
//         } else {
//             addCommands(
//                 RobotContainer.drivetrain.applyRequest(() -> angle
//                     // .withVelocityX(
//                     //     DriverStation.getAlliance().get() == Alliance.Red ? 
//                     //     -(RobotContainer.drivetrain.findClosestApriltag(
//                     //         Constants.apriltagPosesXYRedCoral
//                     //     ).getX() - RobotContainer.drivetrain.getPoseEstimate().getX() - driveRadius) * xKP :

//                     //     (RobotContainer.drivetrain.findClosestApriltag(
//                     //         Constants.apriltagPosesXYBlueCoral
//                     //     ).getX() - RobotContainer.drivetrain.getPoseEstimate().getX() - driveRadius) * xKP)
//                     // .withVelocityY(
//                     //     DriverStation.getAlliance().get() == Alliance.Red ? 
//                     //     ((RobotContainer.drivetrain.findClosestApriltag(Constants.apriltagPosesXYRedCoral
//                     //     ).getY() - (0.25 
//                     //     * Math.cos(angleToTurnTo)
//                     //     ))
//                     //         - RobotContainer.drivetrain.getPoseEstimate().getY()) * yKP : 
                            
//                     //         ((RobotContainer.drivetrain.findClosestApriltag(Constants.apriltagPosesXYBlueCoral
//                     //         ).getY() + (0.12 
//                     //         * Math.cos(angleToTurnTo)
//                     //         ))
//                     //             - RobotContainer.drivetrain.getPoseEstimate().getY()) 
//                     //             * yKP)
//                                 )
//             );
//         }

//     }
// }

