package frc.robot.commands.TeleopAlignment;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;

public class BargeAlign extends SequentialCommandGroup {
    
    public BargeAlign() {
        SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(Constants.bargeTags[0].getRotation())
            .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01)
            .withDeadband(RobotContainer.MaxSpeed * 0.008);
        angle.HeadingController = new PhoenixPIDController(4.52, 0, 0);
        angle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        double xKP = 4.25;
        double yKP = 4.25;

        addCommands(
            RobotContainer.drivetrain.applyRequest(() -> angle
                .withVelocityX(
                    ((Constants.bargeTags[0].getX())
                     - RobotContainer.drivetrain.getPoseEstimate().getX())
                      * xKP)
                .withVelocityY(
                    ((Constants.bargeTags[0].getY())
                    - RobotContainer.drivetrain.getPoseEstimate().getY()) 
                        * yKP)
            )
        );
    }
}
