package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class SwerveTurn extends SequentialCommandGroup{
    
    

    public SwerveTurn () {
        SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
            .withTargetDirection(Rotation2d.fromDegrees(RobotContainer.drivetrain.angleToTurnTo))
            .withRotationalDeadband(RobotContainer.MaxAngularRate * 0.01);
        angle.HeadingController = new PhoenixPIDController(3, 0, 0);
        addCommands(
            RobotContainer.drivetrain.applyRequest(() -> angle
                .withVelocityX(-RobotContainer.joystick.getLeftY() * RobotContainer.MaxSpeed)
                .withVelocityY(-RobotContainer.joystick.getLeftX() * RobotContainer.MaxSpeed))
        );
    }
}
