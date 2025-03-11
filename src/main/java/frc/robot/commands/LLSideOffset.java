package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LLSideOffset extends SequentialCommandGroup{

    public static enum Side {
        Left,
        Right
    }
    
    public LLSideOffset(double angleToTurnTo, Side side) {

        if (side == Side.Right) {
            addCommands(
                RobotContainer.drivetrain.applyRequest(() -> RobotContainer.driveRC
                    .withVelocityX(null)
                )
            );
        } else {
            addCommands(
            );
        }

    }
}
