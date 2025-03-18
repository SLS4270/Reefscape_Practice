package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
