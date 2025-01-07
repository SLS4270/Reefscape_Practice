package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignCommand extends SequentialCommandGroup{

    public AlignCommand(CommandSwerveDrivetrain.RobotCentricDirectionOfCoral direction) {
        addCommands(
            new SequentialCommandGroup(
                new SwerveDriveLLComand(RobotContainer.drivetrain),
                RobotContainer.drivetrain.applyRequest(() -> 
                RobotContainer.driveRC.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(CommandSwerveDrivetrain.limelight_turn_to_coral(direction)))
            )
        );
    }
}
