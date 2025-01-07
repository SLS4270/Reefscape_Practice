package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveDriveLLComand extends Command {
    CommandSwerveDrivetrain swerveSubsys;

    public SwerveDriveLLComand(CommandSwerveDrivetrain swerveSubsys) {
        this.swerveSubsys = swerveSubsys;
        addRequirements(swerveSubsys);
    }
    
    @Override
    public void execute() {
        swerveSubsys.setControl(RobotContainer.driveRC
        .withVelocityX(CommandSwerveDrivetrain.limelight_forward_proportional())
        .withVelocityY(CommandSwerveDrivetrain.limelight_side_to_side_proportional()));
    }

    @Override
    public boolean isFinished() {
        return (CommandSwerveDrivetrain.limelight_forward_proportional() < 0.4)
         && (CommandSwerveDrivetrain.limelight_side_to_side_proportional() < 0.4);
    }
}
