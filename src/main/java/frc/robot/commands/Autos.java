// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos extends SequentialCommandGroup{
  /** Example static factory for an autonomous command. */

  public Autos(String pathName) {
    Command auto = RobotContainer.drivetrain.followPath(pathName);
    addCommands(auto);
  }
}
