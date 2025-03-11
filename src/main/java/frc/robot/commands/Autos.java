// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos extends SequentialCommandGroup{
  /** Example static factory for an autonomous command. */
  public static double tareAngle;

  public Autos(String pathName) {
    if (pathName.equals("L4 + Alagae")) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        tareAngle = 180;
      } else {
        tareAngle = 0;
      }
    } else if (pathName.equals("PUSH")) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        tareAngle = 90;
      } else {
        tareAngle = 270;
      }
    } else {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        tareAngle = 180;
      } else {
        tareAngle = 0;
      }
    }

    Command auto = RobotContainer.drivetrain.followPath(pathName);
    addCommands(auto);
  }
}
