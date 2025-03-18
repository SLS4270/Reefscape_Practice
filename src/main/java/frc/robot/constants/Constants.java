// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //intakes
  public static final int intakeID = 3;
  public static final int indexerID = 13;
  public static final int armIntakeID = 23;
  public static final int ballIntakeID = 33;

  public static final int intakeSensor = 9;
  public static final int armIntakeSensor = 19;

  //wrists
  public static final int climbID = 4;
  public static final int intakeWristID1 = 14;
  public static final int intakeWristID2 = 24;
  public static final int rotatorID = 34;
  public static final int ballWristID = 44;

  public static final int rotatorEncoderPort = 0;

  //elevator
  public static final int elevatorID1 = 5;
  public static final int elevatorID2 = 15;

  //apriltag poses
  public static final Pose2d[] apriltagPosesXYBlueCoral = {
    new Pose2d(4.07, 3.31, Rotation2d.fromDegrees(240)),//17
    new Pose2d(3.66, 4.03, Rotation2d.fromDegrees(180)),//18
    new Pose2d(4.07, 4.75, Rotation2d.fromDegrees(120)),//19
    new Pose2d(4.90, 4.75, Rotation2d.fromDegrees(60)),//20
    new Pose2d(5.32, 4.03, Rotation2d.fromDegrees(0)),//21
    new Pose2d(4.90, 3.31, Rotation2d.fromDegrees(300))//22
  };

  public static final Pose2d[] apriltagPosesXYRedCoral = {
    new Pose2d(13.47, 3.31, Rotation2d.fromDegrees(300)),//6
    new Pose2d(13.89, 4.03, Rotation2d.fromDegrees(0)),//7
    new Pose2d(13.47, 4.75, Rotation2d.fromDegrees(60)),//8
    new Pose2d(12.64, 4.75, Rotation2d.fromDegrees(120)),//9
    new Pose2d(12.23, 4.03, Rotation2d.fromDegrees(180)),//10
    new Pose2d(12.64, 3.31, Rotation2d.fromDegrees(240))//11
  };

  public static final Pose2d[] allApriltagPoses = {

    //Blue
    new Pose2d(4.07, 3.31, Rotation2d.fromDegrees(240)),//17
    new Pose2d(3.66, 4.03, Rotation2d.fromDegrees(180)),//18
    new Pose2d(4.07, 4.75, Rotation2d.fromDegrees(120)),//19
    new Pose2d(4.90, 4.75, Rotation2d.fromDegrees(60)),//20
    new Pose2d(5.32, 4.03, Rotation2d.fromDegrees(0)),//21
    new Pose2d(4.90, 3.31, Rotation2d.fromDegrees(300)),//22

    //Red
    new Pose2d(13.47, 3.31, Rotation2d.fromDegrees(300)),//6
    new Pose2d(13.89, 4.03, Rotation2d.fromDegrees(0)),//7
    new Pose2d(13.47, 4.75, Rotation2d.fromDegrees(60)),//8
    new Pose2d(12.64, 4.75, Rotation2d.fromDegrees(120)),//9
    new Pose2d(12.23, 4.03, Rotation2d.fromDegrees(180)),//10
    new Pose2d(12.64, 3.31, Rotation2d.fromDegrees(240))//11
  };


  
}
