// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  //intakes
  public static final int intakeID = 3;
  public static final int indexerID = 13;
  public static final int armIntakeID = 23;
  public static final int ballIntakeID = 33;
  public static final int climbIntakeID = 43;
  public static final int l1IntakeID = 53;

  public static final int armIntakeSensor = 40;

  //wrists
  public static final int climbID = 4;
  public static final int intakeWristID1 = 14;
  public static final int intakeWristID2 = 24;
  public static final int rotatorID = 34;
  public static final int ballWristID = 44;

  public static final int rotatorEncoderPort = 0;

  //rElevator
  public static final int rElevatorID1 = 5;
  public static final int rElevatorID2 = 15;

  //leds
  public static final int ledsID = 61;

  //coral levels
  public enum CoralLevels {
    L1,
    L2,
    L3,
    L4,
    L4Auto
  }

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

  public final class Setpoints {
    public final class Elevator {
      public static final double rEleDefault = 3.81;
      public static final double rEleIntake = 1.0;
      public static final double rElePrepL2 = 0.45;
      public static final double rElePrepL3 = 13.965;
      public static final double rElePrepL4 = 43.56;
      public static final double rElePrepL1 = 6.18;
      public static final double rEleAlgaeL1 = 5.24;
      public static final double rEleAlgaeL2 = 27.384;
      public static final double rEleThrowAlgae = 45.01;
    }

    public final class Rotator {
      public static final double rotDefault = 0.61;
      public static final double rotBallDefault = 0.63;
      public static final double rotIntake = 0.125;
      public static final double rotPrepL2 = 0.435;
      public static final double rotPrepL3 = 0.465;
      public static final double rotPrepL4 = 0.503;
      public static final double rotPrepL1 = 0.31;
      public static final double rotReturn = 0.13;
      public static final double rotScoreL4Auto = 0.405;
      public static final double rotAlgaeIntake = 0.381;
      public static final double rotBargeThrowPrep = 0.38;
      public static final double rotBargeThrowRrElease = 0.755;
      public static final double rotPrepClimb = 0.381;
      public static final double rotProcessor = 0.275;
    }

    public final class IntakeWrist {
      public static final double wristUp = -4;
      public static final double wristOuttake = -9.71;
      public static final double wristIntake = -24.4;
      public static final double wristClimbPrep = -4.1;
    }

    public final class Climb {
      public static final double climbOut = 220;
      public static final double climbDefault = 58;
      public static final double climbIn = 40;
    }
  }
}
