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
  public static final int climbIntakeID = 43;

  public static final int armIntakeSensor = 40;

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

  public static final Pose2d[] bargeTags = {
    // new Pose2d(8.27, 6.14, Rotation2d.fromDegrees(180)),//blue
    // new Pose2d(9.28, 1.91, Rotation2d.fromDegrees(0))//red
    new Pose2d(6.5, 4.75, Rotation2d.fromDegrees(180)),//blue
    new Pose2d(9.28, 1.91, Rotation2d.fromDegrees(0))//red
  };


  public final class JakeSetpoints {
    public final class Elevator {
      public static final double eleDefault = 4.2; //right elevator motor
      public static final double eleIntake = 1.3;
      public static final double elePrepL2 = 0.66;
      public static final double elePrepL3 = 18.16;
      public static final double elePrepL4 = 52.8;
      public static final double elePrepL1 = 7.1;
      public static final double eleAlgaeL1 = 5.95;
      public static final double eleAlgaeL2 = 31.5;
      public static final double eleThrowAlgae = 54;
      public static final double eleTransferDown = 28;
      public static final double eleTransferStow = 37;
    }

    public final class Rotator {
      public static final double rotDefault = 0.605;
      public static final double rotBallDefault = 0.625;
      public static final double rotIntake = 0.115;
      public static final double rotPrepL2 = 0.43;
      public static final double rotPrepL3 = 0.46;
      public static final double rotPrepL4 = 0.498;
      public static final double rotPrepL1 = 0.305;
      public static final double rotReturn = 0.11;
      public static final double rotScoreL4Auto = 0.4;
      public static final double rotAlgaeIntake = 0.376;
      public static final double rotBargeThrowPrep = 0.3;
      public static final double rotBargeThrowRelease = 0.75;
      public static final double rotPrepClimb = 0.295;
      public static final double rotProcessor = 0.27;
    }

    public final class IntakeWrist {
      public static final double wristUp = -7.7;
      public static final double wristTransferOut = -10.5;
      public static final double wristTransfer = -6.5;
      public static final double wristOuttake = -24.6;
      public static final double wristIntake = -31.95;
    }

    public final class Climb {
      public static final double climbOut = 220;
      public static final double climbDefault = 58;
      public static final double climbIn = 5;
    }

  }

  public final class Jake2Setpoints {//change
    public final class Elevator {


      public static final double lEleDefault = -3.13; //left elevator motor
      public static final double lEleIntake = -1.02;
      public static final double lElePrepL2 = 0.26;
      public static final double lElePrepL3 = -13.27;
      public static final double lElePrepL4 = -42.84;
      public static final double lElePrepL1 = -5.49;
      public static final double lEleAlgaeL1 = -4.56;
      public static final double lEleAlgaeL2 = -26.69;
      public static final double lEleThrowAlgae = -44.325;

      public static final double eleDefault = 3.81; //right elevator motor
      public static final double eleIntake = 1.73;
      public static final double elePrepL2 = 0.45;
      public static final double elePrepL3 = 13.965;
      public static final double elePrepL4 = 43.56;
      public static final double elePrepL1 = 6.18;
      public static final double eleAlgaeL1 = 5.24;
      public static final double eleAlgaeL2 = 27.384;
      public static final double eleThrowAlgae = 45.01;

    }

    public final class Rotator {
      public static final double rotDefault = 0.61;
      public static final double rotBallDefault = 0.63;
      public static final double rotIntake = 0.117;
      public static final double rotPrepL2 = 0.435;
      public static final double rotPrepL3 = 0.465;
      public static final double rotPrepL4 = 0.503;
      public static final double rotPrepL1 = 0.31;
      public static final double rotReturn = 0.115;
      public static final double rotScoreL4Auto = 0.405;
      public static final double rotAlgaeIntake = 0.381;
      public static final double rotBargeThrowPrep = 0.305;
      public static final double rotBargeThrowRelease = 0.755;
      public static final double rotPrepClimb = 0.30;
      public static final double rotProcessor = 0.275;
    }

    public final class IntakeWrist {
      public static final double wristUp = -1.77;
      public static final double wristOuttake = -9.71;
      public static final double wristIntake = -24.49;
    }

    public final class Climb {
      public static final double climbOut = 220;//268.31
      public static final double climbDefault = 58;
      public static final double climbIn = 5;
    }
  }
  
}
