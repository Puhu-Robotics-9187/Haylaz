// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
  public static class kRed{
    public static Pose2d _1 = new Pose2d(13, 2,Rotation2d.fromDegrees(25));
    public static Pose2d _2 = new Pose2d(0, 0,Rotation2d.fromDegrees(0));
    public static Pose2d _3 = new Pose2d(0, 0,Rotation2d.fromDegrees(0));
  }
  public static class kBlue{
    public static Pose2d _1 = new Pose2d(0, 0,Rotation2d.fromDegrees(180));
    public static Pose2d _2 = new Pose2d(0, 0,Rotation2d.fromDegrees(180));
    public static Pose2d _3 = new Pose2d(0, 0,Rotation2d.fromDegrees(180));
  }
  public static class DriveConstants {
  public static int kR1 = 1;
  public static int kL1 = 2; 
  public static int kR2 = 3;
  public static int kL2 = 4;

  public static class lEncoder{
   public static int port0 = 0;
   public static int port1 = 1;
  }

  public static class rEncoder{
    public static int port0 = 2;
    public static int port1 = 3;
   }  
  public static final double kWheelRadius = 0.0508;
  public static final int kEncoderResolution = -4096;

  public static class pidLeft{
    public static double kp=0.14;
    public static double kd=0.014;
  }
  public static class pidRight{
    public static double kp=0.14;
    public static double kd=0.014;
  }
  public static double kTrackWidthMeters = Units.inchesToMeters(27);
  
  }

  public static class startArmPose{
    public static double armSetpoint = Units.degreesToRadians(0);
    public static double intakeSetpoint = Units.degreesToRadians(180);
  }
  public static class intakeArmPose{
    public static double armSetpoint = Units.degreesToRadians(15);
    public static double intakeSetpoint = Units.degreesToRadians(15);
  }
  public static class firstArmPose{
    public static double armSetpoint = Units.degreesToRadians(06);
    public static double intakeSetpoint = Units.degreesToRadians(180);
  }
  public static class secondArmPose{
    public static double armSetpoint = Units.degreesToRadians(0);
    public static double intakeSetpoint = Units.degreesToRadians(180);
  }
  public static class thirdArmPose{
    public static double armSetpoint = Units.degreesToRadians(0);
    public static double intakeSetpoint = Units.degreesToRadians(180);
  }
  public static class stationArmPose{
    public static double armSetpoint = Units.degreesToRadians(0);
    public static double intakeSetpoint = Units.degreesToRadians(180);
  }



  //Arm constants
  public static int kArm = 5;
  public static double massArm = 7.2 ;
  public static double reductionArm = 64;
  public static double armlengths = 1.2;

  public static int kIntake = 6; 
  public static double massIntake = 1.5;
  public static double reductionIntake = 40; 
  public static double intakeLengths = 0.5;


  public static class ArmEncoder{
    public static int port0 = 4;
    public static int port1 = 5;
   }
}
