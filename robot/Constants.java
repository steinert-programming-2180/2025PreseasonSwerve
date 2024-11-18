// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static final int driveControllerPort = 0;
    public static final double Deadband=0.1;
  }

  public static final class DriveConstants{
    //max speed in meters per second
    //max angular speed in radians per second
    public static final double maxSpeed=8.0;
    public static final double maxAngularSpeed=4*Math.PI;


    //imma be honest idk what the units on this are, imma guess meters per second squared and radians per second squared
    public static final double driveAccMax=3;
    public static final double turnAccMax=3;
    
    //makes a SwerveDriveKinematics object that is later used in the drive command.
    //tells the robot where everything is so the computer can do the math
    public static final double trackWidth=Units.inchesToMeters(1);
    public static final double wheelBase=Units.inchesToMeters(1);
    public static final SwerveDriveKinematics driveKinematics=new SwerveDriveKinematics(
      new Translation2d(wheelBase/2, -trackWidth/2),
      new Translation2d(wheelBase/2, trackWidth/2),
      new Translation2d(-wheelBase/2, -trackWidth/2),
      new Translation2d(-wheelBase/2, trackWidth/2)
    );

    //these are long. Contain everything needed to define each swerve module
    public static final int FLDrivePort=5;
    public static final int FLTurnPort=6;
    public static final int FLAbsEncoderPort=6;
    public static final double FLAbsEncoderOffset=0; //0.263
    public static final boolean FLDriveReversed=false;
    public static final boolean FLTurnReversed=false;
    public static final boolean FLAbsEncoderReversed=false;

    public static final int FRDrivePort=1;
    public static final int FRTurnPort=2;
    public static final int FRAbsEncoderPort=2;
    public static final double FRAbsEncoderOffset=0; //0.443
    public static final boolean FRDriveReversed=false;
    public static final boolean FRTurnReversed=false;
    public static final boolean FRAbsEncoderReversed=false;

    public static final int BLDrivePort=7;
    public static final int BLTurnPort=8;
    public static final int BLAbsEncoderPort=8;
    public static final double BLAbsEncoderOffset=0; //0.293
    public static final boolean BLDriveReversed=false;
    public static final boolean BLTurnReversed=false;
    public static final boolean BLAbsEncoderReversed=false;

    public static final int BRDrivePort=3;
    public static final int BRTurnPort=4;
    public static final int BRAbsEncoderPort=4;
    public static final double BRAbsEncoderOffset=0; //0.986
    public static final boolean BRDriveReversed=false;
    public static final boolean BRTurnReversed=false;
    public static final boolean BRAbsEncoderReversed=false;
    
  }

  public static final class ModuleConstants{
    public static final double wheelDiameterM=Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = 5.9/1;
    public static final double turnMotorGearRatio = 5.9/1;

    //drive encoder value converts from rotations and RPM to meters and meters per second
    //turn encoder value converts from rotations and ROM to radians and radians per second
    public static final double driveEncoderConv=driveMotorGearRatio * Math.PI * wheelDiameterM;
    public static final double driveEncoderConvROC=driveEncoderConv/60;
    public static final double turnEncoderConv=turnMotorGearRatio * 2 * Math.PI;
    public static final double turnEncoderConvROC=turnEncoderConv/60;
    public static final double turningP=0.5;
    public static final double turningI=0;
    public static final double turningD=0;
  }
}
