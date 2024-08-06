// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
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
 
  
  
  public static final class DriveConstants {
    
    //CAN Spark IDS
    public static final int kDriverControllerPort = 0;
    public static int kLeftMotorPort;
    public static int kRightMotorPort;
    public static final double kDriveDeadband = 0.05;
    public static final int FRONT_LEFT_DRIVE_ID = 3;
    public static final int FRONT_RIGHT_DRIVE_ID = 1;
    public static final int REAR_LEFT_DRIVE_ID = 5;
    public static final int REAR_RIGHT_DRIVE_ID = 7;
    public static final int FRONT_LEFT_TURN_ID = 4;
    public static final int FRONT_RIGHT_TURN_ID = 2;
    public static final int REAR_LEFT_TURN_ID = 6;
    public static final int REAR_RIGHT_TURN_ID = 8;
    public static final double kWheelBase = Units.inchesToMeters(21.5);
    public static final double kTrackWidth = Units.inchesToMeters(17.5);
    public static Constraints kAimProfile = new Constraints(3 * Math.PI, 2 * Math.PI);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
      );
    //slew
    public static final double kDirectionSlewRate = 8; // radians per second Higher is faster
    public static final double kMagnitudeSlewRate = 3; // percent per second (1 = 100%) Higher is faster
    public static final double kRotationalSlewRate = 15; // percent per second (1 = 100%) Higher is faster
    public static final double kAimP = 1.7;
    public static final double kAimI = 0;
    public static final double kAimD = 0;
    
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {

    public static double kDrivingEncoderVelocityFactor;
    public static double kDrivingEncoderPositionFactor;
    public static double kTurningEncoderPositionFactor;
    public static double kTurningEncoderVelocityFactor;
    public static boolean kTurningEncoderInverted;
    public static double kTurningEncoderPositionPIDMinInput;
    public static double kTurningEncoderPositionPIDMaxInput;
    public static double kDrivingP;
    public static double kDrivingI;
    public static double kDrivingD;
    public static double kDrivingFF;
    public static double kDrivingMinOutput;
    public static double kDrivingMaxOutput;
    public static double kTurningP;
    public static double kTurningI;
    public static double kTurningD;
    public static double kTurningFF;
    public static double kTurningMinOutput;
    public static double kTurningMaxOutput;
    public static IdleMode kDrivingMotorIdleMode;
    public static IdleMode kTurningMotorIdleMode;
    public static int kDrivingMotorCurrentLimit;
    public static int kTurningMotorCurrentLimit;

  }
  public enum StageSide {
    LEFT,
    RIGHT,
    CENTRE
}
}
