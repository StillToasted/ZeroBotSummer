// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import org.ejml.equation.Variable;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTable;
import frc.utils.SwerveUtils;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/*note: need to define spi_port_id or replace it to f*/

public class DriverSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 
  private final AHRS gyroAhrs = new AHRS(SPI.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */
  
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    Constants.DriveConstants.FRONT_LEFT_DRIVE_ID,
    Constants.DriveConstants.FRONT_LEFT_TURN_ID,
    Constants.DriveConstants.kFrontLeftChassisAngularOffset);

   private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
    Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID,
    Constants.DriveConstants.FRONT_RIGHT_TURN_ID,
    Constants.DriveConstants.kFrontRightChassisAngularOffset);

   private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
    Constants.DriveConstants.REAR_LEFT_DRIVE_ID,
    Constants.DriveConstants.REAR_LEFT_TURN_ID,
    Constants.DriveConstants.kBackLeftChassisAngularOffset);

   private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
    Constants.DriveConstants.REAR_RIGHT_DRIVE_ID,
    Constants.DriveConstants.REAR_RIGHT_TURN_ID,
    Constants.DriveConstants.kBackRightChassisAngularOffset);

 /*  private final CANSparkMax m_FrontLeftDrive = new CANSparkMax(
      Constants.DriveConstants.FRONT_LEFT_DRIVE_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
  private final CANSparkMax m_FrontLeftTurn = new CANSparkMax(
      Constants.DriveConstants.FRONT_LEFT_TURN_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
  private final CANSparkMax m_FrontRightDrive = new CANSparkMax(
      Constants.DriveConstants.FRONT_RIGHT_DRIVE_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
  private final CANSparkMax m_FrontRightTurn = new CANSparkMax(
      Constants.DriveConstants.FRONT_RIGHT_TURN_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
  private final CANSparkMax m_RearLeftDrive  = new CANSparkMax(
      Constants.DriveConstants.REAR_LEFT_DRIVE_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
  private final CANSparkMax m_RearLeftTurn  = new CANSparkMax(
      Constants.DriveConstants.REAR_LEFT_TURN_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
  private final CANSparkMax m_RearRightDrive = new CANSparkMax(
      Constants.DriveConstants.REAR_RIGHT_DRIVE_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
  private final CANSparkMax m_RearRightTurn = new CANSparkMax(
      Constants.DriveConstants.REAR_RIGHT_TURN_ID,
      CANSparkLowLevel.MotorType.kBrushless
      );
*/
      private SwerveDrivePoseEstimator m_poseEstimator;

    RelativeEncoder m_FrontLeftEncoder;
    RelativeEncoder m_FrontRightEncoder;
    RelativeEncoder m_RearLeftEncoder;
    RelativeEncoder m_RearRightEncoder;

  private final XboxController m_controller = new XboxController(0);
  private final Timer m_timer = new Timer();

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

     private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private Field2d fieldDisplay = new Field2d();


  public DriverSubsystem() {

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
    Rotation2d.fromDegrees(gyroAhrs.getAngle()),
    new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    }, new Pose2d());
  }
  /*
   * Example command factory method.
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    //update odometry
    m_poseEstimator.update(
      Rotation2d.fromDegrees(gyroAhrs.getAngle()),
       new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
       }
    );

  }
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(gyroAhrs.getAngle()),

        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

   /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)); 

    // Calculate the direction slew rate based on an estimate of the lateral acceleration
    double directionSlewRate;
    if (m_currentTranslationMag != 0.0) {
      directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
    } else {
      directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
    }
    
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

    if (angleDif < 0.45*Math.PI) {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
    }
    else if (angleDif > 0.85*Math.PI) {
      if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
        // keep currentTranslationDir unchanged
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      else {
        m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
    }
    else {
      m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
      m_currentTranslationMag = m_magLimiter.calculate(0.0);
    }
    m_prevTime = currentTime;
    
    xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
    ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
    m_currentRotation = m_rotLimiter.calculate(rot);
  
  } else {
    xSpeedCommanded = xSpeed;
    ySpeedCommanded = ySpeed;
    m_currentRotation = rot;
  }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;
    

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyroAhrs.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }
/**
 * 
 * 
   * X-Lock
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

   /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

    /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroAhrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyroAhrs.getAngle()).getDegrees();
  }
  /*
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyroAhrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  } 
  
}