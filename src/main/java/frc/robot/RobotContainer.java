// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.DriverSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
  



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriverSubsystem m_robotDrive = new DriverSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick m_driverController = new Joystick(Constants.DriveConstants.kDriverControllerPort);

  private final DoubleSupplier driverLeftStickY = () -> MathUtil.applyDeadband(
            m_driverController.getRawAxis(XboxController.Axis.kLeftY.value),
            Constants.DriveConstants.kDriveDeadband);
    private final DoubleSupplier driverLeftStickX = () -> MathUtil.applyDeadband(
            m_driverController.getRawAxis(XboxController.Axis.kLeftX.value),
            Constants.DriveConstants.kDriveDeadband);
    private final DoubleSupplier driverRightStickX = () -> MathUtil.applyDeadband(
            m_driverController.getRawAxis(XboxController.Axis.kRightX.value),
            Constants.DriveConstants.kDriveDeadband);
            private final DoubleSupplier operatorRightStickX = () -> MathUtil.applyDeadband(m_driverController.getRawAxis(
              XboxController.Axis.kRightX.value), Constants.DriveConstants.kDriveDeadband);
              
  /** container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(new TeleopDriveCommand(m_robotDrive,
    driverLeftStickY, driverLeftStickX, driverRightStickX, -1.0));
    
  }
     
    private final JoystickButton driverYButton = new JoystickButton( m_driverController, XboxController.Button.kY.value);
    private final JoystickButton driverAButton = new JoystickButton( m_driverController, XboxController.Button.kA.value);
    private final JoystickButton driverXButton = new JoystickButton( m_driverController, XboxController.Button.kX.value);
    private final JoystickButton driverBButton = new JoystickButton( m_driverController, XboxController.Button.kB.value);

  

 private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }
}

