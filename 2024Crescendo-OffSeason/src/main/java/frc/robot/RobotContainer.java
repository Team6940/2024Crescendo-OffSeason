// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Library.team1706.MathUtils;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Blocker;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ImprovedPS4Controller;
import frc.robot.subsystems.ImprovedXboxController;
import frc.robot.subsystems.Intaker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Chassis.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.objectTracker;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix6.hardware.Pigeon2;

// import frc.robot.subsystems.SwerveDriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
/* Controllers */
public static ImprovedPS4Controller m_driverController = new ImprovedPS4Controller(0);

/* Drive Controls */

/* Subsystems */
public static ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
public static Arm m_Arm = Arm.GetInstance();
public static Blocker m_Blocker = Blocker.GetInstance();
public static Climber m_Climber = Climber.GetInstance();
public static Intaker m_Intaker = Intaker.GetInstance();
public static Shooter m_Shooter = Shooter.GetInstance();
public static String m_PickLimelight=LimelightConstants.AUTP_LLname;
public static String m_SPKRLimelight=LimelightConstants.SPKR_LLname;
//         gyro.getConfigurator().apply(new Pigeon2Configuration());
private static VisionIO visionIO = new VisionIO();
public static DriveSubsystem m_Swerve = new DriveSubsystem();
// private final objectTracker objectTracker = new objectTracker(visionIO, m_Swerve);
public static CommandXboxController m_CommandController = new CommandXboxController(0);

// public static SwerveDriveTrain m_Swerve = SwerveDriveTrain.GetInstance();

// Replace with CommandPS4Controller or CommandJoystick if needed

/**
 * The container for the robot. Contains subsystems, OI devices, and commands.
 */
public RobotContainer() {
  // Configure the trigger bindings
  configureKeyBindings();
  m_Swerve.setDefaultCommand(new ManualDrive(true));
  // m_Arm.setDefaultCommand(Commands.runOnce(()->m_Arm.SetArmDegree(-72.29)));
}

/**
 * Use this method to define your trigger->command mappings. Triggers can be
 * created via the
 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
 * an arbitrary
 * predicate, or via the named factories in {@link
 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
 * {@link
 * CommandXboxController
 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
 * PS4} controllers or
 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
 * joysticks}.
 */
private void configureKeyBindings() {
  // m_CommandController.b().whileTrue(new TestSPKUP());
  // m_CommandController.b().onTrue(Commands.runOnce(()->m_Shooter.SetPCT(0.2)));
  // m_CommandController.x().onTrue(Commands.runOnce(()->m_Shooter.SetPCT(0.8)));
  // m_CommandController.y().onTrue(Commands.runOnce(()->m_Shooter.SetPCT(0)));
}

/**
 * Use this to pass the autonomous command to the main {@link Robot} class.
 *
 * @return the command to run in autonomous
 */
public Command getAutonomousCommand() {
  // An example command will be run in autonomous
  return Autos.exampleAuto(m_exampleSubsystem);
}
}
