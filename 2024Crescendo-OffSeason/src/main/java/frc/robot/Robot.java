// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoAMPConstants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Library.team3476.net.editing.LiveEditableValue;
import frc.robot.commands.Autos;
import frc.robot.commands.AMPCommands.AMP;
import frc.robot.commands.AMPCommands.AutoAMP;
import frc.robot.commands.AutoCommand.C131;
import frc.robot.commands.IntakeCommands.NoteIntake;
import frc.robot.commands.IntakeCommands.NoteOut;
import frc.robot.commands.IntakeCommands.SemiAutoPick;
import frc.robot.commands.SPKCommands.NewAutoSPKUP;
import frc.robot.commands.SPKCommands.ManualSPKDown;
import frc.robot.commands.SPKCommands.ManualSPKUp;
import frc.robot.commands.SPKCommands.TestSPKUP;
// import frc.robot.commands.PassNote;
import frc.robot.subsystems.ImprovedXboxController;
import frc.robot.subsystems.Chassis.CTREConfigs;
import frc.robot.subsystems.ImprovedXboxController.Button;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private LiveEditableValue<Double> m_TestRPSValue=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TestRPS"));

  private LiveEditableValue<Double> m_TestArmValue=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TestArm"));


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    RobotContainer.m_Swerve.resetModulesToAbsolute();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  
    SmartDashboard.putNumber("XboxLeftX", RobotContainer.m_driverController.getLeftX());
    SmartDashboard.putNumber("XboxLeftY", RobotContainer.m_driverController.getLeftY());
    SmartDashboard.putNumber("XboxRightX", RobotContainer.m_driverController.getRightX());
    SmartDashboard.putNumber("XboxRightY", RobotContainer.m_driverController.getRightY());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    new C131().schedule();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.m_Swerve.setPose(new Pose2d(15.135,5.579, new Rotation2d(Math.PI)));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    if(RobotContainer.m_driverController.getRightBumperPressed()){//TODO AMP按钮
      new NewAutoSPKUP(Button.kRightBumper.value).schedule();;
    }
    if(RobotContainer.m_driverController.getLeftBumperPressed())
    {
      new NoteIntake(Button.kLeftBumper.value).schedule();;
    }
    if(RobotContainer.m_driverController.getAButtonPressed())
    {
      new AutoAMP(Button.kA.value, Button.kRightTrigger.value).schedule();
    }
    if(RobotContainer.m_driverController.getBButtonPressed()){
      new NoteOut(Button.kB.value).schedule();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // if(RobotContainer.m_driverController.getAButton())
    // {
    //   RobotContainer.m_Swerve.setChassisSpeeds(new ChassisSpeeds(1, 0, 0));
    // }
    // else
    // {
    //    RobotContainer.m_Swerve.drive(new Translation2d(0, 0), 0, false);
    
    // }
    // if(RobotContainer.m_driverController.getPOVUp()){
    //   RobotContainer.m_Arm.SetPCT(0.05);
    // }
    // else if(RobotContainer.m_driverController.getPOVDown()){
    //   RobotContainer.m_Arm.SetPCT(-0.05);
    // }
    // else{
    //   RobotContainer.m_Arm.SetPCT(0);
    // }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
