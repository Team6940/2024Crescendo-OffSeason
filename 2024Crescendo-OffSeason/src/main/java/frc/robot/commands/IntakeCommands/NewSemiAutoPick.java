package frc.robot.commands.IntakeCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Chassis.controllers.AutoMoveAlignController;
import frc.robot.Library.NumberLimiter;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team1706.MathUtils;
public class NewSemiAutoPick extends Command {
  PIDController m_LRController;
  int m_ButtonID;
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public NewSemiAutoPick(int _ButtonID) {
    addRequirements(RobotContainer.m_Swerve);
    m_ButtonID=_ButtonID;
  }
   
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LRController=new PIDController( AutoAlignConstants.kAutoAlignMovementP, 
                                      AutoAlignConstants.kAutoAlignMovementI, 
                                      AutoAlignConstants.kAutoAlignMovementD);
    m_LRController.reset();
    m_LRController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double _controllerY=m_LRController.calculate(-LimelightHelpers.getTX(RobotContainer.m_PickLimelight));
    _controllerY=NumberLimiter.Limit(-AutoAlignConstants.kAutoAlignPeak,-AutoAlignConstants.kAutoAlignPeak, _controllerY);
    double _controllerX=-RobotContainer.m_driverController.getLeftTriggerAxis();
    Translation2d _controllerTranslation2d=new Translation2d(_controllerX, _controllerY);
    _controllerTranslation2d=MathUtils.signedSquare(_controllerTranslation2d);
    _controllerTranslation2d=MathUtils.applyDeadband(_controllerTranslation2d);
    double _controllerOmega=-RobotContainer.m_driverController.getRightX();
    _controllerOmega=MathUtils.signedSquare(_controllerOmega);
    _controllerOmega=MathUtils.applyDeadband(_controllerOmega);
    Translation2d _desireVelocity=_controllerTranslation2d.times(SwerveConstants.maxSpeed);
    double _desireOmega=_controllerOmega*SwerveConstants.stationaryAngularVelocity;
    RobotContainer.m_Swerve.drive(_desireVelocity, _desireOmega, false);
  }

 
// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Swerve.drive(new Translation2d(), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_driverController.getRawButtonReleased(m_ButtonID);
  }
}
