package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.Library.team1706.MathUtils;
public class ManualDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    boolean m_fieldRelative;
  public ManualDrive(boolean _fieldRelative) {
    addRequirements(RobotContainer.m_Swerve);
    m_fieldRelative=_fieldRelative;
  }
   
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double _controllerX=RobotContainer.m_driverController.getLeftY();
    double _controllerY=RobotContainer.m_driverController.getLeftX();
    Translation2d _controllerTranslation2d=new Translation2d(_controllerX, _controllerY);
    _controllerTranslation2d=MathUtils.signedSquare(_controllerTranslation2d);
    _controllerTranslation2d=MathUtils.applyDeadband(_controllerTranslation2d);
    double _controllerOmega=RobotContainer.m_driverController.getRightX();
    _controllerOmega=MathUtils.signedSquare(_controllerOmega);
    _controllerOmega=MathUtils.applyDeadband(_controllerOmega);
    Translation2d _desireVelocity=_controllerTranslation2d.times(SwerveConstants.maxSpeed);
    double _desireOmega=_controllerOmega*SwerveConstants.stationaryAngularVelocity;
    RobotContainer.m_Swerve.drive(_desireVelocity, _desireOmega, m_fieldRelative);
  }

 
// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Swerve.drive(new Translation2d(), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
