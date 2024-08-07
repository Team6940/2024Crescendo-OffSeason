package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ManualShootConstants;

/** An example command that uses an example subsystem. */
public class ManualSPKUp extends Command {
  private int m_ButtonID;
  private double m_ArmAngle = 0.;
  private double m_RPS = 0.;
  private ShootState m_State;
  public ManualSPKUp(int _ButtonID, int _ArmAngle, int _RPS){
    m_ButtonID=_ButtonID;
    m_ArmAngle=_ArmAngle;
    m_RPS=_RPS;
    addRequirements(RobotContainer.m_Shooter);
    addRequirements(RobotContainer.m_Arm);
    addRequirements(RobotContainer.m_Blocker);
  }
  public enum ShootState{
    Preparing, Shooting;
  }
  @Override
  public void initialize() {
    m_State=ShootState.Preparing;
  }

  @Override
  public void execute() {
    switch(m_State){
      case Preparing:
        prepare();
        break;
      case Shooting:
        shoot();
        break;
      default:
        break;
    }
  }

  private void prepare(){
    RobotContainer.m_Arm.SetArmDegree(m_ArmAngle);
    RobotContainer.m_Shooter.SetRPS(m_RPS);
    if(RobotContainer.m_Arm.IsAtTargetDegree() && RobotContainer.m_Shooter.IsAtTargetRPS() && RobotContainer.m_Swerve.getChassisSpeed() <= ManualShootConstants.ChassisSpeedTolerance){
      m_State = ShootState.Shooting;
    }
  }
  private void shoot(){
    RobotContainer.m_Blocker.SetOutPut(BlockerConstants.GiveNoteOutput);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Shooter.SetPCT(0);
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
  }

  @Override
  public boolean isFinished() {
    if(RobotContainer.m_driverController.getRawButton(m_ButtonID)) return false;
    return true;
  }
}