package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.ShooterConstants;

/** An example command that uses an example subsystem. */
public class ManualSPKUp extends Command {
  private int m_ButtonID;
  public ManualSPKUp(int _ButtonID){
    m_ButtonID=_ButtonID;
    addRequirements(RobotContainer.m_Shooter);
    addRequirements(RobotContainer.m_Arm);
    }
  @Override
  public void initialize() {
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmUpSPKDegree);
    RobotContainer.m_Shooter.SetRPS(ShooterConstants.ShooterManualSPKRPS);
  }

  @Override
  public void execute() {
    if(RobotContainer.m_Shooter.IsAtTargetRPS()&&RobotContainer.m_Arm.IsAtTargetDegree())
      RobotContainer.m_Blocker.SetOutPut(BlockerConstants.GiveNoteOutput);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Shooter.SetPCT(0);
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
  }

  @Override
  public boolean isFinished() {
    if(!RobotContainer.m_driverController.getButton(m_ButtonID)) return true;
    else return false;
  }
}