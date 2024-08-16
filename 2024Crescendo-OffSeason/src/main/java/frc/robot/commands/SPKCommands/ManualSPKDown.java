package frc.robot.commands.SPKCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.ShooterConstants;

/** An example command that uses an example subsystem. */
public class ManualSPKDown extends Command {
  private int m_ButtonID, m_ExecuteID;
  public ManualSPKDown(int _ButtonID, int _ExecuteID){
    m_ButtonID=_ButtonID;
    m_ExecuteID = _ExecuteID;
    addRequirements(RobotContainer.m_Shooter);
    addRequirements(RobotContainer.m_Arm);
    }
  @Override
  public void initialize() {
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDownSPKDegree);
    RobotContainer.m_Shooter.SetRPS(ShooterConstants.ShooterManualSPKRPS);
  }

  @Override
  public void execute() {
    if(RobotContainer.m_driverController.getButton(m_ExecuteID))
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