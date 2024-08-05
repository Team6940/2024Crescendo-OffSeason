package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlockerConstants;

/** An example command that uses an example subsystem. */
public class AMP extends Command {
  private int m_ButtonID;
  private int m_ExecuteButtonID;
  public AMP(int _ButtonID,int _ExecuteButtonID){
      m_ButtonID=_ButtonID;
      addRequirements(RobotContainer.m_Blocker);
      addRequirements(RobotContainer.m_Arm);
    }
  @Override
  public void initialize() {
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmAMPDegree);
  }

  @Override
  public void execute() {
    if(RobotContainer.m_Arm.IsAtTargetDegree()&&RobotContainer.m_driverController.getButton(m_ExecuteButtonID))
      RobotContainer.m_Blocker.SetOutPut(BlockerConstants.AMPOutput);
    else{
      RobotContainer.m_Blocker.SetOutPut(0.);
    }
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.m_Blocker.SetOutPut(0);
      RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
  }

  @Override
  public boolean isFinished() {
    if(!RobotContainer.m_driverController.getButton(m_ButtonID)) return true;
    else return false;
  }
}
