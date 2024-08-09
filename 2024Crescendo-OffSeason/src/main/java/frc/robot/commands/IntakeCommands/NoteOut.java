package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.IntakerConstants;
import frc.robot.commands.Rumble;
/** An example command that uses an example subsystem. */
public class NoteOut extends Command {
  private int m_ButtonID;
  public NoteOut(int _ButtonID){
      m_ButtonID=_ButtonID;
      addRequirements(RobotContainer.m_Intaker);
      addRequirements(RobotContainer.m_Blocker);
      addRequirements(RobotContainer.m_Arm);
    }
  @Override
  public void initialize() {
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
  }

  @Override
  public void execute() {
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
    
    RobotContainer.m_Intaker.SetOutput(IntakerConstants.NoteOutOutput);
    RobotContainer.m_Blocker.SetOutPut(BlockerConstants.NoteOutOutput);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Intaker.SetOutput(0);
    RobotContainer.m_Blocker.SetOutPut(0);
  }

  @Override
  public boolean isFinished() {
    if(m_ButtonID==0)return false;
    if(!RobotContainer.m_driverController.getButton(m_ButtonID)) return true;
    else return false;
  }
}
