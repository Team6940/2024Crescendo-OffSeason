package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.IntakerConstants;

/** An example command that uses an example subsystem. */
public class PassNote extends Command {
  public PassNote(){
      addRequirements(RobotContainer.m_Intaker);
      addRequirements(RobotContainer.m_Arm);
      addRequirements(RobotContainer.m_Blocker);
    }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
      RobotContainer.m_Intaker.SetOutput(IntakerConstants.PassNoteOutput);
      RobotContainer.m_Blocker.SetOutPut(BlockerConstants.PassNoteOutput);
  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.m_Intaker.SetOutput(0);
      RobotContainer.m_Blocker.SetOutPut(0);
  }

  @Override
  public boolean isFinished() {
    if(!RobotContainer.m_Intaker.HasNote()&&RobotContainer.m_Blocker.HasNote()) return true;
    else return false;
  }
}