package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.IntakerConstants;

/** An example command that uses an example subsystem. */
public class testCommand extends Command {
    private int m_ButtonID;
  public testCommand(int _ButtonID){
        m_ButtonID=_ButtonID;
      addRequirements(RobotContainer.m_Intaker);
      addRequirements(RobotContainer.m_Arm);
      addRequirements(RobotContainer.m_Blocker);
    }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
      RobotContainer.m_Intaker.SetOutput(10);

  }

  @Override
  public void end(boolean interrupted) {
      RobotContainer.m_Intaker.SetOutput(0);

  }

  @Override
  public boolean isFinished() {
    if(!RobotContainer.m_driverController.getButton(m_ButtonID)) return true;
    else return false;
  }
}