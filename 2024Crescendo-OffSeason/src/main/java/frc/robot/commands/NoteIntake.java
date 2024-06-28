package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class NoteIntake extends Command {
  private int m_ButtonID;
  public NoteIntake(int _ButtonID){
      m_ButtonID=_ButtonID;
      addRequirements(RobotContainer.m_Intaker);
    }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
      RobotContainer.m_Intaker.NoteIn();
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
