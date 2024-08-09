package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoAMPConstants;

public class AutoAMP extends Command{
    int m_ButtonID;
    PIDController m_RotationController=new PIDController(AutoAMPConstants.AutoAMPRotationkP, AutoAMPConstants.AutoAMPRotationkI, AutoAMPConstants.AutoAMPRotationkD);
    AutoAMP(int _ButtonID)
    {
        m_ButtonID=_ButtonID;
        addRequirements(RobotContainer.m_Swerve);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Blocker);
    }
    @Override
    public void initialize()
    {

    }
    
}
