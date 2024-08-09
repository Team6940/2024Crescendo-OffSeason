package frc.robot.commands.AMPCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team1706.MathUtils;
import frc.robot.commands.Rumble;
import frc.robot.Constants.AutoAMPConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.Constants.ArmConstants;

public class AutoAMP extends Command{
    int m_ButtonID, m_ExecuteButtonID;
    PIDController m_RotationController=new PIDController(AutoAMPConstants.AutoAMPRotationkP, AutoAMPConstants.AutoAMPRotationkI, AutoAMPConstants.AutoAMPRotationkD);
    PIDController m_TranslationXController = new PIDController(AutoAMPConstants.AutoAMPTranslationkP, AutoAMPConstants.AutoAMPTranslationkI, AutoAMPConstants.AutoAMPTranslationkD);
    PIDController m_TranslationYController = m_TranslationXController;
    Pose2d m_TargetPose = new Pose2d();
    public AMPState m_State;
    public double _Omega = 0., _controllerX = 0., _controllerY = 0.;
    AutoAMP(int _ButtonID, int _ExecuteButtonID)
    {
        m_ButtonID=_ButtonID;
        m_ExecuteButtonID = _ExecuteButtonID;
        addRequirements(RobotContainer.m_Swerve);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Blocker);
    }
    public enum AMPState{
        Aligning, Shooting, END;
    }

    @Override
    public void initialize()
    {
        LimelightHelpers.setPriorityTagID(RobotContainer.m_SPKRLimelight, DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?AutoAMPConstants.AMPTagID.Blue:AutoAMPConstants.AMPTagID.Red);
        m_State = AMPState.Aligning;
        m_RotationController.setTolerance(AutoAMPConstants.AutoAMPDegreeTolerance, AutoAMPConstants.AutoAMPDegreeOmegaTolerance);
        m_TranslationXController.setTolerance(AutoAMPConstants.AutoAMPTranslationTolerance);
        m_TranslationYController.setTolerance(AutoAMPConstants.AutoAMPTranslationTolerance);
        m_TargetPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? AutoAMPConstants.AMPPose.Blue : AutoAMPConstants.AMPPose.Red; 
        m_RotationController.setSetpoint(m_TargetPose.getRotation().getDegrees());
        m_TranslationXController.setSetpoint(m_TargetPose.getX());
        m_TranslationYController.setSetpoint(m_TargetPose.getY());
    }

    @Override
    public void execute(){
        switch (m_State){
            case Aligning:
                align();
                break;
            case Shooting:
                shoot();
                break;
            default:
                break;
        }
    }

    public void align(){
        _Omega = m_RotationController.calculate(RobotContainer.m_Swerve.getPose().getRotation().getDegrees());
        _controllerX = m_TranslationXController.calculate(RobotContainer.m_Swerve.getPose().getX());
        _controllerY = m_TranslationYController.calculate(RobotContainer.m_Swerve.getPose().getY());
        RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmAMPDegree);
        RobotContainer.m_Swerve.drive(new Translation2d(_controllerX, _controllerY), _Omega, true);
        if(m_RotationController.atSetpoint() && m_TranslationXController.atSetpoint() && m_TranslationYController.atSetpoint() && RobotContainer.m_Arm.IsAtTargetDegree()){
            new Rumble(RumbleType.kRightRumble, 0.1).schedule();
            if(RobotContainer.m_driverController.getRawButton(m_ExecuteButtonID)){
                m_State = AMPState.Shooting;
            }
        }
    }

    public void shoot(){
        RobotContainer.m_Blocker.SetOutPut(BlockerConstants.AMPOutput);
    }

    @Override 
    public void end(boolean interrupted){

    }
    
    @Override
    public boolean isFinished(){
        if(RobotContainer.m_driverController.getRawButton(m_ButtonID)) return false;
        return true;
    }
}
