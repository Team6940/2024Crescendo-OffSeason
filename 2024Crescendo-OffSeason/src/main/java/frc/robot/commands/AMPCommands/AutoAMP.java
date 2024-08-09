package frc.robot.commands.AMPCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Library.NumberLimiter;
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
    PIDController m_TranslationYController = new PIDController(AutoAMPConstants.AutoAMPTranslationkP, AutoAMPConstants.AutoAMPTranslationkI, AutoAMPConstants.AutoAMPTranslationkD);
    Pose2d m_TargetPose = new Pose2d();
    public AMPState m_State;
    public double _Omega = 0., _controllerX = 0., _controllerY = 0.;
    public AutoAMP(int _ButtonID, int _ExecuteButtonID)
    {
        m_ButtonID=_ButtonID;
        m_ExecuteButtonID = _ExecuteButtonID;
        addRequirements(RobotContainer.m_Swerve);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Blocker);
    }
    public enum AMPState{
        Aligning, Pushing,Shooting, END;
    }

    @Override
    public void initialize()
    {
        LimelightHelpers.setPriorityTagID(RobotContainer.m_SPKRLimelight, DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?AutoAMPConstants.AMPTagID.Blue:AutoAMPConstants.AMPTagID.Red);
        m_State = AMPState.Aligning;
        m_RotationController.setTolerance(AutoAMPConstants.AutoAMPDegreeTolerance, AutoAMPConstants.AutoAMPDegreeOmegaTolerance);
        m_TranslationXController.setTolerance(AutoAMPConstants.AutoAMPTranslationTolerance);
        m_TranslationYController.setTolerance(AutoAMPConstants.AutoAMPTranslationTolerance);
        m_TargetPose = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? AutoAMPConstants.AutoAMPPose.Blue : AutoAMPConstants.AutoAMPPose.Red; 
         m_RotationController.enableContinuousInput(-180, 180);
        m_RotationController.setSetpoint(m_TargetPose.getRotation().getDegrees());
      
        m_TranslationXController.setSetpoint(m_TargetPose.getX());
        m_TranslationYController.setSetpoint(m_TargetPose.getY());
        m_TranslationXController.reset();
        m_TranslationYController.reset();
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("TargetX",m_TranslationXController.getSetpoint());
        SmartDashboard.putNumber("TargetY",m_TranslationYController.getSetpoint());
        SmartDashboard.putNumber("TargetOmega",m_RotationController.getSetpoint());
        switch (m_State){
            case Aligning:
                align();
                break;
            case Pushing:
                push();
                break;
            case Shooting:
                shoot();
                break;
            case END:
                END();
                break;
            default:
                break;
        }
    }

    public void align(){
        moveto();
        
        if(m_RotationController.atSetpoint() && RobotContainer.m_Arm.IsAtTargetDegree()){
             m_RotationController.setSetpoint(m_TargetPose.getRotation().getDegrees());
       m_TranslationXController.setSetpoint(m_TargetPose.getX());
        m_TranslationYController.setSetpoint(m_TargetPose.getY()+AutoAMPConstants.AMPDistancetoGo);
        m_State=AMPState.Pushing;
        }
    }
    private void moveto()
    {
    _Omega = m_RotationController.calculate(RobotContainer.m_Swerve.getPose().getRotation().getDegrees());
        _controllerX=0.;
        _controllerY=0.;
        _controllerX = m_TranslationXController.calculate(RobotContainer.m_Swerve.getPose().getX());
        _controllerY = m_TranslationYController.calculate(RobotContainer.m_Swerve.getPose().getY());
        _controllerX=NumberLimiter.Limit(-2, 2, _controllerX);
        _controllerY=NumberLimiter.Limit(-2, 2, _controllerY);
        _Omega=NumberLimiter.Limit(-3, 3, _Omega);
        RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmAMPDegree);
        RobotContainer.m_Swerve.drive(new Translation2d(_controllerX, _controllerY), _Omega, true);
        
    }
    public void push(){
        moveto();
        if(RobotContainer.m_driverController.getRawButton(m_ExecuteButtonID)){
                m_State = AMPState.Shooting;
            }
    }
    public void shoot(){
        RobotContainer.m_Blocker.SetOutPut(BlockerConstants.AMPOutput);
        if(!RobotContainer.m_driverController.getRawButton(m_ExecuteButtonID))
        {
            m_State=AMPState.END;
            m_RotationController.setSetpoint(m_TargetPose.getRotation().getDegrees());
       m_TranslationXController.setSetpoint(m_TargetPose.getX());
        m_TranslationYController.setSetpoint(m_TargetPose.getY());
        
        }
    }
    public void END()
    {
        moveto();
        
    }

    @Override 
    public void end(boolean interrupted){

        RobotContainer.m_Blocker.SetOutPut(0);
        RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
    }
    
    @Override
    public boolean isFinished(){
        if(RobotContainer.m_driverController.getRawButton(m_ButtonID)) return false;
        return true;
    }
}
