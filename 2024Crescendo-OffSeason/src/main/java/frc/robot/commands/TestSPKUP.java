package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team2910.control.PidController;
import frc.robot.Library.team3476.net.editing.LiveEditableValue;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.LimelightConstants;;

/** An example command that uses an example subsystem. */
public class TestSPKUP extends Command {
  // private int m_ButtonID;
  AutoShootState m_State;
  double m_TargetDegree;
  double m_TargetRPS;
  PIDController m_PidController;
  // private LiveEditableValue<Double> m_TestRPSValue=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TestRPS"));
  // private LiveEditableValue<Double> m_TestArmValue=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TestArm"));
  public TestSPKUP(double _TargetArmAngle,double _TargetRPS){
    // m_ButtonID=_ButtonID;
    m_TargetDegree=_TargetArmAngle;
    m_TargetRPS=_TargetRPS;
    addRequirements(RobotContainer.m_Shooter);
    addRequirements(RobotContainer.m_Arm);
  }

  public TestSPKUP(){
    // m_ButtonID=_ButtonID;
    // m_TargetDegree=m_TestArmValue.get();
    // m_TargetRPS=m_TestRPSValue.get();
    addRequirements(RobotContainer.m_Shooter);
    addRequirements(RobotContainer.m_Arm);
  }
  enum AutoShootState {
    Aim,Accelerate,Shoot;
  }
  
  void Aim(){
        double _Omega=0.; 
        RobotContainer.m_Shooter.SetRPS(AutoShootConstants.RPSInAdvance);
        if(LimelightHelpers.getTV("limelight"))
        {
          // if(LimelightHelpers.getTX("limelight")>AutoShootCommandConstants.NewShootAngleTolerance)
          // _Omega=-AutoShootCommandConstants.NewShootFixingOmega;
          // else if(LimelightHelpers.getTX("limelight")<-AutoShootCommandConstants.NewShootAngleTolerance)
          // _Omega=AutoShootCommandConstants.NewShootFixingOmega;
          if(!m_PidController.atSetpoint())
          {
            _Omega=m_PidController.calculate(LimelightHelpers.getTX(RobotContainer.m_SPKRLimelight));
          }
          else
          {
          RobotContainer.m_Swerve.drive(new Translation2d(), _Omega, false);
        
          
          m_State=AutoShootState.Accelerate;
          }
        }
        RobotContainer.m_Swerve.drive(new Translation2d(), _Omega, false);
  }

  void Accelerate(){
    RobotContainer.m_Arm.SetArmDegree(m_TargetDegree);
    RobotContainer.m_Shooter.SetRPS(m_TargetRPS);
    if(RobotContainer.m_Arm.IsAtTargetDegree()&&RobotContainer.m_Shooter.IsAtTargetRPS())
    {
        m_State=AutoShootState.Shoot;
    }
        
  }

  void Shoot(){
    RobotContainer.m_Blocker.SetOutPut(BlockerConstants.GiveNoteOutput);
  }


  @Override
  public void initialize() {
    m_State=AutoShootState.Accelerate;
    m_PidController=new PIDController(AutoShootConstants.kP, AutoShootConstants.kI, 0.);
    m_PidController.setSetpoint(0.);
    //m_PidController.setIZone(5);
    m_PidController.setTolerance(AutoShootConstants.DegreeTolerance);
    m_PidController.reset();
  }

  @Override
  public void execute() {
    if(m_State==AutoShootState.Aim){
        Aim();
    }
    if(m_State==AutoShootState.Accelerate){
        Accelerate();
    }
    if(m_State==AutoShootState.Shoot){
        Shoot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_Swerve.drive(new Translation2d(), 0., false);
    RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
    RobotContainer.m_Shooter.SetPCT(0.);;
  }

  @Override
  public boolean isFinished() {
    if(RobotContainer.m_driverController.getBButton()) return false;
    else return true;
    // return false;
  }
}