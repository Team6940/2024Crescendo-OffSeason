package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.RobotContainer;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team2910.control.PidController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.LimelightConstants;;

/** An example command that uses an example subsystem. */
public class AutoSPKUP extends Command {
  private int m_ButtonID;
  enum AutoShootState {
    Aim,Accelerate,Shoot;
  }
  AutoShootState m_State;
  double _TargetDegree;
  double _TargetRPS;
  PIDController m_PidController;
  public AutoSPKUP(int _ButtonID){
    m_ButtonID=_ButtonID;
    // m_PidController.setTolerance(AutoShootConstants.DegreeTolerance);
    addRequirements(RobotContainer.m_Shooter);
    addRequirements(RobotContainer.m_Arm);
    // for(var a: AutoShootConstants.ArmPoints)
    // AutoShootConstants.ArmTable.put(a.getX(),a.getY());
    // for(var a: AutoShootConstants.RPSPoints)
    // AutoShootConstants.RPSTable.put(a.getX(),a.getY());
    addRequirements(RobotContainer.m_Swerve);
    }
  
  void Aim(){
        double _Omega=0.; 
        RobotContainer.m_Shooter.SetRPS(AutoShootConstants.RPSInAdvance);
        if(LimelightHelpers.getTV(RobotContainer.m_SPKRLimelight))
        {
          // if(LimelightHelpers.getTX("limelight")>AutoShootCommandConstants.NewShootAngleTolerance)
          // _Omega=-AutoShootCommandConstants.NewShootFixingOmega;
          // else if(LimelightHelpers.getTX("limelight")<-AutoShootCommandConstants.NewShootAngleTolerance)
          // _Omega=AutoShootCommandConstants.NewShootFixingOmega;
          if(!m_PidController.atSetpoint())
          {
            _Omega=m_PidController.calculate(-LimelightHelpers.getTX(RobotContainer.m_SPKRLimelight));
            SmartDashboard.putNumber("Omega", _Omega);
          }
          else
          {
          RobotContainer.m_Swerve.drive(new Translation2d(), _Omega, false);
        
          _TargetDegree=AutoShootConstants.ArmTable.get(LimelightHelpers.getTY(RobotContainer.m_SPKRLimelight));
          _TargetRPS=AutoShootConstants.RPSTable.get(LimelightHelpers.getTY(RobotContainer.m_SPKRLimelight));

          m_State=AutoShootState.Accelerate;
          }
        }
        RobotContainer.m_Swerve.drive(new Translation2d(), _Omega, false);
  }

  void Accelerate(){
    RobotContainer.m_Arm.SetArmDegree(_TargetDegree);
    RobotContainer.m_Shooter.SetRPS(_TargetRPS);
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
    m_State=AutoShootState.Aim;
    m_PidController=new PIDController(AutoShootConstants.kP, AutoShootConstants.kI, AutoShootConstants.kD);
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
    if(RobotContainer.m_driverController.getButton(m_ButtonID)) return false;
    else return true;
  }
}