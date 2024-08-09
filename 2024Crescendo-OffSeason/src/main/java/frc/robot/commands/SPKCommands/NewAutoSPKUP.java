package frc.robot.commands.SPKCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Library.team1706.MathUtils;
import frc.robot.commands.Rumble;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.BlockerConstants;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class NewAutoSPKUP extends Command{
    int m_ButtonID;
    PIDController m_RotationPidController = new PIDController(AutoShootConstants.kP, AutoShootConstants.kI, AutoShootConstants.kD);
    Translation2d AimTranslation = new Translation2d(0, 0);
    double _Omega = 0., _controllerX = 0., _controllerY = 0.;
    double _ArmAngle = 0., _RPS = 0.;
    
    public NewAutoSPKUP(int _ButtonID){
        m_ButtonID = _ButtonID;
    }

    @Override
    public void initialize(){
        m_RotationPidController.setTolerance(AutoShootConstants.DegreeTolerance);
        addRequirements(RobotContainer.m_Blocker);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Swerve);
    }

    @Override
    public void execute(){
        AimTranslation = RobotContainer.m_Swerve.getToSPKTranslation2d();
        _Omega = m_RotationPidController.calculate(RobotContainer.m_Swerve.getPose().getRotation().getDegrees(), AimTranslation.getAngle().getDegrees());
        _controllerX=-RobotContainer.m_driverController.getLeftY();
        _controllerY=-RobotContainer.m_driverController.getLeftX();
        Translation2d _controllerTranslation2d=new Translation2d(_controllerX, _controllerY);
        _controllerTranslation2d=MathUtils.signedSquare(_controllerTranslation2d);
        _controllerTranslation2d=MathUtils.applyDeadband(_controllerTranslation2d);
        Translation2d _desireVelocity=_controllerTranslation2d.times(SwerveConstants.maxSpeed/2.);
        RobotContainer.m_Swerve.drive(_desireVelocity, _Omega, true);

        _ArmAngle = AutoShootConstants.DisToArmTable.get(AimTranslation.getNorm());
        _RPS = AutoShootConstants.DisToRPSTable.get(AimTranslation.getNorm());
        RobotContainer.m_Arm.SetArmDegree(_ArmAngle);
        RobotContainer.m_Shooter.SetRPS(_RPS);

        if(AimTranslation.getNorm()<=AutoShootConstants.MaxRange && 
            RobotContainer.m_Swerve.getChassisSpeed() <= AutoShootConstants.CoastVelocity  &&
            RobotContainer.m_Arm.IsAtTargetDegree() &&
            RobotContainer.m_Shooter.IsAtTargetRPS()
        ) {
            RobotContainer.m_Blocker.SetOutPut(BlockerConstants.GiveNoteOutput);
        }
    }

    @Override
    public void end(boolean interrupted){
        RobotContainer.m_Arm.SetArmDegree(ArmConstants.ArmDefaultDegree);
        RobotContainer.m_Shooter.SetPCT(0.);
        RobotContainer.m_Blocker.SetOutPut(0.);
        new Rumble(RumbleType.kRightRumble, 0.1).schedule();
    }

    @Override
    public boolean isFinished(){
        if(!RobotContainer.m_driverController.getRawButton(m_ButtonID)) return true;
        if(!RobotContainer.m_Blocker.HasNote()) return true;
        return false;
    }
}