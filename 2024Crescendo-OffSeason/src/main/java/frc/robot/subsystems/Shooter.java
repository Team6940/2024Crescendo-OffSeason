package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase{
    public static Shooter m_Instance;

    private static TalonFX m_ShooterLeft = new TalonFX(ShooterConstants.ShooterLeft_ID,"*");
    private static TalonFX m_ShooterRight = new TalonFX(ShooterConstants.ShooterRight_ID,"*");

    private TalonFXConfiguration m_Configs = new TalonFXConfiguration();
    final VelocityVoltage m_DutyCycle = new VelocityVoltage(0,0,false,0.,0,false,false,false);

    private double m_TargetRPS;

    public static Shooter GetInstance()
    {
        return m_Instance==null?m_Instance=new Shooter():m_Instance;
    }

    Shooter(){
        ShooterConfig();
    for(var a: AutoShootConstants.ArmPoints)
        AutoShootConstants.ArmTable.put(a.getX(),a.getY());
    for(var a: AutoShootConstants.RPSPoints)
        AutoShootConstants.RPSTable.put(a.getX(),a.getY());
    }

    private void ShooterConfig(){
        m_Configs.MotorOutput.NeutralMode=NeutralModeValue.Coast;
        m_Configs.MotorOutput.PeakReverseDutyCycle=0;
        m_Configs.Slot0.kP=ShooterConstants.kP;
        m_Configs.Slot0.kI=ShooterConstants.kI;
        m_Configs.Slot0.kD=ShooterConstants.kD;
        m_Configs.Slot0.kV=ShooterConstants.kV;
        m_Configs.Slot0.kS=ShooterConstants.kS;

        m_ShooterLeft.getConfigurator().apply(m_Configs);
        m_ShooterRight.getConfigurator().apply(m_Configs);
        m_ShooterLeft.setInverted(true);
        m_ShooterRight.setInverted(false);
    }

    public void SetRPS(double _RPS){
        m_TargetRPS=_RPS;
        m_ShooterLeft.setControl(m_DutyCycle.withVelocity(_RPS));
        m_ShooterRight.setControl(m_DutyCycle.withVelocity(_RPS+5));
    }

    public void SetPCT(double _PCT){
        m_ShooterLeft.set(_PCT);
        m_ShooterRight.set(_PCT);
    }

    public double GetRPS(){
        return m_ShooterLeft.getVelocity().getValue();
    }

    public double GetTargetRPS(){
        return m_TargetRPS;
    }

    public boolean IsRPSSimilar(){
        return Math.abs(m_ShooterLeft.getVelocity().getValue()-m_ShooterRight.getVelocity().getValue())<ShooterConstants.ShooterDifferenceTolerence;
    }

    public boolean IsAtTargetRPS(){
        return Math.abs(GetRPS()-GetTargetRPS())<ShooterConstants.ShooterSpeedTolerence;
    }

    public boolean IsShooterReady(){
        return IsAtTargetRPS();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("LeftRPS", m_ShooterLeft.getVelocity().getValue());
        SmartDashboard.putNumber("RightRPS", m_ShooterRight.getVelocity().getValue());
    }
}
