package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    public static Shooter m_Instance;

    private static TalonFX m_ShooterLeft = new TalonFX(ShooterConstants.ShooterLeft_ID);
    private static TalonFX m_ShooterRight = new TalonFX(ShooterConstants.ShooterRight_ID);

    private TalonFXConfiguration m_Configs = new TalonFXConfiguration();
    final VelocityDutyCycle m_DutyCycle = new VelocityDutyCycle(0,0,false,ShooterConstants.kF,0,true,false,false);

    private double m_TargetRPS;

    public static Shooter GetInstance()
    {
        return m_Instance==null?m_Instance=new Shooter():m_Instance;
    }

    Shooter(){
        ShooterConfig();
    }

    private void ShooterConfig(){
        m_Configs.MotorOutput.NeutralMode=NeutralModeValue.Coast;
        m_Configs.MotorOutput.PeakReverseDutyCycle=0;
        m_Configs.Slot0.kP=ShooterConstants.kP;
        m_Configs.Slot0.kI=ShooterConstants.kI;
        m_Configs.Slot0.kD=ShooterConstants.kD;
        m_Configs.Slot0.kV=ShooterConstants.kV;

        m_ShooterLeft.setInverted(true);
        m_ShooterRight.setInverted(false);
        m_ShooterLeft.getConfigurator().apply(m_Configs);
        m_ShooterRight.getConfigurator().apply(m_Configs);
    }

    public void SetRPS(double _RPS){
        m_TargetRPS=_RPS;
        m_ShooterLeft.setControl(m_DutyCycle.withVelocity(_RPS));
        m_ShooterRight.setControl(m_DutyCycle.withVelocity(_RPS));
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
        return IsAtTargetRPS()&&IsRPSSimilar();
    }

    @Override
    public void periodic(){
        //SmartDashboard
    }
}
