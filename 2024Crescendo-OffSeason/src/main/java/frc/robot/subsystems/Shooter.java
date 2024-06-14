package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    public static Shooter m_Instance;
    //TODO 定义电机

    public static Shooter GetInstance()
    {
        return m_Instance==null?m_Instance=new Shooter():m_Instance;
    }

    Shooter(){
        ShooterConfig();
    }

    private void ShooterConfig(){

    }

    public void SetRPS(double _RPS){

    }

    public void SetPCT(double _PCT){

    }

    public double GetRPS(){
        return 0.;
    }

    public double GetTargetRPS(){
        return 0.;
    }

    public boolean IsRPSSimilar(){
        return true;
    }

    public boolean IsAtTargetRPS(){
        return true;
    }

    public boolean IsShooterReady(){
        return IsAtTargetRPS()&&IsRPSSimilar();
    }

    @Override
    public void periodic(){
        //SmartDashboard
    }
}
