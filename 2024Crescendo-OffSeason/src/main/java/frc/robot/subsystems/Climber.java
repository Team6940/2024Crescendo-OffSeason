package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    public static Climber m_Instance;

    public static Climber GetInstance()
    {
        return m_Instance==null?m_Instance=new Climber():m_Instance;
    }   
    
    Climber(){
        ClimberConfig();
    }

    private void ClimberConfig(){

    }

    public void SetOutPut(){

    }

    public double GetOutput(){
        return 0.;
    }

    public boolean IsAtTargetLength(){
        return true;
    }

    @Override
    public void periodic(){
        //SmartDashboard
    }
}
