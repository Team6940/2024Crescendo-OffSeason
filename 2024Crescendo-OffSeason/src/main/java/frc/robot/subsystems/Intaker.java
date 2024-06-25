package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intaker extends SubsystemBase{
    public static Intaker m_Instance;
    
    public static Intaker GetInstance()
    {
        return m_Instance==null?m_Instance=new Intaker():m_Instance;
    }

    Intaker(){
        IntakerConfig();
    }

    private void IntakerConfig(){

    }
    
    public void SetOutput(double _Out){

    }

    public double GetOutput(){
        return 0.;
    }

    public boolean HasNote(){
        return true;//注意传感器默认检测到有球返回0
    }

    public void NoteIn(){

    }

    public void NoteOut(){

    }

    @Override
    public void periodic(){
        //SmartDashboard
    }
}
