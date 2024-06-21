package frc.robot.subsystems;

public class Climber {
    public static Climber m_Instance;
    void SetOutPut(){
        
    }
    boolean IfReached() {

    }
    Climber(){
        
    }
    public Intake GetInstance()
    {
        return m_Instance==null?m_Instance=new Climber():m_Instance;
    }    
}
