package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakerConstants;

public class Intaker extends SubsystemBase{
    public static Intaker m_Instance;
    
    private static TalonFX m_Intaker = new TalonFX(IntakerConstants.Intaker_ID);
    DigitalInput m_Sensor = new DigitalInput(IntakerConstants.Sensor_ID);

    private MotorOutputConfigs m_OutputConfigs=new MotorOutputConfigs();
    private Slot0Configs m_Slot0Configs = new Slot0Configs();

    public static Intaker GetInstance()
    {
        return m_Instance==null?m_Instance=new Intaker():m_Instance;
    }

    Intaker(){
        IntakerConfig();
    }

    private void IntakerConfig(){
        m_OutputConfigs.NeutralMode=NeutralModeValue.Brake;
        m_OutputConfigs.Inverted=InvertedValue.CounterClockwise_Positive;//TODO
        m_OutputConfigs.PeakForwardDutyCycle=1;
        m_OutputConfigs.PeakReverseDutyCycle=-1;

        m_Slot0Configs.kP=IntakerConstants.kP;
        m_Slot0Configs.kI=IntakerConstants.kI;
        m_Slot0Configs.kD=IntakerConstants.kD;

        m_Intaker.getConfigurator().apply(m_OutputConfigs);
        m_Intaker.getConfigurator().apply(m_Slot0Configs);
        
    }
    
    public void SetOutput(double _Out){
        m_Intaker.set(_Out); 
    }

    public double GetOutput(){
        return m_Intaker.get();
    }

    public boolean HasNote(){
        return !m_Sensor.get();
    }

    public void NoteIn(){
        if(HasNote()) SetOutput(0);
        else SetOutput(IntakerConstants.NoteInOutput);
    }

    public void NoteOut(){
        SetOutput(IntakerConstants.NoteOutOutput);
    }

    @Override
    public void periodic(){
        //SmartDashboard
    }
}
