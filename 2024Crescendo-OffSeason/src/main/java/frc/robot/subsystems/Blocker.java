package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlockerConstants;

public class Blocker extends SubsystemBase{
    public static Blocker m_Instance;

    private static TalonFX m_Blocker = new TalonFX(BlockerConstants.Blocker_ID);
    final DutyCycleOut m_DutyCycleOut = new DutyCycleOut(0);

    DigitalInput m_Sensor = new DigitalInput(BlockerConstants.Sensor_ID);

    public static Blocker GetInstance()
    {
        return m_Instance==null?m_Instance=new Blocker():m_Instance;
    }

    Blocker(){
        BlockerConfig();
    }

    private void BlockerConfig(){
        m_Blocker.setNeutralMode(NeutralModeValue.Brake);
        m_Blocker.setInverted(false);//TODO
    }

    public void SetOutPut(double _PCT){
        m_Blocker.setControl(m_DutyCycleOut.withOutput(_PCT));
    }

    public boolean HasNote(){
        return !m_Sensor.get();
    }
}
