package frc.robot.subsystems.ShooterLogged;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIOInterface {

    default void setVoltage(double leftVoltage, double rghtVoltage) {
    }

    default void setRPS(double leftRPS, double rghtRPS) {
    }

    default void updateInputs(ShooterIOInputs inputs) {
    }

    // default void updateOutput() {}

    @AutoLog
    class ShooterIOInputs {
        public boolean leftconnected = false;
        public boolean rghtconnected = false;
        public double leftSensorRPS;
        public double rghtSensorRPS;
        public double leftCurrentAmps;
        public double rghtCurrentAmps;
        public double leftVoltage;
        public double rghtVoltage;
    }

    // @AutoLog
    // public class ShooterIOOutputs{
        
    // }
}
