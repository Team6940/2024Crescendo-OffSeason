package frc.robot.Library.team8814.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class ChassisOptimize {
    public static Translation2d optimizeDesireChassisVelocity(Translation2d desireChassisVelocity,Translation2d currentDesireChassisVelocity){
        Translation2d currentChassisSpeed=currentDesireChassisVelocity;
        Rotation2d headingAngle=currentChassisSpeed.getAngle();
        Rotation2d desireDirection=desireChassisVelocity.getAngle();
        double rotatedCurrentSpeed=currentChassisSpeed.rotateBy(desireDirection.times(-1)).getX();
        double desireChassisSpeed=desireChassisVelocity.getNorm();

        Translation2d deltaV=desireChassisVelocity.minus(currentChassisSpeed);
        Translation2d acceleration=deltaV.times(Constants.SwerveConstants.maxDeceleration/(deltaV.getNorm()));
           
        if (rotatedCurrentSpeed<0){
        desireChassisSpeed=0;
        }else if(rotatedCurrentSpeed<desireChassisSpeed){
            desireChassisSpeed=rotatedCurrentSpeed;
        }
        desireChassisVelocity=new Translation2d(desireChassisSpeed,0).rotateBy(desireDirection); 
        Translation2d rotatedDesireAcceleration=acceleration.rotateBy(headingAngle.times(-1));
        if((Constants.SwerveConstants.maxAcceleration-rotatedDesireAcceleration.getX())<0){
            acceleration=acceleration.times(Constants.SwerveConstants.maxAcceleration/Math.abs(rotatedDesireAcceleration.getX()));
            
        }
        Translation2d changeSpeed=acceleration.times(Constants.SwerveConstants.loopDuration);
            if(changeSpeed.getNorm()<deltaV.getNorm()){
                desireChassisVelocity=currentChassisSpeed.plus(changeSpeed);
        }
        return desireChassisVelocity;
    }

    public static double optimizeDesireChassisRotation(double desireAngularSpeed,double currentDesireChassisAngularSpeed){
        double currentAngularSpeed = currentDesireChassisAngularSpeed;
        double angularAcceleration;
        double deltaOmega=desireAngularSpeed-currentAngularSpeed;
        if (Math.abs(desireAngularSpeed)>Math.abs(currentAngularSpeed)){
            angularAcceleration=Constants.SwerveConstants.maxAngularAcceleration;
        }else{
            angularAcceleration=Constants.SwerveConstants.maxAngularDeceleration;
        }
        double changeSpeed=angularAcceleration*Constants.SwerveConstants.loopDuration;
        if(deltaOmega<0){
            changeSpeed=changeSpeed*-1;
        }
        if(Math.abs(changeSpeed)<Math.abs(deltaOmega)){
            desireAngularSpeed=currentAngularSpeed+changeSpeed;
        }
        return desireAngularSpeed;
    }

}
