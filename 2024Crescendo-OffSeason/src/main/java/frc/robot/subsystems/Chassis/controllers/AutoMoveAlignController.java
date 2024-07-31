// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis.controllers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Chassis.PoseEstimator;
/** Add your docs here. */
public class AutoAlignController {

 private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;
    private boolean fieldRelative = false;
    private String m_limelight="Limelight";
    PoseEstimator m_poseEstimator;
    public Translation2d inputDesireVelocity=new Translation2d(0,0);
    public PIDController m_RotationPidController=new PIDController( AutoAlignConstants.kAutoAlignRotationP,
                                                                    AutoAlignConstants.kAutoAlignRotationI,
                                                                    AutoAlignConstants.kAutoAlignRotationD);
    public double inputRotation=0;
    public AutoAlignController(PoseEstimator poseEstimator,String limelight){
        this.m_poseEstimator=poseEstimator;
        this.m_limelight=limelight;
    }

    public Translation2d currentDesireSpeed=new Translation2d(0,0);
    public double currentDesireAngularSpeed=0;
    /**
     * Many of the parameters here have no actual usage, they are just for api adaptment only
     * @param x the x axis of the controller, with no actual use
     * @param y the y axis of the controller , controlling the L&R movement of the robot
     * @param omega the omega input from the controller
     * @param fieldRelative have no actual use, the robot will drive robot relatively no matter the parameter is 
     */
    public void updateControllerInput(double x, double y, double omega, boolean fieldRelative){
        controllerX = x;
        controllerY = y;
        controllerOmega = omega;
        this.fieldRelative = true;
        inputDesireVelocity= applyDeadBand(controllerX, controllerY).times(Constants.Swerve.maxSpeed);
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red){
				inputDesireVelocity=inputDesireVelocity.rotateBy(new Rotation2d(Math.PI));
            }
        }
        
        inputRotation=controllerOmega* Math.max(Constants.Swerve.stationaryAngularVelocity,0);
    }
    public Translation2d applyDeadBand(double rawX,double rawY)
    {
        
        Translation2d rawTranslation2d=new Translation2d(rawX, rawY);
        Translation2d translation2d=rawTranslation2d.getNorm()>DriveConstants.kInnerDeadband?rawTranslation2d:new Translation2d();
        return translation2d;
    }

    public ChassisSpeeds getDesireSpeeds(){
        Translation2d desireVelocity=inputDesireVelocity;
        double rotation=inputRotation;
        desireVelocity=optimizeDesireChassisVelocity(desireVelocity,currentDesireSpeed);
        currentDesireSpeed=desireVelocity;
        rotation=optimizeDesireChassisAngularSpeed(rotation,currentDesireAngularSpeed);
        currentDesireAngularSpeed=rotation;

        return fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            desireVelocity.getX(), 
            desireVelocity.getY(), 
            rotation, 
            m_poseEstimator.getEstimatedPosition().getRotation()
        )
        : new ChassisSpeeds(
            desireVelocity.getX(), 
            desireVelocity.getY(), 
            rotation);
    }
    public Translation2d optimizeDesireChassisVelocity(Translation2d desireChassisVelocity,Translation2d currentDesireChassisVelocity){
        Translation2d currentChassisSpeed=currentDesireChassisVelocity;
        Rotation2d headingAngle=currentChassisSpeed.getAngle();
        Rotation2d desireDirection=desireChassisVelocity.getAngle();
        double rotatedCurrentSpeed=currentChassisSpeed.rotateBy(desireDirection.times(-1)).getX();
        double desireChassisSpeed=desireChassisVelocity.getNorm();

        Translation2d deltaV=desireChassisVelocity.minus(currentChassisSpeed);
        Translation2d acceleration=deltaV.times(Constants.Swerve.maxDeceleration/(deltaV.getNorm()));
           
        if (rotatedCurrentSpeed<0){
        desireChassisSpeed=0;
        }else if(rotatedCurrentSpeed<desireChassisSpeed){
            desireChassisSpeed=rotatedCurrentSpeed;
        }
        desireChassisVelocity=new Translation2d(desireChassisSpeed,0).rotateBy(desireDirection); 
        Translation2d rotatedDesireAcceleration=acceleration.rotateBy(headingAngle.times(-1));
        if((Constants.Swerve.maxAcceleration-rotatedDesireAcceleration.getX())<0){
            acceleration=acceleration.times(Constants.Swerve.maxAcceleration/Math.abs(rotatedDesireAcceleration.getX()));
            
        }
        Translation2d changeSpeed=acceleration.times(Constants.Swerve.loopDuration);
            if(changeSpeed.getNorm()<deltaV.getNorm()){
                desireChassisVelocity=currentChassisSpeed.plus(changeSpeed);
        }
        return desireChassisVelocity;
    }

    public double optimizeDesireChassisAngularSpeed(double desireAngularSpeed,double currentDesireChassisAngularSpeed){
        double currentAngularSpeed = currentDesireChassisAngularSpeed;
        double angularAcceleration;
        double deltaOmega=desireAngularSpeed-currentAngularSpeed;
        if (Math.abs(desireAngularSpeed)>Math.abs(currentAngularSpeed)){
            angularAcceleration=Constants.Swerve.maxAngularAcceleration;
        }else{
            angularAcceleration=Constants.Swerve.maxAngularDeceleration;
        }
        double changeSpeed=angularAcceleration*Constants.Swerve.loopDuration;
        if(deltaOmega<0){
            changeSpeed=changeSpeed*-1;
        }
        if(Math.abs(changeSpeed)<Math.abs(deltaOmega)){
            desireAngularSpeed=currentAngularSpeed+changeSpeed;
        }
        return desireAngularSpeed;
    }

}
