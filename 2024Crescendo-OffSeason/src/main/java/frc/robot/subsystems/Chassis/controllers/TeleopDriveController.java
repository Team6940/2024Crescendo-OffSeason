// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//staled
package frc.robot.subsystems.Chassis.controllers;

import java.security.Principal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Library.team8814.util.ChassisOptimize;
import frc.robot.subsystems.Chassis.DriveSubsystem;
import frc.robot.subsystems.Chassis.PoseEstimator;

/** Add your docs here. */
public class TeleopDriveController {

    private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;
    private boolean fieldRelative = false;
    PoseEstimator m_poseEstimator;
    public Translation2d inputDesireVelocity=new Translation2d(0,0);
    public double inputRotation=0;
    public TeleopDriveController(PoseEstimator poseEstimator){
        this.m_poseEstimator=poseEstimator;
    }

    public Translation2d currentDesireSpeed=new Translation2d(0,0);
    public double currentDesireAngularSpeed=0;
    public void updateControllerInput(double x, double y, double omega, boolean fieldRelative){
        controllerX = x;
        controllerY = y;
        controllerOmega = omega;
        this.fieldRelative = fieldRelative;
        inputDesireVelocity= applyDeadBand(controllerX, controllerY).times(Constants.SwerveConstants.maxSpeed);
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if(alliance.get() == DriverStation.Alliance.Red){
				inputDesireVelocity=inputDesireVelocity.rotateBy(new Rotation2d(Math.PI));
            }
        }
        
        inputRotation=controllerOmega* Math.max(Constants.SwerveConstants.stationaryAngularVelocity,0);
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
        desireVelocity=ChassisOptimize.optimizeDesireChassisVelocity(desireVelocity,currentDesireSpeed);
        currentDesireSpeed=desireVelocity;
        rotation=ChassisOptimize.optimizeDesireChassisRotation(rotation,currentDesireAngularSpeed);
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
    
   
}
