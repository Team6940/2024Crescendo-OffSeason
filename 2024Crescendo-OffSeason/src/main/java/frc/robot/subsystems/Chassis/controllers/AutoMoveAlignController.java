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
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team8814.util.ChassisOptimize;
import frc.robot.subsystems.Chassis.PoseEstimator;
/** Add your docs here. */
public class AutoMoveAlignController {

 private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;
    private boolean fieldRelative = false;
    private String m_limelight="Limelight";
    PoseEstimator m_poseEstimator;
    public Translation2d inputDesireVelocity=new Translation2d(0,0);
    public PIDController m_MovementPidController=new PIDController( AutoAlignConstants.kAutoAlignMovementP,
                                                                    AutoAlignConstants.kAutoAlignMovementI,
                                                                    AutoAlignConstants.kAutoAlignMovementD);
    public double inputRotation=0;
    public AutoMoveAlignController(PoseEstimator poseEstimator,String limelight){
        this.m_poseEstimator=poseEstimator;
        this.m_limelight=limelight;
    }

    public Translation2d currentDesireSpeed=new Translation2d(0,0);
    public double currentDesireAngularSpeed=0;
    /**
     * Some of the parameters here have no actual usage, they are just for api adaptment only
     * @param x the x axis of the controller,with no actual use
     * @param y the y axis of the controller 
     * @param omega the omega input from the controller,with no actual use
     * @param fieldRelative fieldRelative or not ,with no actual use
     */
    public void updateControllerInput(double x, double y, double omega, boolean fieldRelative){
        controllerX = x;
        controllerY = y;
        controllerOmega = omega;
        this.fieldRelative = false;
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
        double desireYVelocity=inputDesireVelocity.getY();
        double tx=LimelightHelpers.getTX(m_limelight);
        double desireXVelocity=m_MovementPidController.calculate(tx);
        Translation2d desireVelocity=new Translation2d(desireXVelocity, desireYVelocity);
        desireVelocity=ChassisOptimize.optimizeDesireChassisVelocity(desireVelocity,currentDesireSpeed);
        double rotation=0;
        currentDesireSpeed=desireVelocity;
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
