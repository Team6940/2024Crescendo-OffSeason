package frc.robot.subsystems.Chassis;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;

import frc.robot.subsystems.Chassis.SwerveModule;
import frc.robot.subsystems.Chassis.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.subsystems.Chassis.controllers.TeleopDriveController;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class DriveSubsystem extends SubsystemBase {

    PoseEstimator m_poseEstimator;
    VisionIO m_visionIO;
    Gyro m_gyro;

    public SwerveModule[] mSwerveMods;

    public Rotation2d directionFromSpeaker;
    public boolean speakerTagDetected=false;
    private Field2d m_field = new Field2d();

    public Translation2d currentDesireSpeed=new Translation2d(0,0);
    public double currentDesireAngularSpeed=0;

    private static final double PERIOD = 0.01;  // 100Hz (10ms period)
    private Notifier odometryNotifier;

    private final TeleopDriveController teleopDriveController;

    private ChassisSpeeds desireSpeeds;

    public enum DriveMode{
        STOP,
        TELEOP,
        AUTONOMOUS,
        AUTO_ALIGN,
    }
    DriveMode currentDriveMode=DriveMode.STOP;
    public DriveSubsystem(Gyro gyro,VisionIO visionIO) {
        this.m_visionIO=visionIO;
        this.m_gyro=gyro;
        this.m_gyro.zeroHeading();
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        m_poseEstimator=new PoseEstimator();
        odometryNotifier = new Notifier(this::updateOdometry);
        startOdometry();
        teleopDriveController = new TeleopDriveController(m_poseEstimator);
        SmartDashboard.putData("Field",m_field);
    }

    public void updateControllerInput(
      double controllerX, double controllerY, double controllerOmega, boolean robotRelative) {
        if (DriverStation.isTeleopEnabled()) {
        if (currentDriveMode != DriveMode.AUTO_ALIGN) {
            currentDriveMode = DriveMode.TELEOP;
        }
        teleopDriveController.updateControllerInput(
            controllerX, controllerY, controllerOmega, robotRelative);
        }
    }

    public Translation2d getDriverDesireSpeeds(){
        return teleopDriveController.inputDesireVelocity;
    }
    public double getDriverDesireRotation(){
        return teleopDriveController.inputRotation;
    }
    private void updateOdometry() {
        // swerveOdometry.update(getGyroYaw(), getModulePositions());
        m_poseEstimator.updateSwerve(getGyroYaw(), getModulePositions());
        // SmartDashboard.putNumber("odometry time", Timer.getFPGATimestamp());
    }
    public void stopOdometry() {
        if (odometryNotifier != null) {
            odometryNotifier.stop();
        }
    }

    public void startOdometry() {
        if (odometryNotifier != null) {
            odometryNotifier.startPeriodic(PERIOD);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
            //SmartDashboard.putString(mod.moduleNumber+" Desire",desiredStates[mod.moduleNumber].toString());
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        // return swerveOdometry.getPoseMeters();
        return m_poseEstimator.getEstimatedPosition();
    }
    // public Pose2d getPoseWithTime(double timestamp) {
    //     // return swerveOdometry.getPoseMeters();
    //     return m_poseEstimator.getRobotPoseWithTime(timestamp);
    // }

    public void setPose(Pose2d pose) {
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        m_poseEstimator.setPose(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        m_poseEstimator.setPose(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        m_gyro.zeroHeading();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public Rotation2d getGyroYaw() {
        return m_gyro.getGyroYaw();
    }

    public Pose2d inversePose2dUsingAlliance(Pose2d pose,DriverStation.Alliance allianceColor){
        if(allianceColor!=getAlliance()){
            return new Pose2d(16.54-pose.getX(),pose.getY(),new Rotation2d(Math.PI).minus(pose.getRotation()));
        }
        return pose;
    }


    public ChassisSpeeds getFieldRelativeSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates()),getHeading());
    }

    public double getChassisSpeed(){
        return Math.sqrt(Math.pow(getRobotRelativeSpeeds().vxMetersPerSecond, 2)+Math.pow(getRobotRelativeSpeeds().vyMetersPerSecond, 2));
    }


    
    //return the chassis speed for the followpathholonomic method
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    //ChassisSpeeds supplier for the followpathholonomic method
    public void setChassisSpeeds (ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Swerve.maxSpeed);
        setModuleStates(moduleStates);
    }
    //auto***************************************************************************************************

    // generate a trajectory(from .path to .json)
    public PathPlannerPath generatePath(String pathName){
        return PathPlannerPath.fromPathFile(pathName);
    }

    //return an auto command
    public Command followPathCommand(PathPlannerPath path){
        // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
        return 
            new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                            new PIDConstants(Constants.AutoConstants.kPTranslationController, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(Constants.AutoConstants.kPRotationController, 0.0, 0.0), // Rotation PID constants
                            Constants.Swerve.maxModuleSpeed, // Max module speed, in m/s
                            Math.sqrt(2)*Constants.Swerve.wheelBase/2, // Drive base radius in meters. Distance from robot center to furthest module.
                            new ReplanningConfig() // Default path replanning config. See the API for the options here
                    ),
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
    }

    public DriverStation.Alliance getAlliance(){
        try{
            return DriverStation.getAlliance().get();
        }catch(Exception e){
            return DriverStation.Alliance.Red;
        }
    }
    

    @Override
    public void periodic(){      

        ChassisSpeeds teleopSpeeds = teleopDriveController.getDesireSpeeds();
        switch (currentDriveMode) {
            case TELEOP -> {
                // Plain teleop drive
                desireSpeeds = teleopSpeeds;
            }
            case AUTO_ALIGN -> {
                // Run auto align with drive input
                // desiredSpeeds = autoAlignController.update();
            }
            default ->{
            }
        }        
        setChassisSpeeds(teleopSpeeds);

        List<TimestampedVisionUpdate> visionPoses=m_visionIO.getRobotPose3ds();
        // m_poseEstimator.visionInput(visionPoses,10*(Math.PI)/180);

        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putString("teleSpeed", teleopSpeeds.toString());
        //print debuging information
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    
}