package frc.robot.Subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Constants.PathPlannerConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

//Powered by YAGSL! Very much thanks for the developers of it!

public class SwerveDriveTrain extends SubsystemBase {
    public static SwerveDriveTrain m_Instance;
    public static String m_SPKRLLName;
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    SwerveDrive m_SwerveDrive = new SwerveDrive(null, null, 0);
    int driveMode = 0; // 0 for brake, 1 for manual drive, 2 for semiauto, 3 for fullauto;

    public static SwerveDriveTrain GetInstance() {
        return m_Instance == null ? m_Instance = new SwerveDriveTrain() : m_Instance;
    }

    SwerveDriveTrain() {
        SwerveDriveTrainConfig();
    }

    public int getAllianceSPKRTagID() {
        return DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4;
    }

    public void setupLimelight() {
        m_SPKRLLName = LimelightConstants.SPKR_LLname;
        LimelightHelpers.setPriorityTagID(m_SPKRLLName, getAllianceSPKRTagID());
    }

    public void setupPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        PathPlannerConstants.TRANSLATION_PID,
                        // Translation PID constants
                        PathPlannerConstants.ROTATION_PID,
                        // Rotation PID constants
                        SwerveConstants.MaxSpeed,
                        // Max module speed, in m/s
                        m_SwerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()
                // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    private void SwerveDriveTrainConfig() {
        try {
            m_SwerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(SwerveConstants.MaxSpeed);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        setupPathPlanner();
        setupLimelight();
    }

    public void drive(double translationX, double translationY, double headingX, double headingY) {
        // Make the robot move
        m_SwerveDrive.driveFieldOriented(m_SwerveDrive.swerveController.getTargetSpeeds(translationX, translationY,
                headingX,
                headingY,
                m_SwerveDrive.getYaw().getRadians(),
                m_SwerveDrive.getMaximumVelocity()));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_SwerveDrive.driveFieldOriented(chassisSpeeds, null);
    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return run(() -> drive(translationX.getAsDouble(), translationY.getAsDouble(), headingX.getAsDouble(),
                headingY.getAsDouble()));
    }

    public Command driveCommand(Supplier<ChassisSpeeds> chassisSpeeds) {
        return run(() -> m_SwerveDrive.driveFieldOriented(chassisSpeeds.get()));
    }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by
     * odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() {
        return m_SwerveDrive.getPose();
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0.
     */
    public void zeroGyro() {
        m_SwerveDrive.zeroGyro();
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        return m_SwerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return m_SwerveDrive.getRobotVelocity();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        m_SwerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this
     * method. However, if either gyro angle or module position is reset, this must
     * be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose) {
        m_SwerveDrive.resetOdometry(initialHolonomicPose);
    }

    /**
     * Get the drive Mode of the drive train;
     * 0 for brake, 1 for full manual, 2 for semiauto, 3 for full auto;
     *
     * @param
     */
    public int getDriveMode() {
        return driveMode;
    }

    /**
     * Set the drive Mode of the drive train;
     *
     * @param Mode 0 for brake, 1 for full manual, 2 for semiauto, 3 for full auto;
     */
    public void setDriveMode(int Mode) {
        this.driveMode = Mode;
    }

    /*
     * Return the Chassis Speed in the specific Mode
     * 
     * @param Mode 0 for brake, 1 for full manual, 2 for aiming, 3 for auto-picking,
     * 4 for full auto;
     */
    public ChassisSpeeds getChassisSpeeds(Double translationX, Double translationY,
            Double headingX,
            Double headingY,
            int DriveMode) {
        if (DriveMode == 4 || DriveMode == 0)
            return new ChassisSpeeds(); // in Full auto no common drive commands should be scheduled;
        if (DriveMode == 2 && LimelightHelpers.getTV(m_SPKRLLName)) {
            return m_SwerveDrive.swerveController.getTargetSpeeds(translationX,
                    translationY, LimelightHelpers.getBotPose_TargetSpace(m_SPKRLLName)[0],
                    LimelightHelpers.getBotPose_TargetSpace(m_SPKRLLName)[1],
                    m_SwerveDrive.getYaw().getRadians(), m_SwerveDrive.getMaximumVelocity());
        }
        if (DriveMode == 3) {
            // TODO
        }
        return m_SwerveDrive.swerveController.getTargetSpeeds(translationX,
                translationY, headingX, headingY,
                m_SwerveDrive.getYaw().getRadians(), m_SwerveDrive.getMaximumVelocity());
    };

    @Override
    public void periodic() {
        // Networktable Stuff
    }
}
