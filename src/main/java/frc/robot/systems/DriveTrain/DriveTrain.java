package frc.robot.systems.DriveTrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Controller;

import com.kauailabs.navx.frc.*;

public class DriveTrain {
    // public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxSpeed = 1.1; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI * .8; // 1/2 rotation per second
    private final Controller xboxCtrlr = Controller.getInstance();

    private final Translation2d m_frontLeftLocation = new Translation2d(0.3127375, 0.3127375);
    private final Translation2d m_frontRightLocation = new Translation2d(0.3127375, -0.3127375);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.3127375, 0.3127375);
    private final Translation2d m_backRightLocation = new Translation2d(-0.3127375, -0.3127375);

    private final RevSwerve m_frontLeft = new RevSwerve(20, 6, "FrontLeft");
    private final RevSwerve m_frontRight = new RevSwerve(16, 8, "FrontRight");
    private final RevSwerve m_backLeft = new RevSwerve(17, 7, "BackLeft");
    private final RevSwerve m_backRight = new RevSwerve(2, 12, "BackRight");

    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation,
            m_frontRightLocation,
            m_backLeftLocation,
            m_backRightLocation);

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
            m_kinematics,
            navx.getRotation2d(),getPositions());

    private double turbo;

    public void Reset() {
        navx.reset();
    }

    public void Drive(boolean fieldRelative) {

        if (xboxCtrlr.getLeftTriggerAxis() > .5) {
            turbo = 1;
        } else {
            turbo = .7;
        }

        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xboxCtrlr.getLeftY(), 0.05))
                * DriveTrain.kMaxSpeed * turbo;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(xboxCtrlr.getLeftX(), 0.05))
                * DriveTrain.kMaxSpeed * turbo;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(xboxCtrlr.getRightX(), 0.05))
                * DriveTrain.kMaxAngularSpeed * turbo;

        drive(xSpeed, ySpeed, rot, fieldRelative);
    }

    public void Reverse(double speed) {
        drive(-speed, 0, 0, false);
    }

    public void Stop() {
        drive(0, 0, 0, false);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    private void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        if (xboxCtrlr.getLeftBumperPressed() && xboxCtrlr.getRightBumperPressed()) {
            Reset();
        }

        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rot", rot);
        SmartDashboard.putBoolean("fieldRelative", fieldRelative);

    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        odometry.update(navx.getRotation2d(),getPositions());

        SmartDashboard.putString("NavX", navx.getRotation2d().toString());
        SmartDashboard.putString("FrontLeftAngle", m_frontLeft.getPosition().toString());
        SmartDashboard.putString("FrontRightAngle", m_frontRight.getPosition().toString());
        SmartDashboard.putString("BackLeftAngle", m_backLeft.getPosition().toString());
        SmartDashboard.putString("BackRightAngle", m_backRight.getPosition().toString());
    }

    private SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
        };
    }

    private static final DriveTrain instance = new DriveTrain();

    private DriveTrain() {
        Reset();
    }

    public static DriveTrain getInstance() {
        return instance;
    }
}