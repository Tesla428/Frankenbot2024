package frc.robot.systems.DriveTrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RevSwerve {
    private static final double kWheelRadius = 0.01; // 310mm
    private static final int kEncoderResolution = 1024;

    private CANSparkMax turningMotor;
    private RelativeEncoder turningEncoder;
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;

    private String Name;

    public RevSwerve(int driveMotorCanbusAddress, int turningMotorCanbusAddress, String Name) {
        turningMotor = new CANSparkMax(turningMotorCanbusAddress, MotorType.kBrushless);
        turningEncoder = turningMotor.getEncoder();
        driveMotor = new CANSparkMax(driveMotorCanbusAddress, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        this.Name = Name;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(WheelSpeed(), WheelAngle());
    }

    private double WheelSpeed() {
        return driveEncoder.getVelocity() / 2 * Math.PI * kWheelRadius / 60;
    };

    private Rotation2d WheelAngle() {
        return new Rotation2d(turningEncoder.getPosition() / kEncoderResolution);
    };

    public void setDesiredState(SwerveModuleState desiredState) {
        SmartDashboard.putString(Name + "desiredState", desiredState.angle.toString());
        SmartDashboard.putNumber(Name + "desiredState", desiredState.speedMetersPerSecond);
       
        // Optomize avoids spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turningEncoder.getPosition() / kEncoderResolution * 2 * Math.PI));
        double turnRadians = turningEncoder.getPosition() / kEncoderResolution * 2 * Math.PI;

        turningMotor.set((state.angle.getRadians() - turnRadians) / 2);

        driveMotor.set(state.speedMetersPerSecond / 3);

        SmartDashboard.putString(Name + "desiredState", desiredState.angle.toString());
    }

    
}
