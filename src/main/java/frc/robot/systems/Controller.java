package frc.robot.systems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Controller {
    private final XboxController xboxCtrlr = new XboxController(0);

    private final static Controller instance = new Controller();

    public static Controller getInstance() {
        return instance;
    }

    public boolean getLeftBumperPressed() {
        return xboxCtrlr.getLeftBumperPressed();
    }

    public boolean getRightBumperPressed() {
        return xboxCtrlr.getRightBumperPressed();
    }

    public double getLeftX() {
        return xboxCtrlr.getLeftX();
    }

    public double getLeftY() {
        return xboxCtrlr.getLeftY();
    }

    public double getRightX() {
        return xboxCtrlr.getRightX();
    }

    public double getRightY() {
        return xboxCtrlr.getRightY();
    }

    public PovAngle getPOV() {
        int rawAngle = xboxCtrlr.getPOV();
        if (rawAngle == 0) {
            return PovAngle.North;
        } else if (rawAngle == 45) {
            return PovAngle.NE;
        } else if (rawAngle == 90) {
            return PovAngle.East;
        } else if (rawAngle == 135) {
            return PovAngle.SE;
        } else if (rawAngle == 180) {
            return PovAngle.South;
        } else if (rawAngle == 225) {
            return PovAngle.SW;
        } else if (rawAngle == 270) {
            return PovAngle.West;
        } else if (rawAngle == 315) {
            return PovAngle.NW;
        } else {
            return PovAngle.Bad;
        }
    }

    public boolean getBackButton() {
        return xboxCtrlr.getBackButton();
    }

    public boolean getStartButton() {
        return xboxCtrlr.getStartButton();
    }

    public boolean getLeftStickButtonPressed(){
        return xboxCtrlr.getLeftStickButtonPressed();        
    }

    public boolean getRightStickButtonPressed(){
        return xboxCtrlr.getRightStickButtonPressed();        
    }

    public void setRumble(float Amount) {
        xboxCtrlr.setRumble(RumbleType.kLeftRumble, Amount);
        xboxCtrlr.setRumble(RumbleType.kRightRumble, Amount);
    }

    public void stopRumble() {
        xboxCtrlr.setRumble(RumbleType.kLeftRumble, 0);
        xboxCtrlr.setRumble(RumbleType.kRightRumble, 0);
    }

    public boolean getAButton() {
        return xboxCtrlr.getAButton();
    }

    public boolean getXButton() {
        return xboxCtrlr.getXButton();
    }

    public boolean getBButton() {
        return xboxCtrlr.getBButton();
    }

    public boolean getYButtonPressed() {
        return xboxCtrlr.getYButtonPressed();
    }

    public boolean getYButton() {
        return xboxCtrlr.getYButton();
    }

    public boolean getRightBumper() {
        return xboxCtrlr.getRightBumper();
    }

    public boolean getLeftBumper() {
        return xboxCtrlr.getLeftBumper();
    }

    public double getRightTriggerAxis() {
        return xboxCtrlr.getRightTriggerAxis();
    }

    public boolean getRightTriggerPress() {
        if (getRightTriggerAxis() > .5) {
            return true;
        } else {
            return false;
        }
    }

    public double getLeftTriggerAxis() {
        return xboxCtrlr.getLeftTriggerAxis();
    }

    private boolean getLeftTriggerPress() {
        if (getLeftTriggerAxis() > .5) {
            return true;
        } else {
            return false;
        }
    }

    public void updateMeasurements() {
        SmartDashboard.putBoolean("A Button", getAButton());
        SmartDashboard.putBoolean("B Button", getBButton());
        SmartDashboard.putBoolean("X Button", getXButton());
        SmartDashboard.putBoolean("Y Button", getYButton());
        SmartDashboard.putBoolean("Right Trigger Bool", getRightTriggerPress());
        SmartDashboard.putNumber("Right Trigger Position", getRightTriggerAxis());
        SmartDashboard.putBoolean("Left Trigger Bool", getLeftTriggerPress());
        SmartDashboard.putNumber("Left Trigger Position", getLeftTriggerAxis());
    }

    public enum PovAngle {
        North,
        NW,
        NE,
        South,
        SW,
        SE,
        West,
        East,
        Bad
    }
}
