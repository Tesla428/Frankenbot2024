
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.systems.Controller;
import frc.robot.systems.DriveTrain.DriveTrain;

public class Robot extends TimedRobot {
  private final DriveTrain swerve = DriveTrain.getInstance();
  private final Controller controller = Controller.getInstance();

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    swerve.updateOdometry();
    controller.updateMeasurements();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    swerve.Drive(false);
  }

}
