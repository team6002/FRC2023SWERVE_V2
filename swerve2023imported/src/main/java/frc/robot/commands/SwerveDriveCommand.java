package frc.robot.commands;

// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain m_drivetrain;
  private final XboxController controller;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  public boolean fieldMode = false;
  double deadzone = 0.2;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double turn = 0;        //variable for turning movement
  
  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.m_drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.controller = controller;
  }

  @Override
  public void initialize() {
  }
 
  @Override
  public void execute() {
    // y = controller.getLeftY();

    // x = controller.getLeftX();

    // turn = controller.getRightX();
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    final var xSpeed = xspeedLimiter.calculate(modifyAxis(-controller.getLeftY()))
        * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    
    final var ySpeed = yspeedLimiter.calculate(modifyAxis(-controller.getLeftX()))
        * DriveConstants.kMaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    
    final var rot = rotLimiter.calculate(modifyAxis(-controller.getRightX()))
        * SwerveDrivetrain.kMaxAngularSpeed;

    boolean fieldRelative = true;
    if (controller.getRightTriggerAxis() >= 0.7) {
      fieldRelative = false;  
    }

    // SmartDashboard.putNumber("xspeed", xSpeed);
    // SmartDashboard.putNumber("yspeed", ySpeed);
    // SmartDashboard.putNumber("rotspeed", rot);
    // SmartDashboard.putNumber("yaxis", controller.getLeftY());
    // SmartDashboard.putNumber("x-axis", controller.getRightX());



    m_drivetrain.drive( xSpeed, ySpeed, rot,true);
  
  }


  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0.0, 0.0, 0.0, true);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);

    // Square the axis
    // value = Math.copySign(value * value, value);

    return value;
  }

}