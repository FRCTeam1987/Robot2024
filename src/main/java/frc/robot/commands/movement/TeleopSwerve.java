package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s). This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class TeleopSwerve extends Command {

  private PIDController thetaController;
  private PIDController yController;
  private IntSupplier mPovDegree;
  private BooleanSupplier mShouldYLock;
  private double mYSetPoint = 0.0;
  private DoubleSupplier mSpeedMultiplier;
  private boolean useDPad = false;
  private boolean useYLock = false;
  private double setPoint = 0.0;

  private final Drivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final SlewRateLimiter translationXSlewRate = new SlewRateLimiter(4.0);
  private final SlewRateLimiter translationYSlewRate = new SlewRateLimiter(4.0);
  private final SlewRateLimiter rotationSlewRate = new SlewRateLimiter(6.0);
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(Constants.MaxSpeed * 0.1)
          .withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  public static final double DEADBAND = 0.05;

  public static final double MAX_VELOCITY_METERS_PER_SECOND = Constants.MaxSpeed;
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = Constants.MaxAngularRate;

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param translationYSupplier the supplier of the translation y value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param rotationSupplier the supplier of the rotation value as a percentage of the maximum
   *     rotational velocity as defined by the standard field or robot coordinate system
   */
  public TeleopSwerve(
      Drivetrain drivetrain,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier speedMultiplier,
      IntSupplier povDegree,
      BooleanSupplier shouldYLock) {

    mSpeedMultiplier = speedMultiplier;
    mPovDegree = povDegree;
    mShouldYLock = shouldYLock;

    this.drivetrain = drivetrain;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivetrain);
    thetaController = new PIDController(0.02, 0.0, 0.0);
    thetaController.enableContinuousInput(-180, 180);
    yController = new PIDController(0.7, 0.0, 0.0);
    yController.enableContinuousInput(-1, 9);
  }

  @Override
  public void initialize() {
    useDPad = false;
    useYLock = false;
    setPoint = 0.0;
    mYSetPoint = 0.0;
  }

  @Override
  public void execute() {
    // invert the controller input and apply the deadband and squaring to make the robot more
    // responsive to small changes in the controller
    double xPercentage =
        translationXSlewRate.calculate(modifyAxis(translationXSupplier.getAsDouble()));
    double yPercentage =
        translationYSlewRate.calculate(modifyAxis(translationYSupplier.getAsDouble()));
    double rotationPercentage =
        rotationSlewRate.calculate(modifyAxis(-rotationSupplier.getAsDouble()));

    if (useYLock && !Util.isWithinTolerance(translationYSupplier.getAsDouble(), 0.0, 0.5)) {
      useYLock = false;
      mYSetPoint = 0.0;
      DriverStation.reportWarning("Stop using Y Lock.", false);
    } else if (mShouldYLock.getAsBoolean()) {
      mYSetPoint = 90.0; // Degree to rotate to?
    }
    if (useYLock) {
      yController.setSetpoint(mYSetPoint);
      yPercentage = yController.calculate(drivetrain.getPose().getY(), yController.getSetpoint());
    }

    if (useDPad && !Util.isWithinTolerance(rotationSupplier.getAsDouble(), 0.0, 0.25)) {
      useDPad = false;
      setPoint = 0.0;
      DriverStation.reportWarning("Stop using DPad.", false);
    } else if (mPovDegree.getAsInt() >= 0) {
      useDPad = true;
      switch (mPovDegree.getAsInt()) {
        case 0:
          setPoint = 0;
          break;
        case 90:
          setPoint = -90;
          break;
        case 180:
          setPoint = 180;
          break;
        case 270:
          setPoint = 90;
          break;
        default:
          break;
      }
    }
    if (useDPad) {
      thetaController.setSetpoint(setPoint);
      rotationPercentage =
          thetaController.calculate(
              drivetrain.getPose().getRotation().getDegrees(), thetaController.getSetpoint());
    }

    double xVelocity =
        xPercentage * MAX_VELOCITY_METERS_PER_SECOND * mSpeedMultiplier.getAsDouble();
    double yVelocity =
        yPercentage * MAX_VELOCITY_METERS_PER_SECOND * mSpeedMultiplier.getAsDouble();
    double rotationalVelocity = rotationPercentage * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    FieldCentric driveRequest =
        drive
            .withVelocityX(xVelocity) // Drive forward with
            // negative Y (forward)
            .withVelocityY(yVelocity) // Drive left with negative X (left)
            .withRotationalRate(
                rotationalVelocity); // Drive counterclockwise with negative X (left)

    drivetrain.setControl(driveRequest);
  }

  @Override
  public void end(boolean interrupted) {


    FieldCentric driveRequest =
        drive
            .withVelocityX(0.0) // Drive forward with
            // negative Y (forward)
            .withVelocityY(0.0) // Drive left with negative X (left)
            .withRotationalRate(0.0); // Drive counterclockwise with negative X (left)

    drivetrain.setControl(driveRequest);

    super.end(interrupted);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value input value
   * @return square of the value
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
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
}
