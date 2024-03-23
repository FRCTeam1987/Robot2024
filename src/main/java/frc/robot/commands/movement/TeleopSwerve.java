package frc.robot.commands.movement;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.Util;
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

  private final PIDController thetaController;
  private final PIDController yController;
  private final IntSupplier mPovDegree;
  private final double mIntakeSetPoint = -35.0; // Change Me to match Source Angle
  private final DoubleSupplier mSpeedMultiplier;
  private boolean useDPad = false;
  private BooleanSupplier mShouldIntakeLock = () -> false;
  private boolean useIntakeLock = false;
  private double setPoint = 0.0;

  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;
  private SwerveRequest.FieldCentric drive;

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
      CommandSwerveDrivetrain drivetrain,
      SwerveRequest.FieldCentric drive,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      DoubleSupplier speedMultiplier,
      IntSupplier povDegree,
      BooleanSupplier shouldIntakeLock) {

    mSpeedMultiplier = speedMultiplier;
    mPovDegree = povDegree;
    mShouldIntakeLock = shouldIntakeLock;

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
    useIntakeLock = mShouldIntakeLock.getAsBoolean();
    setPoint = 0.0;
  }

  @Override
  public void execute() {
    // invert the controller input and apply the deadband and squaring to make the robot more
    // responsive to small changes in the controller
    double xPercentage = translationXSupplier.getAsDouble();
    double yPercentage = translationYSupplier.getAsDouble();
    double rotationPercentage = rotationSupplier.getAsDouble();

    useIntakeLock = mShouldIntakeLock.getAsBoolean();

    if (useIntakeLock) {
      // System.out.println("SET INTAKE LOCK");
      thetaController.setSetpoint(mIntakeSetPoint);
      rotationPercentage =
          thetaController.calculate(
              drivetrain.getPose().getRotation().getDegrees(), thetaController.getSetpoint());
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
    value = deadband(value);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double deadband(double value) {
    if (Math.abs(value) > TeleopSwerve.DEADBAND) {
      if (value > 0.0) {
        return (value - TeleopSwerve.DEADBAND) / (1.0 - TeleopSwerve.DEADBAND);
      } else {
        return (value + TeleopSwerve.DEADBAND) / (1.0 - TeleopSwerve.DEADBAND);
      }
    } else {
      return 0.0;
    }
  }
}
