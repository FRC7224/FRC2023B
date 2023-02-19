package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

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
public class ArmExtendCommand extends CommandBase {

  private final ArmSubsystem armsubsystem;
  private final DoubleSupplier translationXSupplier;
  private final JoystickButton extendoverideSupplier;

  /**
   * Create a new ArmExtendCommand command object.
   *
   * @param armsubsystem the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @return
   */
  public ArmExtendCommand(
      ArmSubsystem armsubsystem,
      JoystickButton extendoverideSupplier,
      DoubleSupplier translationXSupplier) {
    this.armsubsystem = armsubsystem;
    this.extendoverideSupplier = extendoverideSupplier;
    this.translationXSupplier = translationXSupplier;

    addRequirements(armsubsystem);
  }

  /** Used to create string thoughout loop */
  StringBuilder _sb = new StringBuilder();

  int _loops = 0;

  /** Track button state for single press event */
  boolean _lastButton1 = false;

  public void execute() {
    /* Gamepad processing */
    double targetPositionRotations = 0;
    double leftYstick = translationXSupplier.getAsDouble();
    /// *******************need to FIX  ************************ */
    boolean button1 = true; // X-Button
    boolean button2 = true; // A-Button
    double motorOutput;

    /* Get Talon/Victor's current output percentage */
    motorOutput = ArmSubsystem.GetMotorOutputPercent();

    /* Deadband gamepad */
    if (Math.abs(leftYstick) < 0.10) {
      /* Within 10% of zero */
      leftYstick = 0;
    }

    /* Prepare line to print */
    _sb.append("\tout:");
    /* Cast to int to remove decimal places */
    _sb.append((int) (motorOutput * 100));
    _sb.append("%"); // Percent

    _sb.append("\tpos:");
    _sb.append(ArmSubsystem.GetSelectedSensorPosition());
    _sb.append("u"); // Native units

    /**
     * When button 1 is pressed, perform Position Closed Loop to selected position, indicated by
     * Joystick position x10, [-10, 10] rotations
     */
    if (!_lastButton1 && button1) {
      /* Position Closed Loop */

      /* 10 Rotations * 4096 u/rev in either direction */
      targetPositionRotations = leftYstick * 10.0 * 4096;
      armsubsystem.SetTargetPositionRotations(targetPositionRotations);
    }

    /* When button 2 is held, just straight drive */
    if (button2) {
      /* Percent Output */
      armsubsystem.SetPercentOutput(leftYstick);
    }

    /* If Talon is in position closed-loop, print some more info */
    if (armsubsystem.GetControlMode() == ControlMode.Position) {
      /* ppend more signals to print when in speed mode. */
      _sb.append("\terr:");
      _sb.append(armsubsystem.GetClosedLoopError());
      _sb.append("u"); // Native Units

      _sb.append("\ttrg:");
      _sb.append(targetPositionRotations);
      _sb.append("u"); // / Native Units
    }

    /** Print every ten loops, printing too much too fast is generally bad for performance. */
    if (++_loops >= 10) {
      _loops = 0;
      System.out.println(_sb.toString());
    }

    /* Reset built string for next loop */
    _sb.setLength(0);

    /* Save button state for on press detect */
    _lastButton1 = button1;
  }

  @Override
  public void end(boolean interrupted) {
    this.armsubsystem.stop();

    super.end(interrupted);

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
  }

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.DEADBAND);

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
