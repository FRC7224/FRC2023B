/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class GyroIOAhrs implements GyroIO {
  private final AHRS gyro;

  public GyroIOAhrs() {
    gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        gyro.isConnected(); // Indicates whether the sensor is currently connected to the host
    // computer.
    inputs.positionDeg = -gyro.getYaw(); // degrees
    inputs.velocityDegPerSec =
        gyro.getRate(); // Return the rate of rotation of the yaw (Z-axis) gyro, in degrees per
    // second.
    inputs.pitch = gyro.getPitch() + Constants.PITCH_CAL_OFFSET;
    inputs.roll = gyro.getRoll();
  }
}
