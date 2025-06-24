package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
  public static final double weight = 0.5;
  public static final double gearing = 50;
  public static final double armLengthMeters = Units.inchesToMeters(17.466246);
  // get full mesurements
  public static final double minAngleRad = 0;
  public static final double maxAngleRad = Units.degreesToRadians(100);

  public static final LoggedTunableNumber kP;

  public static final double loweredPositionDeg = 0;
  public static double stowedPositionDeg;
  public static double rollersSpeedIn = -5;
  public static double rollersSpeedOut = 5;

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Intake/kP", 0.028);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Intake/simkP", 0.1);
        break;
    }
  }
}
