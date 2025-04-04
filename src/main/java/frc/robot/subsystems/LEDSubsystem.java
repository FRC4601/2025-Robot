package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


/**
 * Subsystem for controlling the REV Robotics Blinkin LED Driver
 * The Blinkin is controlled using a PWM signal, similar to how a Spark motor controller
 * would be controlled, but the signal values represent different LED patterns.
 */
public class LEDSubsystem extends SubsystemBase {
  // The Blinkin is controlled via PWM, like a Spark motor controller
  private final Spark blinkin;
  
  // Current pattern being displayed
  private LEDPattern currentPattern;
  private Alliance alliance;


  /**
   * Creates a new LEDSubsystem
   */
  public LEDSubsystem() {
    // Initialize the Blinkin on the specified PWM port
    blinkin = new Spark(0);

    alliance = DriverStation.getAlliance().orElse(null);

    if (alliance == Alliance.Red) {
      setPattern(LEDPattern.SOLID_RED);
    } else if (alliance == Alliance.Blue) {
      setPattern(LEDPattern.SOLID_BLUE);
    } else {
      // If alliance info isn't available 
      setPattern(LEDPattern.SOLID_VIOLET);
    }

  }

  /**
   * Sets the LED pattern on the Blinkin
   * @param pattern The pattern to display
   */
  public void setPattern(LEDPattern pattern) {
    blinkin.set(pattern.getValue());
    currentPattern = pattern;
  }

  /**
   * Gets the current LED pattern
   * @return The current pattern
   */
  public LEDPattern getCurrentPattern() {
    return currentPattern;
  }

  /**
   * Enum containing the values for different LED patterns on the Blinkin
   * Values are from the REV Robotics Blinkin manual:
   * http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
   */
  public enum LEDPattern {
    // Solid colors
    SOLID_BLACK(0.99),
    SOLID_WHITE(0.93),
    SOLID_RED(0.61),
    SOLID_BLUE(0.87),
    SOLID_GREEN(0.77),
    SOLID_YELLOW(0.69),
    SOLID_VIOLET(0.9),
    
    // Color patterns
    RAINBOW_RAINBOW(-0.99),
    RAINBOW_PARTY(-0.97),
    COLOR_WAVES_RAINBOW(-0.45),
    
    // Fixed palette patterns
    FIRE_MEDIUM(-0.59),
    FIRE_LARGE(-0.57),
    TWINKLES_RAINBOW(-0.55),
    COLORWAVES_FOREST(-0.41),
    LARSON_SCANNER_RED(-0.35),
    STROBE_RED(0.15),
    STROBE_BLUE(0.23),
    STROBE_GOLD(0.19),
    
    // Alliance colors
    ALLIANCE_COLOR_WAVES(-0.43);
    
  
    
    private final double value;
    
    LEDPattern(double value) {
      this.value = value;
    }
    
    public double getValue() {
      return this.value;
    }
  }

  @Override
  public void periodic() {


  }
}

