package frc.lib.util;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class MotorConfig {
  // These are public properties which allow them to be accessed directly without need for get/set
  // methods.
  // final means that they are immutable after object creation, attempting to change them will cause
  // a compile error.
  public final boolean inverted;
  public final int currentLimit;
  public final IdleMode idleMode;

  /**
   * RevMotorConfig constructor is protected so that it can only be constructed through the use of
   * the Builder Details of the Rev SparkMAX configuration parameters
   * https://docs.revrobotics.com/brushless/spark-max/parameters
   *
   * @param builder
   */
  protected MotorConfig(Builder builder) {
    this.inverted = builder.inverted;
    this.currentLimit = builder.currentLimit;
    this.idleMode = builder.idleMode;
  }

  /**
   * This class uses a Builder pattern to construct a RevMotorConfig Each of the three parameters
   * gains separate methods in the Builder. This is a more explicit way of creating a data object
   * with immutable data than using constructors. The parameters can be built by applying 0 or more
   * of the gains values.
   */
  public static class Builder {
    // Gains default to a common and safe application of motor current limiting
    public boolean inverted = false;
    public int currentLimit = 40;
    public IdleMode idleMode = IdleMode.kBrake;

    // Parameters that might be considered here:
    // CanID
    // MotorType
    // SensorType

    /**
     * The directionality of the output power. False - When told to apply positive power, the motor
     * controller output will apply positive power on the positve lead. True - When told to apply
     * positive power, the motor controller output will apply positive power on the negative lead.
     *
     * @param invertered
     * @return this MotorConfig Builder
     */
    public Builder inverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }

    /**
     * The current limit for the motor controller. Learn more about current limits here:
     * https://v6.docs.ctr-electronics.com/en/latest/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     *
     * @param currentLimit The current limit applied to the motor controller. Current limiting
     *     improves performance and protects motors from burning out under stall.
     * @return this MotorConfig Builder
     */
    public Builder currentLimit(int currentLimit) {
      if (currentLimit < 0) throw new IllegalArgumentException();
      this.currentLimit = currentLimit;
      return this;
    }

    /**
     * The motor controller mode when it is not providing motor output.
     *
     * @param idleMode The motor controller mode: Coast or Brake
     * @return this MotorConfig Builder
     */
    public Builder idleMode(IdleMode idleMode) {
      this.idleMode = idleMode;
      return this;
    }

    /**
     * Build an immutable MotorConfig object.
     *
     * @return a new MotorConfig
     */
    public MotorConfig build() {
      return new MotorConfig(this);
    }
  }
}
