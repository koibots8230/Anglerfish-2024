package frc.lib.util;

public class FeedforwardGains {
  // These are public properties which allow them to be accessed directly without need for get/set
  // methods.
  // final means that they are immutable after object creation, attempting to change them will cause
  // a compile error.
  public final double ks;
  public final double kv;
  public final double ka;
  public final double kg;

  /**
   * FeedforwardGains constructor is protected so that it can only be constructed through the use of
   * the Builder
   *
   * @param builder
   */
  protected FeedforwardGains(Builder builder) {
    this.ks = builder.ks;
    this.kv = builder.kv;
    this.ka = builder.ka;
    this.kg = builder.kg;
  }

  /**
   * This class uses a Builder pattern to construct a FeedforwardGains Each of the four feedforward
   * gains have separate methods in the Builder. This is a more explicit way of creating a data
   * object with immutable data than using constructors. The FeedforwardGains can be built by
   * applying 0 or more of the gains values.
   */
  public static class Builder {
    // Gains default to 0.0, so only gains with a positive value need to be built
    public double ks = 0.0;
    public double kv = 0.0;
    public double ka = 0.0;
    public double kg = 0.0;

    /**
     * The gain needed to overcome static friction.
     *
     * @param ks The static gain. Must be greater than or equal to 0.0.
     * @return this FeedforwardGains Builder
     */
    public Builder ks(double ks) {
      if (ks < 0.0) throw new IllegalArgumentException();
      this.ks = ks;
      return this;
    }

    /**
     * The gain needed to achieve the desired velocity.
     *
     * @param kv The velocity gain. Must be greater than or equal to 0.0.
     * @return this FeedforwardGains Builder
     */
    public Builder kv(double kv) {
      if (kv < 0.0) throw new IllegalArgumentException();
      this.kv = kv;
      return this;
    }

    /**
     * The gain need to achieve the desired acceleration
     *
     * @param ka The acceleration gain. Must be greater than or equal to 0.0.
     * @return this FeedforwardGains Builder
     */
    public Builder ka(double ka) {
      if (ka < 0.0) throw new IllegalArgumentException();
      this.ka = ka;
      return this;
    }

    /**
     * The gain needed to overcome gravity.
     *
     * @param kg The gravity gain. Must be greater than or equal to 0.0.
     * @return this FeedforwardGains Builder
     */
    public Builder kg(double kg) {
      if (kg < 0.0) throw new IllegalArgumentException();
      this.kg = kg;
      return this;
    }

    /**
     * Build an immutable FeedforwardGains object.
     *
     * @return a new FeedforwardGains
     */
    public FeedforwardGains build() {
      return new FeedforwardGains(this);
    }
  }
}
