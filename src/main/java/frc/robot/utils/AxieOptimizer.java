package frc.robot.utils;

public class AxieOptimizer {
  private final double weight;
  private double c, t;
  
  public AxieOptimizer(double weight) {
    this.weight = weight;
    c = 0; t = 0;
  }

  public double get(double value) {
    t = value;
    double err = t - c;
    if(Math.abs(err) < 0.05)
      c = t;
    else 
      c += err * weight;
    return c;
  }

  public void reset() {
    c = 0;
    t = 0;
  }
}
