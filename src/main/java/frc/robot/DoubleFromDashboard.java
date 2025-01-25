package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.Preferences;

public class DoubleFromDashboard {
  
  private String key;
  private double value;
  private double previousValue;

  private List<Consumer<Double>> callbacks = new ArrayList<Consumer<Double>>();

  public DoubleFromDashboard(String key, double defaultValue) {
    this.key = key;

    if(!Preferences.containsKey(key)) {
      Preferences.initDouble(key, defaultValue);
      this.value = defaultValue;
      this.previousValue = defaultValue;
    } else {
      this.value = Preferences.getDouble(key, defaultValue);
      this.previousValue = value;
    }
  }

  public double get() {
    return this.value;
  }

  public void set(double value) {
    this.previousValue = this.value;
    this.value = value;
    Preferences.setDouble(key, value);
  }

  public void addCallback(Consumer<Double> callback) {
    callbacks.add(callback);
  }

  public void runCallbacks() {
    for(var callback : callbacks) {
      if(previousValue != value) {
        callback.accept(value);
      }
    }
  }
}