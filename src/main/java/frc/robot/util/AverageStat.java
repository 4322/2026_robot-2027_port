package frc.robot.util;

public class AverageStat {
  private double[] samples;
  private int maxSamples;
  private int numSamples = 0;
  private int nextIndex = 0;
  private double runningTotal = 0;

  public AverageStat(int maxSamples) {
    this.maxSamples = maxSamples;
    samples = new double[maxSamples];
  }

  public void addSample(double sample) {
    if (numSamples < maxSamples) {
      samples[nextIndex] = sample;
      runningTotal += sample;
      numSamples++;
    } else {
      runningTotal -= samples[nextIndex];
      samples[nextIndex] = sample;
      runningTotal += sample;
    }
    nextIndex = (nextIndex + 1) % maxSamples;
  }

  public double getAverage() {
    if (numSamples == 0) {
      return 0;
    }
    return runningTotal / numSamples;
  }
}
