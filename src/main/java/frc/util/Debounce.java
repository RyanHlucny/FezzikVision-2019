package frc.util;

/**
 * Class used to debounce boolean signals
 */
public class Debounce {
  private boolean m_lastState = false;
  private boolean m_debouncedState = false;
  private int m_cycleCount = 0;
  private int m_cycles;

  /**
   * Constructs a new Debounce object
   * @param cycles The number of cycles to wait for the signal to settle. Since the
   * loop period on the robot is 20ms, Each cycle will add 20ms onto the debounce timer.
   */
  public Debounce(int cycles) {
    m_cycles = cycles;
  }

  public boolean debounce(boolean input) {
    // If input is the same as previous state, increment cycle counter
    if (input == m_lastState) {
      m_cycleCount += 1;
    }
    // Otherwise, reset cycle counter
    else {
      m_cycleCount = 0;
    }
    m_lastState = input;

    // If input has been stable long enough, set debounced state to the input
    if (m_cycleCount >= m_cycles) {
      m_debouncedState = input;
    }

    return m_debouncedState;
  }

  public boolean debounce(boolean input, boolean desiredState) {
    // If input is the same as previous state, increment cycle counter
    if (input == desiredState) {
      m_cycleCount += 1;
    }
    // Otherwise, reset cycle counter
    else {
      m_cycleCount = 0;
    }

    // If input has been stable long enough, set debounced state to the input
    if (m_cycleCount >= m_cycles) {
      m_debouncedState = input;
    }
    // Otherwise set m_debouncedState to false
    else {
      m_debouncedState = false;
    }

    return m_debouncedState;
  }
}