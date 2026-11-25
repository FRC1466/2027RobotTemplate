package first.robot.subsystems.example;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.SimHooks;

/**
 * Runs the example pivot against its simulation, since no robot code constructs it. Shows the
 * starter moves to a goal, and why a coordinator checks the requested goal with isNear.
 */
class ExamplePivotTest {
  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
  }

  private static void loop(ExamplePivot pivot) {
    pivot.periodic();
    pivot.periodicAfterScheduler();
    SimHooks.stepTiming(0.02);
  }

  @Test
  void reachesANewGoalAndIsNearOnlyChecksTheRequestedOne() {
    var pivot = new ExamplePivot(new ExamplePivotIOSim());
    for (int i = 0; i < 100; i++) {
      loop(pivot);
    }
    assertTrue(pivot.atGoal(), "holds the stow goal");

    double requested = 0.0;
    // A coordinator requests a new position; the pivot gets it later in the loop.
    assertTrue(pivot.atGoal(), "atGoal still compares against the old goal");
    assertFalse(pivot.isNear(requested), "isNear compares against the new request");

    pivot.setGoal(requested);
    int loops = 0;
    while (!pivot.isNear(requested) && loops < 150) {
      loop(pivot);
      loops++;
    }
    assertTrue(pivot.isNear(requested), "reached the goal within 3 s");
    assertTrue(loops > 5, "the move took time: " + loops + " loops");
  }
}
