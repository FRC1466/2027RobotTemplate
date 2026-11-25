package first.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.subsystems.Choreographer.Source;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;

/** Mechanism requests only change while their source owns the robot's state. */
class ChoreographerTest {
  /** One instance: alerts can only be created once per process. */
  private static Choreographer robot;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize());
    // These checks never reach the drive. A second Drive would reallocate its alerts, which
    // fails when another test class in this JVM already built one.
    robot = new Choreographer(null);
  }

  private static void autonomous() {
    robot.periodic(true, false, false, true);
  }

  private static void teleop() {
    robot.periodic(false, true, false, true);
  }

  private static void disabled() {
    robot.periodic(false, false, false, true);
  }

  @Test
  void onlyTheOwnerCanChangeARequest() {
    disabled();
    var goal = robot.held(0.0);

    autonomous();
    assertFalse(goal.request(Source.DRIVER, 1.0), "driver refused in autonomous");
    assertEquals(0.0, goal.get());
    assertTrue(goal.request(Source.AUTO, 2.0));
    assertEquals(2.0, goal.get());

    teleop();
    assertFalse(goal.request(Source.AUTO, 3.0), "autonomous refused in teleop");
    assertTrue(goal.request(Source.DRIVER, 4.0));

    disabled();
    assertFalse(goal.request(Source.DRIVER, 5.0), "nobody owns a disabled robot");
    assertFalse(goal.request(Source.AUTO, 5.0));
    assertEquals(4.0, goal.get(), "a held request keeps its value");
  }

  @Test
  void latchedRequestsStopWhenTheOwnerChanges() {
    disabled();
    var rollers = robot.latched("STOP");
    var goal = robot.held(0.0);

    autonomous();
    assertTrue(rollers.request(Source.AUTO, "INTAKE"));
    assertTrue(goal.request(Source.AUTO, 1.0));
    autonomous();
    assertEquals("INTAKE", rollers.get(), "stays latched while the owner is unchanged");

    teleop();
    assertEquals("STOP", rollers.get(), "autonomous rollers do not run into teleop");
    assertEquals(1.0, goal.get(), "the position goal is kept");
  }
}
