package first.robot.subsystems.example;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.subsystems.example.ExampleRollers.Request;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.simulation.SimHooks;

/** Current-based game piece detection against the roller simulation. */
class ExampleRollersTest {
  private static ExampleRollersIOSim io;
  private static ExampleRollers rollers;

  @BeforeAll
  static void setUp() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    io = new ExampleRollersIOSim();
    rollers = new ExampleRollers(io);
  }

  private static void run(Request request, double seconds) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      rollers.periodic();
      rollers.request(request);
      rollers.periodicAfterScheduler();
      SimHooks.stepTiming(0.02);
    }
  }

  @Test
  void detectsAJammedPieceAndClearsOnEject() {
    // Spin-up draws high current at low speed, but is ignored; free rollers are not a piece.
    run(Request.INTAKE, 1.0);
    assertFalse(rollers.hasGamePiece(), "free-spinning rollers");

    io.setPieceInRollers(true);
    run(Request.INTAKE, 0.5);
    assertTrue(rollers.hasGamePiece(), "jammed rollers");

    // Stopping keeps the piece; a short eject does not clear it, a long one does.
    run(Request.STOP, 0.5);
    assertTrue(rollers.hasGamePiece(), "kept while stopped");
    io.setPieceInRollers(false);
    run(Request.EJECT, 0.1);
    assertTrue(rollers.hasGamePiece(), "not cleared by a short eject");
    run(Request.EJECT, 0.3);
    assertFalse(rollers.hasGamePiece(), "cleared by ejecting");
  }

  @Test
  void aPieceAlreadyJammedAtStartIsNotDetectedDuringSpinUp() {
    run(Request.STOP, 0.2);
    io.setPieceInRollers(true);
    run(Request.INTAKE, 0.2);
    assertFalse(rollers.hasGamePiece(), "within the spin-up window");
    run(Request.INTAKE, 0.3);
    assertTrue(rollers.hasGamePiece(), "after spin-up and confirm");
    io.setPieceInRollers(false);
    run(Request.EJECT, 0.5);
  }
}
