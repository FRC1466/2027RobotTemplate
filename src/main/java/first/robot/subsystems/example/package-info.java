/**
 * Starter pattern for a closed-loop position mechanism, shown as a two-motor pivot. Copy the
 * package, rename it for the real mechanism, and replace the constants with CAD or measured values.
 * {@code RobotContainer} does not construct it, so it does not run until a robot adds it.
 *
 * <ul>
 *   <li>{@link first.robot.subsystems.example.ExamplePivotIO} is the hardware boundary. Inputs are
 *       in mechanism units, per motor where there are several, and include the profiled setpoint
 *       the controller is tracking this loop.
 *   <li>{@link first.robot.subsystems.example.ExamplePivotIOSim} is a physics model with an onboard
 *       trapezoid profile standing in for Motion Magic. It checks sequencing and logging, not real
 *       gains.
 *   <li>{@link first.robot.subsystems.example.ExamplePivot} holds the goal and applies it in {@code
 *       periodicAfterScheduler()}, after commands and interlocks run. It raises an alert per motor
 *       for disconnect and overtemperature, reports a goal that is not reached in time through
 *       {@link first.robot.util.GoalTimeout}, and logs goal error, setpoint-vs-measured tracking
 *       error, time on goal, and current under {@code Diagnostics/ExamplePivot/}.
 * </ul>
 *
 * <p>Tracking error (profiled setpoint minus measured) shows how well the controller follows the
 * profile. A large tracking error during a move points at gains or feedforward. Goal error (final
 * goal minus measured) is large during every move and only means something once the profile ends.
 *
 * <p>Checking "at position" from a coordinator: compare against the goal that was just requested,
 * with {@link first.robot.subsystems.example.ExamplePivot#isNear(double)}, not against {@code
 * atGoal()}. A coordinator usually passes its requested goal to the mechanism later in the loop, so
 * right after a new request {@code atGoal()} still compares against last loop's goal, which the
 * mechanism may already be holding. A routine that requests a new position and then waits until
 * {@code atGoal()} would continue immediately, before the mechanism moved.
 *
 * <pre>{@code
 * public boolean atPosition() {
 *   return pivot.isNear(setpoints.get(requestedPosition));
 * }
 * }</pre>
 */
package first.robot.subsystems.example;
