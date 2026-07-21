package application;

import javax.inject.Inject;
import com.kuka.nav.rel.RelativeMotion;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.nav.robot.MobileRobotManager;
import com.kuka.nav.task.NavTaskCategory;
import com.kuka.resource.locking.LockException;
import com.kuka.task.ITaskLogger;
import com.kuka.task.RoboticsAPITask;

@NavTaskCategory
public class KMP_Bewegung_Relative_Motion_Test extends RoboticsAPITask {

    @Inject private ITaskLogger _log;
    @Inject private MobileRobotManager _robMan;

    private MobileRobot _rob;
    private int _robotId = 1;

    @Override
    public void initialize() throws Exception {
        _rob = _robMan.getRobot(_robotId);
        _log.info("TestRelativeMotion initialized for Robot ID: " + _robotId);
    }

    @Override
    public void run() throws Exception {
        _log.info("Starting Relative Motion Test...");

        try {
            // Lock robot for exclusive execution
            _rob.lock();

            // Motion parameters (in robot frame):
            double x = 0.5;                       // Move 0.5 meters forward
            double y = 0.0;                       // 0 meters sideways
            double theta = Math.toRadians(15.0);  // Rotate 15 degrees counter-clockwise

            // Instantiate RelativeMotion
            RelativeMotion relMo = new RelativeMotion(x, y, theta);

            _log.info(String.format("Executing relative move: X=%.2fm, Y=%.2fm, Theta=%.1f deg", 
                      x, y, Math.toDegrees(theta)));

            // Execute motion using default speed profile
            _rob.execute(relMo);

            _log.info("Relative motion completed successfully!");

        } catch (LockException e) {
            _log.error("Robot is already locked by another process.", e);
        } catch (InterruptedException e) {
            _log.error("Motion execution was interrupted.", e);
        } finally {
            _rob.unlock();
        }
    }

    @Override
    public void dispose() throws Exception {
        _log.info("Dispose finished.");
    }
}