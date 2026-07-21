package application;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;
import com.kuka.task.ITaskLogger;

public class TestKmpMoveAsync extends RoboticsAPIApplication {

    @Inject private KmpOmniMove kmp;
    @Inject private ITaskLogger logger;

    // Track open-loop pose (X/Y in meters, Alpha in degrees)
    private double posX = 0.0;
    private double posY = 0.0;
    private double posAlphaDeg = 0.0;

    @Override
    public void initialize() {
        logger.info(String.format("TestKmpMoveAsync initialized. Initial Pose -> [X: %.3fm, Y: %.3fm, Alpha: %.2f°]", 
                posX, posY, posAlphaDeg));
    }

    @Override
    public void run() {
        logger.info("Starting relative moveAsync test...");

        // Define relative displacements
        double dxMeters = 0.5;      // 0.5m forward
        double dyMeters = 0.0;      // 0.0m lateral
        double dAlphaDeg = 15.0;    // 15° turn
        double dAlphaRad = Math.toRadians(dAlphaDeg);

        // Update target position estimate
        updatePose(dxMeters, dyMeters, dAlphaDeg);

        MobilePlatformRelativeMotion relMotion = 
                new MobilePlatformRelativeMotion(dxMeters, dyMeters, dAlphaRad);

        // 1. Dispatch non-blocking command
        long startTime = System.currentTimeMillis();
        ICommandContainer commandContainer = kmp.moveAsync(relMotion);
        long elapsedTime = System.currentTimeMillis() - startTime;

        // 2. Log non-blocking return time and target pose
        logger.info(String.format("kmp.moveAsync() called in %d ms (Non-blocking).", elapsedTime));
        logger.info(String.format("Target Pose -> [X: %.3fm, Y: %.3fm, Alpha: %.2f°]", posX, posY, posAlphaDeg));

        // 3. Monitor execution in background
        while (!commandContainer.isFinished()) {
            logger.info(String.format("Moving... Estimated Pose -> [X: %.3fm, Y: %.3fm, Alpha: %.2f°]", 
                    posX, posY, posAlphaDeg));
            try {
                Thread.sleep(500); // Poll status every 500ms
            } catch (InterruptedException e) {
                logger.warn("Monitoring sleep interrupted.");
                break;
            }
        }

        logger.info(String.format("Relative motion finished! Final Pose -> [X: %.3fm, Y: %.3fm, Alpha: %.2f°]", 
                posX, posY, posAlphaDeg));
    }

    private void updatePose(double dx, double dy, double dAlpha) {
        double radAlpha = Math.toRadians(posAlphaDeg);
        posX += dx * Math.cos(radAlpha) - dy * Math.sin(radAlpha);
        posY += dx * Math.sin(radAlpha) + dy * Math.cos(radAlpha);
        posAlphaDeg += dAlpha;
    }

    @Override
    public void dispose() {
        super.dispose();
    }
}