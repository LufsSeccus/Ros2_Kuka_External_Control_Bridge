package application;

import javax.inject.Inject;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.util.Locale;

import com.kuka.geometry.Frame;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;
import com.kuka.task.ITaskLogger;

public class UDP_bridge extends RoboticsAPIApplication {

    @Inject private KmpOmniMove kmp;
    @Inject private LBR lbr;
    @Inject private ITaskLogger logger; 

    private DatagramSocket udpSocket;
    private InetAddress ros2Address = null;

    private final int PORT_ROBOT = 30300;  
    private int PORT_CLIENT = 30333; 

    private boolean isRunning = true;
    private boolean isApplicationActive = false;

    // Protocol tracking
    private long lastRxCounter = -1;
    private long txCounter = 0;
    private boolean lastAppStartSignal = false;
    private long lastVelCommandTime = 0;

    // Non-blocking motion containers
    private Thread activePlatformThread = null;
    private ICommandContainer activePlatformMotionContainer = null;
    private IMotionContainer activeArmMotionContainer = null;

    // Open-loop local pose tracking (Dead reckoning without global map requirement)
    private class KmpPose {
        double x = 0;
        double y = 0;
        double alpha = 0;

        void update(double dx, double dy, double dAlpha) {
            double radAlpha = Math.toRadians(alpha);
            x += dx * Math.cos(radAlpha) - dy * Math.sin(radAlpha);
            y += dx * Math.sin(radAlpha) + dy * Math.cos(radAlpha);
            alpha += dAlpha;
        }
    }
    private KmpPose kmpPose = new KmpPose();

    @Override
    public void initialize() {
        try {
            udpSocket = new DatagramSocket(PORT_ROBOT);
            ros2Address = null;     

            logger.info("KMP + LBR UDP Bridge initialized on port " + PORT_ROBOT + ". Awaiting ROS2 handshake...");
        } catch (Exception e) {
            logger.error("Setup failed: " + e.getMessage());
        }
    }

    @Override
    public void run() {
        byte[] receiveBuf = new byte[512];

        while (isRunning) {
            // 1. INBOUND: Listen for ROS2 command strings
            try {
                DatagramPacket receivePacket = new DatagramPacket(receiveBuf, receiveBuf.length);
                udpSocket.receive(receivePacket);

                InetAddress senderAddr = receivePacket.getAddress();
                int senderPort = receivePacket.getPort();
                String rawMsg = new String(receivePacket.getData(), 0, receivePacket.getLength(), "UTF-8").trim();

                String[] parts = rawMsg.split(";");
                if (parts.length >= 4) {
                    long rxTimestamp = Long.parseLong(parts[0]);
                    long rxCounter = Long.parseLong(parts[1]);
                    String command = parts[2].trim();
                    String value = parts[3].trim();

                    // Handshake recovery logic
                    boolean isHandshake = command.equalsIgnoreCase("App_Start");
                    boolean isNewSender = (ros2Address == null) || 
                                          (!ros2Address.equals(senderAddr)) || 
                                          (PORT_CLIENT != senderPort);

                    if (isHandshake || isNewSender) {
                        ros2Address = senderAddr;
                        PORT_CLIENT = senderPort;
                        lastRxCounter = -1;
                        isApplicationActive = false; 
                        logger.info(String.format("ROS2 Session Connected: %s:%d", ros2Address.getHostAddress(), PORT_CLIENT));
                    }

                    // Drop stale out-of-order packets
                    if (rxCounter <= lastRxCounter) {
                        continue; 
                    }
                    lastRxCounter = rxCounter;

                    // Process commands
                    if (command.equalsIgnoreCase("App_Start")) {
                        boolean currentSignal = Boolean.parseBoolean(value);
                        if (!lastAppStartSignal && currentSignal) {
                            logger.info("Core loop activated via App_Start rising edge.");
                            isApplicationActive = true;
                        }
                        lastAppStartSignal = currentSignal;

                    } else if (command.equalsIgnoreCase("Set_Shutdown")) {
                        shutdownBridge(); 

                    } else if (isApplicationActive) {
                        processActiveParameters(command, value);
                    }
                }
            } catch (SocketTimeoutException e) {
                // Allows loop iteration to trigger outbound telemetry smoothly
            } catch (Exception e) {
                logger.error("Socket reading error: " + e.getMessage());
            }

            // 2. OUTBOUND: Send active platform pose telemetry back to ROS2
            if (isApplicationActive) {
                sendNativeTelemetry();
            }
        }
    }

    private void processActiveParameters(String command, String value) {
        try {
            // ==========================================================
            // 1. KMP VELOCITY MODE
            // ==========================================================
            if (command.equalsIgnoreCase("Set_Vel")) {
                String[] vel = value.split(",");
                double vx = Double.parseDouble(vel[0]); 
                double vy = Double.parseDouble(vel[1]); 
                double omega = Double.parseDouble(vel[2]); 

                long currentTime = System.currentTimeMillis();
                if (lastVelCommandTime == 0) {
                    lastVelCommandTime = currentTime;
                    return;
                }

                double dt = (currentTime - lastVelCommandTime) / 1000.0; 
                lastVelCommandTime = currentTime;

                if (dt <= 0 || dt > 0.5) dt = 0.05;

                double dx = vx * dt; // mm
                double dy = vy * dt; // mm
                double dAlphaDegrees = Math.toDegrees(omega * dt);

                executeRelativePlatformMotion(dx, dy, dAlphaDegrees);
            } 

            // ==========================================================
            // 2. KMP POSE P2P MODE
            // ==========================================================
            else if (command.equalsIgnoreCase("Set_Pose")) {
                String[] pose = value.split(",");
                double targetX = Double.parseDouble(pose[0]); 
                double targetY = Double.parseDouble(pose[1]); 
                double targetAlpha = Double.parseDouble(pose[2]); 

                double dX_world = targetX - kmpPose.x;
                double dY_world = targetY - kmpPose.y;
                double dAlphaDegrees = targetAlpha - kmpPose.alpha;

                double radCurr = Math.toRadians(kmpPose.alpha);
                double dx = dX_world * Math.cos(radCurr) + dY_world * Math.sin(radCurr);
                double dy = -dX_world * Math.sin(radCurr) + dY_world * Math.cos(radCurr);

                executeRelativePlatformMotion(dx, dy, dAlphaDegrees);
            }

            // ==========================================================
            // 3. LBR ARM JOINT CONTROL MODE (7 Joint Angles in Degrees)
            // ==========================================================
            else if (command.equalsIgnoreCase("Set_Arm_Joint")) {
                String[] joints = value.split(",");
                if (joints.length == 7) {
                    executeArmJointMotion(
                        Math.toRadians(Double.parseDouble(joints[0])),
                        Math.toRadians(Double.parseDouble(joints[1])),
                        Math.toRadians(Double.parseDouble(joints[2])),
                        Math.toRadians(Double.parseDouble(joints[3])),
                        Math.toRadians(Double.parseDouble(joints[4])),
                        Math.toRadians(Double.parseDouble(joints[5])),
                        Math.toRadians(Double.parseDouble(joints[6]))
                    );
                }
            }

            // 4. LBR ARM CARTESIAN END-EFFECTOR MODE (x, y, z [m], a, b, c [rad])
            else if (command.equalsIgnoreCase("Set_Arm_End_Effector")) {
                String[] pose = value.split(",");
                if (pose.length == 6) {
                    double x_mm = Double.parseDouble(pose[0]) * 1000.0; // convert meters to mm
                    double y_mm = Double.parseDouble(pose[1]) * 1000.0;
                    double z_mm = Double.parseDouble(pose[2]) * 1000.0;
                    double a_rad = Double.parseDouble(pose[3]); // KUKA A (Z rotation)
                    double b_rad = Double.parseDouble(pose[4]); // KUKA B (Y rotation)
                    double c_rad = Double.parseDouble(pose[5]); // KUKA C (X rotation)

                    executeCartesianArmMotion(x_mm, y_mm, z_mm, a_rad, b_rad, c_rad);
                }
            }
        } catch (Exception e) {
            logger.error("Command parsing error: " + e.getMessage());
        }
    }

    private void executeRelativePlatformMotion(double dx, double dy, double dAlphaDegrees) {
        // Preempt any active platform motion if still running
        if (activePlatformThread != null && activePlatformThread.isAlive()) {
        activePlatformThread.interrupt(); // This aborts the blocking KUKA move()
        }
        // Update local open-loop pose tracking
        kmpPose.update(dx, dy, dAlphaDegrees);

        double dAlpha_rad = Math.toRadians(dAlphaDegrees);

        final MobilePlatformRelativeMotion relMotion = new MobilePlatformRelativeMotion(dx, dy, dAlpha_rad);
        
        // Non-blocking platform command
        activePlatformThread = new Thread(new Runnable() {
        @Override
        public void run() {
            try {
                kmp.move(relMotion);
            } catch (Exception e) {
                // Thread interrupted or motion preempted, safely ignored
            }
        }
    }
    }

    private void executeArmJointMotion(double j1, double j2, double j3, double j4, double j5, double j6, double j7) {
        // Preempt active arm motion if still running
        if (activeArmMotionContainer != null && !activeArmMotionContainer.isFinished()) {
            logger.info("[Set_Arm] Preempting active arm motion for new command.");
            activeArmMotionContainer.cancel();
        }
        try {
        // 2. Fetch current joint positions for comparison
        JointPosition currentJoints = lbr.getCurrentJointPosition();
        
        String currentPosStr = String.format(java.util.Locale.US,
            "Current Pos (deg) -> J1: %.2f | J2: %.2f | J3: %.2f | J4: %.2f | J5: %.2f | J6: %.2f | J7: %.2f",
            Math.toDegrees(currentJoints.get(0)),
            Math.toDegrees(currentJoints.get(1)),
            Math.toDegrees(currentJoints.get(2)),
            Math.toDegrees(currentJoints.get(3)),
            Math.toDegrees(currentJoints.get(4)),
            Math.toDegrees(currentJoints.get(5)),
            Math.toDegrees(currentJoints.get(6))
        );

        // 3. Format target joint positions in degrees
        String targetPosStr = String.format(java.util.Locale.US,
            "Target  Pos (deg) -> J1: %.2f | J2: %.2f | J3: %.2f | J4: %.2f | J5: %.2f | J6: %.2f | J7: %.2f",
            Math.toDegrees(j1), Math.toDegrees(j2), Math.toDegrees(j3),
            Math.toDegrees(j4), Math.toDegrees(j5), Math.toDegrees(j6), Math.toDegrees(j7)
        );

        // 4. Output structured logs to smartPAD Task Logger
        logger.info("=== [LBR Arm Motion Triggered] ===");
        logger.info(currentPosStr);
        logger.info(targetPosStr);

    } catch (Exception e) {
        logger.warn("[Set_Arm] Could not read current position for logging: " + e.getMessage());
    }

        JointPosition targetJoints = new JointPosition(j1, j2, j3, j4, j5, j6, j7);
        PTP ptpMotion = new PTP(targetJoints);

        // Non-blocking LBR PTP motion
        activeArmMotionContainer = lbr.moveAsync(ptpMotion);
    }

    private void executeCartesianArmMotion(double x_mm, double y_mm, double z_mm, double a_rad, double b_rad, double c_rad) {
        // Preempt active arm motion if still running
        if (activeArmMotionContainer != null && !activeArmMotionContainer.isFinished()) {
            logger.info("[Set_Arm_EE] Preempting active arm motion for new command.");
            activeArmMotionContainer.cancel();
        }

        try {
            // Fetch current Cartesian position for comparison
            com.kuka.roboticsAPI.geometricModel.Frame currentFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());

            String currentCartStr = String.format(Locale.US,
                "Current EE -> X: %.2f mm | Y: %.2f mm | Z: %.2f mm | A: %.2f deg | B: %.2f deg | C: %.2f deg",
                currentFrame.getX(),
                currentFrame.getY(),
                currentFrame.getZ(),
                Math.toDegrees(currentFrame.getAlphaRad()),
                Math.toDegrees(currentFrame.getBetaRad()),
                Math.toDegrees(currentFrame.getGammaRad())
            );

            String targetCartStr = String.format(Locale.US,
                "Target  EE -> X: %.2f mm | Y: %.2f mm | Z: %.2f mm | A: %.2f deg | B: %.2f deg | C: %.2f deg",
                x_mm, y_mm, z_mm,
                Math.toDegrees(a_rad),
                Math.toDegrees(b_rad),
                Math.toDegrees(c_rad)
            );

            logger.info("=== [LBR Arm Cartesian Motion Triggered] ===");
            logger.info(currentCartStr);
            logger.info(targetCartStr);

        } catch (Exception e) {
            logger.warn("[Set_Arm_EE] Could not read current Cartesian frame for logging: " + e.getMessage());
        }

     // Build target Frame relative to LBR Root Frame
        com.kuka.roboticsAPI.geometricModel.Frame targetFrame = 
            new com.kuka.roboticsAPI.geometricModel.Frame(lbr.getRootFrame(), x_mm, y_mm, z_mm, a_rad, b_rad, c_rad);

        // Non-blocking LBR PTP motion using BasicMotions helper
        activeArmMotionContainer = lbr.moveAsync(com.kuka.roboticsAPI.motionModel.BasicMotions.ptp(targetFrame));
    }

    private void sendNativeTelemetry() {
        if (ros2Address == null) return;
        try {
            long currentTime = System.currentTimeMillis();
            txCounter++;

            // 1. KMP Base Pose Payload (X, Y, Alpha)
            String posePayload = String.format(java.util.Locale.US, "%.3f,%.3f,%.3f", kmpPose.x, kmpPose.y, kmpPose.alpha);

            // 2. Read LBR Arm Joint Positions (Returns values in radians, converted to degrees)
            JointPosition currentJoints = lbr.getCurrentJointPosition();
            StringBuilder armPayloadBuilder = new StringBuilder();
            for (int i = 0; i < 7; i++) {
                double deg = Math.toDegrees(currentJoints.get(i));
                armPayloadBuilder.append(String.format(java.util.Locale.US, "%.2f", deg));
                if (i < 6) {
                    armPayloadBuilder.append(",");
                }
            }
            String armPayload = armPayloadBuilder.toString();

            // 3. Format complete packet: Timestamp;ErrorCode;Counter;BasePose;ArmPose
            String stateMsg = String.format(java.util.Locale.US, "%d;0;%d;%s;%s", 
                    currentTime, txCounter, posePayload, armPayload);

            byte[] data = stateMsg.getBytes("UTF-8");
            DatagramPacket packet = new DatagramPacket(data, data.length, ros2Address, PORT_CLIENT);
            udpSocket.send(packet);
        } catch (Exception e) {
            logger.error("Outbound telemetry error: " + e.getMessage());
        }
    }

    private void shutdownBridge() {
        logger.warn("Stopping KMP/LBR application loop.");
        isRunning = false;
        isApplicationActive = false;

        if (activePlatformThread != null && activePlatformThread.isAlive()) {
        activePlatformThread.interrupt();
    }
        if (activeArmMotionContainer != null && !activeArmMotionContainer.isFinished()) {
            activeArmMotionContainer.cancel();
        }

        ros2Address = null;
        lastRxCounter = -1;
    }

    @Override
    public void dispose() {
        shutdownBridge();
        if (udpSocket != null && !udpSocket.isClosed()) {
            udpSocket.close();
        }
        super.dispose();
    }
}