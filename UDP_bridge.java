import javax.inject.Inject;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;

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

                double dx = vx * 1000.0 * dt; // mm
                double dy = vy * 1000.0 * dt; // mm
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
            else if (command.equalsIgnoreCase("Set_Arm")) {
                String[] joints = value.split(",");
                if (joints.length == 7) {
                    double j1 = Math.toRadians(Double.parseDouble(joints[0]));
                    double j2 = Math.toRadians(Double.parseDouble(joints[1]));
                    double j3 = Math.toRadians(Double.parseDouble(joints[2]));
                    double j4 = Math.toRadians(Double.parseDouble(joints[3]));
                    double j5 = Math.toRadians(Double.parseDouble(joints[4]));
                    double j6 = Math.toRadians(Double.parseDouble(joints[5]));
                    double j7 = Math.toRadians(Double.parseDouble(joints[6]));

                    executeArmMotion(j1, j2, j3, j4, j5, j6, j7);
                } else {
                    logger.warn("Set_Arm command requires exactly 7 joint values. Received: " + joints.length);
                }
            }
        } catch (Exception e) {
            logger.error("Command parsing error: " + e.getMessage());
        }
    }

    private void executeRelativePlatformMotion(double dx, double dy, double dAlphaDegrees) {
        // Preempt any active platform motion if still running
        if (activePlatformMotionContainer != null && !activePlatformMotionContainer.isFinished()) {
            activePlatformMotionContainer.cancel();
        }

        // Update local open-loop pose tracking
        kmpPose.update(dx, dy, dAlphaDegrees);

        double dx_meters = dx / 1000.0;
        double dy_meters = dy / 1000.0;
        double dAlpha_rad = Math.toRadians(dAlphaDegrees);

        MobilePlatformRelativeMotion relMotion = new MobilePlatformRelativeMotion(dx_meters, dy_meters, dAlpha_rad);
        
        // Non-blocking platform command
        activePlatformMotionContainer = kmp.moveAsync(relMotion);
    }

    private void executeArmMotion(double j1, double j2, double j3, double j4, double j5, double j6, double j7) {
        // Preempt active arm motion if still running
        if (activeArmMotionContainer != null && !activeArmMotionContainer.isFinished()) {
            activeArmMotionContainer.cancel();
        }

        JointPosition targetJoints = new JointPosition(j1, j2, j3, j4, j5, j6, j7);
        PTP ptpMotion = new PTP(targetJoints);

        // Non-blocking LBR PTP motion
        activeArmMotionContainer = lbr.moveAsync(ptpMotion);
    }

    private void sendNativeTelemetry() {
        if (ros2Address == null) return;
        try {
            long currentTime = System.currentTimeMillis();
            txCounter++;

            String posePayload = String.format("%.3f,%.3f,%.3f", kmpPose.x, kmpPose.y, kmpPose.alpha);
            String stateMsg = String.format("%d;0;%d;%s", currentTime, txCounter, posePayload);

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

        if (activePlatformMotionContainer != null && !activePlatformMotionContainer.isFinished()) {
            activePlatformMotionContainer.cancel();
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