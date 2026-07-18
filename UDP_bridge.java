package application;

import javax.inject.Inject;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketTimeoutException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;
import com.kuka.task.ITaskLogger;

public class UDP_bridge extends RoboticsAPIApplication {
    //@Inject private Controller kukaController;
    //@Inject private LBR lbr;
    @Inject private KmpOmniMove kmp;
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

    // Simple Background Worker (Keeps exactly 1 thread alive background-side)
    private ExecutorService motionExecutor;
    private Future<?> activeMotionTask = null;

    // Inner class tracking coordinates open-loop (From your masterarbeit_stachon package)
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
            //udpSocket.setSoTimeout(10); 
            ros2Address = null; 	
            
            motionExecutor = Executors.newSingleThreadExecutor();

            logger.info("KMP Simple UDP Bridge initialized on port " + PORT_ROBOT + ". Awaiting ROS2 handshake...");
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

                    // Handshake recovery logic if ROS2 node restarts or changes dynamic ports
                    boolean isHandshake = command.equalsIgnoreCase("App_Start");
                    boolean isNewSender = (ros2Address == null) || 
                                          (!ros2Address.equals(senderAddr)) || 
                                          (PORT_CLIENT != senderPort);

                    if (isHandshake || isNewSender) {
                        ros2Address = senderAddr;
                        PORT_CLIENT = senderPort;
                        lastRxCounter = -1; // Clear counter lockout
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
                // Heartbeat/timeout step to let outbound telemetry stream smoothly
            } catch (Exception e) {
                logger.error("Socket reading error: " + e.getMessage());
            }

            // 2. OUTBOUND: Send active pose telemetry back to ROS2
            if (isApplicationActive) {
                sendNativeTelemetry();
            }
        }
    }

    private void processActiveParameters(String command, String value) {
        try {
            // ==========================================================
            // 1. VELOCITY EMULATION MODE
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

                if (dt <= 0 || dt > 0.5) dt = 0.05; // Fallback step

                double dx = vx * 1000.0 * dt;
                double dy = vy * 1000.0 * dt;
                double dAlphaDegrees = Math.toDegrees(omega * dt);

                executeMotionAsync(dx, dy, dAlphaDegrees);
            } 
            
            // ==========================================================
            // 2. POSE P2P MODE
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
                
                executeMotionAsync(dx, dy, dAlphaDegrees);
            } 
        } catch (Exception e) {
            logger.error("Command parsing error: " + e.getMessage());
        }
    }

    private void executeMotionAsync(final double dx, final double dy, final double dAlphaDegrees) {
        // 1. Cancel the active background task if it's currently running a previous command
        if (activeMotionTask != null && !activeMotionTask.isDone()) {
            activeMotionTask.cancel(true); // Safely sends an interrupt signal to the underlying motion call
        }

        // 2. Keep your open-loop mathematical coordinate updates exactly as originally written
        kmpPose.update(dx, dy, dAlphaDegrees);

        // 3. Submit the job to our single background thread instance
        activeMotionTask = motionExecutor.submit(new Runnable() {
            @Override
            public void run() {
                try {
                    double dx_meters = dx / 1000.0;
                    double dy_meters = dy / 1000.0;
                    
                    // Native blocking call operates safely inside our single pre-allocated worker thread
                    kmp.move(new MobilePlatformRelativeMotion(dx_meters, dy_meters, Math.toRadians(dAlphaDegrees)));
                } catch (Exception e) {
                    // Catches standard expected interruptions when a new twist command comes in early
                }
            }
        });
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
        logger.warn("Stopping KMP application loop.");
        isRunning = false;
        isApplicationActive = false;
        
        if (activeMotionTask != null) {
            activeMotionTask.cancel(true);
        }
        if (motionExecutor != null) {
            motionExecutor.shutdownNow(); // Clean up OS thread pool allocation
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