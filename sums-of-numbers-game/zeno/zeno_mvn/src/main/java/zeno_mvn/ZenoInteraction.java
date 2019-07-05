package zeno_mvn;

import java.net.Socket;
import org.mechio.api.animation.Animation;
import org.mechio.api.animation.messaging.RemoteAnimationPlayerClient;
import org.mechio.api.animation.player.AnimationJob;
import org.mechio.api.motion.Robot.RobotPositionMap;
import org.mechio.api.motion.messaging.RemoteRobot;
import org.mechio.api.speech.messaging.RemoteSpeechServiceClient;
import org.mechio.client.basic.MechIO;
import org.mechio.client.basic.UserSettings;
import java.nio.ByteBuffer;
import java.io.DataInputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.io.BufferedReader;
import org.json.JSONObject;

import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.jflux.api.common.rk.position.NormalizedDouble;
import org.mechio.api.animation.Animation;
import org.mechio.api.animation.messaging.RemoteAnimationPlayerClient;

import org.mechio.api.motion.Robot;
import org.mechio.api.motion.messaging.RemoteRobot;
import org.mechio.api.speech.SpeechJob;
import org.mechio.api.speech.messaging.RemoteSpeechServiceClient;
import org.mechio.api.speech.utils.DefaultSpeechJob;
import org.mechio.client.basic.MechIO;
import org.mechio.client.basic.UserSettings;
import org.mechio.client.basic.R25RobotJoints;
import org.mechio.api.motion.Joint;

public class ZenoInteraction {

    private static RemoteRobot myRobot;
    private static RemoteAnimationPlayerClient myPlayer;
    private static RemoteSpeechServiceClient mySpeaker;
    private static RobotPositionMap myGoalPositions;
    private static Socket socket;
    
    private static Robot.JointId left_shoulder_yaw;
    private static Robot.JointId left_shoulder_roll;
    private static Robot.JointId right_shoulder_yaw;
    private static Robot.JointId right_shoulder_roll;
    private static Robot.JointId waist;
    private static Robot.JointId left_wrist;
    private static Robot.JointId right_wrist;

    private static DefaultSpeechJob currentSpeechJob = null;
    private static Animation waveAnim;
    private static PrintStream robotOut;
    private static SimpleDateFormat dateFormat;

    public static void main(String[] args) {
        long animLen;

        // If running on a robot instead of an avatar:
        //     uncomment the next five lines and change the IP to the robot's IP
        String ipAddress = "192.168.0.112";
        UserSettings.setRobotAddress(ipAddress);
        UserSettings.setSpeechAddress(ipAddress);
        UserSettings.setAnimationAddress(ipAddress);
        UserSettings.setRobotId("myRobot");
        dateFormat = new SimpleDateFormat("yyyy-MM-dd HH-mm-ss.SSS");

        waveAnim = MechIO.loadAnimation("IROS_animations/wave-anim.xml");
        mySpeaker = MechIO.connectSpeechService();
        myPlayer = MechIO.connectAnimationPlayer();
        myRobot = MechIO.connectRobot();
        myGoalPositions = new org.mechio.api.motion.Robot.RobotPositionHashMap();
        myGoalPositions = myRobot.getDefaultPositions();
//        myRobot.move(myGoalPositions, 1000);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            Logger.getLogger("zenorobot").log(Level.SEVERE, null, ex);
        }
        myGoalPositions.clear();
        
        Animation introAnim;
        AnimationJob introJob;

        String hostName = "192.168.0.127";

        int portNumber = 1932;
        String inputFromBroker = null;
        String[] inputSplit;

        String event = "{ \"class\": \"iristk.system.Event\", \"event_name\": \"athena.zeno.behavior.done\"}\n";


        try {
            Socket echoSocket = new Socket(hostName, portNumber);
            PrintWriter out =
                new PrintWriter(echoSocket.getOutputStream(), true);
            BufferedReader in =
                new BufferedReader(
                    new InputStreamReader(echoSocket.getInputStream()));
            out.println("CONNECT furhat zeno_system\n");
            inputFromBroker = in.readLine();
            out.println("SUBSCRIBE athena.zeno.behavior\n");

            while (true) {
                inputSplit = in.readLine().split(" ");
                if (inputSplit[0].equals("EVENT")) {
                    if (inputSplit[1].equals("athena.zeno.behavior")) {
                        inputFromBroker = in.readLine();
                        JSONObject jsonObj = new JSONObject(inputFromBroker);
                        if (jsonObj.getString("name").equals("IROS_animations/happiness.anim.xml")) {
                            // play the animation defined in the xml file
                            introAnim
                                    = MechIO.loadAnimation(jsonObj.getString("name"));
                            introJob = myPlayer.playAnimation(introAnim);
                            animLen = introAnim.getLength();
                        MechIO.sleep(500 + animLen);
//                        out.println(String.format("EVENT athena.zeno.behavior.done %s\n",event.length()));
//                        out.println(event);
                        }
                        else if (jsonObj.getString("name").equals("IROS_animations/sadness.anim.xml")) {
                            // play the animation defined in the xml file
                            introAnim
                                    = MechIO.loadAnimation(jsonObj.getString("name"));
                            introJob = myPlayer.playAnimation(introAnim);
                            animLen = introAnim.getLength();
                        MechIO.sleep(500 + animLen);
//                        out.println(String.format("EVENT athena.zeno.behavior.done %s\n",event.length()));
//                        out.println(event);
                        }
                        else if (jsonObj.getString("name").equals("IROS_animations/neutral.anim.xml")) {
                            // play the animation defined in the xml file
                            introAnim
                                    = MechIO.loadAnimation(jsonObj.getString("name"));
                            introJob = myPlayer.playAnimation(introAnim);
                            animLen = introAnim.getLength();
                        MechIO.sleep(500 + animLen);
//                        out.println(String.format("EVENT athena.zeno.behavior.done %s\n",event.length()));
//                        out.println(event);
                        }
                        else if (jsonObj.getString("name").equals("IROS_animations/FitInBox.anim.xml")) {
                            // play the animation defined in the xml file
                            introAnim
                                    = MechIO.loadAnimation(jsonObj.getString("name"));
                            introJob = myPlayer.playAnimation(introAnim);
                            animLen = introAnim.getLength();
                        MechIO.sleep(500 + animLen);
                        myGoalPositions.clear();
                        waist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.WAIST));
                        myGoalPositions.put(waist, new NormalizedDouble(0.5));
                        myRobot.move(myGoalPositions, 200);
//                        out.println(String.format("EVENT athena.zeno.behavior.done %s\n",event.length()));
//                        out.println(event);
                        }
                        else if (jsonObj.getString("name").equals("zeno_select_card_0")) {
                            myGoalPositions.clear();
                            right_shoulder_roll = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.RIGHT_SHOULDER_ROLL));
                            right_shoulder_yaw = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.RIGHT_SHOULDER_YAW));
                            waist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.WAIST));
                            right_wrist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.RIGHT_WRIST_YAW));
                            myGoalPositions.put(right_shoulder_roll, new NormalizedDouble(0.7));
                            myGoalPositions.put(right_shoulder_yaw, new NormalizedDouble(0.5));
                            myGoalPositions.put(waist, new NormalizedDouble(0.7));
                            myGoalPositions.put(right_wrist, new NormalizedDouble(0.0));
                            myRobot.move(myGoalPositions, 200);
                        }
                        else if (jsonObj.getString("name").equals("zeno_select_card_1")) {
                            myGoalPositions.clear();
                            right_shoulder_roll = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.RIGHT_SHOULDER_ROLL));
                            right_shoulder_yaw = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.RIGHT_SHOULDER_YAW));
                            waist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.WAIST));
                            right_wrist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.RIGHT_WRIST_YAW));
                            myGoalPositions.put(right_shoulder_roll, new NormalizedDouble(0.7));
                            myGoalPositions.put(right_shoulder_yaw, new NormalizedDouble(0.5));
                            myGoalPositions.put(waist, new NormalizedDouble(0.6));
                            myGoalPositions.put(right_wrist, new NormalizedDouble(0.0));
                            myRobot.move(myGoalPositions, 200);
                        }
                        else if (jsonObj.getString("name").equals("zeno_select_card_2")) {
                            myGoalPositions.clear();
                            left_shoulder_roll = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.LEFT_SHOULDER_ROLL));
                            left_shoulder_yaw = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.LEFT_SHOULDER_YAW));
                            left_wrist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.LEFT_WRIST_YAW));
                            myGoalPositions.put(left_shoulder_roll, new NormalizedDouble(0.55));
                            myGoalPositions.put(left_shoulder_yaw, new NormalizedDouble(0.4));
                            myGoalPositions.put(left_wrist, new NormalizedDouble(0.0));
                            myRobot.move(myGoalPositions, 200);
                        }
                        else if (jsonObj.getString("name").equals("zeno_select_card_3")) {
                            myGoalPositions.clear();
                            left_shoulder_roll = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.LEFT_SHOULDER_ROLL));
                            left_shoulder_yaw = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.LEFT_SHOULDER_YAW));
                            waist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.WAIST));
                            myGoalPositions.put(left_shoulder_roll, new NormalizedDouble(0.7));
                            myGoalPositions.put(left_shoulder_yaw, new NormalizedDouble(0.5));
                            myGoalPositions.put(waist, new NormalizedDouble(0.4));
                            myRobot.move(myGoalPositions, 200);
                        }
                        else if (jsonObj.getString("name").equals("zeno_select_card_4")) {
                            myGoalPositions.clear();
                            left_shoulder_roll = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.LEFT_SHOULDER_ROLL));
                            left_shoulder_yaw = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.LEFT_SHOULDER_YAW));
                            waist = new Robot.JointId(myRobot.getRobotId(), new Joint.Id(R25RobotJoints.WAIST));
                            myGoalPositions.put(left_shoulder_roll, new NormalizedDouble(0.7));
                            myGoalPositions.put(left_shoulder_yaw, new NormalizedDouble(0.5));
                            myGoalPositions.put(waist, new NormalizedDouble(0.3));
                            myRobot.move(myGoalPositions, 200);
                        }
                        else {
                            
                        }
                    }
                }
            }
        }
        catch (Exception e) {
                    e.printStackTrace();
                    
                } finally {
                    try {
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }

        MechIO.disconnect();
        System.exit(0);
    }
}
