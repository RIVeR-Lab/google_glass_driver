package com.riverlab.robotmanager;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Set;

import com.google.android.glass.media.Sounds;
import com.google.glass.logging.Log;
import com.riverlab.robotmanager.bluetooth.ConnectedThread;
import com.riverlab.robotmanager.messages.MessageListActivity;
import com.riverlab.robotmanager.messages.RobotMessage;
import com.riverlab.robotmanager.robot.Robot;
import com.riverlab.robotmanager.voice_recognition.VoiceRecognitionThread;

import android.app.Application;
import android.content.Context;
import android.media.AudioManager;
import android.os.Handler;
import android.os.Message;


public class RobotManagerApplication extends Application
{
	private Handler mMainThreadHandler;
	private Handler mConnectedThreadHandler;
	private Handler mVoiceThreadHandler;
	private HashMap<String, Robot> mRobotMap = new HashMap<String, Robot>();
	private boolean isConnected;
	private Robot robotInFocus;
	private ArrayList<RobotMessage> mMessages = new ArrayList<RobotMessage>();
	private MessageListActivity msgListActivity = null;

	public Handler getMainActivityHandler()
	{
		return mMainThreadHandler;
	}

	public Handler getConnectedThreadHandler()
	{
		return mConnectedThreadHandler;
	}

	public Handler getVoiceThreadHandler()
	{
		return mVoiceThreadHandler;
	}


	public void setMainThreadHandler(Handler mainHandler)
	{
		mMainThreadHandler = mainHandler;
	}

	public void setConnectedThreadHandler(Handler connectedHandler)
	{
		mConnectedThreadHandler = connectedHandler;
	}

	public void setVoiceThreadHandler(Handler voiceHandler)
	{
		mVoiceThreadHandler = voiceHandler;
	}

	public void setMsgListActivity(MessageListActivity mla)
	{
		msgListActivity = mla;
	}

	public boolean getConnectionStatus()
	{
		return isConnected;
	}


	public void setConnectionStatus(boolean isConnected)
	{
		this.isConnected = isConnected;
	}


	public Set<String> getRobotNames()
	{
		return mRobotMap.keySet();
	}

	public Collection<Robot> getRobots()
	{
		return mRobotMap.values();
	}

	public Robot getRobot(String name)
	{
		return mRobotMap.get(name);
	}

	public void addRobot(Robot newRobot)
	{
		Log.i("Application", "In addRobot");
		mRobotMap.put(newRobot.getName(), newRobot);

		Message msg = mVoiceThreadHandler.obtainMessage(VoiceRecognitionThread.ADD_VOCAB_MESSAGE, newRobot.getName());
		mVoiceThreadHandler.sendMessageAtFrontOfQueue(msg);

		Log.i("Application", "Finished addRobot");
	}

	public void removeRobot(Robot robot)
	{
		mRobotMap.remove(robot.getName());

		Message msg = mVoiceThreadHandler.obtainMessage(VoiceRecognitionThread.REMOVE_VOCAB_MESSAGE, robot.getName());
		mVoiceThreadHandler.sendMessageAtFrontOfQueue(msg);
	}

	public Robot getRobotInFocus()
	{
		return robotInFocus;
	}

	public void setRobotInFocus(String robotName)
	{
		if (robotName.equals("All"))
		{
			robotInFocus = null;
		}
		else
		{
			robotInFocus = mRobotMap.get(robotName);
		}	
	}

	public void addMessage(RobotMessage newMsg)
	{
		mMessages.add(newMsg);
		if (msgListActivity != null)
		{
			Runnable task = new Runnable() {
				public void run() {
					msgListActivity.onMessageAddition();
				}
			};
			
			mMainThreadHandler.post(task);
		}
		
		Message msg = mMainThreadHandler.obtainMessage(MainActivity.NEW_MESSAGE_MESSAGE);
		mMainThreadHandler.sendMessage(msg);
		
		AudioManager audio = (AudioManager) getSystemService(Context.AUDIO_SERVICE);
		audio.playSoundEffect(Sounds.SUCCESS);
	}

	public ArrayList<RobotMessage> getMessages()
	{
		return mMessages;
	}

	public void onShutdown()
	{
		mRobotMap = null;
		mMessages = null;
		
		Message msgMain = mMainThreadHandler.obtainMessage();
		msgMain.what = MainActivity.SHUTDOWN_MESSAGE;
		mMainThreadHandler.sendMessageAtFrontOfQueue(msgMain);
		
		Message msgConnected = mConnectedThreadHandler.obtainMessage(ConnectedThread.SHUTDOWN_MESSAGE);
		msgConnected.what = ConnectedThread.SHUTDOWN_MESSAGE;
		mConnectedThreadHandler.sendMessageAtFrontOfQueue(msgConnected);
		
		Message msgVoice = mVoiceThreadHandler.obtainMessage(VoiceRecognitionThread.SHUTDOWN_MESSAGE);
		msgVoice.what = VoiceRecognitionThread.SHUTDOWN_MESSAGE;
		mVoiceThreadHandler.sendMessageAtFrontOfQueue(msgVoice);
	}
}
