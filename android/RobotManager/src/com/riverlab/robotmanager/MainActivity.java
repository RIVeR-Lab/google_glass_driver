package com.riverlab.robotmanager;

import android.app.Activity;
import android.app.PendingIntent;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.content.res.Resources;
import android.media.AudioManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.Parcelable;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.RemoteViews;
import android.widget.TextView;

import com.google.android.glass.media.Sounds;
import com.google.android.glass.timeline.LiveCard;
import com.google.android.glass.touchpad.Gesture;
import com.google.android.glass.touchpad.GestureDetector;
import com.google.glass.input.VoiceInputHelper;
import com.google.glass.input.VoiceListener;
import com.google.glass.logging.FormattingLogger;
import com.google.glass.logging.FormattingLoggers;
import com.google.glass.logging.Log;
import com.google.glass.voice.VoiceCommand;
import com.google.glass.voice.VoiceConfig;
import com.riverlab.robotmanager.bluetooth.BluetoothDevicesListActivity;
import com.riverlab.robotmanager.bluetooth.ConnectedThread;
import com.riverlab.robotmanager.messages.MessageActivity;
import com.riverlab.robotmanager.messages.MessageListActivity;
import com.riverlab.robotmanager.robot.ViewRobotListActivity;
import com.riverlab.robotmanager.voice_recognition.VoiceRecognitionThread;

import java.util.ArrayList;
import java.util.Set;

/*This is the Main Activity for the Robot Manager App.  It displays the
 * main GUI, and initializes the other parts of the application.
 */

public class MainActivity extends Activity {

	//Thread global variables
	private ConnectedThread mConnectedThread;
	private VoiceRecognitionThread mVoiceThread;
	
	//Handler constants and definition
	public static final int MESSAGE_LIST_MESSAGE = 0;
	public static final int IMAGE_MESSAGE = 1;
	public static final int VIDEO_MESSAGE = 2;
	public static final int CONNECTION_MESSAGE = 3;
	public static final int FOCUS_MESSAGE = 4;
	public static final int COMMAND_MESSAGE = 5;
	public static final int SHUTDOWN_MESSAGE = 6;
	public static final int ROBOT_LIST_MESSAGE = 7;
	public static final int NEW_MESSAGE_MESSAGE = 8;


	//This Handler handles messages sent from the ConnectedThread for received bluetooth messages
	private final Handler mHandler = new Handler() {
		@Override
		public void handleMessage(Message msg) {
			switch (msg.what) {
			
			case MESSAGE_LIST_MESSAGE:
				launchMessageListActivity();
				break;
				
			case CONNECTION_MESSAGE:
				String text = (String)msg.obj;
				
				if (text.equals("connected"))
				{
					//Set the image view to show the bluetooth image
					imageView.setImageResource(R.drawable.ic_bluetooth_on_big);
				}
				else if (text.equals("disconnected"))
				{
					//Set the image view to show the not connected image
					imageView.setImageResource(R.drawable.not_connected_cross);
				}
				break;
				
			case FOCUS_MESSAGE:
				String focus = (String)msg.obj;
				setFocusView(focus);
				break;
				
			case COMMAND_MESSAGE:
				String cmdText = (String)msg.obj;
				setCommandView(cmdText);
				break;
				
			case SHUTDOWN_MESSAGE:
				shutdown();
				break;
				
			case ROBOT_LIST_MESSAGE:
				launchRobotListActivity();
				break;
				
			case NEW_MESSAGE_MESSAGE:
				updateMessageView(1);
			}
		}
	};

	//Timeline global variables
	private RobotManagerApplication mApplication;
	private RemoteViews aRV;
	private LiveCard mLiveCard;
	private static final String LIVE_CARD_ID = "robot_manager";

	//GUI global variables
	private ImageView imageView;
	private GestureDetector mGestureDetector;
	private TextView commandView;
	private TextView focusView;
	private TextView messageView;
	private int numMsgs = 0;
	private int numNewMsgs = 0;
	private boolean msgFlag = false;

	
	/*
	 * This function initializes the GUI and worker threads, 
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) 
	{
		super.onCreate(savedInstanceState);

		//Uncomment the following line to use the ADT debugger
		//android.os.Debug.waitForDebugger();
		
		mGestureDetector = createGestureDetector(this);

		//Setup GUI resources
		setContentView(R.layout.activity_main);
		imageView = (ImageView) findViewById(R.id.imageView);
		messageView = (TextView) findViewById(R.id.messageView);
		commandView = (TextView)findViewById(R.id.commandView);
		focusView = (TextView)findViewById(R.id.focusView);
		mApplication = ((RobotManagerApplication) this.getApplication());
		
		//Initialize the image view with the not connected image
		imageView.setImageResource(R.drawable.not_connected_cross);
		
		//Create and start the primary worker threads
		mConnectedThread = new ConnectedThread(mHandler, mApplication);
		mVoiceThread = new VoiceRecognitionThread(mApplication, this);

		mConnectedThread.start();
		mVoiceThread.start();
		
		//Wait until the worker threads have finished initializing, then
		//store their handlers within the application
		while(!mConnectedThread.isReady() && !mVoiceThread.isReady());
		
		mApplication.setMainThreadHandler(mHandler);
		mApplication.setConnectedThreadHandler(mConnectedThread.getHandler());
		mApplication.setVoiceThreadHandler(mVoiceThread.getHandler());
		
		//Provide the worker threads with each other's handlers
		mConnectedThread.setHandlers(mHandler, mVoiceThread.getHandler());
		mVoiceThread.setHandlers(mHandler, mConnectedThread.getHandler());
		
		//Name the threads for debugging purposes
		Thread.currentThread().setName("Main Activity Thread");
		mConnectedThread.setName("Connected Thread");
		mVoiceThread.setName("Voice Recognition Thread");
	}

	/*
	 * This function sets up the voice listener and resets new messages if needed
	 */
	@Override
	protected void onResume() {
		super.onResume();
		//Do not turn off screen
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		
		//Set the voice listener thread to use the context of this activity,
		//
		Handler voiceHandler = mApplication.getVoiceThreadHandler();
		
		Message msg = voiceHandler.obtainMessage(
				VoiceRecognitionThread.CONTEXT_MESSAGE, 
				this);
		voiceHandler.sendMessage(msg);
		
		//Configure voice listener to listen for default commands
		msg = voiceHandler.obtainMessage(
				VoiceRecognitionThread.ENABLE_SYSTEM_CMD_MESSAGE, 
				true);
		voiceHandler.sendMessage(msg);
		
		msg = voiceHandler.obtainMessage(
				VoiceRecognitionThread.CHANGE_VOCAB_MESSAGE,
				new ArrayList<String>());
		voiceHandler.sendMessage(msg);
		
		msg = voiceHandler.obtainMessage(
				VoiceRecognitionThread.LISTENING_MESSAGE, 
				true);
		voiceHandler.sendMessage(msg);
		
		//If we have just exited MessageListActivity reset new messages to 0
		if (msgFlag)
		{
			numNewMsgs = 0;
			msgFlag = false;
			updateMessageView(0);
		}
	}

	/*
	 * This function pauses the voice listener and allows the screen to turn off
	 */
	@Override
	protected void onPause() {
		super.onPause();
		
		//Pause the voice listener
		Handler voiceHandler = mApplication.getVoiceThreadHandler();
		if (voiceHandler != null)
		{
			Message msg = voiceHandler.obtainMessage(VoiceRecognitionThread.LISTENING_MESSAGE, false);
			voiceHandler.sendMessageAtFrontOfQueue(msg);
		}
		
		//Allow the screen to turn off
		getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
	}

	/*
	 * This function creates the menu
	 */
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.main, menu);
		return true;
	}

	/*
	 * This function decides which menu item was selected and
	 * applies the appropriate action
	 */
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle item selection.
		switch (item.getItemId()) {
		case R.id.viewDevices:
			launchBluetoothListActivity();
			return true;
		case R.id.viewRobots:
			launchRobotListActivity();
			return true;
		case R.id.viewMessages:
			launchMessageListActivity();
			return true;
		case R.id.close:
			mApplication.onShutdown();
			return true;
		default:
			return super.onOptionsItemSelected(item);
		}
	}

	/*
	 * This function listens for the user to tap on the Card and
	 * then opens the menu on tap.
	 */
	private GestureDetector createGestureDetector(Context context) {
		GestureDetector gestureDetector = new GestureDetector(context);
		//Create a base listener for generic gestures
		gestureDetector.setBaseListener( new GestureDetector.BaseListener() {
			@Override
			public boolean onGesture(Gesture gesture) {
				if (gesture == Gesture.TAP) {
					//On tap, play tap sound and open menu
					AudioManager audio = (AudioManager) getSystemService(Context.AUDIO_SERVICE);
					audio.playSoundEffect(Sounds.TAP);
					openOptionsMenu();
					return true;
				}
				return false;
			}
		});
		return gestureDetector;
	}

	/*
	 * Handles motion events
	 */
	@Override
	public boolean onGenericMotionEvent(MotionEvent event) {
		if (mGestureDetector != null) {
			return mGestureDetector.onMotionEvent(event);
		}
		return false;
	}
	
	/*
	 * This function sets the command text view to the
	 * passed string
	 */
	public void setCommandView(String cmdText)
	{
		commandView.setText("Command: " + cmdText);
	}
	
	/*
	 * This function sets the focus text view to the
	 * passed string
	 */
	public void setFocusView(String robotName)
	{
		focusView.setText("Focus: " + robotName);
	}
	
	/*
	 * This function sets the message text view to reflect
	 * the total number and the number of new messages
	 * after a change.  The change is given by the integer
	 * argument
	 */
	public void updateMessageView(int change) 
	{
		numMsgs += change;
		numNewMsgs += change;
		
		messageView.setText("\u2709\t" + numMsgs + "\tNew: " + numNewMsgs);
	}
	
	/*
	 * This function starts the Bluetooth Devices List Activity
	 */
	public void launchBluetoothListActivity()
	{
		startActivity(new Intent(this, BluetoothDevicesListActivity.class));
	}
	
	/*
	 * This function starts the Robot List Activity
	 */
	public void launchRobotListActivity()
	{
		startActivity(new Intent(this, ViewRobotListActivity.class));
	}
	
	/*
	 * This function starts the Message List Activity
	 */
	public void launchMessageListActivity()
	{
		msgFlag = true;
		startActivity(new Intent(this, MessageListActivity.class));
	}
	
	/*
	 * This function shuts down the activity and kills the worker threads
	 */
	public void shutdown()
	{
		//Make sure all threads are done before shutting down
		while (mApplication.getConnectedThreadHandler() != null &&
				mApplication.getVoiceThreadHandler() != null);
		mConnectedThread.interrupt();
		mVoiceThread.interrupt();
		mConnectedThread = null;
		mVoiceThread = null;
		
		finish();            	
	}
}