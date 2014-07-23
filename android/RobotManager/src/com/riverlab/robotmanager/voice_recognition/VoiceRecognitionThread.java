package com.riverlab.robotmanager.voice_recognition;

import java.util.ArrayList;
import java.util.HashMap;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.Intent;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Message;
import android.os.Parcelable;

import com.google.glass.input.VoiceInputHelper;
import com.google.glass.input.VoiceListener;
import com.google.glass.logging.FormattingLogger;
import com.google.glass.logging.FormattingLoggers;
import com.google.glass.logging.Log;
import com.google.glass.voice.VoiceCommand;
import com.google.glass.voice.VoiceConfig;
import com.riverlab.robotmanager.MainActivity;
import com.riverlab.robotmanager.RobotManagerApplication;
import com.riverlab.robotmanager.bluetooth.ConnectedThread;
import com.riverlab.robotmanager.robot.Robot;

/*
 * The voice recognition thread inherits the HandlerThread class.
 * This thread is used to continuously listen to voice input
 * It also manages the current vocabulary the voice listener
 * is listening for.  
 * 
 * Voice recognition is done by the VoiceListenerImpl inner class.
 * When a command is recognized the inner class selects the appropriate
 * action and performs it.
 */

public class VoiceRecognitionThread extends HandlerThread
{
	//Globals
	private RobotManagerApplication mApplication;
	private BluetoothAdapter mBluetoothAdapter;
	private Context mContext;

	//Global flags
	private boolean isShutdown = false;
	private boolean isListening = true;
	private boolean isConnected = false;
	private boolean usingSystemCommands = false;
	
	//Continuous listening
	private VoiceInputHelper mVoiceInputHelper;
	private VoiceConfig mVoiceConfig;
	private ArrayList<String> mSystemCommands;
	private ArrayList<String> mConnectCommands;
	private ArrayList<String> mVoiceConfigList;
	private HashMap<String, Runnable> mActionMap;

	//Thread globals
	private VoiceHelperThread mHelper = null;
	private Handler mHelperHandler = null;

	//Handler message type constants
	public static final int ENABLE_SYSTEM_CMD_MESSAGE = 0;
	public static final int ADD_VOCAB_MESSAGE = 1;
	public static final int REMOVE_VOCAB_MESSAGE = 2;
	public static final int CHANGE_VOCAB_MESSAGE = 3;
	public static final int CHANGE_VOCAB_ACTION_MESSAGE = 4;
	public static final int RESET_MESSAGE = 5;
	public static final int LISTENING_MESSAGE = 6;
	public static final int CONTEXT_MESSAGE = 7;
	public static final int CONNECTION_MESSAGE = 8;
	public static final int SHUTDOWN_MESSAGE = 9;

	//Handlers
	private Handler mainHandler;
	private Handler connectedHandler;
	private Handler mHandler = null;

	/*
	 * Constructor
	 */
	public VoiceRecognitionThread(RobotManagerApplication app, Context context)
	{
		super("Voice Recognition Thread");
		this.mApplication  = app;
		this.mContext = context;
	}

	/*
	 * This function sets up separate ArrayLists of Strings which
	 * define standard voice commands for the user to use.
	 * These lists are:
	 * 		- Connect commands: Used to connect to Bluetooth devices
	 * 		- System commands: Used to navigate the app
	 * In addition, it sets up and starts voice recognition
	 */
	private void setup()
	{
		//Standard lists of voice commands
		mSystemCommands = new ArrayList<String>();
		mConnectCommands = new ArrayList<String>();


		//Prefix all bluetooth device names with "connect to " to allow the
		//user to connect through voice control and add them to the connect command list
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		for (BluetoothDevice device : mBluetoothAdapter.getBondedDevices())
		{
			mConnectCommands.add("Connect to " + device.getName());
		}
		mConnectCommands.add("Close robot manager");


		//Hard coded system commands
		mSystemCommands.add("All robots");
		mSystemCommands.add("Start listening");
		mSystemCommands.add("Stop listening");
		mSystemCommands.add("End connection");
		mSystemCommands.add("Close robot manager");
		mSystemCommands.add("View robots");
		//mSystemCommands.add("Create group"); NOT IMPLEMENTED YET
		mSystemCommands.add("View messages");
		//mSystemCommands.add("View map"); NOT IMPLEMENTED YET
		//mSystemCommands.add("Show commands"); NOT IMPLEMENTED YET


		//Set up command listener
		mVoiceConfigList = new ArrayList<String>(mConnectCommands);
		mVoiceConfig = new VoiceConfig("MyVoiceConfig", mConnectCommands.toArray(new String[mConnectCommands.size()]));
		mVoiceInputHelper = new VoiceInputHelper(mContext, new VoiceListenerImpl(mVoiceConfig),
				VoiceInputHelper.newUserActivityObserver(mContext));

		mVoiceInputHelper.addVoiceServiceListener();
	}

	/*
	 * This function starts the voice recognition thread.  
	 * It also creates a handler which is associated with this thread's
	 * looper.
	 */
	@Override
	public void start()
	{
		super.start();

		mHandler = new Handler(getLooper()){
			@Override
			public void handleMessage(Message msg) {
				switch (msg.what) {
				
				case ENABLE_SYSTEM_CMD_MESSAGE:
					Boolean enable = (Boolean)msg.obj;
					useSystemCommands(enable);
					break;
					
				case ADD_VOCAB_MESSAGE:
					String addition = (String)msg.obj;
					addToVocab(addition);
					break;
					
				case REMOVE_VOCAB_MESSAGE:
					String deletion = (String)msg.obj;
					removeFromVocab(deletion);
					break;
					
				case CHANGE_VOCAB_MESSAGE:
					ArrayList<String> newVocab = (ArrayList<String>)msg.obj;
					changeVocab(newVocab);
					break;
					
				case CHANGE_VOCAB_ACTION_MESSAGE:
					HashMap<String, Runnable> actionMap = (HashMap<String, Runnable>)msg.obj;
					changeVocabWithAction(actionMap);
					break;
					
				case RESET_MESSAGE:
					String resetMessage = (String)msg.obj;
					reset(resetMessage);
					break;
					
				case LISTENING_MESSAGE:
					boolean isListening = (Boolean) msg.obj;
					VoiceRecognitionThread.this.isListening = isListening;
					break;
					
				case CONTEXT_MESSAGE:
					Context newContext = (Context)msg.obj;
					setContext(newContext);
					break;
					
				case CONNECTION_MESSAGE:
					String status = (String)msg.obj;
					if (status.equals("connected"))
					{
						useSystemCommands(true);
						changeVocab(new ArrayList<String>());
					}
					else if (status.equals("disconnected"))
					{
						useSystemCommands(false);
						changeVocab(mConnectCommands);
					}
					break;
					
				case SHUTDOWN_MESSAGE:
					shutdown();
					break;
				}

			}
		};

		setup();
	}

	/*
	 * This function returns true if the thread has been started and the thread's 
	 * handler is available
	 */
	public synchronized boolean isReady()
	{
		return mHandler != null;
	}

	/*
	 * This function accepts the handlers of the main and connected threads
	 * and saves them in global variables
	 */
	public void setHandlers(Handler mainHandler, Handler connectedHandler)
	{
		this.mainHandler = mainHandler;
		this.connectedHandler = connectedHandler;
	}

	/*
	 * Returns this thread's handler
	 */
	public Handler getHandler()
	{
		return mHandler;
	}

	/*
	 * This thread sets the listening status of the 
	 * voice listener
	 */
	public void setListeningStatus(boolean isListening)
	{
		this.isListening = isListening;
	}

	/*
	 * This function returns a list of default commands that the robot
	 * in focus can understand
	 */
	public ArrayList<String> getDefaultRobotCommands()
	{
		ArrayList<String> newVocab = new ArrayList<String>();

		//Get the instance of the robot in focus
		Robot robot = mApplication.getRobotInFocus();
		
		//If no robot is in focus, get the default commands of all robots
		if (robot == null)
		{
			for (Robot rbt : mApplication.getRobots())
			{
				newVocab.addAll(rbt.getNextPhrases(new ArrayList<String>()).newVocab);
			}
			return newVocab;
		}
		else
		{
			NewPhrasesMessage msg = robot.getNextPhrases(new ArrayList<String>());
			return msg.newVocab;
		}
	}

	/*
	 * This function accepts a string and adds that string to
	 * the vocabulary used by the voice listener
	 */
	public void addToVocab(String newVoiceCommand)
	{
		Log.d("VoiceThread", "Adding " + newVoiceCommand + " to vocab");

		//Add the new command to the current list of commands
		mVoiceConfigList.add(newVoiceCommand);

		Log.d("VoiceThread", "Vocab: " + mVoiceConfigList.toString());

		//Remove the current voice listener, replace it with a new one which
		//uses the new vocabulary, and restart the listener
		mVoiceInputHelper.removeVoiceServiceListener();
		mVoiceConfig = new VoiceConfig("MyVoiceConfig", mVoiceConfigList.toArray(new String[mVoiceConfigList.size()]));
		mVoiceInputHelper = new VoiceInputHelper(mContext, new VoiceListenerImpl(mVoiceConfig),
				VoiceInputHelper.newUserActivityObserver(mContext));

		mVoiceInputHelper.addVoiceServiceListener();
	}

	/*
	 * This function accepts a string and removes that string from
	 * the vocabulary used by the voice listener
	 */
	public void removeFromVocab(String vcToRemove)
	{
		//Remove the provided command from the current list of commands
		mVoiceConfigList.remove(vcToRemove);

		//Remove the current voice listener, replace it with a new one which
		//uses the new vocabulary, and restart the listener
		mVoiceInputHelper.removeVoiceServiceListener();
		mVoiceConfig = new VoiceConfig("MyVoiceConfig", mVoiceConfigList.toArray(new String[mVoiceConfigList.size()]));
		mVoiceInputHelper = new VoiceInputHelper(mContext, new VoiceListenerImpl(mVoiceConfig),
				VoiceInputHelper.newUserActivityObserver(mContext));

		mVoiceInputHelper.addVoiceServiceListener();
	}

	/*
	 * This function accepts an ArrayList of Strings and sets it
	 * as the current list of accepted commands
	 */
	public void changeVocab(ArrayList<String> newVocab)
	{
		Log.i("VoiceThread", "Changing vocab");
		
		//Only update vocab if connected or changing to connection commands
		if (mApplication.getConnectionStatus() || newVocab.equals(mConnectCommands))
		{
			//Empty current list and repopulate using the provided list
			mVoiceConfigList = new ArrayList<String>();
			mVoiceConfigList.addAll(newVocab);
			
			if (usingSystemCommands)
			{
				mVoiceConfigList.addAll(mSystemCommands);
				mVoiceConfigList.addAll(mApplication.getRobotNames());
			}
		}
		else //If we are not connected, make sure the connect commands are available
		{
			mVoiceConfigList.addAll(mConnectCommands);
		}

		//Remove, replace, and restart the voice listener
		mVoiceInputHelper.removeVoiceServiceListener();
		mVoiceConfig = new VoiceConfig("MyVoiceConfig", mVoiceConfigList.toArray(new String[mVoiceConfigList.size()]));
		mVoiceInputHelper = new VoiceInputHelper(mContext, new VoiceListenerImpl(mVoiceConfig),
				VoiceInputHelper.newUserActivityObserver(mContext));

		mVoiceInputHelper.addVoiceServiceListener();

		Log.d("VoiceRecognitionThread", "New listening list: " + mVoiceConfigList.toString());

	}

	/*
	 * This function accepts a hash map with keys of type String and values of type Runnable
	 * The key is what the voice listener will listen for and the Runnable value is what will
	 * be executed upon detection of the key.
	 */
	public void changeVocabWithAction(HashMap<String, Runnable> vocabWithAction)
	{
		mActionMap = vocabWithAction;

		//Extract voice commands from the map's keys
		ArrayList<String> commands = new ArrayList<String>();
		for (String command : mActionMap.keySet())
		{
			commands.add(command);
		}

		//Do not use system commands
		useSystemCommands(false);
		changeVocab(commands);
	}

	/*
	 * This function sets the context global variable
	 */
	public void setContext(Context newContext)
	{
		mContext = newContext;

	}

	/*
	 * This function sets the usingSystemCommands global variables
	 */
	public void useSystemCommands(boolean use)
	{
		usingSystemCommands = use;
	}

	/*
	 * This function is used after a full command has been detected.
	 * It resets the helper to null and sends the detected command to the connected thread
	 * to be sent to the google glass driver ros node
	 */
	public void reset(String expression)
	{
		Message msg = connectedHandler.obtainMessage(ConnectedThread.WRITE_MESSAGE, expression);
		connectedHandler.sendMessage(msg);

		mHelper = null;
		usingSystemCommands = true;
		changeVocab(getDefaultRobotCommands());
	}

	/*
	 * This function cleans up the helper thread and the voice recognizer
	 */
	public void shutdown()
	{
		isShutdown = true;
		mVoiceInputHelper.removeVoiceServiceListener();
		mVoiceInputHelper = null;
		if (mHelper != null)
		{
			mHelper.interrupt();
			mHelper = null;
		}
		mApplication.setVoiceThreadHandler(null);
	}


	//This class is used to continuously listen for user input 
	public class VoiceListenerImpl implements VoiceListener {
		protected final VoiceConfig voiceConfig;

		public VoiceListenerImpl(VoiceConfig voiceConfig) {
			this.voiceConfig = voiceConfig;
		}

		@Override
		public void onVoiceServiceConnected() {
			mVoiceInputHelper.setVoiceConfig(mVoiceConfig);
		}

		@Override
		public void onVoiceServiceDisconnected() {	

		}

		//This method handles recognized voice commands
		@Override
		public VoiceConfig onVoiceCommand(VoiceCommand vc) {
			String recognizedStr = vc.getLiteral();

			Log.i("VoiceRecognitionThread", "Recognized text: "+recognizedStr);

			if (isListening)
			{				
				//Check to see if it is a robot focus
				if (recognizedStr.equals("All robots"))
				{
					Log.d("VoiceRecognitionThread", "Setting focus on All");
					//focus on all robots
					mApplication.setRobotInFocus("All");

					Message msg = mainHandler.obtainMessage(MainActivity.FOCUS_MESSAGE, "All");
					mainHandler.sendMessageAtFrontOfQueue(msg);

					changeVocab(getDefaultRobotCommands());
				}

				else if (mApplication.getRobotNames().contains(recognizedStr))
				{
					Log.d("VoiceRecognitionThread", "Setting focus on: " + recognizedStr);
					//Set focus on robot whose name was recognized
					mApplication.setRobotInFocus(recognizedStr);


					Message msg = mainHandler.obtainMessage(MainActivity.FOCUS_MESSAGE, recognizedStr);
					mainHandler.sendMessageAtFrontOfQueue(msg);

					changeVocab(getDefaultRobotCommands());
				}
				else
				{
					//Print Command to screen
					Message msg = mainHandler.obtainMessage(MainActivity.COMMAND_MESSAGE, recognizedStr);
					mainHandler.sendMessageAtFrontOfQueue(msg);

					//Check to see if it is a system command.
					if (recognizedStr.startsWith("Connect to"))
					{
						if (!isConnected) //If already connected do not connect
						{
							Message msgConnected = connectedHandler.obtainMessage(ConnectedThread.CONNECT_MESSAGE, recognizedStr.substring(11));
							connectedHandler.sendMessageAtFrontOfQueue(msgConnected);
						}
					}
					else if (recognizedStr.equals("End connection"))
					{
						Message msgConnected = connectedHandler.obtainMessage();
						msgConnected.what = ConnectedThread.DISCONNECT_MESSAGE;
						connectedHandler.sendMessage(msgConnected);
					}

					else if (recognizedStr.equals("Close robot manager"))
					{
						mApplication.onShutdown();
					}

					else if (recognizedStr.equals("Stop listening"))
					{
						setListeningStatus(false);
					}

					else if (recognizedStr.equals("View robots"))
					{
						Log.i("VoiceRecognitionThread", "Requesting launch of RobotListActivity");
						Message msgView = mainHandler.obtainMessage();
						msgView.what = MainActivity.ROBOT_LIST_MESSAGE;
						mainHandler.sendMessage(msgView);
					}

					else if (recognizedStr.equals("Create group"))
					{
						//NOT YET IMPLEMENTED
					}

					else if (recognizedStr.equals("View messages"))
					{
						Log.i("VoiceRecognitionThread", "Requesting launch of MessageListActivity");
						Message msgView = mainHandler.obtainMessage();
						msgView.what = MainActivity.MESSAGE_LIST_MESSAGE;
						mainHandler.sendMessage(msgView);
					}

					else if (recognizedStr.equals("View map"))
					{
						//NOT YET IMPLEMENTED
					}
					
					else if (recognizedStr.equals("Cancel"))
					{
						mHelper.interrupt();
						mHelper = null;
						changeVocab(getDefaultRobotCommands());
						Message mainMsg = mainHandler.obtainMessage(MainActivity.COMMAND_MESSAGE, "Cancel");
						mainHandler.sendMessage(mainMsg);
					} 

					else if (mActionMap != null && mActionMap.containsKey(recognizedStr))
					{
						mActionMap.get(recognizedStr).run();
					}

					//If the recognized string is not a system command, it must be a robot command
					else if (!recognizedStr.equals("Start listening"))
					{
						if (mHelper == 	null)
						{
							Robot focus = mApplication.getRobotInFocus();
							if (focus == null)
							{
								mHelper = new VoiceHelperThread(mApplication, recognizedStr, 
										"All", mHandler);
							}
							else
							{
								mHelper = new VoiceHelperThread(mApplication, recognizedStr, 
										focus.getName(), mHandler);
							}

							mHelper.start();
							while(!mHelper.isReady());
							mHelperHandler = mHelper.getHandler();
							//addToVocab("Cancel");

							usingSystemCommands = false;
						}
						else
						{
							Message newPhraseMsg = mHelperHandler.obtainMessage(VoiceHelperThread.RECEIVE_PHRASE_MESSAGE, recognizedStr);
							mHelperHandler.sendMessageAtFrontOfQueue(newPhraseMsg);
						}
					}
				}
			}
			else if (recognizedStr.equals("Start listening"))
			{
				Message msgShow = mainHandler.obtainMessage(MainActivity.COMMAND_MESSAGE, recognizedStr);
				mainHandler.sendMessageAtFrontOfQueue(msgShow);
				setListeningStatus(true);
			}

			return null;
		}

		@Override
		public FormattingLogger getLogger() {
			return FormattingLoggers.getContextLogger();
		}

		@Override
		public boolean isRunning() {
			return true;
		}

		@Override
		public boolean onResampledAudioData(byte[] arg0, int arg1, int arg2) {
			return false;
		}

		//This may be considered in order to differentiate commands from normal
		//speech
		@Override
		public boolean onVoiceAmplitudeChanged(double arg0) {
			return false;
		}

		@Override
		public void onVoiceConfigChanged(VoiceConfig arg0, boolean arg1) 
		{
		}
	}
}
