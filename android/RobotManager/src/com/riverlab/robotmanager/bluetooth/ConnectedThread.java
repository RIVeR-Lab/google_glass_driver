package com.riverlab.robotmanager.bluetooth;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.concurrent.TimeoutException;

import com.riverlab.robotmanager.MainActivity;
import com.riverlab.robotmanager.RobotManagerApplication;
import com.riverlab.robotmanager.messages.RobotMessage;
import com.riverlab.robotmanager.robot.Robot;
import com.riverlab.robotmanager.voice_recognition.Vocabulary;
import com.riverlab.robotmanager.voice_recognition.VoiceRecognitionThread;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.Message;
import android.util.Log;

public class ConnectedThread extends HandlerThread
{
	private BluetoothAdapter mBluetoothAdapter;
	private BluetoothSocket mBtSocket;
	private InputStream mInStream;
	private OutputStream mOutStream;
	private RobotManagerApplication mApplication;
	private boolean isShutdown = false;
	private Object readWriteLock = new Object();

	public static final int CONNECT_MESSAGE = 0;
	public static final int DISCONNECT_MESSAGE = 1;
	public static final int WRITE_MESSAGE = 2;
	public static final int SHUTDOWN_MESSAGE = 3;


	private Handler mHandler = null;
	private Handler mainHandler;
	private Handler voiceHandler;

	Thread socketThread;

	public ConnectedThread(Handler mainHandler, RobotManagerApplication app) 
	{
		super("Bluetooth Connection Thread");
		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
		mApplication = app;

		socketThread = new Thread()
		{
			@Override
			public void run()
			{
				receivePackets();
			}
		};
	}

	@Override
	public void start()
	{
		super.start(); 

		mHandler = new Handler(getLooper()){
			public void handleMessage(Message msg) 
			{
				switch (msg.what)
				{
				case CONNECT_MESSAGE:
					String deviceName = (String)msg.obj;
					connect(deviceName);
					break;
				case DISCONNECT_MESSAGE:
					Log.d("ConnectedThread", "Disconnect request received");
					disconnect();
					break;
				case WRITE_MESSAGE:
					String msgText = (String)msg.obj;
					write(msgText.getBytes());
					break;
				case SHUTDOWN_MESSAGE:
					shutdown();
					break;
				}
			}
		};		
	}

	public synchronized boolean isReady()
	{
		return mHandler != null;
	}

	public void setHandlers(Handler mainHandler, Handler voiceHandler) 
	{
		this.mainHandler = mainHandler;
		this.voiceHandler = voiceHandler;
	}

	public Handler getHandler()
	{
		return mHandler;
	}

	public boolean connect(String deviceName)
	{
		for (BluetoothDevice device : mBluetoothAdapter.getBondedDevices())
		{
			if (device.getName().equals(deviceName))
			{				
				Log.d("RobotManagerBluetooth", "Attempting to connect to device");
				// Discovery is resource intensive.  Make sure it isn't going on
				// when you attempt to connect and pass your message.
				mBluetoothAdapter.cancelDiscovery();

				// Create a Rfcomm Socket between the Server and Glass
				Log.d("RobotManagerBluetooth", "Attempting to create Rfcomm Socket");
				Method m;
				try {
					m = device.getClass().getMethod("createRfcommSocket", new Class[] {int.class}); 
					mBtSocket = (BluetoothSocket) m.invoke(device, 1); 
					Log.d("RobotManagerBluetooth", "Rfcomm Socket created");
				} 
				catch (NoSuchMethodException e) {
					e.printStackTrace();
					return false;
				}
				catch (IllegalArgumentException e) {
					e.printStackTrace();
					return false;
				} catch (IllegalAccessException e) {
					e.printStackTrace();
					return false;
				} catch (InvocationTargetException e) {
					e.printStackTrace();
					return false;
				}

				// Establish the connection.  This will block until it connects.
				Log.d("RobotManagerBluetooth", "Attempting to open socket");
				try {
					mBtSocket.connect();
					Log.d("RobotManagerBluetooth", "Connection established");
				} catch (IOException e) {
					e.printStackTrace();
					return false;
				}

				// Create a data stream so we can talk to server.
				Log.d("RobotManagerBluetooth", "Sending confirmation message to server");
				try {
					mOutStream = mBtSocket.getOutputStream();
				} catch (IOException e) {
					e.printStackTrace();
					return false;
				}

				String message = "Confirm connection\n";
				byte[] msgBuffer = message.getBytes();
				try {
					Log.d("RobotManagerBluetooth", "Writing message");
					mOutStream.write(msgBuffer);
					Log.d("RobotManagerBluetooth", "Message written");
				} catch (IOException e) {
					e.printStackTrace();   
					return false;
				}

				// Listen for confirmation from the server.
				Log.d("RobotManagerBluetooth", "Listening for confirmation message from server");
				try {
					mInStream = mBtSocket.getInputStream();
				} catch (IOException e) {
					e.printStackTrace();
					return false;
				}

				String confirmString = "Connection confirmed\n";
				byte[] receivedBytes = new byte[confirmString.getBytes().length];
				try
				{
					mInStream.read(receivedBytes);
				} catch (IOException e){
					e.printStackTrace();
					return false;
				}
				String receivedString = new String(receivedBytes);
				if (receivedString.equals(confirmString))
				{
					Log.d("RobotManagerBluetooth", "Connection confirmed");
					mApplication.setConnectionStatus(true);

					Message connectionMessage = mainHandler.obtainMessage(MainActivity.CONNECTION_MESSAGE, "connected");
					mainHandler.sendMessageAtFrontOfQueue(connectionMessage);
					
					Message voiceConnectionMessage = voiceHandler.obtainMessage(VoiceRecognitionThread.CONNECTION_MESSAGE,
							"connected");
					voiceHandler.sendMessage(voiceConnectionMessage);

					socketThread.start();

					return true;
				}
				else
				{
					Log.d("RobotManagerBluetooth", "Confirmation not received");
					mApplication.setConnectionStatus(false);
					return false;
				}
			}
		}
		return false;
	}

	public synchronized byte[] read()
	{
		try {
			int available = mInStream.available(); 
			if (available > 0) 
			{ 
				byte[] buffer = new byte[available]; 
				mInStream.read(buffer); 
				return buffer;
			}
			else
			{
				return null;
			}
		} catch (IOException e) {
			e.printStackTrace();
			Log.d("ConnectedThread", "Error reading message");
			return null;
		}
	}

	public synchronized byte[] read(int length)
	{
		byte[] buffer = new byte[length];

		try {
			buffer = new byte[2048];
			mInStream.read(buffer, 0, length);
			return buffer;
		} catch (IOException e) {
			e.printStackTrace();
			Log.d("ConnectedThread", "Error reading message");
			return null;
		}
	}

	/*public void pollSocket() 
	{

		Log.d("ConnectedThread", "Polling socket");

		while (!isShutdown)
		{
			// Keep listening to the InputStream until an exception occurs	
			String bufferString = "";
			String[] pkts = null;

			while (mApplication.getConnectionStatus()) 
			{
				Log.d("ConnectedThread", "Listening for message");

				boolean incoming = false;
				int packetNum = 0;
				int packetSize = 0;
				boolean invalid = false;
				int bufferSlack = 100;



				byte[] pktBuffer = new byte[packetSize + bufferSlack];
				pktBuffer = read();

				String pktString = new String(pktBuffer);
				pkts = new String[packetNum];


				String message = "";
				for (String str : pkts)
				{
					message += str;
				}



			}//While connected loop
		}//While alive


	}//Function */

	private void receivePackets()
	{
		Log.i("ConnectedThread", "Packet listener thread enabled");

		byte[] buffer = new byte[30];  // buffer store for the stream
		int packetNum = 0;
		int packetSize = 0;
		String message;

		while(mApplication.getConnectionStatus())
		{
			Log.i("ConnectedThread", "Listening for packet header");

			String tempString = "";
			while (!hasFullHeader(tempString))
			{	
				byte[] bytes = read(1);
				if (bytes != null)
				{
					String readString = new String(bytes).trim();
					tempString += readString;
				}
			}

			String headerString = tempString;
			Log.i("ConnectedThread", "Header: " + headerString);

			if ((headerString.substring(0, 5).equals("<msg>")) && (headerString.indexOf("</msg>") != -1))
			{
				headerString = headerString.replace("<msg>", "").replace("</msg>", "");
				String[] parts = headerString.split("_DELIM_");
				packetNum = Integer.parseInt(parts[0]);
				packetSize = Integer.parseInt(parts[1]);
			}

			Log.i("ConnectedThread", "Expecting " + Integer.toString(packetNum)
					+ " packets of size " + Integer.toString(packetSize)
					+ " bytes");

			message = getPacketData(packetNum, packetSize);
			String[] messageParts = message.split("_DELIM_");
			Log.d("ConnectedThread", "Message type: " + messageParts[0]);


			if(messageParts[0].equals("robot_configuration"))
			{
				parseRobotConfiguration(messageParts[1], messageParts[2], messageParts[3]);
			}
			else if(messageParts[0].equals("text_message"))
			{
				parseTextMessage(messageParts[1], messageParts[2], messageParts[3]);
			}
			else if(messageParts[0].equals("image_message"))
			{
				parseImageMessage(messageParts[1], messageParts[2], messageParts[3], messageParts[4]);
			}
		}
	}

	private String getPacketData(int numPackets, int pktSize)
	{
		Log.i("ConnectedThread", "Getting packet data");

		int metaSize = 27;
		String pktData = "";
		int deltaT = 3000;
		double stopTime = System.currentTimeMillis() + deltaT;

		for (int i = 0; i < numPackets && 
				System.currentTimeMillis() != stopTime; i++)
		{
			Log.i("ConnectedThread", "Listening for packets");
			String tempString = "";
			while (!isShutdown)
			{
				while (!hasFullPacket(tempString))
				{	
					byte[] bytes = read();
					if (bytes != null)
					{
						String readString = new String(bytes).trim();
						tempString += readString;
					}
				}
				Log.i("ConnectedThread", "Packet received");
				Log.i("ConnectedThread", "Raw: " + tempString);
				tempString = conditionData(tempString, i+1);
				Log.i("ConnectedThread", "Conditioned: " + tempString);
				Log.i("ConnectedThread", "Asking for next packet");
				write("_NEXT_\n".getBytes());
				break;
			}
			if (tempString == null)
			{
				Log.d("ConnectedThread", "Could not read packet #" + Integer.toString(i));
			}
			else
			{
				pktData += tempString;
			}
		}
		pktData = pktData.replace("_SPACE_", " ");
		return pktData;
	}

	private boolean hasFullHeader(String strBuffer)
	{
		int start = strBuffer.indexOf("<msg>");
		if (start >= 0)
		{
			int end = strBuffer.indexOf("</msg>", start);
			if (end >= 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	
	private boolean hasFullPacket(String strBuffer)
	{
		int start = strBuffer.indexOf("<pkt>");
		if (start >= 0)
		{
			int end = strBuffer.indexOf("</pkt>", start);
			if (end >= 0)
			{
				return true;
			}
			else
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	private String conditionData(String rawString, int expectedIndex)
	{
		String conditionedString = "";
		boolean invalid = false;

		if ((rawString.substring(0, 5).equals("<pkt>")) && (rawString.indexOf("</pkt>") != -1))
		{
			rawString = rawString.replace("<pkt>", "");
			Log.d("ConnectedThread", "Remove pkt tags " + rawString);

			if ((rawString.substring(0, 7).equals("<index>")) && (rawString.indexOf("</index>") != -1))
			{
				String pktIndexString = rawString.substring(7, rawString.indexOf("</index>"));
				int pktIndex = Integer.parseInt(pktIndexString);

				String pktBody = rawString.substring(
						rawString.indexOf("</index>") + 8);

				if (pktBody.indexOf("</pkt>") != -1)
				{
					if (expectedIndex == pktIndex)
					{
						conditionedString = pktBody.substring(0,
								pktBody.indexOf("</pkt>"));

						conditionedString = conditionedString.replace("_SPACE_", " ");
					}
					else
					{
						invalid = true;
						Log.i("ConnectedThread", "Packet index does not match expected index");
					}
				}
				else
				{
					invalid = true;
					Log.i("ConnectedThread", "End of packet not read");
				}
			}//Check index tags
			else
			{
				invalid = true;
				Log.i("ConnectedThread", "Index tags not read");
			}
		}//Check packet tags
		else
		{
			invalid = true;
			Log.i("ConnectedThread", "Begin packet tag not read");
		}

		if (invalid)
		{
			conditionedString = null;
			Log.i("ConnectedThread", "Invalid packet");
		}

		return conditionedString;
	}

	private void parseRobotConfiguration(String sender, String info, String rawVocab)
	{
		Log.d("ConnectedThread", "Configuring Robot");
		Robot newRobot = new Robot();
		newRobot.setName(sender);
		newRobot.setInfo(info);
		newRobot.setVocabulary(new Vocabulary(rawVocab));

		Message msg = voiceHandler.obtainMessage(VoiceRecognitionThread.ADD_VOCAB_MESSAGE, newRobot.getName());
		voiceHandler.sendMessageAtFrontOfQueue(msg);

		mApplication.addRobot(newRobot);

		Log.d("ConnectedThread", "Writing confirmation");
		write("configuration complete\n".getBytes());
	}


	private void parseTextMessage(String sender, String text, String strPriority)
	{
		Log.d("ConnectedThread", "Reading text message");
		RobotMessage msg = new RobotMessage();
		msg.setType("Text");
		msg.setSender(sender);
		msg.setText(text);
		msg.setPriority(Integer.parseInt(strPriority));

		Calendar cal = Calendar.getInstance();
		SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");
		msg.setTimestamp(sdf.format(cal.getTime()));

		Log.d("ConnectedThread", "Adding text message to messages");
		mApplication.addMessage(msg);
	}

	private void parseImageMessage(String sender, String text, String base64Image, String strPriority)
	{
		Log.d("ConnectedThread", "Reading image message");
		RobotMessage msg = new RobotMessage();
		msg.setType("Image");
		msg.setSender(sender);
		msg.setText(text);
		msg.setImage(base64Image);
		msg.setPriority(Integer.parseInt(strPriority));

		Calendar cal = Calendar.getInstance();
		SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");
		msg.setTimestamp(sdf.format(cal.getTime()));

		Log.d("ConnectedThread", "Adding text message to messages");
		mApplication.addMessage(msg);
	}

	public void write(byte[] bytes) {
		String sentString = new String(bytes);
		String confirmString = "Copy: " + sentString;

		Log.d("RobotManagerBluetooth", "Acquiring read/write lock");
		//synchronized (readWriteLock) 
		//{
		try {
			Log.d("RobotManagerBluetooth", "Writing");
			mOutStream.write(bytes);
		} catch (IOException e) { 
			//Something went wrong, end connection

			e.printStackTrace();
			disconnect();
		}
		/*
			// Listen for confirmation of receipt.
			Log.d("RobotManagerBluetooth", "Listening for confirmation message from server");

			byte[] receivedBytes = read();

			String receivedString = new String(receivedBytes).trim();
			if (receivedString.equals(confirmString))
			{
				Log.d("RobotManagerBluetooth", "Receipt confirmed");

			}
			else
			{
				Log.d("RobotManagerBluetooth", "Confirmation of receipt not received, instead: " + receivedString);
				return;
			}
		 */
		//	}
	}

	private void sendMainMessage(RobotMessage msg)
	{
		Message mainMsg = new Message();

		if (msg.getType().equals("simple"))
		{
			mainMsg.what = MainActivity.MESSAGE_LIST_MESSAGE;
		}

		mainMsg.obj = msg;
		mHandler.sendMessageAtFrontOfQueue(mainMsg);
	}

	public void disconnect() 
	{
		//try 
		//{
			write("end connection\n".getBytes());
			//mBtSocket.close();
			socketThread.interrupt();
			socketThread = null;
			mApplication.setConnectionStatus(false); 
			
			Message connectionMessage = mainHandler.obtainMessage(MainActivity.CONNECTION_MESSAGE, "disconnected");
			mainHandler.sendMessage(connectionMessage);
			
			Message voiceConnectionMessage = voiceHandler.obtainMessage(VoiceRecognitionThread.CONNECTION_MESSAGE, "disconnected");
			voiceHandler.sendMessage(voiceConnectionMessage);
			//}
		/*catch (IOException e) { 
			Log.d("ConnectedThread", "Exception in disconnect");
			e.printStackTrace();
		}*/
	}

	public void shutdown()
	{
		if (mApplication.getConnectionStatus())
		{
			disconnect();
		}
		isShutdown = true;
		mApplication.setConnectedThreadHandler(null);
	}
}
