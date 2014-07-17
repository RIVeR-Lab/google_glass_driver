package com.riverlab.robotmanager.bluetooth;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.media.AudioManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;

import com.google.android.glass.media.Sounds;
import com.google.android.glass.widget.CardScrollView;
import com.riverlab.robotmanager.R;
import com.riverlab.robotmanager.RobotManagerApplication;
import com.riverlab.robotmanager.R.id;
import com.riverlab.robotmanager.R.menu;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class BluetoothDevicesListActivity extends Activity implements AdapterView.OnItemClickListener {

	BluetoothAdapter mBluetoothAdapter;
	List<BluetoothDevice> mBondedDevices;
	CardScrollView mCardScrollView;
	BluetoothDevice mSelectedDevice;
	BluetoothDeviceCardScrollAdapter adapter;
	BluetoothSocket btSocket;
	RobotManagerApplication mApplication;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

		mCardScrollView = new CardScrollView(this);
		mCardScrollView.activate();
		mCardScrollView.setOnItemClickListener(this);
		mApplication = ((RobotManagerApplication) this.getApplication());
		setContentView(mCardScrollView);
	}

	@Override
	protected void onResume() {
		super.onResume();

		Set<BluetoothDevice> devices = mBluetoothAdapter.getBondedDevices();
		mBondedDevices = new ArrayList<BluetoothDevice>(devices);
		adapter = new BluetoothDeviceCardScrollAdapter(this, mBondedDevices);
		mCardScrollView.setAdapter(adapter);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
	}

	@Override
	public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
		mSelectedDevice = (BluetoothDevice) mCardScrollView.getItemAtPosition(position);
		AudioManager audio = (AudioManager) getSystemService(Context.AUDIO_SERVICE);
		audio.playSoundEffect(Sounds.TAP);
		openOptionsMenu();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.device, menu);

		return true;
	}

	@Override
	public boolean onMenuItemSelected(int featureId, MenuItem item) {
		switch (item.getItemId()) {
		case R.id.connectMenuItem:
			//Connect to mSelectedDevice
			
			Handler connectedHandler = mApplication.getConnectedThreadHandler();
			Message msg = connectedHandler.obtainMessage(ConnectedThread.CONNECT_MESSAGE, mSelectedDevice.getName());
			connectedHandler.sendMessage(msg);
			
			long now = System.currentTimeMillis();
			int threshold = 2000;
			
			while(!mApplication.getConnectionStatus() && System.currentTimeMillis() - now < threshold);
			
			if (mApplication.getConnectionStatus())
			{
				Intent returnIntent = new Intent();
				returnIntent.putExtra("result", "Success");
				setResult(RESULT_OK,returnIntent);     
				finish();
			} else
			{       
				Intent returnIntent = new Intent();
				returnIntent.putExtra("result", "Failure");
				setResult(RESULT_OK,returnIntent);     
				finish();
			}
			return true;
		default:
			return super.onOptionsItemSelected(item);
		}
	}

	
}
