package com.riverlab.robotmanager.messages;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import android.app.Activity;
import android.content.Context;
import android.media.AudioManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.Message;

import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.ImageView;
import android.widget.TextView;

import com.google.android.glass.media.Sounds;
import com.google.android.glass.widget.CardScrollView;
import com.riverlab.robotmanager.R;
import com.riverlab.robotmanager.RobotManagerApplication;
import com.riverlab.robotmanager.voice_recognition.VoiceRecognitionThread;

public class MessageListActivity extends Activity implements AdapterView.OnItemClickListener
{

	List<RobotMessage> mMessages;
	CardScrollView mCardScrollView;
	RobotMessage mSelectedMessage;
	MessageCardScrollAdapter adapter;
	RobotManagerApplication mApplication;

	Runnable close = new Runnable() {
		@Override
		public void run() 
		{
			finish();
		}
	};

	Runnable previous = new Runnable() {
		@Override
		public void run() 
		{
			int current = mCardScrollView.getSelectedItemPosition();
			int next = current - 1;

			if(next >= 0)
			{
				mCardScrollView.setSelection(next);
			}
		}
	};

	Runnable next = new Runnable() {
		@Override
		public void run() 
		{
			int last = mCardScrollView.getChildCount();
			int current = mCardScrollView.getSelectedItemPosition();
			int next = current + 1;

			if(next < last)
			{
				mCardScrollView.setSelection(next);
			}
		}
	};
	
	Runnable viewImage = new Runnable() {
		@Override
		public void run() 
		{
			Runnable guiUpdate = new Runnable() {
				@Override
				public void run() {
					int index = mCardScrollView.getSelectedItemPosition();
					if (mMessages.get(index).getType() == "Text")
						return;
					
					View curView = mCardScrollView.getChildAt(mCardScrollView.getSelectedItemPosition());
					ImageView imgView = (ImageView)curView.findViewById(R.id.imageView);
					TextView msgView = (TextView)curView.findViewById(R.id.messageText);
					
					imgView.setVisibility(View.VISIBLE);
					msgView.setVisibility(View.INVISIBLE);
				}
			};
			
			Handler handler = new Handler(Looper.getMainLooper());
			handler.post(guiUpdate);
		}
	};
	
	Runnable viewText = new Runnable() {
		@Override
		public void run() 
		{
			Runnable guiUpdate = new Runnable() {
				@Override
				public void run() {
					View curView = mCardScrollView.getChildAt(mCardScrollView.getSelectedItemPosition());
					ImageView imgView = (ImageView)curView.findViewById(R.id.imageView);
					TextView msgView = (TextView)curView.findViewById(R.id.messageText);
					
					imgView.setVisibility(View.INVISIBLE);
					msgView.setVisibility(View.VISIBLE);
				}
			};
			
			Handler handler = new Handler(Looper.getMainLooper());
			handler.post(guiUpdate);
		}
	};

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		mCardScrollView = new CardScrollView(this);
		mCardScrollView.activate();
		mCardScrollView.setOnItemClickListener(this);
		mApplication = ((RobotManagerApplication) this.getApplication());
		setContentView(mCardScrollView);
	}

	@Override
	protected void onResume() {
		super.onResume();

		mApplication.setMsgListActivity(this);
		mMessages = mApplication.getMessages();
		adapter = new MessageCardScrollAdapter(this, mMessages);
		mCardScrollView.setAdapter(adapter);
		mCardScrollView.setSelection(mMessages.size() - 1);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		HashMap<String, Runnable> commands = new HashMap<String, Runnable>();
		commands.put("Close", close);
		commands.put("Next", next);
		commands.put("Previous", previous);
		commands.put("Image", viewImage);
		commands.put("Text", viewText);
		
		Handler voiceHandler = mApplication.getVoiceThreadHandler();
		Message msg = voiceHandler.obtainMessage(
				VoiceRecognitionThread.CHANGE_VOCAB_ACTION_MESSAGE,
				commands);
		voiceHandler.sendMessage(msg);
		
		msg = voiceHandler.obtainMessage(
				VoiceRecognitionThread.CONTEXT_MESSAGE, 
				this);
		voiceHandler.sendMessage(msg);
		
		msg = voiceHandler.obtainMessage(
				VoiceRecognitionThread.LISTENING_MESSAGE, 
				true);
		voiceHandler.sendMessage(msg);
	}

	@Override
	protected void onPause()
	{
		super.onPause();

		mApplication.setMsgListActivity(null);
	}

	@Override
	public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
		mSelectedMessage = (RobotMessage) mCardScrollView.getItemAtPosition(position);
		AudioManager audio = (AudioManager) getSystemService(Context.AUDIO_SERVICE);
		audio.playSoundEffect(Sounds.TAP);
		openOptionsMenu();
	}

	public void onMessageAddition()
	{
		int curPosition = 0;
		curPosition = mCardScrollView.getSelectedItemPosition();

		mMessages = mApplication.getMessages();
		adapter = new MessageCardScrollAdapter(this, mMessages);
		mCardScrollView.setAdapter(adapter);

		if (mMessages.get(0).getPriority() == 0)
		{
			mCardScrollView.setSelection(curPosition);
		}
		else
		{
			mCardScrollView.setSelection(mMessages.size() - 1);
		}
	}
}
