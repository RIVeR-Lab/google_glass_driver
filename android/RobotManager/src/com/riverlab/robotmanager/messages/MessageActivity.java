package com.riverlab.robotmanager.messages;

import com.riverlab.robotmanager.R;
import com.riverlab.robotmanager.R.id;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public class MessageActivity extends Activity
{
	@Override
	protected void onCreate(Bundle savedInstanceState) 
	{
		RobotMessage msg = (RobotMessage)this.getIntent().getParcelableExtra("Message");
		String type = "";
		
		if (msg.getType().equals("text"))
		{
			
		}
		
		TextView headlineTextView = (TextView)findViewById(R.id.headline);
		headlineTextView.setText("Message from " + msg.getSender());
		
		TextView messageTextView = (TextView)findViewById(R.id.messageText);
		messageTextView.setText("Alert from " + msg.getText());
		
	}
	
	@Override
	protected void onResume() {}
	
	@Override
	protected void onPause() {}
	
	
}
