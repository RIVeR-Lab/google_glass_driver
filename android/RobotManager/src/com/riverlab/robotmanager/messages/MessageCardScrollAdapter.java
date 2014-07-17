package com.riverlab.robotmanager.messages;

import java.util.List;

import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.util.Log;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;
import android.widget.TextView;

import com.google.android.glass.widget.CardScrollAdapter;
import com.riverlab.robotmanager.R;

public class MessageCardScrollAdapter extends CardScrollAdapter
{
	private List<RobotMessage> mMessages;
	private Context context;
	ViewHolder holder;
	
	public MessageCardScrollAdapter(Context context, List<RobotMessage> messages) 
	{
        mMessages = messages;
        this.context = context;
    }
	
	public void addMessage(RobotMessage msg) {
        mMessages.add(msg);
    }

	@Override
	public int getCount() {
		return mMessages.size();
	}

	@Override
	public RobotMessage getItem(int index) 
	{
		return mMessages.get(index);
	}

	@Override
	public int getPosition(Object msg)
	{
		return mMessages.indexOf((RobotMessage)msg);
	}
	
	 class ViewHolder {
	        TextView headline;
	        TextView text;
	        ImageView img;
	        TextView timestamp;
	        TextView nextTxt;
	        TextView prevTxt;
	        TextView msgCounter;
	        ImageView indicator;
	    }

	@Override
	public View getView(int position, View convertView, ViewGroup parent) {
        if (convertView == null) {
            convertView = View.inflate(context, R.layout.message_card, null);
            holder = new ViewHolder();
            Log.d("MessageActivity", "Getting views");
            holder.headline = (TextView) convertView.findViewById(R.id.headline);
            holder.text = (TextView) convertView.findViewById(R.id.messageText);
            holder.img = (ImageView) convertView.findViewById(R.id.imageView);
            holder.timestamp = (TextView) convertView.findViewById(R.id.timestampText);
            holder.nextTxt = (TextView) convertView.findViewById(R.id.msgNextText);
            holder.prevTxt = (TextView) convertView.findViewById(R.id.msgPreviousText);
            holder.msgCounter = (TextView) convertView.findViewById(R.id.msgCounterText);
            holder.indicator = (ImageView) convertView.findViewById(R.id.imageIndicator);
            
            convertView.setTag(holder);
        }
        else {
            holder = (ViewHolder) convertView.getTag();
        }

        RobotMessage msg = getItem(position);
        holder.headline.setText(msg.getSender() + ": ");
        holder.text.setText(msg.getText());
  
        if (msg.getType().equals("Image") && msg.getImage() != null)
        {
        	holder.img.setImageBitmap(msg.getImage());
        	holder.indicator.setVisibility(View.VISIBLE);
        }
        holder.timestamp.setText(msg.getTimestamp());
        
        
        if (hasNext(position))
        {
        	holder.nextTxt.setVisibility(View.VISIBLE);
        	holder.nextTxt.setText(R.string.next);
        }
        else
        {
        	holder.nextTxt.setVisibility(View.INVISIBLE);
        	//holder.nextTxt.setText(R.string.no_next);
        }
        
        if (hasPrevious(position))
        {
        	holder.prevTxt.setVisibility(View.VISIBLE);
        	holder.prevTxt.setText(R.string.previous);
        }
        else
        {
        	holder.prevTxt.setVisibility(View.INVISIBLE);
        	//holder.prevTxt.setText(R.string.no_previous);
        }
        
        holder.msgCounter.setText((position + 1) + " of " + mMessages.size());

        return convertView;
	}
	
	private boolean hasNext(int position)
	{
		int last = mMessages.size();
		int next = position + 1;

		return next < last;
	}
	
	private boolean hasPrevious(int position)
	{
		return position > 0;
	}
	
	public ViewHolder getHolder()
	{
		return holder;
	}

}
