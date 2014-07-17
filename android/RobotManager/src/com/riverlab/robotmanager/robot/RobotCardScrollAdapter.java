package com.riverlab.robotmanager.robot;

import android.app.ActionBar.LayoutParams;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.webkit.WebView.FindListener;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.TextView;

import com.google.android.glass.widget.CardScrollAdapter;
import com.google.android.search.core.util.HttpHelper.GetRequest;
import com.riverlab.robotmanager.R;
import com.riverlab.robotmanager.R.id;
import com.riverlab.robotmanager.R.layout;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class RobotCardScrollAdapter extends CardScrollAdapter {
	List<Robot> mRobotList;
	Context context;

	public RobotCardScrollAdapter(Context context, Collection<Robot> robots) {
		mRobotList = new ArrayList<Robot>(robots);
		this.context = context;
	}

	public void addRobot(Robot robot) {
		mRobotList.add(robot);
	}

	@Override
	public int getCount() {
		return mRobotList.size();
	}

	@Override
	public Robot getItem(int position) {
		return mRobotList.get(position);
	}

	class ViewHolder {
		TextView name;
		TextView infoTextView;
		TextView prevTxt;
		TextView nextTxt;
		TextView robotCounter;
	}

	@Override
	public View getView(int position, View convertView, ViewGroup parent) {
		ViewHolder holder;
		int nextID = 3;

		if (convertView == null)
		{
			convertView = View.inflate(context, R.layout.robot_card, null);
			holder = new ViewHolder();

			Robot robot = mRobotList.get(position);

			LinearLayout ll = (LinearLayout) convertView
					.findViewById(R.id.scrollLinearLayout);
			LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
					LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT);

			holder.name = new TextView(this.context);
			holder.name.setId(nextID);
			nextID++;
			holder.name.setTextSize(50);
			holder.name.setText(robot.getName());
			holder.name.setLayoutParams(lp);
			holder.name.setGravity(Gravity.CENTER);
			ll.addView(holder.name);

			holder.infoTextView = new TextView(this.context);
			holder.infoTextView.setId(nextID);
			holder.infoTextView.setTextSize(24);
			holder.infoTextView.setText(robot.getInfo());
			holder.infoTextView.setLayoutParams(lp);
			holder.infoTextView.setGravity(Gravity.LEFT);
			holder.infoTextView.setPadding(30, 10, 30, 10);
			holder.infoTextView.setTextColor(context.getResources().getColor(R.color.blue));
			ll.addView(holder.infoTextView);
			
			holder.prevTxt = (TextView) convertView.findViewById(R.id.robotPreviousText);
			holder.nextTxt = (TextView) convertView.findViewById(R.id.robotNextText);
			holder.robotCounter = (TextView) convertView.findViewById(R.id.robotCounterText);
			
			convertView.setTag(holder);
		} else 
		{
			holder = (ViewHolder) convertView.getTag();
		}
		
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
        
        holder.robotCounter.setText((position + 1) + " of " + mRobotList.size());

		return convertView;
	}
	
	private boolean hasNext(int position)
	{
		int last = mRobotList.size();
		int next = position + 1;

		return next < last;
	}
	
	private boolean hasPrevious(int position)
	{
		return position > 0;
	}

	@Override
	public int getPosition(Object arg0) {
		// TODO Auto-generated method stub
		return 0;
	}
}
