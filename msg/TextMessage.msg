#This message file is used to send text messages from a robot to Glass

int64 LOW_PRIORITY = 0		#Alert the user the message has come in but do not interrupt
int64 HIGH_PRIORITY = 1 	#Interrupt the user and cause the message to pop up automatically

string sender				#The name of the robot sending the text message
string text 				#The body of text to be sent with the message. This should be short, ~100 characters
int64 priority				#See notes on constants