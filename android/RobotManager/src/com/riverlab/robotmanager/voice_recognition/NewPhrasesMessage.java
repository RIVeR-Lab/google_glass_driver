package com.riverlab.robotmanager.voice_recognition;

import java.util.ArrayList;

public class NewPhrasesMessage
{
	ArrayList<String> newVocab;
	boolean isRequired;
	boolean reset;
	
	public NewPhrasesMessage(ArrayList<String> newVocab, boolean isRequired, boolean reset)
	{
		this.newVocab = newVocab;
		this.isRequired = isRequired;
		this.reset = reset;
	}
}
