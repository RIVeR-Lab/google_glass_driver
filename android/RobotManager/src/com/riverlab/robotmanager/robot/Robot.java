package com.riverlab.robotmanager.robot;

import java.util.ArrayList;

import com.riverlab.robotmanager.voice_recognition.NewPhrasesMessage;
import com.riverlab.robotmanager.voice_recognition.Vocabulary;

public class Robot 
{
	private String name;
	private String info;
	private Vocabulary vocab;
	
	public Robot()
	{
		//No data
	}
	
	public Robot(String name)
	{
		this.name = name;
	}
	
	public Robot (String name, String info, Vocabulary vocab)
	{
		this.name = name;
		this.info = info;
		this.vocab = vocab;
	}
	
	public String getName()
	{
		return name;
	}
	
	public String getInfo()
	{
		return info;
	}
	
	public void setName(String newName)
	{
		name = newName;
	}
	
	public void setInfo(String newInfo)
	{
		info = newInfo;
	}
	
	public void setVocabulary(Vocabulary vocab)
	{
		this.vocab = vocab;
	}
	
	public NewPhrasesMessage getNextPhrases(ArrayList<String> previous)
	{
		return vocab.getNextPhrases(previous);
	}
}
