package com.riverlab.robotmanager.voice_recognition;

import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

import android.util.Log;

public class Vocabulary 
{
	List<String> mDefaultSubVocabList;
	private HashMap<String, ArrayList<Phrase>> mSubVocabMap;

	public Vocabulary (String rawXML)
	{
		Log.d("Vocabulary", "Creating vocabulary from XML");
		//Create the builder factory
		DocumentBuilderFactory builderFactory = DocumentBuilderFactory.newInstance();
		DocumentBuilder builder = null;

		//Create the builder from the builder factory
		try {
			builder = builderFactory.newDocumentBuilder();
		} catch (ParserConfigurationException e) {
			e.printStackTrace();  
		}

		//Use InputSource and StringReader to represent the rawXML String object
		//as a file and parse it using the builder. 
		Log.d("Vocabulary", "Reading document");
		Document document = null;
		try {
			InputSource is = new InputSource(new StringReader(rawXML));
			document = builder.parse(is);
		} catch (SAXException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		//Get the default sub-vocabs from the root vocab elemement's attributes
		Element rootElement = document.getDocumentElement();
		String defaultSubVocabs = rootElement.getAttribute("default");
		Log.d("Vocabulary", "Root element: " + rootElement.getTagName());
		String[] defaultSubVocabArray = defaultSubVocabs.split(",");
		this.mDefaultSubVocabList = Arrays.asList(defaultSubVocabArray);

		NodeList subVocabNodes = rootElement.getChildNodes();
		mSubVocabMap = new HashMap<String, ArrayList<Phrase>>();

		Log.d("Vocabulary", "Reading sub-vocabs");
		for (int i = 0; i < subVocabNodes.getLength(); i++)
		{
			if (!(subVocabNodes.item(i).getNodeType() == Node.ELEMENT_NODE))
			{
				continue;
			}

			Element subVocab = (Element) subVocabNodes.item(i);
			String subVocabName = subVocab.getAttribute("name");

			NodeList phraseNodes = subVocab.getChildNodes();
			ArrayList<Phrase> phrases = new ArrayList<Phrase>();
			for (int j = 0; j < phraseNodes.getLength(); j++)
			{
				if (!(phraseNodes.item(j).getNodeType() == Node.ELEMENT_NODE))
				{
					continue;
				}

				Element phraseNode = (Element) phraseNodes.item(j);

				String tempName = phraseNode.getAttribute("text");

				NodeList modifiers = phraseNode.getChildNodes();

				if (modifiers.getLength() == 0)
				{
					phrases.add(new Phrase(tempName, new ArrayList<Modifier>()));
				}
				else
				{
					ArrayList<Modifier> tempModList = new ArrayList<Vocabulary.Modifier>();
					for (int k = 0; k < modifiers.getLength(); k++)
					{
						if (!(modifiers.item(k).getNodeType() == Node.ELEMENT_NODE))
						{
							continue;
						}
						//Construct the remainder depth of this tree using 
						//recursive modifier constructor
						Element mod = (Element)modifiers.item(k);
						tempModList.add(new Modifier(mod));
					}
					phrases.add(new Phrase(tempName, tempModList));
				}
			}
			mSubVocabMap.put(subVocabName, phrases);
		}
	}

	public Phrase findPhrase(String phraseText)
	{
		//Search sub-vocabularies for phrase
		for (Map.Entry<String, ArrayList<Phrase>> subVocab : mSubVocabMap.entrySet())
		{
			ArrayList<Phrase> phrases = subVocab.getValue();

			//Search through phrases in sub-vocabulary for phrase
			for (Phrase phrase : phrases)
			{
				if (phrase.text.equals(phraseText))
				{
					return phrase;
				}
			}
		}
		//No such phrase
		return null;
	}

	public Modifier findModifier(String phraseText, ArrayList<Modifier> modifiers)
	{
		String modifierUses = "";

		//Search sub-vocabularies for phrase
		searchloop:
			for (Map.Entry<String, ArrayList<Phrase>> subVocab : mSubVocabMap.entrySet())
			{
				ArrayList<Phrase> phrases = subVocab.getValue();

				//Search through phrases in sub-vocabulary for phrase
				for (Phrase phrase : phrases)
				{
					if (phrase.text.equals(phraseText))
					{
						modifierUses = subVocab.getKey();
						break searchloop;
					}
				}
			}			

		for (Modifier mod : modifiers)
		{
			if (mod.getUses() == modifierUses)
			{
				return mod;
			}
		}

		return null;
	}

	public NewPhrasesMessage getNextPhrases(ArrayList<String> previous)
	{
		ArrayList<String> rtnList = new ArrayList<String>();
		boolean isRequired = true;
		boolean reset = false;

		//If no phrases have been said, return the default list
		if (previous.size() == 0)
		{
			for (String subVocabName : mDefaultSubVocabList)
			{
				for (Phrase phrase : mSubVocabMap.get(subVocabName))
				{
					rtnList.add(phrase.getText());
				}
			}
		}
		else //If a phrase has already been said
		{
			Phrase rootPhrase = findPhrase(previous.remove(0));
			VocabMessage msg = rootPhrase.getNextSubVocabs(previous);
			ArrayList<String> nextSubVocabs = msg.subVocabNames;
			isRequired = msg.isRequired();

			//If there is nothing left to say, return default list and reset
			if (nextSubVocabs.size() == 0)
			{
				for (String subVocabName : mDefaultSubVocabList)
				{
					for (Phrase phrase : mSubVocabMap.get(subVocabName))
					{
						rtnList.add(phrase.getText());
					}
				}
				reset = true;
			}
			else //If there are still phrases left to say, get them from their subVocabs
			{			
				for (String subVocabName : nextSubVocabs)
				{
					try
					{
						for (Phrase phrase : mSubVocabMap.get(subVocabName))
						{
							rtnList.add(phrase.getText());
						}
					}
					catch (NullPointerException e)
					{
						Log.w("Vocabulary", 
								"No subvocabulary \"" + subVocabName + "\" was found. Please check your vocabulary XML file.");
						e.printStackTrace();
					}
				}
			}
		}
		return new NewPhrasesMessage(rtnList, isRequired, reset);
	}



	public class Phrase
	{
		private String text;
		private boolean isRequired = true;
		private ArrayList<Modifier> modifiers;

		public Phrase(String text, ArrayList<Modifier> modifiers)
		{
			this.text = text;
			this.modifiers = modifiers;
		}

		public String getText()
		{
			return text;
		}

		public boolean isRequired()
		{
			return isRequired;
		}

		public ArrayList<Modifier> getModifiers()
		{
			return modifiers;
		}

		public void setText(String text)
		{
			this.text = text;
		}

		public void setRequired(boolean required)
		{
			this.isRequired = required;
		}

		public void setModifiers(ArrayList<Modifier> modifiers)
		{
			this.modifiers = modifiers;
		}

		public VocabMessage getNextSubVocabs(ArrayList<String> previous)
		{
			ArrayList<String> rtnList = new ArrayList<String>();

			if (previous.size() == 0)
			{
				if (modifiers.size() == 0)
				{
					return new VocabMessage(rtnList, false);
				}
				else
				{
					boolean required = false;
					for (Modifier mod : modifiers)
					{
						rtnList.add(mod.getUses());
						required |= mod.isRequired();
					}
					return new VocabMessage(rtnList, required);
				}
			}
			else
			{
				Modifier usedMod = findModifier(previous.remove(0), modifiers);
				if (usedMod == null)
				{
					Log.d("CommandInterpreter", "Error finding modifier, check for voice recognition bounce");
					return new VocabMessage(new ArrayList<String>(), false);
				}
				return usedMod.getNextSubVocabs(previous);
			}
		}
	}

	public class Modifier
	{
		private String uses;
		private boolean isRequired;
		private ArrayList<Modifier> modifiers;

		public Modifier(Element modifierNode)
		{
			this.uses = modifierNode.getAttribute("use");
			this.isRequired = !(modifierNode.getAttribute("required").equals("false"));

			Log.d("Vocabulary", "Constructing modifier using " + this.uses);

			NodeList additionalMods = modifierNode.getChildNodes();

			this.modifiers = new ArrayList<Vocabulary.Modifier>();

			for (int i = 0; i < additionalMods.getLength(); i++)
			{
				if (!(additionalMods.item(i).getNodeType() == Node.ELEMENT_NODE))
				{
					continue;
				}

				Element elm = (Element)additionalMods.item(i);

				this.modifiers.add(new Modifier(elm));
			}
		}

		public String getUses()
		{
			return uses;
		}

		public boolean isRequired()
		{
			return isRequired;
		}

		public ArrayList<Modifier> getModifiers()
		{
			return modifiers;
		}

		public void setUses(String uses)
		{
			this.uses = uses;
		}

		public void setRequired(boolean required)
		{
			isRequired = required;
		}

		public void setModifiers(ArrayList<Modifier> modifiers)
		{
			this.modifiers = modifiers;
		}

		public VocabMessage getNextSubVocabs(ArrayList<String> previous)
		{
			ArrayList<String> rtnList = new ArrayList<String>();

			if (previous.size() == 0)
			{
				if (modifiers.size() == 0)
				{
					return new VocabMessage(rtnList, false);
				}
				else
				{
					boolean required = false;
					for (Modifier mod : modifiers)
					{
						rtnList.add(mod.getUses());
						required |= mod.isRequired();
					}

					return new VocabMessage(rtnList, required);
				}
			}
			else
			{
				Modifier usedMod = findModifier(previous.remove(0), modifiers);
				return usedMod.getNextSubVocabs(previous);
			}
		}
	}

	public class VocabMessage
	{
		private ArrayList<String> subVocabNames;
		private boolean isRequired;

		public VocabMessage(ArrayList<String> subVocabNames, boolean isRequired)
		{
			this.subVocabNames = subVocabNames;
			this.isRequired = isRequired;
		}

		public ArrayList<String> getSubVocabNames() 
		{
			return subVocabNames;
		}

		public void setSubVocabNames(ArrayList<String> subVocabNames) 
		{
			this.subVocabNames = subVocabNames;
		}

		public boolean isRequired() 
		{
			return isRequired;
		}

		public void setRequired(boolean isRequired) 
		{
			this.isRequired = isRequired;
		}

	}
}
