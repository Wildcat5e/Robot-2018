package org.usfirst.frc.team6705.robot;

import java.io.File;
import java.io.IOException;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.PrintWriter;

public class CSVLogger {

	File file;
	
	public CSVLogger(String fileName, String[] columnNames) {
		
		try {
    		file = new File("/U/" + fileName + ".csv");
    		if(!file.exists()){
    			file.createNewFile();
    		} else {
    			file.delete();
    			file.createNewFile();
    		}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		try {
			
			FileWriter fw = new FileWriter(file,true);
	    	BufferedWriter bw = new BufferedWriter(fw);
	    	PrintWriter pw = new PrintWriter(bw);
	        StringBuilder sb = new StringBuilder();
	        
	        for (int i = 0; i < columnNames.length; i++) {
	        	sb.append(columnNames[i]);
	        	if (i != columnNames.length - 1) {
	        		sb.append(',');
	        	}
	        }
	        //sb.append('\n');
	        
	        pw.println(sb.toString());
	        pw.close();
		} catch (IOException ie) {
			ie.printStackTrace();
		}
		
	}
	
	public void reset() {
		if(file.exists()){
			//file.delete();
			//file.createNewFile();
			writeLine(new String[]{"INIT"});
		}
	}
	
	public void writeLine(String[] line) {
		try {
			
			FileWriter fw = new FileWriter(file,true);
	    	BufferedWriter bw = new BufferedWriter(fw);
	    	PrintWriter pw = new PrintWriter(bw);
	        StringBuilder sb = new StringBuilder();
	        
	        for (int i = 0; i < line.length; i++) {
	        	sb.append(line[i]);
	        	if (i != line.length - 1) {
	        		sb.append(',');
	        	}
	        }
	        //sb.append('\n');
	        
	        pw.println(sb.toString());
	        pw.close();
		} catch (IOException ie) {
			ie.printStackTrace();
		}
	}
	
}
