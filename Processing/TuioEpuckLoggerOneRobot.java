/*
	TUIO Java Example - part of the reacTIVision project
	http://reactivision.sourceforge.net/

	Copyright (c) 2005-2009 Martin Kaltenbrunner <mkalten@iua.upf.edu>

	This program is free software;you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation;either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY;without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program;if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

import javax.swing.*;
import java.awt.geom.*;
import java.awt.*;
import java.awt.event.*;
import TUIO.*;
import java.io.*;
import java.util.Timer;
import java.util.TimerTask;
import java.util.Calendar;
import java.util.Date;
import java.text.SimpleDateFormat;


public class TuioEpuckLoggerOneRobot implements TuioListener {

	private static  double [] data = new double[3];
	private static PrintWriter out;
	
	public void addTuioObject(TuioObject tobj) {
		System.out.println("add obj "+tobj.getSymbolID()+" ("+tobj.getSessionID()+") "+tobj.getX()+" "+tobj.getY()+" "+tobj.getAngle());	
	}

	public void updateTuioObject(TuioObject tobj) {
		if (tobj.getSymbolID()==1) {
			data[0]=tobj.getX();
			data[1]=tobj.getY();
			data[2]=tobj.getAngle();
		}
		//System.out.println("set obj "++" ("+tobj.getSessionID()+") "++" "+tobj.getY()+" "+tobj.getAngle()+" "+tobj.getMotionSpeed()+" "+tobj.getRotationSpeed()+" "+tobj.getMotionAccel()+" "+tobj.getRotationAccel());
	}
	
	public void removeTuioObject(TuioObject tobj) {
		System.out.println("del obj "+tobj.getSymbolID()+" ("+tobj.getSessionID()+")");	
	}

	public void addTuioCursor(TuioCursor tcur) {
		System.out.println("add cur "+tcur.getCursorID()+" ("+tcur.getSessionID()+") "+tcur.getX()+" "+tcur.getY());	
	}

	public void updateTuioCursor(TuioCursor tcur) {
		System.out.println("set cur "+tcur.getCursorID()+" ("+tcur.getSessionID()+") "+tcur.getX()+" "+tcur.getY()+" "+tcur.getMotionSpeed()+" "+tcur.getMotionAccel());
	}
	
	public void removeTuioCursor(TuioCursor tcur) {
		System.out.println("del cur "+tcur.getCursorID()+" ("+tcur.getSessionID()+")");	
	}
	
	public void refresh(TuioTime frameTime) {
		//System.out.println("refresh "+frameTime.getTotalMilliseconds());
	}

	public static void main(String argv[]) {
	
		int port = 3333;
		
		Date today = Calendar.getInstance().getTime();
		SimpleDateFormat formatter = new SimpleDateFormat("yyyyMMdd.HHmmss"); 
		String dateStr = formatter.format(today);

		if (argv.length==1) {
			try { port = Integer.parseInt(argv[0]); }
			catch (Exception e) { System.out.println("usage: java TuioEpuckLoggerOneRobot [port]"); }
		}
        try {out = new PrintWriter("realexp"+dateStr+".txt");}
			catch (Exception e) { System.out.println("Error opening file"); }
		
		TuioEpuckLoggerOneRobot demo = new TuioEpuckLoggerOneRobot();
		TuioClient client = new TuioClient(port);

		System.out.println("listening to TUIO messages at port "+port);
		client.addTuioListener(demo);
		client.connect();
		
		
		
		// write every 100 ms
		Timer timer = new Timer();
		timer.scheduleAtFixedRate(new TimerTask()
		{
 
			@Override
			public void run()
			{
				//put code here
				// latest known position of robots (x,y,angle)
				out.println(Double.toString(data[0])+","+Double.toString(data[1])+","+Double.toString(data[2]));
				// reset to -1 to check if its a new value
				data[0]=-1;
				data[1]=-1;
				data[2]=-1;
			}
		}, 0, 100);
		
		// for 2 hours
		//Thread.sleep(7200000); 
		
		try {Thread.sleep(30000); }
			catch (Exception e) { System.out.println("Error while sleeping"); }
		timer.cancel();
		timer.purge();
		client.disconnect();
		out.close();
		return;
	}
}
