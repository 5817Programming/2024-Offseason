// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc.subsystems.Requests;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public abstract class Request {

	public Request(){
	}
    public  void act(){}
	
	public boolean isFinished(){return true;}

	public List<Prerequisite> prerequisites = new ArrayList<>();

	public void initialize(){

	}

	public void withPrerequisites(List<Prerequisite> reqs){
		for(Prerequisite req : reqs){
			prerequisites.add(req);
		}
	}

	public void withPrerequisite(Prerequisite req){
		prerequisites.add(req);
	}

	public boolean allowed(){
		boolean reqsMet = true;
		for(Prerequisite req : prerequisites){
			reqsMet &= req.met();
		}
		return reqsMet;
	}
}
