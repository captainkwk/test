// ------------------------------------------------------------------------
// Simulation script for GettingStarted Application   V1   2004-04-22   ETM
// ------------------------------------------------------------------------
// Automatic model simulation... this script acts like a (very) simplified 
// process resp. sometimes like the PLC. Especially flows and levels are
// periodically calculated out of device information (speed, position,...) 

main()
{
  int   cycleMS, restCycleMS;// loop times of the simulation in milliseconds 
  float pos1, pos2, pos3,    // position / opening grad of valves in [%]
        flow1, flow2, flow3, // flow after each valve in [l/s]
        level1, level2,      // tank levels in [m]
        oldlevel1, oldlevel2,// levels in previous loop
        level1max,level2max, // maximum tank levels in [m]
        level1HI,level2HI,   // switching points for automatic mode 
        level1LO,level2LO,   // switching points for automatic mode
        vol1, vol2,          // tank voluminas in [l]
        area1 = 1000, area2 = 4000,// tank area in [cm²]
        pspeed,              // pump speed in [%]
        maxFlow = 70,        // max flow through a pipe [l/s]
        noise, noise1, noise2,// maximum signal disturbance in percent of level1max
        cycleFloat;
  bool  modelSimOn, noInit = false,
        p1man, v1man, v2man, v3man; // manual operting mode of the pump and two valves
  time  startLoop, now;
  
  // simulate a runDry alarm for the pump whenever 
  // T1 is empty or V2 is closed when P1 is running !
  dpConnect("runDryAlarmCB","T1.level",
                            "V2.state.position",
                            "P1.state.speed",
                            "P1.cmd.speed");
                            
  dpGet("T1.level:_pv_range.._max",level1max, 
        "T2.level:_pv_range.._max",level2max);
        
  level1LO = 0.2*level1max;
  level1HI = 0.8*level1max;
  level2LO = 0.15*level2max;
  level2HI = 0.7*level2max;      
       

  while(true) //
  {
    startLoop = getCurrentTime();
    dpGet("SimControl.model.on",modelSimOn,
          "SimControl.model.cycleTime",cycleMS,
          "SimControl.model.noise",noise);

    if(modelSimOn)
    {
      dpGet("V1.state.position",pos1,
            "V1.state.manual",v1man,
            "V2.state.position",pos2,
            "V2.state.manual",v2man,    
            "V3.state.position",pos3,
            "V3.state.manual",v3man,      
            "T1.level",level1, 
            "T2.level",level2, 
            "P1.state.speed",pspeed,
            "P1.state.manual",p1man);
      
      cycleFloat = cycleMS/1000.0;

      noise1 = noise*level1max*0.01;
      noise2 = noise*level2max*0.01;
            
      flow1 = pos1 * 0.01 * maxFlow;
      flow2 = pos2 * 0.00012 * pspeed * maxFlow * sqrt(2*9.81*level1) * 0.26;
      flow3 = pos3 * 0.015 * maxFlow * sqrt(2*9.81*level2) * 0.23;
      
      level1 = level1 + ((flow1 - flow2) * cycleFloat)/area1;
      level2 = level2 + ((flow2 - flow3) * cycleFloat)/area2;

     if ((level1 < (level1max - noise1)) && (level1 > noise1))
       level1 = level1 + noise1 * ( (rand()/32767.0) - 0.5);

     if ((level2 < (level2max - noise2)) && (level2 > noise2))
       level2 = level2 + noise2 * ( (rand()/32767.0) - 0.5);


      
      if (level1 < 0)
        level1 = 0;
        
      if (level2 < 0)
        level2 = 0;

      if (level1 > level1max)
        level1 = level1max;
        
      if (level2 > level2max)
        level2 = level2max;
      
      if(noInit) 
      {           
        if(oldlevel1 != level1)
          dpSet("T1.level",level1);

        if(oldlevel2 != level2)
          dpSet("T2.level",level2); 
      }
      else
        dpSet("T1.level",level1,
              "T2.level",level2);
      
      // in auto mode, pumps and valves are switched 
      // dependent on the tank levels        
      if(level1 >= level1HI)
      {
        if(!p1man)
          dpSet("P1.cmd.speed",100);
        if(!v2man)
          dpSet("V2.cmd.position",100);
      }
      
      if(level1 <= level1LO)
      {
        if(!p1man)
          dpSet("P1.cmd.speed",0);
        delay(0,50);
        if(!v1man)
          dpSet("V1.cmd.position",85);        
        if(!v2man)
          dpSet("V2.cmd.position",0);
      }

      if(level2 >= level2HI)
      {
        if(!v3man)
          dpSet("V3.cmd.position",100);
      }

      if(level2 <= level2LO)
      {
        if(!v3man)
          dpSet("V3.cmd.position",0);
      }
            
      oldlevel1 = level1;
      oldlevel2 = level2;


      // calculate the used time from start of this loop and correct the 
      // required delay to have approx. correct looptimes
      now = getCurrentTime();
      restCycleMS = cycleMS - (1000 * (second(now - startLoop)) + (milliSecond(now - startLoop))); 
      if(restCycleMS < 5000)    
        delay(0, restCycleMS);
      else
        delay(0,1500);          
    }
    else
      delay(0,1500);  
  }
}
// end of "main()"
// ------------------------------------------------------------------------  


// ------------------------------------------------------------------------  
// Callback function to simulate a run dry problem for P1
runDryAlarmCB(string dp1, float T1level, string dp2, float V2pos, 
              string dp3, float P1speed, string dp4, float P1cmdSpeed)
{
  bool alarm;
 
  if(P1speed > 0)
  {
    if((T1level <= 0.05) || ((V2pos > 5)? (false) : (V2pos < P1speed)))
      dpSet("P1.alarm",true,
            "P1.cmd.speed",0);
  }
  else
    {
      dpGet("P1.alarm",alarm);
      if(alarm)
        dpSet("P1.alarm",0);        
    }
}
// end of "runDryAlarmCB()"
// ------------------------------------------------------------------------  