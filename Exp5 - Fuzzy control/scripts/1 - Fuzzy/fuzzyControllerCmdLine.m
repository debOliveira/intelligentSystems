clear all
%%
GoalSeeking = mamfis('Name',"GoalSeeking");

GoalSeeking = addInput(GoalSeeking,[0 10],'Name',"GoalDistance");
GoalSeeking = addMF(GoalSeeking,"GoalDistance","trimf",[-5 0 5],'Name',"Near");
GoalSeeking = addMF(GoalSeeking,"GoalDistance","trimf",[0 5 10],'Name',"Medium");
GoalSeeking = addMF(GoalSeeking,"GoalDistance","trimf",[5 10 15],'Name',"Far");

GoalSeeking = addInput(GoalSeeking,[-180 180],'Name',"SteeringAngle");
GoalSeeking = addMF(GoalSeeking,"SteeringAngle","trapmf",[-200 -200 -90 -60],'Name',"NB");
GoalSeeking = addMF(GoalSeeking,"SteeringAngle","trimf",[-90 -60 -30],'Name',"NM");
GoalSeeking = addMF(GoalSeeking,"SteeringAngle","trimf",[-60 -30 0],'Name',"NS");
GoalSeeking = addMF(GoalSeeking,"SteeringAngle","trimf",[-30 0 30],'Name',"ZE");
GoalSeeking = addMF(GoalSeeking,"SteeringAngle","trimf",[0 30 60],'Name',"PS");
GoalSeeking = addMF(GoalSeeking,"SteeringAngle","trimf",[30 60 90],'Name',"PM");
GoalSeeking = addMF(GoalSeeking,"SteeringAngle","trapmf",[60 90 200 200],'Name',"PB");

GoalSeeking = addOutput(GoalSeeking,[-30 30],'Name',"TurningAngle");
GoalSeeking = addMF(GoalSeeking,"TurningAngle","trimf",[-45 -30 -15],'Name',"LB");
GoalSeeking = addMF(GoalSeeking,"TurningAngle","trimf",[-22.5 -15 -7.5],'Name',"LM");
GoalSeeking = addMF(GoalSeeking,"TurningAngle","trimf",[-15 -7.5 0],'Name',"LS");
GoalSeeking = addMF(GoalSeeking,"TurningAngle","trimf",[-7.5 0 7.5],'Name',"ZE");
GoalSeeking = addMF(GoalSeeking,"TurningAngle","trimf",[0 7.5 15],'Name',"RS");
GoalSeeking = addMF(GoalSeeking,"TurningAngle","trimf",[7.5 15 22.5],'Name',"RM");
GoalSeeking = addMF(GoalSeeking,"TurningAngle","trimf",[15 30 45],'Name',"RB");

GoalSeeking = addOutput(GoalSeeking,[0 0.3],'Name',"LinearVel");
GoalSeeking = addMF(GoalSeeking,"LinearVel","trimf",[-0.15 0 0.15],'Name',"Slow");
GoalSeeking = addMF(GoalSeeking,"LinearVel","trimf",[0 0.15 0.3],'Name',"Medium");
GoalSeeking = addMF(GoalSeeking,"LinearVel","trimf",[0.15 0.3 0.45],'Name',"Fast");

ruleList = [1 0 0 1 1 1;
            2 0 0 2 1 1;
            3 0 0 3 1 1;
            0 1 1 0 1 1;
            0 2 2 0 1 1;
            0 3 3 0 1 1;
            0 4 4 0 1 1;
            0 5 5 0 1 1;
            0 6 6 0 1 1;
            0 7 7 0 1 1];
GoalSeeking = addRule(GoalSeeking,ruleList);
%%
ObstacleAvoidance = mamfis('Name',"ObstacleAvoidance");

ObstacleAvoidance = addInput(ObstacleAvoidance,[0 2],'Name',"ObstacleDistance");
ObstacleAvoidance = addMF(ObstacleAvoidance,"ObstacleDistance","trapmf",[0 0 0.5 1],'Name',"Near");
ObstacleAvoidance = addMF(ObstacleAvoidance,"ObstacleDistance","trimf",[0.5 1 1.5],'Name',"Medium");
ObstacleAvoidance = addMF(ObstacleAvoidance,"ObstacleDistance","trapmf",[1 1.5 2 2],'Name',"Far");

ObstacleAvoidance = addInput(ObstacleAvoidance,[-2 2],'Name',"SideObstacleDistance");
ObstacleAvoidance = addMF(ObstacleAvoidance,"SideObstacleDistance","trapmf",[-3 -3 -1 -0.5],'Name',"RL");
ObstacleAvoidance = addMF(ObstacleAvoidance,"SideObstacleDistance","trimf",[-1 -0.5 0],'Name',"RS");
ObstacleAvoidance = addMF(ObstacleAvoidance,"SideObstacleDistance","trimf",[-0.1 0 0.1],'Name',"ZE");
ObstacleAvoidance = addMF(ObstacleAvoidance,"SideObstacleDistance","trimf",[0 0.5 1],'Name',"LS");
ObstacleAvoidance = addMF(ObstacleAvoidance,"SideObstacleDistance","trapmf",[0.5 1 3 3],'Name',"LL");

ObstacleAvoidance = addOutput(ObstacleAvoidance,[-30 30],'Name',"TurningAngle");
ObstacleAvoidance = addMF(ObstacleAvoidance,"TurningAngle","trimf",[-45 -30 -15],'Name',"LB");
ObstacleAvoidance = addMF(ObstacleAvoidance,"TurningAngle","trimf",[-22.5 -15 -7.5],'Name',"LM");
ObstacleAvoidance = addMF(ObstacleAvoidance,"TurningAngle","trimf",[-15 -7.5 0],'Name',"LS");
ObstacleAvoidance = addMF(ObstacleAvoidance,"TurningAngle","trimf",[-7.5 0 7.5],'Name',"ZE");
ObstacleAvoidance = addMF(ObstacleAvoidance,"TurningAngle","trimf",[0 7.5 15],'Name',"RS");
ObstacleAvoidance = addMF(ObstacleAvoidance,"TurningAngle","trimf",[7.5 15 22.5],'Name',"RM");
ObstacleAvoidance = addMF(ObstacleAvoidance,"TurningAngle","trimf",[15 30 45],'Name',"RB");

ObstacleAvoidance = addOutput(ObstacleAvoidance,[0 0.3],'Name',"LinearVel");
ObstacleAvoidance = addMF(ObstacleAvoidance,"LinearVel","trimf",[-0.15 0 0.15],'Name',"Slow");
ObstacleAvoidance = addMF(ObstacleAvoidance,"LinearVel","trimf",[0 0.15 0.3],'Name',"Medium");
ObstacleAvoidance = addMF(ObstacleAvoidance,"LinearVel","trimf",[0.15 0.3 0.45],'Name',"Fast");

ruleList = [1 1 7 1 1 1;
            1 2 7 1 1 1;
            1 3 7 1 1 1;
            1 4 1 1 1 1;
            1 5 1 1 1 1;
            2 1 6 2 1 1;
            2 2 6 2 1 1;
            2 3 6 2 1 1;
            2 4 2 2 1 1;
            2 5 2 2 1 1;
            3 1 5 3 1 1;
            3 2 5 3 1 1;
            3 3 5 3 1 1;
            3 4 3 3 1 1;
            3 5 3 3 1 1];
ObstacleAvoidance = addRule(ObstacleAvoidance,ruleList);
%%
Tracking = mamfis('Name',"Tracking");

Tracking = addInput(Tracking,[-180 180],'Name',"SteeringAngle");
Tracking = addMF(Tracking,"SteeringAngle","trapmf",[-200 -200 -90 -60],'Name',"NB");
Tracking = addMF(Tracking,"SteeringAngle","trimf",[-90 -60 -30],'Name',"NM");
Tracking = addMF(Tracking,"SteeringAngle","trimf",[-60 -30 0],'Name',"NS");
Tracking = addMF(Tracking,"SteeringAngle","trimf",[-30 0 30],'Name',"ZE");
Tracking = addMF(Tracking,"SteeringAngle","trimf",[0 30 60],'Name',"PS");
Tracking = addMF(Tracking,"SteeringAngle","trimf",[30 60 90],'Name',"PM");
Tracking = addMF(Tracking,"SteeringAngle","trapmf",[60 90 200 200],'Name',"PB");

Tracking = addInput(Tracking,[-2 2],'Name',"SideObstacleDistance");
Tracking = addMF(Tracking,"SideObstacleDistance","trapmf",[-3 -3 -1 -0.5],'Name',"RL");
Tracking = addMF(Tracking,"SideObstacleDistance","trimf",[-1 -0.5 0],'Name',"RS");
Tracking = addMF(Tracking,"SideObstacleDistance","trimf",[-0.1 0 0.1],'Name',"ZE");
Tracking = addMF(Tracking,"SideObstacleDistance","trimf",[0 0.5 1],'Name',"LS");
Tracking = addMF(Tracking,"SideObstacleDistance","trapmf",[0.5 1 3 3],'Name',"LL");

Tracking = addOutput(Tracking,[-30 30],'Name',"TurningAngle");
Tracking = addMF(Tracking,"TurningAngle","trimf",[-45 -30 -15],'Name',"LB");
Tracking = addMF(Tracking,"TurningAngle","trimf",[-22.5 -15 -7.5],'Name',"LM");
Tracking = addMF(Tracking,"TurningAngle","trimf",[-15 -7.5 0],'Name',"LS");
Tracking = addMF(Tracking,"TurningAngle","trimf",[-7.5 0 7.5],'Name',"ZE");
Tracking = addMF(Tracking,"TurningAngle","trimf",[0 7.5 15],'Name',"RS");
Tracking = addMF(Tracking,"TurningAngle","trimf",[7.5 15 22.5],'Name',"RM");
Tracking = addMF(Tracking,"TurningAngle","trimf",[15 30 45],'Name',"RB");

Tracking = addOutput(Tracking,[0 0.3],'Name',"LinearVel");
Tracking = addMF(Tracking,"LinearVel","trimf",[-0.15 0 0.15],'Name',"Slow");
Tracking = addMF(Tracking,"LinearVel","trimf",[0 0.15 0.3],'Name',"Medium");
Tracking = addMF(Tracking,"LinearVel","trimf",[0.15 0.3 0.45],'Name',"Fast");

ruleList = [1 5 1 2 1 1;
            1 4 1 2 1 1;
            1 2 4 3 1 1;
            1 1 4 3 1 1;
            2 5 2 2 1 1;
            2 4 2 2 1 1;
            2 2 4 3 1 1;
            2 1 4 3 1 1;
            3 5 3 2 1 1;
            3 4 3 2 1 1;
            3 2 4 3 1 1;
            3 1 4 3 1 1;            
            4 0 4 3 1 1;
            0 4 4 1 1 1;
            5 1 5 2 1 1;
            5 2 5 2 1 1;
            5 4 4 3 1 1;
            5 5 4 3 1 1;
            6 1 6 2 1 1;
            6 2 6 2 1 1;
            6 4 4 3 1 1;
            6 5 4 3 1 1;
            7 1 7 2 1 1;
            7 2 7 2 1 1;
            7 4 4 3 1 1;
            7 5 4 3 1 1];
Tracking = addRule(Tracking,ruleList);
%%
DeadlockDisarming = mamfis('Name',"DeadlockDisarming");

DeadlockDisarming = addInput(DeadlockDisarming,[0 2],'Name',"ObstacleDistance");
DeadlockDisarming = addMF(DeadlockDisarming,"ObstacleDistance","trapmf",[0 0 0.5 1],'Name',"Near");
DeadlockDisarming = addMF(DeadlockDisarming,"ObstacleDistance","trimf",[0.5 1 1.5],'Name',"Medium");
DeadlockDisarming = addMF(DeadlockDisarming,"ObstacleDistance","trapmf",[1 1.5 2 2],'Name',"Far");

DeadlockDisarming = addInput(DeadlockDisarming,[-180 180],'Name',"SteeringAngle");
DeadlockDisarming = addMF(DeadlockDisarming,"SteeringAngle","trapmf",[-200 -200 -90 -60],'Name',"NB");
DeadlockDisarming = addMF(DeadlockDisarming,"SteeringAngle","trimf",[-90 -60 -30],'Name',"NM");
DeadlockDisarming = addMF(DeadlockDisarming,"SteeringAngle","trimf",[-60 -30 0],'Name',"NS");
DeadlockDisarming = addMF(DeadlockDisarming,"SteeringAngle","trimf",[-30 0 30],'Name',"ZE");
DeadlockDisarming = addMF(DeadlockDisarming,"SteeringAngle","trimf",[0 30 60],'Name',"PS");
DeadlockDisarming = addMF(DeadlockDisarming,"SteeringAngle","trimf",[30 60 90],'Name',"PM");
DeadlockDisarming = addMF(DeadlockDisarming,"SteeringAngle","trapmf",[60 90 200 200],'Name',"PB");

DeadlockDisarming = addOutput(DeadlockDisarming,[-30 30],'Name',"TurningAngle");
DeadlockDisarming = addMF(DeadlockDisarming,"TurningAngle","trimf",[-45 -30 -15],'Name',"LB");
DeadlockDisarming = addMF(DeadlockDisarming,"TurningAngle","trimf",[-22.5 -15 -7.5],'Name',"LM");
DeadlockDisarming = addMF(DeadlockDisarming,"TurningAngle","trimf",[-15 -7.5 0],'Name',"LS");
DeadlockDisarming = addMF(DeadlockDisarming,"TurningAngle","trimf",[-7.5 0 7.5],'Name',"ZE");
DeadlockDisarming = addMF(DeadlockDisarming,"TurningAngle","trimf",[0 7.5 15],'Name',"RS");
DeadlockDisarming = addMF(DeadlockDisarming,"TurningAngle","trimf",[7.5 15 22.5],'Name',"RM");
DeadlockDisarming = addMF(DeadlockDisarming,"TurningAngle","trimf",[15 30 45],'Name',"RB");

DeadlockDisarming = addOutput(DeadlockDisarming,[0 0.3],'Name',"LinearVel");
DeadlockDisarming = addMF(DeadlockDisarming,"LinearVel","trimf",[-0.15 0 0.15],'Name',"Slow");
DeadlockDisarming = addMF(DeadlockDisarming,"LinearVel","trimf",[0 0.15 0.3],'Name',"Medium");
DeadlockDisarming = addMF(DeadlockDisarming,"LinearVel","trimf",[0.15 0.3 0.45],'Name',"Fast");

ruleList = [1 1 1 1 1 1;
            1 2 1 1 1 1;
            1 3 1 1 1 1;
            1 4 4 1 1 1;
            1 5 7 1 1 1;
            1 6 7 1 1 1;
            1 7 7 1 1 1;
            2 1 2 2 1 1;
            2 2 2 2 1 1;
            2 3 2 2 1 1;
            2 4 4 2 1 1;
            2 5 6 2 1 1;
            2 6 6 2 1 1;
            2 7 6 3 1 1;
            3 1 3 3 1 1;
            3 2 3 3 1 1;
            3 3 3 3 1 1;
            3 4 4 3 1 1;
            3 5 5 3 1 1;
            3 6 5 3 1 1;
            3 7 5 3 1 1];
DeadlockDisarming = addRule(DeadlockDisarming,ruleList);

clear ruleList