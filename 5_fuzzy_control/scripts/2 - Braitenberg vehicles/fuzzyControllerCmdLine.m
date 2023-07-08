clear all
%%
Fear = mamfis('Name',"Fear");

Fear = addInput(Fear,[0 1],'Name',"LeftLight");
Fear = addMF(Fear,"LeftLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Fear = addMF(Fear,"LeftLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Fear = addMF(Fear,"LeftLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Fear = addInput(Fear,[0 1],'Name',"RightLight");
Fear = addMF(Fear,"RightLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Fear = addMF(Fear,"RightLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Fear = addMF(Fear,"RightLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Fear = addOutput(Fear,[0 2],'Name',"LeftMotor");
Fear = addMF(Fear,"LeftMotor","trimf",[-1 0 1],'Name',"Slow");
Fear = addMF(Fear,"LeftMotor","trimf",[0 1 2],'Name',"Medium");
Fear = addMF(Fear,"LeftMotor","trimf",[1 2 3],'Name',"Fast");

Fear = addOutput(Fear,[0 2],'Name',"RightMotor");
Fear = addMF(Fear,"RightMotor","trimf",[-1 0 1],'Name',"Slow");
Fear = addMF(Fear,"RightMotor","trimf",[0 1 2],'Name',"Medium");
Fear = addMF(Fear,"RightMotor","trimf",[1 2 3],'Name',"Fast");

ruleList = [1 0 1 0 1 1;
            2 0 2 0 1 1;
            3 0 3 0 1 1;
            0 1 0 1 1 1;
            0 2 0 2 1 1;
            0 3 0 3 1 1;];
Fear = addRule(Fear,ruleList);
%%
Agression = mamfis('Name',"Agression");

Agression = addInput(Agression,[0 1],'Name',"LeftLight");
Agression = addMF(Agression,"LeftLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Agression = addMF(Agression,"LeftLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Agression = addMF(Agression,"LeftLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Agression = addInput(Agression,[0 1],'Name',"RightLight");
Agression = addMF(Agression,"RightLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Agression = addMF(Agression,"RightLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Agression = addMF(Agression,"RightLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Agression = addOutput(Agression,[0 2],'Name',"LeftMotor");
Agression = addMF(Agression,"LeftMotor","trimf",[-1 0 1],'Name',"Slow");
Agression = addMF(Agression,"LeftMotor","trimf",[0 1 2],'Name',"Medium");
Agression = addMF(Agression,"LeftMotor","trimf",[1 2 3],'Name',"Fast");

Agression = addOutput(Agression,[0 2],'Name',"RightMotor");
Agression = addMF(Agression,"RightMotor","trimf",[-1 0 1],'Name',"Slow");
Agression = addMF(Agression,"RightMotor","trimf",[0 1 2],'Name',"Medium");
Agression = addMF(Agression,"RightMotor","trimf",[1 2 3],'Name',"Fast");

ruleList = [0 1 1 0 1 1;
            0 2 2 0 1 1;
            0 3 3 0 1 1;
            1 0 0 1 1 1;
            2 0 0 2 1 1;
            3 0 0 3 1 1;];
Agression = addRule(Agression,ruleList);
%%
Love = mamfis('Name',"Love");

Love = addInput(Love,[0 1],'Name',"LeftLight");
Love = addMF(Love,"LeftLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Love = addMF(Love,"LeftLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Love = addMF(Love,"LeftLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Love = addInput(Love,[0 1],'Name',"RightLight");
Love = addMF(Love,"RightLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Love = addMF(Love,"RightLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Love = addMF(Love,"RightLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Love = addOutput(Love,[0 2],'Name',"LeftMotor");
Love = addMF(Love,"LeftMotor","trimf",[-1 0 1],'Name',"Slow");
Love = addMF(Love,"LeftMotor","trimf",[0 1 2],'Name',"Medium");
Love = addMF(Love,"LeftMotor","trimf",[1 2 3],'Name',"Fast");

Love = addOutput(Love,[0 2],'Name',"RightMotor");
Love = addMF(Love,"RightMotor","trimf",[-1 0 1],'Name',"Slow");
Love = addMF(Love,"RightMotor","trimf",[0 1 2],'Name',"Medium");
Love = addMF(Love,"RightMotor","trimf",[1 2 3],'Name',"Fast");

ruleList = [3 0 1 0 1 1;
            2 0 2 0 1 1;
            1 0 3 0 1 1;
            0 3 0 1 1 1;
            0 2 0 2 1 1;
            0 1 0 3 1 1;];
Love = addRule(Love,ruleList);
%%
Curious = mamfis('Name',"Curious");

Curious = addInput(Curious,[0 1],'Name',"LeftLight");
Curious = addMF(Curious,"LeftLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Curious = addMF(Curious,"LeftLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Curious = addMF(Curious,"LeftLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Curious = addInput(Curious,[0 1],'Name',"RightLight");
Curious = addMF(Curious,"RightLight","trapmf",[-0.2 -0.2 0.3 0.6],'Name',"Dark");
Curious = addMF(Curious,"RightLight","trimf",[0.3 0.6 0.9],'Name',"Shadow");
Curious = addMF(Curious,"RightLight","trapmf",[0.6 0.9 1.2 1.2],'Name',"Bright");

Curious = addOutput(Curious,[0 2],'Name',"LeftMotor");
Curious = addMF(Curious,"LeftMotor","trimf",[-1 0 1],'Name',"Slow");
Curious = addMF(Curious,"LeftMotor","trimf",[0 1 2],'Name',"Medium");
Curious = addMF(Curious,"LeftMotor","trimf",[1 2 3],'Name',"Fast");

Curious = addOutput(Curious,[0 2],'Name',"RightMotor");
Curious = addMF(Curious,"RightMotor","trimf",[-1 0 1],'Name',"Slow");
Curious = addMF(Curious,"RightMotor","trimf",[0 1 2],'Name',"Medium");
Curious = addMF(Curious,"RightMotor","trimf",[1 2 3],'Name',"Fast");

ruleList = [0 3 1 0 1 1;
            0 2 2 0 1 1;
            0 1 3 0 1 1;
            3 0 0 1 1 1;
            2 0 0 2 1 1;
            1 0 0 3 1 1;];
Curious = addRule(Curious,ruleList);

clear ruleList