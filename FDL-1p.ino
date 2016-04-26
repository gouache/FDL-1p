
//setup photon to connect to wifi on demand
SYSTEM_MODE(SEMI_AUTOMATIC);

//define pins
int stepperAngleStep = D0;
int stepperAngleDir = D1;
int stepperAltStep = D2;
int stepperAltDir = D3;
int stepperTurnEnable = D4;
int stepperFireStep = D5;
int stepperAdvanceStep = D6;
int stepperEnable = D7;

int advSenseIn = A0;
int plungerSenseIn = A1;
int connSenseIn = A3;
int speedSenseIn = A5;
int triggerSenseIn = TX;

// create servo object to control the ESC's
Servo flywheelESC;
int escControlPin = A4;

double azPosition = 0.0;
bool firstRun = true;
unsigned long disableMillis = millis();

// This routine runs only once upon reset
void setup() {

  //associates esc control pin to servo object
  flywheelESC.attach(escControlPin);

  // Initialize pins
  // It's important you do this here, inside the setup() function rather than outside it or in the loop function.
  pinMode(stepperAdvanceStep, OUTPUT);
  pinMode(stepperFireStep, OUTPUT);
  pinMode(stepperAngleStep, OUTPUT);
  pinMode(stepperAngleDir, OUTPUT);
  pinMode(stepperEnable, OUTPUT);
  pinMode(stepperTurnEnable, OUTPUT);
  pinMode(stepperAltStep, OUTPUT);
  pinMode(stepperAltDir, OUTPUT);

  pinMode(advSenseIn, INPUT_PULLDOWN);
  pinMode(plungerSenseIn, INPUT_PULLDOWN);
  pinMode(triggerSenseIn, INPUT_PULLDOWN);
  pinMode(connSenseIn, INPUT_PULLDOWN);

  pinMode(speedSenseIn, INPUT);

  // Turn off steppers (HIGH)
  digitalWrite(stepperEnable, HIGH);
  digitalWrite(stepperTurnEnable, HIGH);

  Particle.function("azmove", handleAzMove);
  Particle.function("altmove", handleAltMove);
  Particle.function("fire", handleFire);
  Particle.function("advance", handleAdvance);

  Particle.variable("angle", &azPosition, DOUBLE);
}

// This routine gets called repeatedly, like once every 5-15 milliseconds.
// Particle firmware interleaves background CPU activity associated with WiFi + Cloud activity with your code.
// Make sure none of your code delays or blocks for too long (like more than 5 seconds), or weird things can happen.
void loop() {

    //trigger pulled
    if(digitalRead(triggerSenseIn) == HIGH){

        if(firstRun){
            //for esc calibration trigger pulled and held on bootup
            //photon should be powered by usb
            while(digitalRead(triggerSenseIn) == HIGH){
                flywheelESC.write(180);
            }

            flywheelESC.write(0);
            delay(500);
        }
        else{
            //read value from speed knob
            int speedVal = analogRead(speedSenseIn);
            speedVal = map(speedVal, 0, 4094, 15, 100);
            fireByTrigger(speedVal);
        }
    }
    else{
        //shut down flywheels if trigger not pressed
        flywheelESC.write(0);
    }

    //if wifi wire connected, try to connect
    if(digitalRead(connSenseIn) == HIGH){
      Particle.connect();
    }

    firstRun = false;

    delay(10);

    //shut down steppers after set time
    //this helps keep the steppers enabled between trigger pulls
    if(millis() > disableMillis){
        digitalWrite(stepperEnable, HIGH);
        digitalWrite(stepperTurnEnable, HIGH);
    }
}


//advance entry
int handleAdvance(String args){

    int advanceCount = args.toInt();

    digitalWrite(stepperEnable, LOW);
    delay(300);

    for(int index = 0; index < advanceCount; index++){

        if(!advance()){ break; }

        delay(100);
    }

    delay(100);
    digitalWrite(stepperEnable, HIGH);

    return 1;
}

void advanceWithEnableDisable(){

    digitalWrite(stepperEnable, LOW);
    delay(200);

    advance();

    delay(100);
    digitalWrite(stepperEnable, HIGH);
}




boolean advance(){
    int senseTest = digitalRead(advSenseIn);

    int successCheck = 0;

    while(senseTest == HIGH){
        if(successCheck > 300){
            return false;
        }

        StepRange(stepperAdvanceStep, 700, 700, 1);
        senseTest = digitalRead(advSenseIn);

        successCheck++;
    }

    successCheck = 0;

    while(senseTest == LOW){
        if(successCheck > 300){
            return false;
        }


        StepRange(stepperAdvanceStep, 700, 700, 1);
        senseTest = digitalRead(advSenseIn);

        successCheck++;
    }

    StepRange(stepperAdvanceStep, 800, 800, 20);

    return true;
}


int handleFire(String args){

    int delimIndex = args.indexOf(',');

    double spinupMicros = args.substring(0, delimIndex).toFloat();
    int shots = args.substring(delimIndex + 1, args.length()).toInt();

    fireBrushless(spinupMicros, shots);

    return 1;
}


void fireBrushless(double powerVal, int shots){
    fireBrushless(powerVal, shots, true, true);
}

void fireBrushless(double powerVal, int shots, bool powerUp, bool powerDown){

    if(shots <= 0){ return; }

    if(powerUp){
        if(!brushlessPowerUpAndAdvance(powerVal, 400)){
            return;
        }
    }
    else{
        if(!advance()){
            brushlessPowerDown(1000);
            return;
        }
    }

    for(int index = 0; index < shots; index++){

        if(index > 0){
            if(!advance()){ break; }
        }

        spinPlungerToSwitch();
    }

    if(powerDown){
        brushlessPowerDown(1000);
    }

}

void fireByTrigger(double powerVal){

    bool firstShot = true;

    if(!brushlessPowerUpAndAdvance(powerVal, 300)){
        return;
    }

    while(digitalRead(triggerSenseIn) == HIGH || firstShot){

        if(!firstShot){
            if(!advance()){ break; }
        }

        firstShot = false;

        spinPlungerToSwitch();
    }

    brushlessPowerDown(1000);
}

bool brushlessPowerUpAndAdvance(double powerVal, double spinUpDelay){

    int minStepperWarmup = 260;

    double spinupPower = powerVal;
    spinupPower += 40;
    spinupPower = min(spinupPower, 80);

    flywheelESC.write(spinupPower);

    //kick on steppers first and foremost
    digitalWrite(stepperEnable, LOW);
    delay(minStepperWarmup);

    //advance, bail all firing if fail
    if(digitalRead(advSenseIn) == LOW){
        if(!advance()){
            digitalWrite(stepperEnable, HIGH);
            flywheelESC.write(0);
            return false;
        }
    }

    flywheelESC.write(powerVal);

    delay(spinUpDelay);

    return true;
}

void brushlessPowerDown(double millisToDisable){

    flywheelESC.write(0);

    //sets disable time for 1 sec
    disableMillis = millis() + millisToDisable;
}

bool spinPlungerToSwitch(){

    //1600 full spin
    //spin enough to let go of the switch (1/2 wayish)
    StepRange(stepperFireStep, 180, 140, 100);
    StepRange(stepperFireStep, 140, 140, 800);

    for(int stepIndex = 0; stepIndex < 1600; stepIndex++){//500
       StepRange(stepperFireStep, 120, 120, 1);

       if(digitalRead(plungerSenseIn) == HIGH){
           return true;
       }
    }

    return false;
}



int handleAltMove(String args){
    double moveAngle = args.toFloat();
    altMove(moveAngle);

    return moveAngle;
}

void altMove(double moveAngle){
    if(moveAngle == 0) { return; }

    if(moveAngle < 0){
        digitalWrite(stepperAltDir, HIGH);
    }
    else{
        digitalWrite(stepperAltDir, LOW);
    }

    //get steps
    double steps = abs(moveAngle / 360.0 * 200000.0);

    digitalWrite(stepperTurnEnable, LOW);
    delay(100);

    if(steps > 200){
        StepRange(stepperAltStep, 300, 80, 200);
        steps -= 200;

        StepRange(stepperAltStep, 80, 80, steps);
    }
    else{
        StepRange(stepperAltStep, 300, 100, steps);
    }

    //azPosition += moveAngle;
    digitalWrite(stepperTurnEnable, HIGH);
}




int handleAzMove(String args){

    int delimIndex = args.indexOf(',');

    //handle legacy
    if(delimIndex == -1){
        double moveAngleRelative = args.toFloat();
        azMove(moveAngleRelative);
        return azPosition;
    }

    double moveAngle = args.substring(0, delimIndex).toFloat();
    String moveMethod = args.substring(delimIndex + 1, args.length());

    if(moveMethod == "A"){
        azMovePosition(moveAngle);
    }
    else{
        azMove(moveAngle);
    }

    return azPosition;
}

void azMove(double moveAngle){
    if(moveAngle == 0) { return; }

    if(moveAngle < 0){
        digitalWrite(stepperAngleDir, LOW);
    }
    else{
        digitalWrite(stepperAngleDir, HIGH);
    }




    //  //get steps
    // double azSteps = abs(azAngle / 360.0 * 20000.0); //(40)2222.222222

    // //radius = 60
    // double pi = 3.1415926536;
    // double circ = pi * 120.0;
    // double mmPerDegree = circ / 360.0;

    // //nut is 1/4-20
    // double travelPerRotation = 1.0 * 25.4 / 20;
    // double stepsPerRotation = 1600.0;

    // double altSteps = abs(altAngle * mmPerDegree / travelPerRotation * stepsPerRotation);





    //get steps
    double steps = abs(moveAngle / 360.0 * 20000.0);

    digitalWrite(stepperTurnEnable, LOW);
    delay(100);

    if(steps > 200){
        StepRange(stepperAngleStep, 800, 500, 200);
        steps -= 200;

        StepRange(stepperAngleStep, 500, 500, steps);
    }
    else{
        StepRange(stepperAngleStep, 800, 500, steps);
    }

    azPosition += moveAngle;
    digitalWrite(stepperTurnEnable, HIGH);
}




int handleAzPositon(String args){
    double movePosition = args.toFloat();
    azMovePosition(movePosition);

    return movePosition;
}

void azMovePosition(double newPosition){
    double moveAngle = newPosition - azPosition;
    azMove(moveAngle);
}




void StepDelay(int stepperPin, double delayMicros, int steps){
     for (int index = 0 ; index < steps ; index ++) {
        digitalWrite(stepperPin, HIGH);
        delayMicroseconds(delayMicros / 2);
        digitalWrite(stepperPin, LOW);
        delayMicroseconds(delayMicros / 2);
    }
}

void StepRange(int stepperPin, double startDelay, double endDelay, int steps){

    double delayChangePerStep = (endDelay - startDelay) / steps;

    double loopDelay = startDelay;

    for (int index = 0 ; index < steps ; index += 1) {
        digitalWrite(stepperPin, HIGH);
        delayMicroseconds(loopDelay / 2);
        digitalWrite(stepperPin, LOW);
        delayMicroseconds(loopDelay / 2);

        loopDelay += delayChangePerStep;
    }
}
