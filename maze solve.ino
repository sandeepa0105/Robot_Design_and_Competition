#define R_motor_A 4
#define R_motor_B 7
#define L_motor_A 2
#define L_motor_B 3

#define R_motor_pwm 6
#define L_motor_pwm 5

int IR_val[6] = {0,0,0,0,0,0} ;
int IR_weight[6] = {-18,-8,-5,5,8,18};


#define JUNCTION_SIZE  100
int MAX_SPEED = 130 ;
int ANGLE_SPEED = 130;
int MIDDLE_SPEED = 80;
int LOWEST_SPEED = 0;

int POP_JUNCTION = 0;
int L_JUNCTION = 1;
int T_JUNCTION = 2;
int CROSS_JUNCTION = 3;

int left_motor_speed = MIDDLE_SPEED ;
int right_motor_speed = MIDDLE_SPEED ;

int junctions[JUNCTION_SIZE];
int taken_directions[JUNCTION_SIZE];

int junction_id = 0;
float p , d , error ;
float i = 0 ;
float prev_error = 0 ;
float KP = 4.3 ;
float KI = 0 ;
float KD = 8 ;

int direction = 1;
  

  
void setup() {

  for (int i = 0; i<JUNCTION_SIZE; i++){
    junctions[i] = 0;
    taken_directions[i] = 0;
  }
  
  Serial.begin(9600) ;
  pinMode(A0, INPUT) ;
  pinMode(A1, INPUT) ;
  pinMode(A2, INPUT) ;
  pinMode(A3, INPUT) ;
  pinMode(A4, INPUT) ;
  pinMode(A5, INPUT) ;


  pinMode(2, OUTPUT) ;
  pinMode(3, OUTPUT) ;
  pinMode(4, OUTPUT) ;
  pinMode(7, OUTPUT) ;
  pinMode(5, OUTPUT) ;
  pinMode(6, OUTPUT) ;

  set_motor_speed()  ;
  forward() ;
  delay(500) ;
  
}











void loop() {

  read_IR();

  Serial.print(IR_val[5] );
  Serial.print(IR_val[4] );
  Serial.print(IR_val[3] );
  Serial.print(IR_val[2] );
  Serial.print(IR_val[1] );
  Serial.println(IR_val[0]); 
  // turning;
  if(direction == 1){

    if (is_LM()){                      //remove the else if 
      little_forward();
      if (is_None()){
        junctions[junction_id++] = L_JUNCTION;
        taken_directions[junction_id++] = 0;        
        turn_left();        
      }else {                         //remove  if (is_M())
        junctions[junction_id++] = T_JUNCTION; 
        taken_directions[junction_id++] = 0; 
        turn_left();                  
      }
      
          
    }else if(is_MR()){
      little_forward();
      
      if (is_M()){            
        junctions[junction_id++] = T_JUNCTION;
        taken_directions[junction_id++] = 0;  
        //taken_directions[junction_id++] = 0;
        forward();
        PID();
      }else{                                             //remove   if (is_None())
        junctions[junction_id++] = L_JUNCTION;
        taken_directions[junction_id++] = 0;
        //forward();
        //PID();
        turn_right();            
      }
      
    }else if (is_LMR()){
      little_forward();
      if (is_None()){      
        junctions[junction_id++] = T_JUNCTION;  
        taken_directions[junction_id++] = 0;  
        turn_left();

      }else if(is_LMR()){
        turn_around();
        direction = 0;
      }else {                                               //remove if(is_M())
        junctions[junction_id++] = CROSS_JUNCTION;
        taken_directions[junction_id++] = 0; 
        turn_left();

      }

    }else if(is_None()){
      turn_around();
     // direction = 1;   //change -1

    }else if(is_M()){
      forward();
      PID();
      set_motor_speed();    
    }
  }


// after turning


  else if(direction == -1){

    if (is_LM()){
      little_forward();
      if (is_None()){
        junctions[junction_id--] = POP_JUNCTION; // L_JUNCTION but when 
  
      }else if (is_M()){
        taken_directions[junction_id] += 1;  //T_JUNCTION 
        direction = 1;

        if (taken_directions[junction_id] == junctions[junction_id]){
          direction = -1;
          junctions[junction_id] = POP_JUNCTION;
          taken_directions[junction_id--] = POP_JUNCTION;
        }              
      }
      turn_left();
          
    }else if(is_MR()){
      little_forward();
      if (is_None()){
        turn_right();
        junctions[junction_id--] = POP_JUNCTION; // L_JUNCTION but when 
      }else if (is_M()){            
        taken_directions[junction_id] +=1;
        direction = 1;

        if (taken_directions[junction_id] == junctions[junction_id]){
          direction = -1;
          junctions[junction_id] = POP_JUNCTION;
          taken_directions[junction_id--] = POP_JUNCTION;
        }     //T
        PID();
        forward();                  
      }
      
    }else if (is_LMR()){
      little_forward();
      if (is_None()){      
        junctions[junction_id--] = POP_JUNCTION;     
      }else if(is_M()){
        direction = 1;
        taken_directions[junction_id] += 1;   

        if (taken_directions[junction_id] == junctions[junction_id]){
          direction = -1;
          junctions[junction_id] = POP_JUNCTION;
          taken_directions[junction_id--] = POP_JUNCTION;
        }    
      }else if(is_LMR()){
        direction = 0;
      }
      turn_left();

    }else if(is_None()){
      turn_around();
      direction = -1;

    }else if(is_M()){
      PID();
      forward();    
    }
    



  }else if(direction == 0){

    if (is_LM()){
      little_forward();
      if (is_None()){
        turn_left();
      }else if (is_M()){
        // -|
        if(taken_directions[junction_id--] == 0){
          PID();
          forward();
        }else{
          turn_left();
        }          
      }

          
    }else if(is_MR()){
      little_forward();
      if (is_None()){
        turn_right();
      }else if (is_M()){            
        // |-
        if(taken_directions[junction_id--] == 0){
          turn_right();
        }else{
          PID();
          forward();
        }  
      }
      
    }else if (is_LMR()){
      little_forward();
      if (is_None()){      
        // T
        if(taken_directions[junction_id--] == 0){
          turn_right();
        }else{
          turn_left();
        } 

      }else if(is_M()){
        // -|-
        if(taken_directions[junction_id] == 0){
          turn_right();
        }else if(taken_directions[junction_id] == 1){
          PID();
          forward();
        }else {
          turn_left();
        } 
      }
      turn_left();

    }else if(is_None()){
      Serial.println("Cannot happen");
    // cannot happen


    }else if(is_M()){
      PID();
      forward();    
    }
  }
  
  PID() ;
  set_motor_speed() ;

  
}

void turn_left(){
  left_motor_speed = LOWEST_SPEED;
  right_motor_speed = ANGLE_SPEED;
  set_motor_speed();
  forward();
  delay(510);
  PID();
  set_motor_speed();
}

void turn_right(){
  left_motor_speed = ANGLE_SPEED;
  right_motor_speed = LOWEST_SPEED;
  set_motor_speed();
  forward();
  delay(510);
  PID();
  set_motor_speed();    
}

void turn_around(){
    stop();
    delay(200);
    left_motor_speed = ANGLE_SPEED;
    right_motor_speed = ANGLE_SPEED;
    set_motor_speed();
    around();
    delay(600);
    stop();
    delay(200);
    forward();    
    PID();
}

void read_IR(){
   
      IR_val[0] = analogRead(A5)  ;
      IR_val[1] = analogRead(A4) ;
      IR_val[2] = analogRead(A3) ;
      IR_val[3] = analogRead(A2) ;
      IR_val[4] = analogRead(A1) ;
      IR_val[5] = analogRead(A0) ;

      if(IR_val[0]>512){
        IR_val[0] = 1;
      }else{
        IR_val[0] = 0;
      }

      if(IR_val[1]>512){
        IR_val[1] = 1;
      }else{
        IR_val[1] = 0;
      }

      if(IR_val[2]>512){
        IR_val[2] = 1;
      }else{
        IR_val[2] = 0;
      }

      if(IR_val[3]>512){
        IR_val[3] = 1;
      }else{
        IR_val[3] = 0;
      }

      if(IR_val[4]>512){
        IR_val[4] = 1;
      }else{
        IR_val[4] = 0;
      }

      if(IR_val[5]>512){
        IR_val[5] = 1;
      }else{
        IR_val[5] = 0;
      }
}

//110000 ->L
int is_LM(){
  if( IR_val[0]  && !IR_val[2] && !IR_val[3] && !IR_val[4] && !IR_val[5] ){
    return 1;
  }else{
    return 0;
  }
}



//000011 -> R
int is_MR(){
  read_IR();
  if( !IR_val[0] && !IR_val[1] && !IR_val[2] && !IR_val[3]  && IR_val[5]){
    return 1;
  }else{
    return 0;
  }
}

int is_M(){
  read_IR();
  //if (IR_val[2] || IR_val[3]){
  if( IR_val[0] && IR_val[5] && (!IR_val[2] || !IR_val[3])){                //remove   IR_val[1] && IR_val[4] && 
    return 1;
  }else{
    return 0;
  }
}

int is_LMR(){
  read_IR();  
  if( !IR_val[0] && !IR_val[2] && !IR_val[3]  && !IR_val[5] && !IR_val[1] && !IR_val[4]){
    return 1;
  }else{
    return 0;
  }
}

int is_None(){
  read_IR();  
  if( IR_val[0] && IR_val[1] && IR_val[2] && IR_val[3] && IR_val[4] && IR_val[5]){
    return 1;
  }else{
    return 0;
  }  
}


void little_forward(){
    // Serial.println("is cross");
    stop();
    delay(100);
  
    left_motor_speed = MIDDLE_SPEED;
    right_motor_speed = MIDDLE_SPEED;
    set_motor_speed();
    forward();
    delay(310);
    stop();
    delay(100);
    read_IR();
}

void forward() {
  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_B, HIGH) ;
  digitalWrite(L_motor_A, LOW) ;
}

void stop () {
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_A, LOW) ;
  digitalWrite(L_motor_B, LOW) ;
}

void around(){
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, HIGH) ;
  digitalWrite(L_motor_A, LOW) ;
  digitalWrite(L_motor_B, HIGH) ;
}
void right(){
  digitalWrite(R_motor_A, LOW) ;
  digitalWrite(R_motor_B, HIGH) ;
  digitalWrite(L_motor_A, LOW) ;
  digitalWrite(L_motor_B, HIGH) ;
} 
void left(){
  digitalWrite(R_motor_A, HIGH) ;
  digitalWrite(R_motor_B, LOW) ;
  digitalWrite(L_motor_A, HIGH) ;
  digitalWrite(L_motor_B, LOW) ;
}
void set_motor_speed() {
  analogWrite(R_motor_pwm, right_motor_speed) ;
  analogWrite(L_motor_pwm, left_motor_speed) ;
   
  }
  
void PID() {
  read_IR();
  error = 0 ;

  for (int i=0; i<=5; i++) {
    error += IR_val[i]*IR_weight[i] ;
    }

  p = error ;
  i = i + error ;
  d = error - prev_error ;
  prev_error = error ;

  float PID_val = (p*KP + i*KI + d*KD)/2 ;
  //int PID_val_int = round(PID_val);
  Serial.print(PID_val);

  right_motor_speed = MIDDLE_SPEED - PID_val ;
  left_motor_speed = MIDDLE_SPEED + PID_val ;

  if (right_motor_speed < 0)         {right_motor_speed = LOWEST_SPEED ; }
  if (left_motor_speed < 0)          {left_motor_speed = LOWEST_SPEED ; }
  if (right_motor_speed > MAX_SPEED) {right_motor_speed = MAX_SPEED ; }
  if (left_motor_speed > MAX_SPEED)  {left_motor_speed = MAX_SPEED ; }
 
  }