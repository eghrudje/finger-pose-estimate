 // Define the pins connected to the A4988 driver
const int stepPin = 6;  // Step pin
const int dirPin = 5;   // Direction pin
const int enablePin = 7; //Enable Pin
const int interruptPin = 2;

//const int A = 8;
//const int B = 9;

// Define variables
const int stepsPerRevolution = 25;  // Change this to match your stepper motor
int stepCount = 0;  // Initialize step counter variable


void setup() {
  // Set the control pins as OUTPUT
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(interruptPin, OUTPUT);
  
  // Set the initial direction (clockwise or counterclockwise)
  digitalWrite(dirPin, LOW); // Set direction to HIGH for clockwise rotation

  digitalWrite(enablePin, LOW);
  
}

void rot() {
  
for (int i = 0; i < stepsPerRevolution; i++) {
  digitalWrite(interruptPin, HIGH);
  
  digitalWrite(stepPin, HIGH);
  delay(10); // Adjust this delay as needed
  digitalWrite(stepPin, LOW);
  delay(10); // Adjust this delay as needed

  // Step AC
  digitalWrite(stepPin, HIGH);
  delay(10); // Adjust this delay as needed
  digitalWrite(stepPin, LOW);
  delay(10); // Adjust this delay as needed

  // Step BC
  digitalWrite(stepPin, HIGH);
  delay(10); // Adjust this delay as needed
  digitalWrite(stepPin, LOW);
  delay(10); // Adjust this delay as needed

  // Step BA
  digitalWrite(stepPin, HIGH); 
  delay(10); // Adjust this delay as needed
  digitalWrite(stepPin, LOW);
  delay(10); // Adjust this delay as needed

  stepCount++;  // Increment the step counter
  digitalWrite(interruptPin, LOW);
  
    
  if (stepCount >= stepsPerRevolution) {
    // If the desired number of steps is reached, disable the motor driver
    digitalWrite(enablePin, HIGH); // Disable the motor driver
    //digitalWrite(B, HIGH);
    return;
  }
    
  }
  
  
  }

void loop() {
  // Rotate one full revolution (360 degrees)
 rot();

  // Stop rotating
  //while (true) {
    // This will keep the program in a loop and effectively stop the rotation
  //}

}
