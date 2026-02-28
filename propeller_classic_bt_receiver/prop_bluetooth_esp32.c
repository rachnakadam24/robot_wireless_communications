#include "simpletools.h"
#include "fdserial.h"
#include "servo.h"

// ---------- Pin Definitions ----------
#define RX_PIN      7     
#define TX_PIN      6
#define ROB_ID      24     
#define LEFT_SERVO  13    
#define RIGHT_SERVO 14
#define GRIP_SERVO  12

#define BAUD        9600

// ---------- Heartbeat Function ----------
// This will run continuously in its own Cog
void heartbeatLED() {
  while(1) {
    high(26);    // Turn LED on Pin 26 ON
    pause(500);  // Wait half a second
    low(26);     // Turn LED OFF
    pause(500);  // Wait half a second
  }
}

int main() {
  // 1. Launch the Heartbeat LED in a parallel Cog!
  // We give it 128 words of stack memory, which is plenty for blinking an LED.
  cog_run(heartbeatLED, 128);

  // 2. Start the background serial cog
  fdserial *bt = fdserial_open(RX_PIN, TX_PIN, 0, BAUD);
  
  int idLo, idHi, leftV, rightV, gripV;
  int rid;
  int packetCount = 0;

  print("--- Propeller BT Receiver Ready ---\n");
  print("Waiting for ESP32 Data...\n");

  while(1) {
    // Wait for the double header "!!"
    if (fdserial_rxChar(bt) == '!') {
      if (fdserial_rxChar(bt) == '!') {
        
        // Read the 5 data bytes
        idLo   = fdserial_rxChar(bt);
        idHi   = fdserial_rxChar(bt);
        leftV  = fdserial_rxChar(bt);
        rightV = fdserial_rxChar(bt);
        gripV  = fdserial_rxChar(bt);

        packetCount++;
        rid = idLo + (idHi * 256);
        
        if (rid == ROB_ID) {

          // Display Results
          print("%c", CLS); // Clear cursor
          print("PACKET RECEIVED! (#%d) \n", packetCount);
          print("--------------------\n");
          print("Full ID:  %d      \n", rid);
          print("Left:     %d      \n", leftV);
          print("Right:    %d      \n", rightV);
          print("Gripper:  %d      \n", gripV);
          print("--------------------\n");
          
          // Control Servos
          if (leftV != 255)  { servo_angle(LEFT_SERVO,  leftV  * 10); }
          if (rightV != 255) { servo_angle(RIGHT_SERVO, rightV * 10); }
          if (gripV != 255)  { servo_angle(GRIP_SERVO,  gripV  * 10); }
        }
        else {
          print("%c", CLS); // Clear cursor
          print("Invalid ID\n");
        }          
      }
    }
  }
}