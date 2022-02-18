// TCtrl with Ethernet-Shield on Arduino Uno
// read data: <ip>/data/get/
// set value: <ip>/write/<parameter>/<value>/
// explanation getdate: See TestShield_date_2.1.ino and 2.2 
// reading works on DCS (output {"value":{"I":"100.00","P":"5.00","updated":"2022-02-02 13:29:07","T":"60.00","setpoint":"60.00","D":"0.00","error":"0.00","output":"63.05"},"response":"get"} )
// writing does not work on DCS

// dictionaries needed
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// MAC address of shield
byte mac[] = { 0x08, 0x00, 0x27, 0xCE, 0xB0, 0xD8};
EthernetServer server(80); // Using port 80 (normal for http)

/* Define variables for date*/
unsigned int localPort = 8888;       // local port to listen for UDP packets
const char timeServer[] = "ntp.kip.uni-heidelberg.de";
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
String date;
EthernetUDP Udp; // A UDP instance to let us send and receive packets over UDP


/* Define global objects for PID calculation*/
int Temp_in = A0;
float V_out;
float Temperature;
int analogpin = 9;

const int nmeas = 10; //number of measurements before eval

unsigned long now;
long sumval;
int measnum;
int lc;
int lcmax = 100;

/* Working variables for PID calculation */
unsigned long lastTime;
double input, output, setpoint, error;
double errSum, lastErr;
double kp, ki, kd, G, tauI, tauD;

char mode;


/* Start processes */  
void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  Ethernet.init(10);  // Most Arduino shields
  
  Ethernet.begin(mac); // Start the Ethernet shield
  server.begin();
  Serial.begin(9600); // Start serial communication
  Udp.begin(localPort); //Start UDP connenction

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
    }
    if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
    }

  // start the server
  server.begin();
 
  Serial.println("Server address:"); // Print server address
  // (Arduino shield)
  Serial.println(Ethernet.localIP());

  pinMode(Temp_in, INPUT); // setup pins for analog read
  pinMode(analogpin, OUTPUT);

  setpoint = 30;

  ////////PID parameters
  G = 3; //gain that we want to use. We find it by adjusting it to be small enough such that the system is not oscillating
  tauI = 100; // in s and obtained from the time constant as we apply a step function
  tauD = 0;
  kp = G;
  ki = G / tauI;
  kd = G * tauD;

  //initialize integrator
  errSum = 20 / lcmax; // let the loop start at a nice value
  lc = 0;
  measnum = 0;
  output = 0;
  sumval = 0;
}


void read(EthernetClient client);
void write(EthernetClient client);
void getdate();
void sendNTPpacket(const char * address); 

/* Actual code */
void loop() {
  // Get clients coming from server
  EthernetClient client = server.available();
  // There is a new client?
  if (client) {
    // Process request
    // server.write("Successful write!");
   // process(client);

  delay(50); // Poll every 50ms   
  lc++;
  
  ////// measure every 10th loop cycle
  if ((lc % 10) == 0) {
    measnum++;
    //V_out = analogRead(Temp_in)*(5/1024.0);
    V_out = analogRead(Temp_in) * 0.004953289; //new calibration
    // Temperature = (V_out-1.25)/0.005; //conversion from voltage to temp

    Temperature = 0.85744118 * analogRead(Temp_in) - 215.49289717;

    sumval = sumval + Temperature;
  }

  
  /////Calculation of PID if enough measurements
  if (measnum == nmeas) {
     getdate(); // ask for date now
    
    input = double(sumval) / double(nmeas);
    now = millis();

    // time since last evaluation
    // we want the timeChange to be in seconds and not ms
    // this is necessary as we only know the time constant in actual units
    double timeChange = (double)((now - lastTime) / 1000);

    if (timeChange > 0) {
      //calculate error signal
      error = setpoint - input;

      //update integrator
      errSum += (error * timeChange);

      //limit the integrator to the bounds of the output
      if (errSum * ki > 255) errSum = 255 / ki;
      if (errSum * ki < 0) errSum = 0;

      //calculate derivative part
      double dErr = (error - lastErr) / timeChange;

      //compute PI output
      output = kp * error + ki * errSum + kd * dErr;

      //limit PID output to the bounds of the output
      if (output > 255) output = 255;
      if (output < 0) output = 0;

      //remember for next time
      lastErr = error;
      lastTime = now;
    }

    analogWrite(analogpin, output);
    // reset number of aquired measurements and measurement accumulator
    sumval = 0;
    measnum = 0;
  }

  /////////// second part of the wavepacket control
  if (lc == lcmax) {
    lc = 0;
  }

 
  //// connection to server
  String command = "";
  while (client.connected()) {
        while(client.available()){
        char c = client.read(); // Read from the Ethernet shield as long as there is something
        command += c; // Add character to string buffer
      if(c == '/' && command=="GET /write/"){
          write(client);
          break;
      }  
      if(c == '/' && command=="GET /data/get/" ){
         read(client);
         break;
      }
//    else{
//       client.println("HTTP/1.1 200 OK"); 
//       client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
//       client.println(); // HTML code for browser 
//       client.print("wrong command");
//       client.stop(); 
      }   
      
   client.println("HTTP/1.1 200 OK"); 
          client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
          client.println(); // HTML code for browser 
          client.print(command);
          client.stop(); 
  }
  }
}


// send an NTP request to the time server at the given address
void sendNTPpacket(const char * address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); // NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void getdate() {
  sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait to see if a reply is available 
  delay(500); //initially 1000, but kip-server is fast
  if (Udp.parsePacket()) {
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // now convert NTP time into everyday time:
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
   
    // convert to date
    int year=(1970+ epoch/31556926); //31556926 seconds in a year with 365.25 days, startes 19707
    int month=(epoch % 31556926)/2629743 +1; //// 2629743 seconds in a month with 30.44 days, +1 because in January, the result of % is 0, we want a 1 though
    int day = (epoch % 2629743)/86400; //86400 seconds in a day
  
    // print the hour, minute and second:
    epoch=epoch+3600; // 1 hour time shift because winter time ( UTC is the time at Greenwich Meridian (GMT))
    int hours= (epoch  % 86400L) / 3600; // print the hour (86400 equals secs per day)
    int minutes = (epoch % 3600) / 60;
    int seconds= epoch % 60; 
    // build date-variable with 2022-02-02 13:29:07
    date="";
    date.concat(year);
    date.concat('-');
    if (month<10){ //leading 0
      date.concat(0);
      } 
    date.concat(month);
    date.concat('-');
    if (day<10){ // leading 0
      date.concat(0);
      }
    date.concat(day); 
    date.concat(" ");
    if (hours<10){ // leading 0
      date.concat(0);
      }
    date.concat(hours);
    date.concat(":");
    if (minutes<10){ // leading 0
      date.concat(0);
      }
    date.concat(minutes);
    date.concat(":");
    if (seconds<10){ // leading 0
      date.concat(0);
      }
    date.concat(seconds);    
}
}


void write(EthernetClient client){
  String component = client.readStringUntil('/');
    if (component == "setpoint") { // edit setpoint
       float value = 0;
       value = client.parseFloat();
       if (value){
        setpoint = value;    
        client.println("HTTP/1.1 200 OK"); 
        client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
        client.println(); // HTML code for browser 
        client.print("new value for ");
        client.print(component);
        client.print(": ");
        client.print(value);
        client.stop();  
       }
       else{
        client.println("HTTP/1.1 200 OK"); 
        client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
        client.println(); // HTML code for browser 
        client.print("Please enter value");
       }
    }
     
    if (component == "P") { // edit gain
      float value = 0;
      value = client.parseFloat();
      if (value){
        G = value;
        kp = G;
        ki = G / tauI;
        kd = G * tauD;
        client.println("HTTP/1.1 200 OK"); 
        client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
        client.println(); // HTML code for browser 
        client.print("new value for ");
        client.print(component);
        client.print(": ");
        client.print(value);
        client.stop();
      }
      else{
        client.println("HTTP/1.1 200 OK"); 
        client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
        client.println(); // HTML code for browser 
        client.print("Please enter value");
       }
    }
  
  if (component == "I") { // edit tauI
    float value = 0;
    value = client.parseFloat();
    if (value){
        tauI = value;
        ki = G / tauI;
        client.println("HTTP/1.1 200 OK"); 
        client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
        client.println(); // HTML code for browser 
        client.print("new value for ");
        client.print(component);
        client.print(": ");
        client.print(value);
        client.stop();
    }
    else{
        client.println("HTTP/1.1 200 OK"); 
        client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
        client.println(); // HTML code for browser 
        client.print("Please enter value");
       }
  }
  
  if (component == "D") { // edit tauD
    float value = 0;
    value = client.parseFloat();
    if (value){
        tauD = value;
        kd = G*tauD;
        client.println("HTTP/1.1 200 OK"); 
        client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
        client.println(); // HTML code for browser 
        client.print("new value for ");
        client.print(component);
        client.print(": ");
        client.print(value);
        client.stop();
    }
  }
}
  
void read(EthernetClient client){ //DCS will als Format: {"value":{"I":"100.00","P":"5.00","updated":"2022-02-02 13:29:07","T":"60.00","setpoint":"60.00","D":"0.00","error":"0.00","output":"63.05"},"response":"get"}
     getdate(); //get date now to print to output
     client.println("HTTP/1.1 200 OK"); 
     client.println("Content-Type: text/html"); //this line and the next line are crucial to display data!
     client.println(); // HTML code for browser 
     client.print("{\"value\":");
     client.print("{\"I\":\"" + String(tauI) + "\",");
     client.print("\"P\":\"" + String(G) + "\",");
     client.print("\"updated\":\""+ String(date) + "\","); 
     client.print("\"T\":\"" + String(input) + "\",");
     client.print("\"setpoint\":\"" + String(setpoint) + "\",");
     client.print("\"D\":\"" + String(tauD) + "\",");
     client.print("\"error\":\"" + String(error) + "\",");
     client.print("\"output\":\"" + String(output) + "\"}");
     client.print(",\"response\":");
     client.print("\"get\"}");
     client.stop();
}
