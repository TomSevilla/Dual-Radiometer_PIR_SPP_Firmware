//
// Dual Radiometer_PIR_SPP_Firmware
//
// Dual Radiometer_PIR_SPP
//
// Author 		Thomas Sevilla
// 				Thomas Sevilla
//
// Date			6/16/16 9:11 AM
// Version		1.0
//
// Copyright	© Thomas Sevilla, 2016
//
//

//Radiometer code FINAL- TOM

/*Included Libraries*/
#include "SPI.h"
#include <SD.h>
#include <virtuabotixRTC.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <stdlib.h>
/*------------------*/

/*Global Variables*/

#define RTC_CLK 6
#define IO 7
#define CE 8

#define sensorOne 9 //Radiometer One PSP
#define sensorTwo 12 //PIR Radiometer
#define sensorThree 4 //Dome resistance
#define sensorFour 5 //Case Resistance
#define fan 46  //Fan activate
#define chipSelect  48 //SD Card CS
#define TempSensor 6

const unsigned int MAX_INPUT = 50;

double v_ref = 5.0;
double input_one = 0.0;
double input_two = 0.0;
double temp1,temp2=0.0;
double SumOne = 0;
double SumTwo = 0;
double SumThree,SumFour =0.0;
double cal1;
double cal2;
double avg1 = 0;
double avg2 = 0;
double TempAvg1,TempAvg2=0.0;


double C1=0.0010295;
double C2=0.0002391;
double C3=0.0000001568;

double Coef[14];
double gain1;
double gain2;

float maxTemp= 0.0;
float minTemp= 0.0;


char filename[] = {'L', 'O', 'G', 'G', 'E', 'R', '0', '0', '.', 'C', 'S', 'V', '\0'};
File logfile;
File myFile;

int serialInput;
int inData;
int _year;
int _month;
int _day;
int _dayWeek;
int _hr;
int _min;
int _sec;
int counter = 0; //NEEDS TO BE CHANGED
int whichCoef;
/*------------------*/

SoftwareSerial wirelessSerial(11, 10); // RX, TX 10,11
SoftwareSerial wiredSerial(1, 0); // RX, TX
virtuabotixRTC myRTC(RTC_CLK, IO, CE);

void splash() {
    wiredSerial.println("                                               *-------------------------------*");
    wiredSerial.println("                                               | Welcome to the NOAA/AOML/PHOD |");
    wiredSerial.println("                                               | Dual Radiometer setup, please |");
    wiredSerial.println("                                               | Follow the onscreen prompts to|");
    wiredSerial.println("                                               |   Set the Real time Clock     |");
    wiredSerial.println("                                               *-------------------------------*");
    wiredSerial.println(" ");
    wiredSerial.println(" ");
    digitalWrite(fan, LOW);
    
    digitalWrite(sensorOne, HIGH);
    digitalWrite(sensorTwo, HIGH);
    digitalWrite(sensorThree, HIGH);
    digitalWrite(sensorFour, HIGH);

}

void initSD() {
    wiredSerial.print("Initializing SD card...");
    
    
    if (!SD.begin(chipSelect)) {
        wiredSerial.println("Card failed, or not present");
        return;
    } else {
        wiredSerial.println("Card Initialized.");
    }
    
    for (uint8_t i = 0; i < 100; i++) {
        filename[6] = i / 10 + '0';
        filename[7] = i % 10 + '0';
        if (! SD.exists(filename)) {
            // only open a new file if it doesn't exist
            logfile = SD.open(filename, FILE_WRITE);
            break;  // leave the loop!
        }
        
    }
}

void read_cal_file(){
    if (SD.exists("CAL.txt")){
    // re-open the file for reading:
    myFile = SD.open("CAL.txt",FILE_READ);
    if (myFile) {
        wiredSerial.println("CAL.txt:");
        
        // read from the file until there's nothing else in it:
        while (myFile.available()) {
            
            Coef[whichCoef] = myFile.parseFloat();
            wiredSerial.println(Coef[whichCoef], 6);
            whichCoef++;
        }
        gain1=Coef[12];
        gain2=Coef[13];
        
        // close the file:
        myFile.close();
    } else {
        // if the file didn't open, print an error:
        wiredSerial.println("error opening Cal.txt");
    }
    }
}

double correction(double input,int AD_used){
    
    int index1,index2,index3;
    
    if(AD_used==1){
       // wiredSerial.println("Correcting ADC 1");
        index1=0;
        index2=1;
        index3=2;
      
       // wiredSerial.println("Using Coefs:");
       // wiredSerial.print(Coef[index1]);
       // wiredSerial.print(Coef[index2]);
       // wiredSerial.print(Coef[index3]);
        
    } else if(AD_used==2){
    
       // wiredSerial.println("Correcting ADC 2");
        index1=3;
        index2=4;
        index3=5;
        
       // wiredSerial.println("Using Coefs:");
       // wiredSerial.print(Coef[index1]);
       // wiredSerial.print(Coef[index2]);
       // wiredSerial.print(Coef[index3]);
        
    } else if(AD_used==3){
        
       // wiredSerial.println("Correcting ADC 3");
        index1=6;
        index2=7;
        index3=8;
        
       // wiredSerial.println("Using Coefs:");
       // wiredSerial.print(Coef[index1]);
       // wiredSerial.print(Coef[index2]);
       // wiredSerial.print(Coef[index3]);
    } else if(AD_used==4){
        
        //wiredSerial.println("Correcting ADC 4");
        index1=9;
        index2=10;
        index3=11;
        
        //wiredSerial.println("Using Coefs:");
        //wiredSerial.print(Coef[index1]);
        //wiredSerial.print(Coef[index2]);
        //wiredSerial.print(Coef[index3]);
    }
    
    
    double corrected = (Coef[index1]*pow(input,2))+(Coef[index2]*input)+Coef[index3];
    wiredSerial.println("The corrected value is: ");
    wiredSerial.println(corrected);

}

double meassure(int chip) {
    
    double volt;
    long int result = 0;
    byte sig = 0;
    byte b;
    
    digitalWrite(chip, LOW);
    
    if (!bit_is_set(PINB, PB3)) {
        
        b = SPI.transfer(0xFF);            // read 4 bytes adc raw data with SPI
        if ((b & 0x20) == 0) {           // is input negative ?
            sig = 1;
        }
        
        b &= 0x1F;                  // discard bit 27..31
        result |= b;
        result <<= 8;
        b = SPI.transfer(0xFF);
        result |= b;
        result <<= 8;
        b = SPI.transfer(0xFF);
        result |= b;
        result <<= 8;
        b = SPI.transfer(0xFF);
        result |= b;
        
        delayMicroseconds(200);
        
        digitalWrite(chip, LOW);
        delay(200);
        
        if (sig) {  // if input negative insert sign bit
            result |= 0xf0000000;
        }
        
        result = result / 16; // scale result down , last 4 bits have no information
        
        
        volt = (result * v_ref ) / 16777216; // max scale
    }
    digitalWrite(chip, HIGH);
    delay(20);
    volt=1; //TAKE THIS OUT BEFORE!!!!!!!
    return volt;
    
}

void calibrate(){
    int i=0;
    float volt_wanted=0.00;
    int AD_num=1;
    
    wiredSerial.println("YOU'VE ENTERED CALIBRATION MODE!!!! -> MUST BE CONNECTED DIRECTLY TO ADC");
    wiredSerial.println("MUST ONLY BE DONE ONCE");
    wiredSerial.println("*********************************************************************************");
    wiredSerial.println("\n");
    
    wiredSerial.println("Starting Calibration for A/D #1 obtaining 50 values type '1' to advance to the next sample");
    wiredSerial.println("Set input volatge to the requested values:");
    while(i<=50){
        wiredSerial.print("Sample #");
        wiredSerial.print(i);
        wiredSerial.print(" should be: ");
        wiredSerial.println(volt_wanted);
        
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        int next = wirelessSerial.parseInt();
        wiredSerial.print("Meassured Value was: ");
        wiredSerial.println(meassure(sensorOne));
        if(next == 1){
        i++;
            volt_wanted=volt_wanted+0.1;
    }
    }
    i=0;
    volt_wanted=0.00;
    
    wiredSerial.println("Starting Calibration for A/D #2 obtaining 50 values type '1' to advance to the next sample");
    wiredSerial.println("Set input volatge to the requested values:");
    while(i<=50){
        wiredSerial.print("Sample #");
        wiredSerial.print(i);
        wiredSerial.print(" should be: ");
        wiredSerial.println(volt_wanted);
        
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        int next = wirelessSerial.parseInt();
        wiredSerial.print("Meassured Value was: ");
        wiredSerial.println(meassure(sensorTwo));
        if(next == 1){
            i++;
            volt_wanted=volt_wanted+0.1;
        }
    }
    i=0;
    volt_wanted=0.00;
    
    wiredSerial.println("Starting Calibration for A/D #3 obtaining 50 values type '1' to advance to the next sample");
    wiredSerial.println("Set input volatge to the requested values:");
    while(i<=50){
        wiredSerial.print("Sample #");
        wiredSerial.print(i);
        wiredSerial.print(" should be: ");
        wiredSerial.println(volt_wanted);
        
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        int next = wirelessSerial.parseInt();
        wiredSerial.print("Meassured Value was: ");
        wiredSerial.println(meassure(sensorThree));
        if(next == 1){
            i++;
            volt_wanted=volt_wanted+0.1;
        }
    }
    i=0;
    volt_wanted=0.00;
    
    wiredSerial.println("Starting Calibration for A/D #4 obtaining 50 values type '1' to advance to the next sample");
    wiredSerial.println("Set input volatge to the requested values:");
    while(i<=50){
        wiredSerial.print("Sample #");
        wiredSerial.print(i);
        wiredSerial.print(" should be: ");
        wiredSerial.println(volt_wanted);
        
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        int next = wirelessSerial.parseInt();
        wiredSerial.print("Meassured Value was: ");
        wiredSerial.println(meassure(sensorFour));
        if(next == 1){
            i++;
            volt_wanted=volt_wanted+0.1;
        }
    }
    i=0;
    volt_wanted=0.00;
    
    
    }

void setRef_Radio() {
    wiredSerial.println("Enter Vref value: ");
    while (wirelessSerial.available() == 0) { // Wait for User to Input Data
    }
    v_ref = wirelessSerial.parseFloat(); //Read the data the user has input
    wiredSerial.println(v_ref);
    
    wiredSerial.println("Enter Calibration Coefficient for Radiometer 1: ");
    while (wirelessSerial.available() == 0) { // Wait for User to Input Data
    }
    cal1 = wirelessSerial.parseFloat(); //Read the data the user has input
    wiredSerial.println(cal1);
    
    wiredSerial.println("Enter Calibration Coefficient for Radiometer 2: ");
    while (wirelessSerial.available() == 0) { // Wait for User to Input Data
    }
    cal2 = wirelessSerial.parseFloat(); //Read the data the user has input
    wiredSerial.println(cal2);
    
    wiredSerial.println("Would you like to enter Calibration mode [Y=1/N=0]??");
    while (wirelessSerial.available() == 0) { // Wait for User to Input Data
    }
     int input = wirelessSerial.parseInt(); //Read the data the user has input
    if (input==1)
    {
        wiredSerial.println("GOING INTO CALIBRATION MODE!");
        calibrate();
        wiredSerial.println("Enter Value on matlab script!!");
        wiredSerial.println("PLEASE UPDATE THE CAL.txt file! and the restart board");
    } else if (input==0)
        
    {
        read_cal_file();
    }
    
    
}

void runFan() {
    
    digitalWrite(fan, HIGH);
}

void stopFan(){
    
    digitalWrite(fan, LOW);
    
}

double readAmbTemp(){
    
    double reading=0;
    
    for(int i=0; i<=40; i++){
        
        reading += analogRead(TempSensor);
    }
    
    reading = reading/40;
    
    float voltage = reading * 5.0;
    voltage /= 1024.0;
    float temperatureC = (voltage - 0.5) * 100 ;
    return temperatureC;
}

int check_temp_board(){
    
    double reading=0;
    
    for(int i=0; i<=50; i++){
        reading += analogRead(TempSensor);
    }
    
    reading=reading/50;
    
    float voltage = reading * 5.0;
    voltage /= 1024.0;
    float temperatureC = (voltage - 0.5) * 100 ;
    
    if(temperatureC>maxTemp)
    {
        //wiredSerial.println(temperatureC);
        runFan();
        int tempFlag=1;
        return tempFlag;
    } else if(temperatureC<maxTemp)
    {
        stopFan();
        //wiredSerial.println(temperatureC);
        int tempFlag=0;
        return tempFlag;
    }
}

void setTemp(){
    wiredSerial.println("Would you like to set the operating tempereature dynamically? [Y=1/N=0]");
    while (wirelessSerial.available() == 0) { // Wait for User to Input Data
    }
    int input = wirelessSerial.parseInt(); //Read the data the user has input
    if (input==1)
    {
                wiredSerial.println("Setting temperature to an operating range +/-10C ...");
                float AmbTemp=readAmbTemp();
         wiredSerial.println("The Ambient Temp is:");
        wiredSerial.println(AmbTemp,4);
        
         wiredSerial.println("The min and max are:");
                minTemp=AmbTemp-5;
                maxTemp=AmbTemp+5;
        wiredSerial.println(minTemp,4);
        wiredSerial.println(maxTemp,4);
    }else
            {
                wiredSerial.println("Setting temperature to an operating range of 75-85F");
                minTemp=18.00;
                maxTemp =34.00;
            }
    }

void setRTC() {
    wiredSerial.println("Enter the year (Format should be 20XX): ");
    
    while (wirelessSerial.available() == 0) { // Wait for User to Input Data
    }
    
    _year = wirelessSerial.parseInt(); //Read the data the user has input
    
        
    if (_year == 9999) {
        wiredSerial.println("!!!!!!!!!!IN TEST MODE!!!!!!!!!!!!");
        _year = 2016;
        _month = 4;
        _day = 8;
        _dayWeek = 5;
        _hr = 12;
        _min = 12;
        _sec = 00;
        v_ref = 5.00;
        cal1 = 8.17;
        cal2 = 8.17;
    } else {
        
        wiredSerial.println(_year);
        wiredSerial.println("Now, enter the month: ");
 
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        _month = wirelessSerial.parseInt(); //Read the data the user has input
        
        wiredSerial.println(_month);
        
        wiredSerial.println("Now, enter the day: ");
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
     
        _day = wirelessSerial.parseInt(); //Read the data the user has input
        wiredSerial.println(_day);
        
        wiredSerial.println("Now, enter the day of the week (Monday = 1, Tuesday =2 ...): ");
       
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        _dayWeek = wirelessSerial.parseInt(); //Read the data the user has input
        wiredSerial.println(_dayWeek);
        
        wiredSerial.println("Now, enter the Hour (24hr Format): ");
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
       
        _hr = wirelessSerial.parseInt(); //Read the data the user has input
        wiredSerial.println(_hr);
        
        wiredSerial.println("Now, enter the minutes: ");
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
      
        _min = wirelessSerial.parseInt(); //Read the data the user has input
        wiredSerial.println(_min);
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        wiredSerial.println("Now, enter seconds: ");
       
        _sec = wirelessSerial.parseInt(); //Read the data the user has input
        wiredSerial.println(_sec);
    
        setRef_Radio();
    }
    
    logfile = SD.open(filename, FILE_WRITE);
    
    if (logfile) {
        char buf_log[70];
        logfile.print("Deployment Date and Time: ");
        sprintf(buf_log, ",%02d,%02d,%02d,%02d,%02d,%02d", _month, _day, _year, _hr, _min, _sec);
        logfile.println(buf_log);
        
        char buf[70];
        wiredSerial.print("Deployment Date and Time: ");
        sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d", _month, _day, _year, _hr, _min, _sec);
        wiredSerial.println(buf);
        
        setTemp();
        
        wiredSerial.println("~~~~~~~~~~~STARTING MEASSURMENTS!~~~~~~~~~~");
        wiredSerial.println("Month/Day/Year , Hour:Min:Sec , Measurment 1, Measurment 2 ");
        logfile.println("Month,Day,Year,Hour,Min,Sec, Measurment 1, Measurment 2 ");
        logfile.close();
    } else {
        wiredSerial.println("ERROR WHEN WRITING TO FILE");
    }
    myRTC.setDS1302Time(_sec, _min, _hr, _dayWeek, _day, _month, _year);
}

void PrintTime_Serial() {
    char buf_serial[70];
    sprintf(buf_serial, "%02d/%02d/%02d %02d:%02d:%02d", myRTC.month, myRTC.dayofmonth, myRTC.year, myRTC.hours, myRTC.minutes, myRTC.seconds);
    wiredSerial.print(buf_serial);
    wirelessSerial.print(buf_serial);
}

void PrintTime_File() {
    char buf_log[70];
    sprintf(buf_log, "%02d,%02d,%02d,%02d,%02d,%02d", myRTC.month, myRTC.dayofmonth, myRTC.year, myRTC.hours, myRTC.minutes, myRTC.seconds);
    logfile.print(buf_log);
}

double calcWm2(double avg, double cal) {
    cal = cal / 1000000;
    double mV = (avg/gain1); //put in the gain that we have to remove
    double irradiance = (mV / cal);
    return avg;
}

double calcWm2_PIR(double avg, double cal,double T1,double T2) {
    double sigma = 5.6704e-8;
    int k=4;
    cal = cal / 1000000;
    double mV = (avg/gain2); //300 is the gain that we have to remove
    double irradiance = (mV / cal);
    irradiance=irradiance+sigma*pow(T1,4)-sigma*(pow(T2,4)-pow(T1,4));
    return irradiance;
}

float CalcResistance(float volt){

    float resistance = volt/0.0001;
}

float realTemp(float Res) {
  
    float T = 1/(C1+C2*log(Res)+C3*(pow(log(Res),3.0)));
    return T;
}

void testFan(){
    int run=1;
    
    wiredSerial.println("Would you like to test the fan? [Y=1/N=0]");
    
    while (wirelessSerial.available() == 0) { // Wait for User to Input Data
    }
    int input = wirelessSerial.parseInt(); //Read the data the user has input
    if (input==1)
    {
        while(run<5){
        wiredSerial.println("Turn Fan on type 1, To turn Fan off type 0");
        
        while (wirelessSerial.available() == 0) { // Wait for User to Input Data
        }
        int input = wirelessSerial.parseInt(); //Read the data the user has input
        
        if (input==1){
        
            digitalWrite(fan, HIGH);
            run++;
        }else if(input==0)
        {
            digitalWrite(fan, LOW);
            run++;
        }
        }
    }else
    {
        return;
    }
}


void setup() {
    
    pinMode(sensorOne, OUTPUT);
    pinMode(sensorTwo, OUTPUT);
    pinMode(sensorThree, OUTPUT);
    pinMode(sensorFour, OUTPUT);
    pinMode(fan, OUTPUT);
    
    digitalWrite(sensorOne, HIGH);
    digitalWrite(sensorTwo, HIGH);
    digitalWrite(sensorThree, HIGH);
    digitalWrite(sensorFour, HIGH);
    digitalWrite(fan, HIGH);
    
    wiredSerial.begin(9600);
    wirelessSerial.begin(9600);

    wiredSerial.setTimeout(10000);
    wirelessSerial.setTimeout(10000);
    
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV16);
    
    splash();
    initSD();
    setRTC();
    testFan();
    
    wiredSerial.print("The Gain 1 value is : ");
    wiredSerial.println(gain1);
    
    wiredSerial.print("The Gain 2 value is : ");
    wiredSerial.println(gain2);
    
}

unsigned int to_pre = (myRTC.seconds);
unsigned int to_cur = to_pre;

void loop() {
    
    myRTC.updateTime();
    
    to_cur = (myRTC.seconds);
    
    if ((to_cur - to_pre) >= 1) {
        input_one = meassure(sensorOne);
        SumOne += input_one;
        
        input_two = meassure(sensorTwo);
        SumTwo += input_two;
        
        temp1=meassure(sensorThree);
        SumThree += temp1;
        
        temp2=meassure(sensorFour);
        SumFour += temp2;
        
        counter++;
        to_pre = to_cur;
    }
    
    if (counter >= 60) {
        logfile = SD.open(filename, FILE_WRITE);
        
        avg1 = SumOne / counter;
        avg1=correction(avg1,1);
        
        avg2 = SumTwo / counter;
        avg2=correction(avg2,2);
        
        TempAvg1 = SumThree / counter;
        TempAvg1=correction(TempAvg1,3);
        TempAvg1=CalcResistance(TempAvg1);
        TempAvg1=realTemp(TempAvg1);
        
        
        TempAvg2= SumFour / counter;
        TempAvg2=correction(TempAvg2,4);
        TempAvg2=CalcResistance(TempAvg2);
        TempAvg2=realTemp(TempAvg2);
        
        PrintTime_Serial();
        wiredSerial.print(" ");
        wiredSerial.print(calcWm2(avg1, cal1), 4);
        
        wirelessSerial.print(" ");
        wirelessSerial.println(calcWm2(avg1, cal1), 4);
        
        
        PrintTime_File();
        logfile.print(",");
        logfile.print(calcWm2(avg1, cal1), 4);
        
        
        wiredSerial.print(" , ");
        wiredSerial.print(" ");
        wiredSerial.println(calcWm2_PIR(avg2, cal2,TempAvg1,TempAvg2), 4);
        
        logfile.print(",");
        logfile.println(calcWm2_PIR(avg2, cal2,TempAvg1,TempAvg2), 4);
        
        
        SumTwo,SumOne,SumThree,SumFour,counter=0;
        
        logfile.close();
    }
  
    if(check_temp_board()==1){
        
        wiredSerial.println("FAN IS RUNNING CIRCUIT MIGHT BE TOO HOT FOR STABLE MEASSUREMENTS!!");
    
    }

    
}


