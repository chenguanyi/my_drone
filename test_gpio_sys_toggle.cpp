#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>

// wPi 10 is GPIO 36 according to readall
#define PIN_NUM 36 

int main() {
    if (wiringPiSetupSys() == -1) {
        printf("Setup Sys failed\n");
        return 1;
    }
    printf("Setup Sys succeeded\n");
    
    pinMode(PIN_NUM, OUTPUT);
    printf("Pin mode set\n");
    
    digitalWrite(PIN_NUM, HIGH);
    printf("Set High\n");
    sleep(1);
    digitalWrite(PIN_NUM, LOW);
    printf("Set Low\n");
    
    return 0;
}
