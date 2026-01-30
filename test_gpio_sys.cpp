#include <wiringPi.h>
#include <stdio.h>

int main() {
    if (wiringPiSetupSys() == -1) {
        printf("Setup Sys failed\n");
        return 1;
    }
    printf("Setup Sys succeeded\n");
    return 0;
}
