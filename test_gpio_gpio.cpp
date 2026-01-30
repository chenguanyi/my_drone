#include <wiringPi.h>
#include <stdio.h>

int main() {
    if (wiringPiSetupGpio() == -1) {
        printf("Setup Gpio failed\n");
        return 1;
    }
    printf("Setup Gpio succeeded\n");
    return 0;
}
