#include <wiringPi.h>
#include <stdio.h>

int main() {
    if (wiringPiSetup() == -1) {
        printf("Setup failed\n");
        return 1;
    }
    printf("Setup succeeded\n");
    return 0;
}
