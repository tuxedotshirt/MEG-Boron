// Example usage for MMA8452Q library by Jim Lindblom <jim@sparkfun.com>.

#include "MMA8452Q.h"

// Initialize objects from the lib
MMA8452Q mMA8452Q;

void setup() {
    // Call functions on initialized library objects that require hardware
    mMA8452Q.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    mMA8452Q.process();
}
