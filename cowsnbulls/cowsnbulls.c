#include <stdio.h>
#include <time.h>
#include <stdlib.h>

/* override: 
/usr/include/stdlib.h:128:0: note: this is the location of the previous definition
 #define RAND_MAX 2147483647
*/

#define LOCAL_RAND_MAX 9999

int main(int argc, char *argv[]) {

    printf("We are playing cows and bulls!\n");

    srand(time(NULL));

    int target = 999;
    while (target <= 999) {
        target = rand() % LOCAL_RAND_MAX; // starts from 0 to LRM
    }

    printf("Target is %d (%d)\n", target, RAND_MAX);
    return 0;
}
