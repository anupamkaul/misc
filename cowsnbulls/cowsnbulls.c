#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdbool.h>

/* override: 
/usr/include/stdlib.h:128:0: note: this is the location of the previous definition
 #define RAND_MAX 2147483647
*/

#define LOCAL_RAND_MAX 9999
#define MAX_TRIES      10

int main(int argc, char *argv[]) {

    printf("We are playing cows and bulls, %d tries!\n", MAX_TRIES);

    srand(time(NULL));

    int target = 999;
    while (target <= 999) {
        target = rand() % LOCAL_RAND_MAX; // starts from 0 to LRM
    }

    printf("Target is %d (%d)\n", target, RAND_MAX);

    int current_try = 0;
    bool pass = false;
    int guess;

    while(current_try < MAX_TRIES && !pass) {
        printf("Try %d :", current_try); 
        scanf("%4d", &guess);
        printf("you guessed %d\n", guess);
        current_try++;
    }

    
    return 0;
}
