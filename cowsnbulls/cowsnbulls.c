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

bool match(int target, int guess);

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
        printf("\nTry %d :", current_try); 
        scanf("%4d", &guess);
        printf("you guessed %d\n", guess);
        pass = match(target, guess);
        current_try++;
    }

    if (pass) {
        printf ("CORRECT !! in %d tries\n", --current_try);
    }
    else {
        printf("FAIL !! Better luck next time\n");
    }


    return 0;
}

bool match(int inp_target, int inp_guess) {

    if (inp_target == inp_guess) return true;

    int cows = 0, bulls = 0;
    char target[4], guess[4];

    // create 2 arrays 
    sprintf(target, "%d", inp_target);
    sprintf(guess, "%d",  inp_guess);

    /* printf("debug: %s and %s ", target, guess); */

    /* you get 1 cow if the number is correct but in the wrong place
     * you get 1 bull if the number is correct and in the correct place
     */

    /* take 1 from guess and run it by all in target */
    for (int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
   
            printf("DEBUG: comp %d and %d\n", guess[i], target[j]);
            if(guess[i] == target[j]) {
                if (i==j) bulls++; else cows++;
            }
        }
    }
    
    printf("match: bulls %d cows %d", bulls, cows);
    return false;
}
