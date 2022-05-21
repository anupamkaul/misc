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
        //printf("you guessed %d\n", guess);
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
    char visited_guess_bull[4] = {0,0,0,0}; // short for an actual DS with guess

    // create 2 arrays 
    sprintf(target, "%d", inp_target);
    sprintf(guess, "%d",  inp_guess);

    /* printf("debug: %s and %s ", target, guess); */

    /* you get 1 cow if the number is correct but in the wrong place
     * you get 1 bull if the number is correct and in the correct place
     */

    /* Insight - take 1 from target and run it by all in guess - this is counter intuitive
     * but target is more restrictive. Skip back to next in target upon a match
     */

    // Algo-1: (bulls and cows are equal)

    /* TODO: 
     *
     * 1. On detecting a cow, continue processing of unanalyzed digits in the same loop of guess.
     * If a better Bull is found, change cow for bull and quit processing.
     * (solution for 2016 (t) and 6666 (g) state should be 1b and not 1c
     *
     * 2. Should target be a unique (non repeating) set of numbers?
     */

    /*
    for (int i=0; i<4; i++) { // 1 of target to all of guest
        for(int j=0; j<4; j++) { //guess
   
            printf("DEBUG: comp %c and %c\n", target[i], guess[j]);
            if(target[i] == guess[j]) {
                if (i==j) 
                { 
                    bulls++;
                } 
                else 
                {
                    cows++;
                }
                break; // stop processing the target loop on a match
            }
        }
    }
    */

    // Algo-2: (take the bull by its horns)

    /* Assumption is that target is all unique/non-repeating chars.
     * this avoids the redundant cow that may match a potential unseen bull problem
     * (the real problem is we are not visual but sequentially blind in computing).
     */

    for (int i=0; i<4; i++) { // outerloop is target

        printf("DEBUG: Bull first: comp %c and %c\n", target[i], guess[i]);
        /* search for a B and iterate if found - make use of o(1) array lookup */
        if ((target[i] == guess[i]) && (!visited_guess_bull[i])) {
            bulls++;
            visited_guess_bull[i] = 1;
            continue; // skip to next target[i]
        }

        /* else search for a cow and iterate if found */
        for(int j=0; j<4; j++) { //innerloop is guest
   
            if (visited_guess_bull[j]) continue; // skip a found bull
            printf("DEBUG: comp %c and %c\n", target[i], guess[j]);

            if(target[i] == guess[j]) {
                cows++;
                break; // stop processing and go back to next target (outerloop)
            }
        }
    }
    // In Algo-2 a visited-bull count=4 indicates success, however I have a faster method at the top
   
    
    printf("match: bulls %d cows %d\n", bulls, cows);
    return false;
}
