/*
* This is a test for the CIS3105 Embedded Systems Module Coursework
* The purpose of this file is to have a controlled environment to test
* code for the Cortex-M3 MCU 
* Created by Erik Thomas & Zach Wharton
* Date of creation - 15/11/2017
*/
//Imports, needs random number generator (time)
#include <stdio.h>
#include <string.h>
//Setting boolean values
#define TRUE    1
#define FALSE   0
//Joystick direction constants (should be defined in a common .h file)
#define STATIONARY          0
#define JOYSTICK_UP			1
#define JOYSTICK_DOWN		2
#define JOYSTICK_LEFT		3
#define JOYSTICK_RIGHT		4
#define JOYSTICK_SELECT		5

extern unsigned int joystick_int;
int gameOver;
int width = 20;
int height = 10;
time_t time;
//Initialises RNG
srand((unsigned) time(&t));

struct food {
    char shape;
    int x;
    int y;
}

struct snake {
    char shape;
    int x;
    int y;
    int xD;
    int yD;
    //Tail segment set to allow increase in length of snake
    int tailLen;
    int tailMax = 100;
    int tailX[tailMax];
    int tailY[tailMax];
}

snake sSnake;
food sFood;

void snakeInit(void) {
    //Initialises the relevant values for drawing the snake and its food
    gameOver = FALSE;

    sSnake.x = width/2;
    sSnake.y = height/2;
    sSnake.xD = STATIONARY;
    sSnake.yD = STATIONARY;

    sFood.x = rand() % width;
    sFood.y = rand() % height;
}

void snakeDrawing(struct food sFood) {
    //Draws the snake to the LCD screen
    sSnake.shape = 'o';
    sFood.shape = 'x';

    GLCD_DisplayString(sSnake.x, sSnake.y, __FI, sSnake.shape);
    GLCD_DisplayString(sFood.x, sFood.y, __FI, sFood.shape);
    for (int i=0; i<tailLen; i++) {
        GLCD_DisplayString(sSnake.tailX[i], sSnake.tailY[i], __FI, sSnake.shape);
    }   
}

void snakeMovement(void) {
    //Put joystick switch in here to control movement of the snake
    //Clear GLCD on loop in main
    joystick_int = 0;

    switch (joystick_int) {
        case JOYSTICK_UP:
            sSnake.xD=0;
            sSnake.yD=-1;
            joystick_int=0;
            break;
        case JOYSTICK_DOWN:
            sSnake.xD=0;
            sSnake.yD=1;
            joystick_int=0;
            break;
        case JOYSTICK_LEFT:
            sSnake.xD=-1;
            sSnake.yD=0;
            joystick_int=0;
            break;
        case JOYSTICK_RIGHT:
            sSnake.xD=1;
            sSnake.yD=0;
            joystick_int=0;
            break;
}

void snakeLogic(void) {
    //Performs the logical operations to move the snake, read jostick inputs etc.
    int lastX = sSnake.tailX[0];
    int lastY = sSnake.tailY[0];
    int lastX_, lastY_;

    sSnake.tailX[0] = x;
    sSnake.tailY[0] = y;

    for (int i=1; i<sSnake.tailLen; i++) {
        lastX_ = sSnake.tailX[i];
        lastY_ = sSnake.tailY[i];
        tailX[i] = lastX;
        tailY[i] = lastY;
        lastX = lastX_;
        lastY = lastY_;
    }
    snakeMovement();

    if (sSnake.x >= width) {
        sSnake.x = 0;
    } else if (sSnake.x < 0) {
        sSnake.x = width - 1;
    }
    if (sSnake.y >= width) {
        sSnake.y = 0;
    } else if (sSnake.y < 0) {
        sSnake.y = height - 1;
    }

    if (sSnake.x == sFood.x && sSnake.y == sFood.y) {
        sFood.x = rand() % width;
        sFood.y = rand() % height;
        sSnake.tailLen++;
    }
    //Checks if the snake is colliding with itself. If true, game over
    for (int i=0; i<sSnake.tailLen; i++) {
        if (sSnake.tailX[i] == sSnake.x && sSnake.tailY[i] == y) {
            gameOver = TRUE;
        }
    }
}

int main(void) {
    snakeInit();
    //Loops whilst the Microcontroller is running
    while (TRUE) {
        //Then checks to see if the game is over. If true, the game can be reset
        if (!gameOver) {
            //Call drawing of snake first, then movement
            //Maybe test with reset button on board first, then allow multiple games in one session?
            snakeDrawing();
            snakeMovement();
            snakeLogic();
            //Uses delay method from Blinky.c
            delay(5);
        } else {
            gameOver = FALSE;
            snakeInit();
        } 
        //Do other MCU stuff here...
    }
}