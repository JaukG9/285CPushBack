#pragma once

#include "main.h"
#include "lemlib/api.hpp"

void skip();
void left_4Rush();
void right_4Rush();
void left_7Rush();
void right_7Rush();
void left_43Split();
void right_43Split();
void left_awp();
void right_awp();
void skills();

class Button{
    public:
        int x, y, width, height;
        std::string text;
        pros::Color buttonColor, textColor;

    Button(int x, int y, int width, int height, std::string text, pros::Color buttonColor, pros::Color textColor)
    : x(x), y(y), width(width), height(height), text(text), buttonColor(buttonColor), textColor(textColor){}

    void render(){
        pros::screen::set_pen(buttonColor);
        pros::screen::fill_rect(x, y, x + width, y + height);
        pros::screen::set_eraser(buttonColor);
        pros::screen::set_pen(textColor);
        pros::screen::print(pros::E_TEXT_MEDIUM, x + 10, y + 10, text.c_str());
    }

    bool isClicked(){
        pros::screen_touch_status_s_t status = pros::c::screen_touch_status();
        int xCoord = status.x;
        int yCoord = status.y;

        if((status.touch_status == pros::E_TOUCH_PRESSED || status.touch_status == pros::E_TOUCH_HELD)
            && xCoord >= x && xCoord <= x + width
            && yCoord >= y && yCoord <= y + height){
                
            while(pros::c::screen_touch_status().touch_status != pros::E_TOUCH_RELEASED){
                pros::delay(10);
            }
            return true;
        }

        return false;
    }
};

extern int autonomousSelection;
void autonSelector(void*);