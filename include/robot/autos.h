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
        const char* cText = text.c_str();
        pros::Color buttonColor, textColor;

    Button(int x, int y, int width, int height, std::string text, pros::Color buttonColor, pros::Color textColor)
    : x(x), y(y), width(width), height(height), text(text), buttonColor(buttonColor), textColor(textColor){}

    void render(){
        pros::screen::set_pen(buttonColor);
        pros::screen::draw_rect(x, y, width, height);
        pros::screen::set_pen(textColor);
        pros::screen::print(pros::E_TEXT_MEDIUM, x + 10, y + 10, cText);
    }

    bool isClicked(){
        pros::screen_touch_status_s_t status = pros::c::screen_touch_status();
        int xCoord = status.x;
        int yCoord = status.y;

        if(status.touch_status && xCoord >= x && yCoord <= x + width && yCoord >= y && yCoord <= y + height){
            return true;
        }

        return false;
    }
};

int autonSelector();